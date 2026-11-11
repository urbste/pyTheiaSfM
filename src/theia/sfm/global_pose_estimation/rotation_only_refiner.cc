// Implements MORE (Multi-view Optimization with Rotation-only Reprojection
// Error), Sec. 5.2 Eq. 21-23 and Alg. 2 of Li, Cai, Wu, arXiv:2611.11770.
// Parameters not given in the paper are pyTheia choices. See
// rotation_only_refiner.h for the details and the citations.

#include "theia/sfm/global_pose_estimation/rotation_only_refiner.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>
#include <map>
#include <random>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "theia/sfm/global_pose_estimation/art_position_estimator.h"
#include "theia/sfm/global_pose_estimation/ligt_constraints.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/util/parallel_for.h"

namespace theia {

namespace {

using Matrix3d = Eigen::Matrix3d;
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;

template <typename T>
using Vec = Eigen::Matrix<T, 3, 1>;
template <typename T>
using Mat = Eigen::Matrix<T, 3, 3>;
template <typename T>
using Mat6x3 = Eigen::Matrix<T, 6, 3>;

// Minimum number of shared tracks of a triplet (or of a view pair in the
// two-view case). pyTheia choice, matches the ART default.
constexpr int kMinSharedTracks = 10;
// Regularizer of the norms ||v|| ~ sqrt(||v||^2 + eps), keeps the derivatives
// finite when a cross product vanishes.
constexpr double kNormEps = 1e-30;
// Tiny offset under the square root of the eigen-solver p, avoids a division
// by zero when S is a multiple of the identity.
constexpr double kTinyP = 1e-300;
// Clamp of the cosine in the closed-form eigenvalue (keeps acos differentiable).
constexpr double kMaxCos = 1.0 - 1e-12;
// Relative eigenvalue floor of the two-view matrix P_ij (rank-1 rejection).
constexpr double kMinTwoViewEigenRatio = 1e-10;
// Convergence threshold of the outer loop (radians).
constexpr double kMinRotationChange = 1e-6;
constexpr double kTwoPiOver3 = 2.0 * M_PI / 3.0;

// Scalar (value) part of a double or of a ceres::Jet. Used for the branches of
// the templated code, which must follow the values, not the derivatives.
double Value(const double x) { return x; }
template <int N>
double Value(const ceres::Jet<double, N>& x) {
  return x.a;
}

template <typename T>
Vec<T> ToVec(const Vector3d& v) {
  return Vec<T>(T(v.x()), T(v.y()), T(v.z()));
}

template <typename T>
Mat<T> Skew(const Vec<T>& f) {
  Mat<T> s;
  s(0, 0) = T(0.0);
  s(0, 1) = -f(2);
  s(0, 2) = f(1);
  s(1, 0) = f(2);
  s(1, 1) = T(0.0);
  s(1, 2) = -f(0);
  s(2, 0) = -f(1);
  s(2, 1) = f(0);
  s(2, 2) = T(0.0);
  return s;
}

// World-to-camera rotation matrix of an angle-axis rotation. Eigen and ceres
// both store the matrix column-major.
template <typename T>
Mat<T> RotationFromAngleAxis(const T* angle_axis) {
  Mat<T> rotation;
  ceres::AngleAxisToRotationMatrix(angle_axis, rotation.data());
  return rotation;
}

template <typename T>
T SmoothNorm(const Vec<T>& v) {
  using std::sqrt;
  return sqrt(v.squaredNorm() + T(kNormEps));
}

// Inverse of a 3x3 matrix through its adjugate.
template <typename T>
Mat<T> Inverse3(const Mat<T>& m) {
  Mat<T> adj;
  adj(0, 0) = m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1);
  adj(0, 1) = m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2);
  adj(0, 2) = m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1);
  adj(1, 0) = m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2);
  adj(1, 1) = m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0);
  adj(1, 2) = m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2);
  adj(2, 0) = m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0);
  adj(2, 1) = m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1);
  adj(2, 2) = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
  const T det = m(0, 0) * adj(0, 0) + m(0, 1) * adj(1, 0) + m(0, 2) * adj(2, 0);
  return adj / det;
}

template <typename T>
T ClampCos(const T& x) {
  if (Value(x) > kMaxCos) {
    return T(kMaxCos);
  }
  if (Value(x) < -kMaxCos) {
    return T(-kMaxCos);
  }
  return x;
}

// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix S. The
// eigenvalue is the trigonometric closed form, the vector is the cross product
// of two rows of (S - lambda I) with the largest norm. The sign follows the
// reference direction. Differentiable with respect to the entries of S.
template <typename T>
Vec<T> SmallestEigenvector(const Mat<T>& S, const Vector3d& reference) {
  using std::acos;
  using std::cos;
  using std::sqrt;
  const T q = (S(0, 0) + S(1, 1) + S(2, 2)) / T(3.0);
  Mat<T> D = S;
  D(0, 0) -= q;
  D(1, 1) -= q;
  D(2, 2) -= q;
  const T p2 = D(0, 0) * D(0, 0) + D(1, 1) * D(1, 1) + D(2, 2) * D(2, 2) +
               T(2.0) * (S(0, 1) * S(0, 1) + S(0, 2) * S(0, 2) +
                         S(1, 2) * S(1, 2));
  const T p = sqrt(p2 / T(6.0) + T(kTinyP));
  const Mat<T> B = D / p;
  const T det = B(0, 0) * (B(1, 1) * B(2, 2) - B(1, 2) * B(2, 1)) -
                B(0, 1) * (B(1, 0) * B(2, 2) - B(1, 2) * B(2, 0)) +
                B(0, 2) * (B(1, 0) * B(2, 1) - B(1, 1) * B(2, 0));
  const T r = ClampCos(det / T(2.0));
  const T phi = acos(r) / T(3.0);
  const T lambda = q + T(2.0) * p * cos(phi + T(kTwoPiOver3));

  Mat<T> M = S;
  M(0, 0) -= lambda;
  M(1, 1) -= lambda;
  M(2, 2) -= lambda;
  const Vec<T> r0(M(0, 0), M(0, 1), M(0, 2));
  const Vec<T> r1(M(1, 0), M(1, 1), M(1, 2));
  const Vec<T> r2(M(2, 0), M(2, 1), M(2, 2));
  const Vec<T> c01 = r0.cross(r1);
  const Vec<T> c02 = r0.cross(r2);
  const Vec<T> c12 = r1.cross(r2);
  Vec<T> e = c01;
  double best = Value(c01.squaredNorm());
  if (Value(c02.squaredNorm()) > best) {
    e = c02;
    best = Value(c02.squaredNorm());
  }
  if (Value(c12.squaredNorm()) > best) {
    e = c12;
  }
  e = e / sqrt(e.squaredNorm() + T(kNormEps));
  if (Value(e.dot(ToVec<T>(reference))) < 0.0) {
    e = -e;
  }
  return e;
}

// ComputeLiGTBC (ligt_constraints.cc) with templated rotations and constant
// rays. B = [x_j]_x R_ij x_i a_il^T R_l and C = ||theta^{il}||^2 [x_j]_x R_j,
// with a_il^T = theta^T [x_l]_x and theta = (R_il x_i) x x_l.
template <typename T>
void LiGTBC(const Vector3d& x_i_d,
            const Vector3d& x_j_d,
            const Vector3d& x_l_d,
            const Mat<T>& R_i,
            const Mat<T>& R_j,
            const Mat<T>& R_l,
            Mat<T>* B,
            Mat<T>* C) {
  const Vec<T> x_i = ToVec<T>(x_i_d);
  const Vec<T> x_j = ToVec<T>(x_j_d);
  const Vec<T> x_l = ToVec<T>(x_l_d);
  const Mat<T> R_ij = R_j * R_i.transpose();
  const Mat<T> R_il = R_l * R_i.transpose();
  const Vec<T> theta = (R_il * x_i).cross(x_l);
  const Vec<T> a_il = -(Skew(x_l) * theta);  // column form of a_il
  const Vec<T> u = Skew(x_j) * (R_ij * x_i);
  *B = (u * a_il.transpose()) * R_l;
  *C = theta.squaredNorm() * (Skew(x_j) * R_j);
}

// Blocks of Theorem 1 for one track (same layout as ComputeARTBlocks).
template <typename T>
void ARTBlocks(const Vector3d& x_eta,
               const Vector3d& x_zeta,
               const Vector3d& x_nu,
               const Mat<T>& R_eta,
               const Mat<T>& R_zeta,
               const Mat<T>& R_nu,
               Mat6x3<T>* B_k,
               Mat6x3<T>* C_k) {
  Mat<T> B1, C1, B2, C2;
  LiGTBC(x_eta, x_nu, x_zeta, R_eta, R_nu, R_zeta, &B1, &C1);
  LiGTBC(x_zeta, x_nu, x_eta, R_zeta, R_nu, R_eta, &B2, &C2);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      (*B_k)(r, c) = B1(r, c);
      (*B_k)(r + 3, c) = -(B2(r, c) + C2(r, c));
      (*C_k)(r, c) = C1(r, c);
      (*C_k)(r + 3, c) = C2(r, c);
    }
  }
}

// Theorem 1 with the rotations as parameters: the translations are the
// closed-form function of the rotations. b_il = c_i - c_l is the smallest
// eigenvector of S, b_ij = c_i - c_j = -T b_il, sign from `reference`.
template <typename T>
void Theorem1(const std::vector<Vector3d>& x_i,
              const std::vector<Vector3d>& x_l,
              const std::vector<Vector3d>& x_j,
              const Mat<T>& R_i,
              const Mat<T>& R_l,
              const Mat<T>& R_j,
              const Vector3d& reference,
              Vec<T>* b_il,
              Vec<T>* b_ij) {
  Mat<T> CtC = Mat<T>::Zero();
  Mat<T> CtB = Mat<T>::Zero();
  Mat<T> BtB = Mat<T>::Zero();
  for (size_t k = 0; k < x_i.size(); ++k) {
    Mat6x3<T> B_k, C_k;
    ARTBlocks(x_i[k], x_l[k], x_j[k], R_i, R_l, R_j, &B_k, &C_k);
    CtC += C_k.transpose() * C_k;
    CtB += C_k.transpose() * B_k;
    BtB += B_k.transpose() * B_k;
  }
  const Mat<T> T_mat = Inverse3(CtC) * CtB;
  Mat<T> S = BtB - CtB.transpose() * T_mat;
  S = T(0.5) * (S + S.transpose());
  const Vec<T> e = SmallestEigenvector(S, reference);
  *b_il = e;
  *b_ij = -(T_mat * e);
}

// Two-view translation (TRRM, Eq. 11): smallest eigenvector of
// P_ij = sum theta theta^T, theta = (R_ij x_i) x x_j, in the frame of camera j.
template <typename T>
Vec<T> TwoViewDirection(const std::vector<Vector3d>& x_i,
                        const std::vector<Vector3d>& x_j,
                        const Mat<T>& R_ij,
                        const Vector3d& reference) {
  Mat<T> P = Mat<T>::Zero();
  for (size_t k = 0; k < x_i.size(); ++k) {
    const Vec<T> theta = (R_ij * ToVec<T>(x_i[k])).cross(ToVec<T>(x_j[k]));
    P += theta * theta.transpose();
  }
  return SmallestEigenvector(P, reference);
}

// Eq. 21-22 for the reference pair (a, b) and the target view j, with the
// world-frame baselines b_ab = c_a - c_b and b_aj = c_a - c_j:
//   Y = ||t_ab x x_b|| R_aj x_a + ||x_b x (R_ab x_a)|| t_aj,
//   residual = pi(Y) - pi(x_j),  t_ab = R_b b_ab,  t_aj = R_j b_aj.
template <typename T>
void ThreeViewResidual(const Vector3d& x_a,
                       const Vector3d& x_b,
                       const Mat<T>& R_a,
                       const Mat<T>& R_j,
                       const Mat<T>& R_b,
                       const Vec<T>& b_ab,
                       const Vec<T>& b_aj,
                       const Vector2d& pixel_j,
                       T* residual) {
  const Vec<T> xa = ToVec<T>(x_a);
  const Vec<T> xb = ToVec<T>(x_b);
  const Vec<T> rot_a_t_x_a = R_a.transpose() * xa;
  const Vec<T> r_ab_x_a = R_b * rot_a_t_x_a;
  const Vec<T> r_aj_x_a = R_j * rot_a_t_x_a;
  const Vec<T> t_ab = R_b * b_ab;
  const Vec<T> t_aj = R_j * b_aj;
  const T norm_1 = SmoothNorm<T>(t_ab.cross(xb));
  const T norm_2 = SmoothNorm<T>(xb.cross(r_ab_x_a));
  const Vec<T> y = norm_1 * r_aj_x_a + norm_2 * t_aj;
  residual[0] = y.x() / y.z() - T(pixel_j.x());
  residual[1] = y.y() / y.z() - T(pixel_j.y());
}

// Two-view residual (l = j): t_ij in the frame of camera j,
//   Y = ||x_j x t|| R_ij x_i + ||(R_ij x_i) x x_j|| t,  residual = pi(Y) - pi(x_j).
template <typename T>
void TwoViewResidual(const Vector3d& x_i_d,
                     const Vector3d& x_j_d,
                     const Mat<T>& R_ij,
                     const Vec<T>& t,
                     const Vector2d& pixel_j,
                     T* residual) {
  const Vec<T> r_ij_x_i = R_ij * ToVec<T>(x_i_d);
  const Vec<T> x_j = ToVec<T>(x_j_d);
  const T norm_1 = SmoothNorm<T>(x_j.cross(t));
  const T norm_2 = SmoothNorm<T>(r_ij_x_i.cross(x_j));
  const Vec<T> y = norm_1 * r_ij_x_i + norm_2 * t;
  residual[0] = y.x() / y.z() - T(pixel_j.x());
  residual[1] = y.y() / y.z() - T(pixel_j.y());
}

// Pixel-like coordinates (normalized image plane) of a homogeneous ray.
Vector2d RayToPlane(const Vector3d& x) {
  return Vector2d(x.x() / x.z(), x.y() / x.z());
}

// Cost of one triplet (i, l, j), reference edge (i, l), with parameters the
// angle-axis rotations of i, l and j. The Theorem 1 tracks (the kept tracks of
// the observation filter) define the translations b_il and b_ij as functions of
// the rotations. Each tuple contributes 4 residuals, eps^{il}_{jik} and its
// symmetric term eps^{li}_{jlk}, multiplied by its IRLS weight sqrt(w).
class TripletFunctor {
 public:
  TripletFunctor(std::vector<Vector3d> thm_x_i,
                 std::vector<Vector3d> thm_x_l,
                 std::vector<Vector3d> thm_x_j,
                 std::vector<Vector3d> tup_x_i,
                 std::vector<Vector3d> tup_x_l,
                 std::vector<Vector3d> tup_x_j,
                 const Vector3d& reference)
      : thm_x_i_(std::move(thm_x_i)),
        thm_x_l_(std::move(thm_x_l)),
        thm_x_j_(std::move(thm_x_j)),
        tup_x_i_(std::move(tup_x_i)),
        tup_x_l_(std::move(tup_x_l)),
        tup_x_j_(std::move(tup_x_j)),
        reference_(reference),
        sqrt_weight_(tup_x_i_.size(), 1.0) {
    tup_pixel_.reserve(tup_x_j_.size());
    for (const Vector3d& x : tup_x_j_) {
      tup_pixel_.push_back(RayToPlane(x));
    }
  }

  int NumTuples() const { return static_cast<int>(tup_x_i_.size()); }

  void SetWeights(const std::vector<double>& sqrt_weight) {
    sqrt_weight_ = sqrt_weight;
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const {
    const Mat<T> R_i = RotationFromAngleAxis(parameters[0]);
    const Mat<T> R_l = RotationFromAngleAxis(parameters[1]);
    const Mat<T> R_j = RotationFromAngleAxis(parameters[2]);

    Vec<T> b_il, b_ij;
    Theorem1(thm_x_i_, thm_x_l_, thm_x_j_, R_i, R_l, R_j, reference_, &b_il,
             &b_ij);
    const Vec<T> b_li = -b_il;
    const Vec<T> b_lj = b_ij - b_il;

    for (size_t m = 0; m < tup_x_i_.size(); ++m) {
      T* out = residual + 4 * m;
      ThreeViewResidual(tup_x_i_[m], tup_x_l_[m], R_i, R_j, R_l, b_il, b_ij,
                        tup_pixel_[m], out);
      ThreeViewResidual(tup_x_l_[m], tup_x_i_[m], R_l, R_j, R_i, b_li, b_lj,
                        tup_pixel_[m], out + 2);
      const T w = T(sqrt_weight_[m]);
      for (int r = 0; r < 4; ++r) {
        out[r] *= w;
      }
    }
    return true;
  }

 private:
  std::vector<Vector3d> thm_x_i_;
  std::vector<Vector3d> thm_x_l_;
  std::vector<Vector3d> thm_x_j_;
  std::vector<Vector3d> tup_x_i_;
  std::vector<Vector3d> tup_x_l_;
  std::vector<Vector3d> tup_x_j_;
  std::vector<Vector2d> tup_pixel_;
  Vector3d reference_;
  std::vector<double> sqrt_weight_;
};

// Cost of the two-view tuples of an ordered view pair (i, j). Parameters are
// the angle-axis rotations of i and j; t_ij is the smallest eigenvector of the
// shared-track matrix P_ij, which is evaluated inside the residual.
class TwoViewFunctor {
 public:
  TwoViewFunctor(std::vector<Vector3d> thm_x_i,
                 std::vector<Vector3d> thm_x_j,
                 std::vector<Vector3d> tup_x_i,
                 std::vector<Vector3d> tup_x_j,
                 const Vector3d& reference)
      : thm_x_i_(std::move(thm_x_i)),
        thm_x_j_(std::move(thm_x_j)),
        tup_x_i_(std::move(tup_x_i)),
        tup_x_j_(std::move(tup_x_j)),
        reference_(reference),
        sqrt_weight_(tup_x_i_.size(), 1.0) {
    tup_pixel_.reserve(tup_x_j_.size());
    for (const Vector3d& x : tup_x_j_) {
      tup_pixel_.push_back(RayToPlane(x));
    }
  }

  int NumTuples() const { return static_cast<int>(tup_x_i_.size()); }

  void SetWeights(const std::vector<double>& sqrt_weight) {
    sqrt_weight_ = sqrt_weight;
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const {
    const Mat<T> R_i = RotationFromAngleAxis(parameters[0]);
    const Mat<T> R_j = RotationFromAngleAxis(parameters[1]);
    const Mat<T> R_ij = R_j * R_i.transpose();
    const Vec<T> t = TwoViewDirection(thm_x_i_, thm_x_j_, R_ij, reference_);

    for (size_t m = 0; m < tup_x_i_.size(); ++m) {
      T* out = residual + 2 * m;
      TwoViewResidual(tup_x_i_[m], tup_x_j_[m], R_ij, t, tup_pixel_[m], out);
      const T w = T(sqrt_weight_[m]);
      out[0] *= w;
      out[1] *= w;
    }
    return true;
  }

 private:
  std::vector<Vector3d> thm_x_i_;
  std::vector<Vector3d> thm_x_j_;
  std::vector<Vector3d> tup_x_i_;
  std::vector<Vector3d> tup_x_j_;
  std::vector<Vector2d> tup_pixel_;
  Vector3d reference_;
  std::vector<double> sqrt_weight_;
};

// Sets the Cauchy IRLS weights w = 1 / (1 + s / c^2), c = loss_scale_px / f_j,
// of the tuples of `functor`, evaluated at the parameters (constant during the
// Ceres solve). s is the squared residual per term, averaged over the `terms`
// two-vector terms of a tuple.
template <typename Functor>
void SetCauchyWeights(const std::vector<double>& focal_j,
                      const double loss_scale_px,
                      const int terms,
                      const double* const* parameters,
                      Functor* functor) {
  const int n = functor->NumTuples();
  std::vector<double> residuals(static_cast<size_t>(2 * terms * n));
  functor->template operator()<double>(parameters, residuals.data());
  std::vector<double> sqrt_weight(n);
  for (int m = 0; m < n; ++m) {
    double sum_sq = 0.0;
    for (int r = 0; r < 2 * terms; ++r) {
      const double v = residuals[2 * terms * m + r];
      sum_sq += v * v;
    }
    const double s = sum_sq / terms;
    const double c = loss_scale_px / focal_j[m];
    sqrt_weight[m] = std::sqrt(1.0 / (1.0 + s / (c * c)));
  }
  functor->SetWeights(sqrt_weight);
}

Matrix3d AngleAxisToMatrix(const Vector3d& angle_axis) {
  Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(angle_axis.data(), rotation.data());
  return rotation;
}

// One feature of a track in a view that has an orientation.
struct Observation {
  ViewId view_id = kInvalidViewId;
  Vector3d ray = Vector3d::Zero();  // normalized homogeneous ray (x, y, 1)
  double focal = 0.0;               // focal length of the view (pixels)
};

// The observations of one track in oriented views, sorted by ViewId.
struct TrackData {
  TrackId track_id = kInvalidTrackId;
  std::vector<Observation> observations;
};

const Observation* FindObservation(const TrackData& track, const ViewId view_id) {
  // The observations are sorted by ViewId.
  const auto it = std::lower_bound(
      track.observations.begin(), track.observations.end(), view_id,
      [](const Observation& obs, const ViewId id) { return obs.view_id < id; });
  if (it != track.observations.end() && it->view_id == view_id) {
    return &*it;
  }
  return nullptr;
}

// Theorem 1 solution of the triplet (i, l, j) at the current rotations, after
// the observation filter. b_il = c_i - c_l and b_ij = c_i - c_j (chirality
// fixed). The kept tracks are the Theorem 1 tracks of the cost function.
struct TripletSolve {
  bool valid = false;
  Vector3d b_il = Vector3d::Zero();
  Vector3d b_ij = Vector3d::Zero();
  std::vector<Vector3d> x_i, x_l, x_j;
  std::set<TrackId> kept;
};

// Two-view solution of the ordered pair (i, j) at the current rotations.
// t_j = t_ij in the frame of camera j, chirality fixed.
struct TwoViewSolve {
  bool valid = false;
  Vector3d t_j = Vector3d::Zero();
  std::vector<Vector3d> x_i, x_j;
};

// One residual tuple (Alg. 2 step 1): target view j of a track, reference pair
// (i, l) (three-view) or the single view i (two-view, l is unused).
struct Tuple {
  bool two_view = false;
  TrackId track_id = kInvalidTrackId;
  ViewId i = kInvalidViewId;
  ViewId l = kInvalidViewId;
  ViewId j = kInvalidViewId;
  Vector3d x_i = Vector3d::Zero();
  Vector3d x_l = Vector3d::Zero();
  Vector3d x_j = Vector3d::Zero();
  double focal_j = 0.0;
  double theta_norm = 0.0;
};

// A candidate tuple of one observation (before the residual gate).
struct Proposal {
  bool two_view = false;
  TrackId track_id = kInvalidTrackId;
  ViewId i = kInvalidViewId;
  ViewId l = kInvalidViewId;
  ViewId j = kInvalidViewId;
  Vector3d x_i = Vector3d::Zero();
  Vector3d x_l = Vector3d::Zero();
  Vector3d x_j = Vector3d::Zero();
  double focal_j = 0.0;
  double theta = 0.0;
  double residual_px = 0.0;  // +inf if not finite
};

// Candidates of one observation (target view j), each list in order of
// decreasing theta.
struct ObservationProposals {
  ViewId j = kInvalidViewId;
  std::vector<Proposal> triplet;
  std::vector<Proposal> two_view;
};

double MedianOf(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double median = values[mid];
  if (values.size() % 2 == 0) {
    const double lower = *std::max_element(values.begin(), values.begin() + mid);
    median = 0.5 * (median + lower);
  }
  return median;
}


// Reference-pair candidate of one observation: theta and the indices of the
// two other observations of the track (a < b), or a = b for two-view.
struct Candidate {
  double theta;
  size_t a;
  size_t b;
};

// Candidates of one observation jn of a track (the top max_reference_candidates
// per kind, in order of decreasing theta).
struct ObsCandidates {
  size_t jn = 0;
  std::vector<Candidate> triplet;
  std::vector<Candidate> two_view;
};

using TripletGroups = std::map<std::array<ViewId, 3>, std::vector<Tuple>>;
using PairGroups = std::map<ViewIdPair, std::vector<Tuple>>;

// Builds the tuples of Alg. 2 step 1 for the current rotations, and the
// Theorem 1 solutions they use (cached per triplet and per view pair).
class TupleBuilder {
 public:
  TupleBuilder(const RotationOnlyRefinerOptions& options,
               const std::vector<TrackData>& tracks,
               const std::unordered_map<ViewId, std::vector<int>>& view_tracks,
               const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
               const std::unordered_map<ViewId, Matrix3d>& rotations)
      : options_(options),
        tracks_(tracks),
        view_tracks_(view_tracks),
        view_pairs_(view_pairs),
        rotations_(rotations) {}

  void Build(TripletGroups* triplet_groups, PairGroups* pair_groups);

  // Residual (px, max of both terms) of the tuple of the triplet solution at
  // the current rotations, for the target observation oj.
  double TripletPx(const TripletSolve& solve,
                   const Observation& oi,
                   const Observation& ol,
                   const Observation& oj) const;
  // Residual (px) of the two-view solution for the pair (oi, oj).
  double TwoViewPx(const TwoViewSolve& solve,
                   const Observation& oi,
                   const Observation& oj) const;

  // Triplet (i, l, j) with reference edge (i, l).
  const TripletSolve& GetTriplet(const ViewId i, const ViewId l, const ViewId j);
  // Ordered two-view pair (i, j).
  const TwoViewSolve& GetTwoView(const ViewId i, const ViewId j);
  // Uncached solutions (pure functions of the inputs, thread-safe).
  TripletSolve ComputeTriplet(const ViewId i, const ViewId l, const ViewId j) const;
  TwoViewSolve ComputeTwoView(const ViewId i, const ViewId j) const;

 private:
  // Ascending indices into tracks_ of the tracks observed in the view, or null.
  const std::vector<int>* ViewTrackList(const ViewId view_id) const {
    const auto it = view_tracks_.find(view_id);
    return it == view_tracks_.end() ? nullptr : &it->second;
  }

  bool HasEdge(const ViewId a, const ViewId b) const {
    return view_pairs_.count(ViewIdPair(a, b)) > 0 ||
           view_pairs_.count(ViewIdPair(b, a)) > 0;
  }

  const Matrix3d& Rotation(const ViewId view_id) const {
    return rotations_.at(view_id);
  }

  const RotationOnlyRefinerOptions& options_;
  const std::vector<TrackData>& tracks_;
  const std::unordered_map<ViewId, std::vector<int>>& view_tracks_;
  const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs_;
  const std::unordered_map<ViewId, Matrix3d>& rotations_;

  ARTPositionEstimator::Options art_options_;
  std::map<std::array<ViewId, 3>, TripletSolve> triplet_cache_;
  std::map<ViewIdPair, TwoViewSolve> two_view_cache_;

 public:
};

const TripletSolve& TupleBuilder::GetTriplet(const ViewId i,
                                             const ViewId l,
                                             const ViewId j) {
  const std::array<ViewId, 3> key = {i, l, j};
  const auto it = triplet_cache_.find(key);
  if (it != triplet_cache_.end()) {
    return it->second;
  }
  return triplet_cache_.emplace(key, ComputeTriplet(i, l, j)).first->second;
}

TripletSolve TupleBuilder::ComputeTriplet(const ViewId i,
                                          const ViewId l,
                                          const ViewId j) const {
  TripletSolve entry;
  std::vector<TrackId> ids;
  std::vector<Vector3d> x_i, x_l, x_j;
  // Tracks seen in i, l and j, in ascending track index (3-way merge of the
  // sorted per-view track lists).
  const std::vector<int>* lists[3] = {ViewTrackList(i), ViewTrackList(l),
                                      ViewTrackList(j)};
  if (lists[0] != nullptr && lists[1] != nullptr && lists[2] != nullptr) {
    size_t p[3] = {0, 0, 0};
    while (p[0] < lists[0]->size() && p[1] < lists[1]->size() &&
           p[2] < lists[2]->size()) {
      const int k0 = (*lists[0])[p[0]];
      const int k1 = (*lists[1])[p[1]];
      const int k2 = (*lists[2])[p[2]];
      if (k0 == k1 && k1 == k2) {
        const TrackData& track = tracks_[k0];
        ids.push_back(track.track_id);
        x_i.push_back(FindObservation(track, i)->ray);
        x_l.push_back(FindObservation(track, l)->ray);
        x_j.push_back(FindObservation(track, j)->ray);
        if (static_cast<int>(x_i.size()) >= options_.max_tracks_per_triplet) {
          break;
        }
        ++p[0];
        ++p[1];
        ++p[2];
        continue;
      }
      const int m = std::max(k0, std::max(k1, k2));
      for (int q = 0; q < 3; ++q) {
        if ((*lists[q])[p[q]] < m) {
          ++p[q];
        }
      }
    }
  }
  if (static_cast<int>(x_i.size()) >= kMinSharedTracks) {
    ARTTripletSolution sol;
    std::vector<int> kept;
    if (SolveARTTripletFiltered(x_i, x_l, x_j, Rotation(i), Rotation(l),
                                Rotation(j), art_options_, &sol, &kept)) {
      entry.valid = sol.b_ez.allFinite() && sol.b_en.allFinite();
      entry.b_il = sol.b_ez;
      entry.b_ij = sol.b_en;
      for (const int k : kept) {
        entry.kept.insert(ids[k]);
        entry.x_i.push_back(x_i[k]);
        entry.x_l.push_back(x_l[k]);
        entry.x_j.push_back(x_j[k]);
      }
    }
  }
  return entry;
}

const TwoViewSolve& TupleBuilder::GetTwoView(const ViewId i, const ViewId j) {
  const ViewIdPair key(i, j);
  const auto it = two_view_cache_.find(key);
  if (it != two_view_cache_.end()) {
    return it->second;
  }
  return two_view_cache_.emplace(key, ComputeTwoView(i, j)).first->second;
}

TwoViewSolve TupleBuilder::ComputeTwoView(const ViewId i, const ViewId j) const {
  TwoViewSolve entry;
  std::vector<Vector3d> x_i, x_j;
  const std::vector<int>* lists[2] = {ViewTrackList(i), ViewTrackList(j)};
  if (lists[0] != nullptr && lists[1] != nullptr) {
    size_t p[2] = {0, 0};
    while (p[0] < lists[0]->size() && p[1] < lists[1]->size()) {
      const int k0 = (*lists[0])[p[0]];
      const int k1 = (*lists[1])[p[1]];
      if (k0 == k1) {
        const TrackData& track = tracks_[k0];
        x_i.push_back(FindObservation(track, i)->ray);
        x_j.push_back(FindObservation(track, j)->ray);
        if (static_cast<int>(x_i.size()) >= options_.max_tracks_per_triplet) {
          break;
        }
        ++p[0];
        ++p[1];
        continue;
      }
      if (k0 < k1) {
        ++p[0];
      } else {
        ++p[1];
      }
    }
  }

  if (static_cast<int>(x_i.size()) >= kMinSharedTracks) {
    Vector3d t;
    if (SolveTwoViewTranslation(x_i, x_j, Rotation(i), Rotation(j),
                                kMinTwoViewEigenRatio, &t, nullptr)) {
      entry.t_j = t;
      entry.valid = true;
      entry.x_i = x_i;
      entry.x_j = x_j;
    }
  }
  return entry;
}

double TupleBuilder::TripletPx(const TripletSolve& solve,
                               const Observation& oi,
                               const Observation& ol,
                               const Observation& oj) const {
  const Matrix3d R_i = Rotation(oi.view_id);
  const Matrix3d R_l = Rotation(ol.view_id);
  const Matrix3d R_j = Rotation(oj.view_id);
  const Vector3d b_il = solve.b_il;
  const Vector3d b_ij = solve.b_ij;
  const Vector3d b_li = -solve.b_il;
  const Vector3d b_lj = solve.b_ij - solve.b_il;
  const Vector2d pixel = RayToPlane(oj.ray);
  double r[4];
  ThreeViewResidual<double>(oi.ray, ol.ray, R_i, R_j, R_l, b_il, b_ij, pixel, r);
  ThreeViewResidual<double>(ol.ray, oi.ray, R_l, R_j, R_i, b_li, b_lj, pixel,
                            r + 2);
  const double px = std::max(Vector2d(r[0], r[1]).norm(),
                             Vector2d(r[2], r[3]).norm()) * oj.focal;
  return std::isfinite(px) ? px : std::numeric_limits<double>::infinity();
}

double TupleBuilder::TwoViewPx(const TwoViewSolve& solve,
                               const Observation& oi,
                               const Observation& oj) const {
  const Matrix3d R_ij = GetRij(Rotation(oi.view_id), Rotation(oj.view_id));
  const Vector3d t = solve.t_j;
  const Vector2d pixel = RayToPlane(oj.ray);
  double r[2];
  TwoViewResidual<double>(oi.ray, oj.ray, R_ij, t, pixel, r);
  const double px = Vector2d(r[0], r[1]).norm() * oj.focal;
  return std::isfinite(px) ? px : std::numeric_limits<double>::infinity();
}

void TupleBuilder::Build(TripletGroups* triplet_groups, PairGroups* pair_groups) {
  CHECK_NOTNULL(triplet_groups)->clear();
  CHECK_NOTNULL(pair_groups)->clear();
  const size_t max_ref =
      static_cast<size_t>(std::max(1, options_.max_reference_candidates));
  const int num_threads = options_.num_threads;
  const size_t num_tracks = tracks_.size();

  // Step 1a (parallel over tracks): the candidate pairs of every observation,
  // in order of decreasing theta (at most max_ref per observation and kind).
  // Which pairs are examined does not depend on the Theorem 1 solutions, so
  // the solutions needed are known before they are computed.
  std::vector<std::vector<ObsCandidates>> cands(num_tracks);
  ParallelFor(num_threads, num_tracks, [&](const size_t t) {
    const std::vector<Observation>& obs = tracks_[t].observations;
    const size_t n = obs.size();
    if (n < 2) {
      return;
    }
    cands[t].resize(n);
    for (size_t jn = 0; jn < n; ++jn) {
      ObsCandidates& oc = cands[t][jn];
      oc.jn = jn;
      // Three-view candidates: reference pairs (a, b), a < b, a, b != j.
      std::vector<Candidate> candidates;
      for (size_t a = 0; a < n; ++a) {
        if (a == jn) {
          continue;
        }
        for (size_t b = a + 1; b < n; ++b) {
          if (b == jn) {
            continue;
          }
          const Observation& oi = obs[a];
          const Observation& ol = obs[b];
          if (options_.require_view_graph_edges &&
              !HasEdge(oi.view_id, ol.view_id)) {
            continue;
          }
          // theta^{il} = (R_il x_i) x x_l, with R_il = R_l R_i^T.
          const Matrix3d R_il = GetRij(Rotation(oi.view_id), Rotation(ol.view_id));
          const double theta = (R_il * oi.ray).cross(ol.ray).norm();
          candidates.push_back({theta, a, b});
        }
      }
      std::sort(candidates.begin(), candidates.end(),
                [](const Candidate& x, const Candidate& y) {
                  if (x.theta != y.theta) {
                    return x.theta > y.theta;
                  }
                  return x.a != y.a ? x.a < y.a : x.b < y.b;
                });
      const size_t num_triplet = std::min(candidates.size(), max_ref);
      oc.triplet.assign(candidates.begin(), candidates.begin() + num_triplet);

      // Two-view candidates (l = j): the single view i with the largest
      // ||theta^{ij}|| = ||(R_ij x_i) x x_j||.
      if (options_.use_two_view_tuples) {
        std::vector<Candidate> two_view_candidates;
        for (size_t a = 0; a < n; ++a) {
          if (a == jn) {
            continue;
          }
          if (options_.require_view_graph_edges &&
              !HasEdge(obs[a].view_id, obs[jn].view_id)) {
            continue;
          }
          const Matrix3d R_ij = GetRij(Rotation(obs[a].view_id), Rotation(obs[jn].view_id));
          const double theta = (R_ij * obs[a].ray).cross(obs[jn].ray).norm();
          two_view_candidates.push_back({theta, a, a});
        }
        std::sort(two_view_candidates.begin(), two_view_candidates.end(),
                  [](const Candidate& x, const Candidate& y) {
                    if (x.theta != y.theta) {
                      return x.theta > y.theta;
                    }
                    return x.a < y.a;
                  });
        const size_t num_two = std::min(two_view_candidates.size(), max_ref);
        oc.two_view.assign(two_view_candidates.begin(),
                           two_view_candidates.begin() + num_two);
      }
    }
  });

  // Step 1b (serial): the distinct triplets and view pairs that are examined.
  std::vector<std::array<ViewId, 3>> tkeys;
  std::vector<ViewIdPair> pkeys;
  for (size_t t = 0; t < num_tracks; ++t) {
    const std::vector<Observation>& obs = tracks_[t].observations;
    for (const ObsCandidates& oc : cands[t]) {
      const ViewId j = obs[oc.jn].view_id;
      for (const Candidate& c : oc.triplet) {
        tkeys.push_back({obs[c.a].view_id, obs[c.b].view_id, j});
      }
      for (const Candidate& c : oc.two_view) {
        pkeys.push_back(ViewIdPair(obs[c.a].view_id, j));
      }
    }
  }
  // Sorted and distinct (the caches are empty at the start of Build).
  std::sort(tkeys.begin(), tkeys.end());
  tkeys.erase(std::unique(tkeys.begin(), tkeys.end()), tkeys.end());
  std::sort(pkeys.begin(), pkeys.end());
  pkeys.erase(std::unique(pkeys.begin(), pkeys.end()), pkeys.end());

  // Step 1c (parallel over the keys): the Theorem 1 solutions, cached. Each
  // solution is a pure function of its key, so the cache does not depend on
  // the thread count.
  std::vector<TripletSolve> tsolves(tkeys.size());
  ParallelFor(num_threads, tkeys.size(), [&](const size_t k) {
    tsolves[k] = ComputeTriplet(tkeys[k][0], tkeys[k][1], tkeys[k][2]);
  });
  for (size_t k = 0; k < tkeys.size(); ++k) {
    triplet_cache_.emplace(tkeys[k], std::move(tsolves[k]));
  }
  std::vector<TwoViewSolve> psolves(pkeys.size());
  ParallelFor(num_threads, pkeys.size(), [&](const size_t k) {
    psolves[k] = ComputeTwoView(pkeys[k].first, pkeys[k].second);
  });
  for (size_t k = 0; k < pkeys.size(); ++k) {
    two_view_cache_.emplace(pkeys[k], std::move(psolves[k]));
  }

  // Step 1d (parallel over tracks): the proposals, with the cached solutions.
  std::vector<std::vector<ObservationProposals>> props(num_tracks);
  ParallelFor(num_threads, num_tracks, [&](const size_t t) {
    const TrackData& track = tracks_[t];
    const std::vector<Observation>& obs = track.observations;
    for (const ObsCandidates& oc : cands[t]) {
      const size_t jn = oc.jn;
      const Observation& oj = obs[jn];
      ObservationProposals op;
      op.j = oj.view_id;
      for (const Candidate& c : oc.triplet) {
        const Observation& oi = obs[c.a];
        const Observation& ol = obs[c.b];
        const TripletSolve& entry = triplet_cache_.at(
            {oi.view_id, ol.view_id, oj.view_id});
        // The observation must pass the filter of the triplet.
        if (!entry.valid || entry.kept.count(track.track_id) == 0) {
          continue;
        }
        Proposal p;
        p.two_view = false;
        p.track_id = track.track_id;
        p.i = oi.view_id;
        p.l = ol.view_id;
        p.j = oj.view_id;
        p.x_i = oi.ray;
        p.x_l = ol.ray;
        p.x_j = oj.ray;
        p.focal_j = oj.focal;
        p.theta = c.theta;
        p.residual_px = TripletPx(entry, oi, ol, oj);
        op.triplet.push_back(p);
      }
      for (const Candidate& c : oc.two_view) {
        const Observation& oi = obs[c.a];
        const TwoViewSolve& entry = two_view_cache_.at(ViewIdPair(oi.view_id, oj.view_id));
        if (!entry.valid) {
          continue;
        }
        Proposal p;
        p.two_view = true;
        p.track_id = track.track_id;
        p.i = oi.view_id;
        p.l = kInvalidViewId;
        p.j = oj.view_id;
        p.x_i = oi.ray;
        p.x_l = Vector3d::Zero();
        p.x_j = oj.ray;
        p.focal_j = oj.focal;
        p.theta = c.theta;
        p.residual_px = TwoViewPx(entry, oi, oj);
        op.two_view.push_back(p);
      }
      props[t].push_back(std::move(op));
    }
  });
  std::vector<ObservationProposals> proposals;
  proposals.reserve(num_tracks);
  for (std::vector<ObservationProposals>& list : props) {
    for (ObservationProposals& op : list) {
      proposals.push_back(std::move(op));
    }
  }

  // Step 2: per observation, the candidate with the smallest residual (among
  // the three-view candidates, else the two-view candidates). The residual gate
  // is max(tuple_gate_px, 3 * 1.4826 * MAD) over these per-observation minima.
  std::vector<const Proposal*> best(proposals.size(), nullptr);
  std::vector<double> minima;
  for (size_t o = 0; o < proposals.size(); ++o) {
    const ObservationProposals& op = proposals[o];
    const std::vector<Proposal>& list = !op.triplet.empty() ? op.triplet : op.two_view;
    for (const Proposal& p : list) {
      if (std::isfinite(p.residual_px) &&
          (best[o] == nullptr || p.residual_px < best[o]->residual_px)) {
        best[o] = &p;
      }
    }
    if (best[o] != nullptr) {
      minima.push_back(best[o]->residual_px);
    }
  }
  if (minima.empty()) {
    return;
  }
  const double median = MedianOf(minima);
  std::vector<double> deviations;
  deviations.reserve(minima.size());
  for (const double r : minima) {
    deviations.push_back(std::abs(r - median));
  }
  const double mad = MedianOf(deviations);
  const double gate =
      std::max(options_.tuple_gate_px, 3.0 * 1.4826 * mad);
  VLOG(1) << "Rotation-only tuple gate: " << gate << " px (MAD " << mad
          << " px, " << minima.size() << " observations).";

  // Step 3: accept the best candidate of each observation if it passes the gate.
  std::map<ViewId, std::vector<Tuple>> per_view;
  for (size_t o = 0; o < proposals.size(); ++o) {
    const Proposal* chosen = best[o];
    if (chosen == nullptr || chosen->residual_px > gate) {
      continue;
    }
    Tuple tuple;
    tuple.two_view = chosen->two_view;
    tuple.track_id = chosen->track_id;
    tuple.i = chosen->i;
    tuple.l = chosen->l;
    tuple.j = chosen->j;
    tuple.x_i = chosen->x_i;
    tuple.x_l = chosen->x_l;
    tuple.x_j = chosen->x_j;
    tuple.focal_j = chosen->focal_j;
    tuple.theta_norm = chosen->theta;
    per_view[proposals[o].j].push_back(tuple);
  }

  // Cap the number of tuples per target view, keeping the largest ||theta||,
  // and group the tuples by their triplet (or view pair).
  for (auto& entry : per_view) {
    std::vector<Tuple>& list = entry.second;
    std::sort(list.begin(), list.end(), [](const Tuple& x, const Tuple& y) {
      if (x.theta_norm != y.theta_norm) {
        return x.theta_norm > y.theta_norm;
      }
      return x.track_id < y.track_id;
    });
    if (static_cast<int>(list.size()) > options_.max_observations_per_view) {
      list.resize(options_.max_observations_per_view);
    }
    for (const Tuple& tuple : list) {
      if (tuple.two_view) {
        (*pair_groups)[ViewIdPair(tuple.i, tuple.j)].push_back(tuple);
      } else {
        (*triplet_groups)[{tuple.i, tuple.l, tuple.j}].push_back(tuple);
      }
    }
  }
}

// Angle (radians) of the relative rotation between two angle-axis rotations.
double RotationChange(const Vector3d& before, const Vector3d& after) {
  const Matrix3d delta = AngleAxisToMatrix(before).transpose() * AngleAxisToMatrix(after);
  return Eigen::AngleAxisd(delta).angle();
}

// Epipolar verification of the observations at the initial rotations (pyTheia
// addition). For each view pair with enough shared tracks, the two-view
// translation is estimated robustly (least median of squares over minimal
// solutions t = theta_p x theta_q, theta_k = (R_ab x_ak) x x_bk), and every
// shared track gets the symmetric epipolar distance in pixels. A pair is
// consistent for a track if the distance is below
// max(verify_px, kVerifyMedianFactor * median distance of the pair). An
// observation is removed if more of its pairs are inconsistent than
// consistent. Matches that went through two-view verification rarely fail
// this; unverified mismatches (random pixels) do, and they would otherwise
// enter the Theorem 1 solves inside the residuals.
// Returns the number of removed observations.
int VerifyObservations(const RotationOnlyRefinerOptions& options,
                       const std::unordered_map<ViewId, Vector3d>& orientations,
                       std::vector<TrackData>* tracks) {
  constexpr int kMinPairTracks = 8;
  constexpr int kNumSamples = 50;
  constexpr double kVerifyMedianFactor = 5.0;
  constexpr double kTiny = 1e-12;
  std::unordered_map<ViewId, Matrix3d> R;
  for (const auto& entry : orientations) {
    R[entry.first] = AngleAxisToMatrix(entry.second);
  }
  // (track, obs a, obs b) per view pair, a < b in view id order.
  struct Entry {
    int track;
    int a;
    int b;
  };
  std::map<ViewIdPair, std::vector<Entry>> pairs;
  for (int t = 0; t < static_cast<int>(tracks->size()); ++t) {
    const auto& obs = (*tracks)[t].observations;
    for (int a = 0; a < static_cast<int>(obs.size()); ++a) {
      for (int b = a + 1; b < static_cast<int>(obs.size()); ++b) {
        pairs[ViewIdPair(obs[a].view_id, obs[b].view_id)].push_back({t, a, b});
      }
    }
  }
  std::vector<const std::pair<const ViewIdPair, std::vector<Entry>>*> pair_list;
  for (const auto& entry : pairs) {
    if (static_cast<int>(entry.second.size()) >= kMinPairTracks) {
      pair_list.push_back(&entry);
    }
  }
  // Per pair and entry: +1 consistent, -1 inconsistent.
  std::vector<std::vector<int8_t>> verdicts(pair_list.size());
  ParallelFor(options.num_threads, pair_list.size(), [&](const size_t p) {
    const ViewIdPair& key = pair_list[p]->first;
    const std::vector<Entry>& entries = pair_list[p]->second;
    const Matrix3d R_ab = GetRij(R.at(key.first), R.at(key.second));
    const size_t n = entries.size();
    std::vector<Vector3d> rx_a(n), x_b(n), theta(n);
    std::vector<double> f_a(n), f_b(n);
    for (size_t k = 0; k < n; ++k) {
      const auto& obs = (*tracks)[entries[k].track].observations;
      rx_a[k] = R_ab * obs[entries[k].a].ray;
      x_b[k] = obs[entries[k].b].ray;
      f_a[k] = obs[entries[k].a].focal;
      f_b[k] = obs[entries[k].b].focal;
      theta[k] = rx_a[k].cross(x_b[k]);
    }
    // Symmetric epipolar distance (pixels) of entry k for translation t (the
    // translation in the frame of b): lines t x R x_a in b and
    // R^T (x_b x t) in a, both with algebraic residual t . theta_k.
    auto distance = [&](const Vector3d& t, const size_t k) {
      const double algebraic = std::abs(t.dot(theta[k]));
      const Vector3d line_b = t.cross(rx_a[k]);
      const Vector3d line_a = R_ab.transpose() * x_b[k].cross(t);
      const double nb = line_b.head<2>().norm();
      const double na = line_a.head<2>().norm();
      const double d_b = nb > kTiny ? f_b[k] * algebraic / nb : 1e12;
      const double d_a = na > kTiny ? f_a[k] * algebraic / na : 1e12;
      return std::max(d_a, d_b);
    };
    std::mt19937 rng(static_cast<unsigned int>(key.first) * 92821u +
                     static_cast<unsigned int>(key.second));
    std::uniform_int_distribution<size_t> pick(0, n - 1);
    double best_median = std::numeric_limits<double>::infinity();
    std::vector<double> best_d;
    std::vector<double> d(n);
    for (int s = 0; s < kNumSamples; ++s) {
      const size_t p0 = pick(rng);
      const size_t p1 = pick(rng);
      Vector3d t = theta[p0].cross(theta[p1]);
      if (p0 == p1 || !(t.norm() > kTiny)) {
        continue;
      }
      t.normalize();
      for (size_t k = 0; k < n; ++k) {
        d[k] = distance(t, k);
      }
      const double median = MedianOf(d);
      if (median < best_median) {
        best_median = median;
        best_d = d;
      }
    }
    if (best_d.empty()) {
      return;
    }
    const double threshold =
        std::max(options.verify_observations_px,
                 kVerifyMedianFactor * best_median);
    verdicts[p].resize(n);
    for (size_t k = 0; k < n; ++k) {
      verdicts[p][k] = best_d[k] <= threshold ? 1 : -1;
    }
  });
  // Votes per observation.
  std::vector<std::vector<int>> consistent(tracks->size()),
      inconsistent(tracks->size());
  for (size_t t = 0; t < tracks->size(); ++t) {
    consistent[t].assign((*tracks)[t].observations.size(), 0);
    inconsistent[t].assign((*tracks)[t].observations.size(), 0);
  }
  for (size_t p = 0; p < pair_list.size(); ++p) {
    const std::vector<Entry>& entries = pair_list[p]->second;
    for (size_t k = 0; k < verdicts[p].size(); ++k) {
      const Entry& e = entries[k];
      auto& counter = verdicts[p][k] > 0 ? consistent : inconsistent;
      ++counter[e.track][e.a];
      ++counter[e.track][e.b];
    }
  }
  int num_removed = 0;
  for (size_t t = 0; t < tracks->size(); ++t) {
    std::vector<Observation>& obs = (*tracks)[t].observations;
    std::vector<Observation> kept;
    for (size_t a = 0; a < obs.size(); ++a) {
      if (inconsistent[t][a] > consistent[t][a]) {
        ++num_removed;
      } else {
        kept.push_back(obs[a]);
      }
    }
    obs.swap(kept);
  }
  return num_removed;
}

}  // namespace

namespace rotation_only_internal {

bool ComputeTheorem1Baselines(const std::vector<Vector3d>& x_i,
                              const std::vector<Vector3d>& x_l,
                              const std::vector<Vector3d>& x_j,
                              const Vector3d& angle_axis_i,
                              const Vector3d& angle_axis_l,
                              const Vector3d& angle_axis_j,
                              const Vector3d& reference,
                              Vector3d* b_il,
                              Vector3d* b_ij) {
  CHECK_NOTNULL(b_il);
  CHECK_NOTNULL(b_ij);
  if (x_i.empty() || x_l.size() != x_i.size() || x_j.size() != x_i.size()) {
    return false;
  }
  const Mat<double> R_i = RotationFromAngleAxis(angle_axis_i.data());
  const Mat<double> R_l = RotationFromAngleAxis(angle_axis_l.data());
  const Mat<double> R_j = RotationFromAngleAxis(angle_axis_j.data());
  Vec<double> bil, bij;
  Theorem1(x_i, x_l, x_j, R_i, R_l, R_j, reference, &bil, &bij);
  *b_il = bil;
  *b_ij = bij;
  return b_il->allFinite() && b_ij->allFinite();
}

ceres::CostFunction* CreateTripletCostFunctionForTest(
    const std::vector<Vector3d>& x_i,
    const std::vector<Vector3d>& x_l,
    const std::vector<Vector3d>& x_j,
    const Vector3d& reference) {
  auto* functor = new TripletFunctor(x_i, x_l, x_j, x_i, x_l, x_j, reference);
  auto* cost =
      new ceres::DynamicAutoDiffCostFunction<TripletFunctor, 9>(functor);
  cost->AddParameterBlock(3);
  cost->AddParameterBlock(3);
  cost->AddParameterBlock(3);
  cost->SetNumResiduals(4 * static_cast<int>(x_i.size()));
  return cost;
}

}  // namespace rotation_only_internal

bool RefineRotationsWithRotationOnlyError(
    const RotationOnlyRefinerOptions& options,
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    std::unordered_map<ViewId, Vector3d>* orientations) {
  CHECK_NOTNULL(orientations);
  if (orientations->empty() || options.num_outer_iterations < 1 ||
      options.max_num_iterations < 1 || options.max_tracks_per_triplet < 1 ||
      options.max_observations_per_view < 1 || !(options.loss_scale_px > 0.0)) {
    LOG(WARNING) << "Rotation-only refinement skipped: invalid input.";
    return false;
  }

  // Collect the observations of every track in the oriented views.
  std::vector<TrackId> track_ids = reconstruction.TrackIds();
  std::sort(track_ids.begin(), track_ids.end());
  std::vector<TrackData> all_tracks;
  for (const TrackId track_id : track_ids) {
    const Track* track = reconstruction.Track(track_id);
    if (track == nullptr) {
      continue;
    }
    std::vector<ViewId> view_ids(track->ViewIds().begin(),
                                 track->ViewIds().end());
    std::sort(view_ids.begin(), view_ids.end());

    TrackData data;
    data.track_id = track_id;
    for (const ViewId view_id : view_ids) {
      if (orientations->count(view_id) == 0) {
        continue;
      }
      const View* view = reconstruction.View(view_id);
      if (view == nullptr || view->GetFeature(track_id) == nullptr) {
        continue;
      }
      const double focal = view->Camera().FocalLength();
      if (!(focal > 0.0)) {
        continue;
      }
      Observation obs;
      obs.view_id = view_id;
      obs.ray = GetNormalizedFeatureRay(*view, track_id);
      obs.focal = focal;
      data.observations.push_back(obs);
    }
    if (data.observations.size() < 2) {
      continue;
    }
    all_tracks.push_back(std::move(data));
  }
  if (options.verify_observations_px > 0.0) {
    const int num_removed =
        VerifyObservations(options, *orientations, &all_tracks);
    VLOG(2) << "Rotation-only refinement: " << num_removed
            << " observations failed the epipolar verification.";
  }
  std::vector<TrackData> tracks;
  std::unordered_map<ViewId, std::vector<int>> view_tracks;
  for (TrackData& data : all_tracks) {
    if (data.observations.size() < 2) {
      continue;
    }
    for (const Observation& obs : data.observations) {
      view_tracks[obs.view_id].push_back(static_cast<int>(tracks.size()));
    }
    tracks.push_back(std::move(data));
  }
  if (tracks.empty()) {
    LOG(WARNING) << "Rotation-only refinement skipped: no shared tracks.";
    return false;
  }

  std::unordered_map<ViewId, Vector3d> current = *orientations;
  bool solved = false;
  for (int outer = 0; outer < options.num_outer_iterations; ++outer) {
    // Step 1: tuples, Theorem 1 filtering and chirality references from the
    // current rotations.
    std::unordered_map<ViewId, Matrix3d> rotations;
    for (const auto& entry : current) {
      rotations[entry.first] = AngleAxisToMatrix(entry.second);
    }
    TupleBuilder builder(options, tracks, view_tracks, view_pairs, rotations);
    TripletGroups triplet_groups;
    PairGroups pair_groups;
    builder.Build(&triplet_groups, &pair_groups);
    if (triplet_groups.empty() && pair_groups.empty()) {
      LOG(WARNING) << "Rotation-only refinement: no valid tuples at outer "
                   << "iteration " << outer << ".";
      break;
    }

    // Step 2: Ceres problem over the angle-axis rotations. The translations are
    // evaluated inside the residuals of each triplet and view pair.
    std::unordered_map<ViewId, Vector3d> params = current;
    ceres::Problem problem;
    std::map<ViewId, int> residual_count;
    for (const auto& group : triplet_groups) {
      const std::array<ViewId, 3>& key = group.first;
      const std::vector<Tuple>& tuples = group.second;
      const TripletSolve& solve = builder.GetTriplet(key[0], key[1], key[2]);
      if (!solve.valid || tuples.empty()) {
        continue;
      }
      std::vector<Vector3d> tup_i, tup_l, tup_j;
      std::vector<double> focal;
      for (const Tuple& t : tuples) {
        tup_i.push_back(t.x_i);
        tup_l.push_back(t.x_l);
        tup_j.push_back(t.x_j);
        focal.push_back(t.focal_j);
      }
      auto* functor = new TripletFunctor(solve.x_i, solve.x_l, solve.x_j, tup_i,
                                         tup_l, tup_j, solve.b_il);
      const double* angle_axes[3] = {current.at(key[0]).data(),
                                     current.at(key[1]).data(),
                                     current.at(key[2]).data()};
      SetCauchyWeights(focal, options.loss_scale_px, 2, angle_axes, functor);

      const int n = static_cast<int>(tuples.size());
      auto* cost =
          new ceres::DynamicAutoDiffCostFunction<TripletFunctor, 9>(functor);
      cost->AddParameterBlock(3);
      cost->AddParameterBlock(3);
      cost->AddParameterBlock(3);
      cost->SetNumResiduals(4 * n);
      problem.AddResidualBlock(cost, nullptr, params.at(key[0]).data(),
                               params.at(key[1]).data(),
                               params.at(key[2]).data());
      // Each tuple adds two residual blocks, each touching i, l and j.
      residual_count[key[0]] += 2 * n;
      residual_count[key[1]] += 2 * n;
      residual_count[key[2]] += 2 * n;
    }
    for (const auto& group : pair_groups) {
      const ViewId i = group.first.first;
      const ViewId j = group.first.second;
      const std::vector<Tuple>& tuples = group.second;
      const TwoViewSolve& solve = builder.GetTwoView(i, j);
      if (!solve.valid || tuples.empty()) {
        continue;
      }
      std::vector<Vector3d> tup_i, tup_j;
      std::vector<double> focal;
      for (const Tuple& t : tuples) {
        tup_i.push_back(t.x_i);
        tup_j.push_back(t.x_j);
        focal.push_back(t.focal_j);
      }
      auto* functor =
          new TwoViewFunctor(solve.x_i, solve.x_j, tup_i, tup_j, solve.t_j);
      const double* angle_axes[2] = {current.at(i).data(), current.at(j).data()};
      SetCauchyWeights(focal, options.loss_scale_px, 1, angle_axes, functor);

      const int n = static_cast<int>(tuples.size());
      auto* cost =
          new ceres::DynamicAutoDiffCostFunction<TwoViewFunctor, 9>(functor);
      cost->AddParameterBlock(3);
      cost->AddParameterBlock(3);
      cost->SetNumResiduals(2 * n);
      problem.AddResidualBlock(cost, nullptr, params.at(i).data(),
                               params.at(j).data());
      residual_count[i] += n;
      residual_count[j] += n;
    }
    if (problem.NumResidualBlocks() == 0) {
      break;
    }

    // Gauge: the view with the most residuals keeps its orientation.
    ViewId gauge = kInvalidViewId;
    int gauge_count = -1;
    for (const auto& entry : residual_count) {
      if (entry.second > gauge_count) {
        gauge = entry.first;
        gauge_count = entry.second;
      }
    }
    problem.SetParameterBlockConstant(params.at(gauge).data());

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solver_options.max_num_iterations = options.max_num_iterations;
    solver_options.num_threads = options.num_threads;
    solver_options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    VLOG(1) << "Rotation-only outer iteration " << outer << ": "
            << summary.BriefReport();
    if (summary.termination_type == ceres::FAILURE) {
      LOG(WARNING) << "Rotation-only refinement: Ceres failed.";
      return false;
    }

    // Write the rotations back and check the change.
    double max_change = 0.0;
    bool finite = true;
    for (auto& entry : params) {
      if (!entry.second.allFinite()) {
        finite = false;
        break;
      }
      max_change = std::max(max_change,
                            RotationChange(current.at(entry.first), entry.second));
    }
    if (!finite) {
      LOG(WARNING) << "Rotation-only refinement: non-finite rotations.";
      return false;
    }
    current = params;
    solved = true;
    if (max_change < kMinRotationChange) {
      break;
    }
  }

  if (!solved) {
    return false;
  }
  *orientations = current;
  return true;
}

}  // namespace theia
