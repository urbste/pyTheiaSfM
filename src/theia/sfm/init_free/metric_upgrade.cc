// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/metric_upgrade.h"

#include <ceres/ceres.h>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>

namespace theia {

namespace {

// Index of the symmetric entry (i, j) in the 10-vector of unknowns.
int SymmetricIndex(int i, int j) {
  if (i > j) std::swap(i, j);
  static const int kIndex[4][4] = {
      {0, 1, 2, 3}, {1, 4, 5, 6}, {2, 5, 7, 8}, {3, 6, 8, 9}};
  return kIndex[i][j];
}

// Coefficients of (P Omega P^T)_{rs} in the 10 unknowns of Omega.
Eigen::Matrix<double, 1, 10> QuadricRow(const OseCamera& P, int r, int s) {
  Eigen::Matrix<double, 1, 10> row = Eigen::Matrix<double, 1, 10>::Zero();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      row[SymmetricIndex(i, j)] += P(r, i) * P(s, j);
    }
  }
  return row;
}

Eigen::Matrix4d QuadricFromVector(const Eigen::Matrix<double, 10, 1>& omega) {
  Eigen::Matrix4d Omega;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Omega(i, j) = omega[SymmetricIndex(i, j)];
    }
  }
  return Omega;
}

Eigen::Matrix<double, 10, 1> SmallestRightSingularVector(
    const Eigen::MatrixXd& A) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  return svd.matrixV().col(9);
}

Eigen::Matrix3d ProjectToRotation(const Eigen::Matrix3d& M) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = -1.0;
    R = svd.matrixU() * D * svd.matrixV().transpose();
  }
  return R;
}

// Rank-3 truncation: Omega ~ H diag(1, 1, 1, 0) H^T.
bool UpgradeFromQuadric(const Eigen::Matrix4d& Omega,
                        Eigen::Matrix4d* H,
                        std::string* error) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(
      0.5 * (Omega + Omega.transpose()));
  const Eigen::Vector4d values = eig.eigenvalues();  // ascending
  const Eigen::Matrix4d vectors = eig.eigenvectors();
  if (!(values[1] > 1e-12 * values[3])) {
    *error =
        "Estimated dual quadric is not rank-3 positive semidefinite; the "
        "projective reconstruction is likely degenerate.";
    return false;
  }
  H->col(0) = vectors.col(3) * std::sqrt(values[3]);
  H->col(1) = vectors.col(2) * std::sqrt(values[2]);
  H->col(2) = vectors.col(1) * std::sqrt(values[1]);
  H->col(3) = vectors.col(0);
  return true;
}

// Focal lengths implied by Omega_3 = H diag(1,1,1,0) H^T for K = diag(f,f,1).
bool FocalLengthsFromUpgrade(const OseCameraVector& cameras,
                             const Eigen::Matrix4d& H,
                             const bool shared,
                             std::vector<double>* focal_lengths,
                             std::string* error) {
  const int n = static_cast<int>(cameras.size());
  focal_lengths->resize(n);
  for (int i = 0; i < n; ++i) {
    const Eigen::Matrix3d M = cameras[i] * H.leftCols<3>();
    const Eigen::Matrix3d w = M * M.transpose();
    const double f2 = 0.5 * (w(0, 0) + w(1, 1)) / w(2, 2);
    if (!(f2 > 0.0) || !std::isfinite(f2)) {
      *error = "Self-calibration gave no valid focal length for camera " +
               std::to_string(i) + ".";
      return false;
    }
    (*focal_lengths)[i] = std::sqrt(f2);
  }
  if (shared) {
    std::vector<double> sorted = *focal_lengths;
    std::nth_element(sorted.begin(), sorted.begin() + n / 2, sorted.end());
    std::fill(focal_lengths->begin(), focal_lengths->end(), sorted[n / 2]);
  }
  return true;
}

bool LinearQuadricCalibrated(const OseCameraVector& cameras,
                             Eigen::Matrix4d* Omega) {
  const int n = static_cast<int>(cameras.size());
  // Five equations per camera: off-diagonals zero, equal diagonal.
  Eigen::MatrixXd A(5 * n, 10);
  for (int i = 0; i < n; ++i) {
    const OseCamera P = cameras[i] / cameras[i].norm();
    const auto m00 = QuadricRow(P, 0, 0);
    A.row(5 * i + 0) = QuadricRow(P, 0, 1);
    A.row(5 * i + 1) = QuadricRow(P, 0, 2);
    A.row(5 * i + 2) = QuadricRow(P, 1, 2);
    A.row(5 * i + 3) = m00 - QuadricRow(P, 1, 1);
    A.row(5 * i + 4) = m00 - QuadricRow(P, 2, 2);
  }
  *Omega = QuadricFromVector(SmallestRightSingularVector(A));
  if (Omega->trace() < 0.0) {
    *Omega = -*Omega;
  }
  return true;
}

// Pollefeys et al., "Visual modeling with a hand-held camera", IJCV 2004:
// omega = P Omega P^T ~ diag(f^2, f^2, 1), with the equations weighted by the
// expected uncertainty of each constraint and by 1 / omega_22.
bool LinearQuadricUncalibrated(const OseCameraVector& cameras,
                               Eigen::Matrix4d* Omega) {
  const int n = static_cast<int>(cameras.size());
  std::vector<OseCamera, Eigen::aligned_allocator<OseCamera>> P(n);
  for (int i = 0; i < n; ++i) {
    P[i] = cameras[i] / cameras[i].norm();
  }
  std::vector<double> nu(n, 1.0);
  Eigen::MatrixXd A(6 * n, 10);
  for (int iteration = 0; iteration < 3; ++iteration) {
    for (int i = 0; i < n; ++i) {
      const auto w00 = QuadricRow(P[i], 0, 0);
      const auto w11 = QuadricRow(P[i], 1, 1);
      const auto w22 = QuadricRow(P[i], 2, 2);
      const double s = 1.0 / nu[i];
      A.row(6 * i + 0) = s / 9.0 * (w00 - w22);  // f ~ 1 (weak)
      A.row(6 * i + 1) = s / 9.0 * (w11 - w22);
      A.row(6 * i + 2) = s / 0.2 * (w00 - w11);  // unit aspect ratio
      A.row(6 * i + 3) = s / 0.01 * QuadricRow(P[i], 0, 1);  // zero skew
      A.row(6 * i + 4) = s / 0.1 * QuadricRow(P[i], 0, 2);   // principal
      A.row(6 * i + 5) = s / 0.1 * QuadricRow(P[i], 1, 2);   // point at 0
    }
    *Omega = QuadricFromVector(SmallestRightSingularVector(A));
    // Sign: P_3 Omega P_3^T = omega_22 > 0 for most cameras.
    int positive = 0;
    for (int i = 0; i < n; ++i) {
      positive += P[i].row(2) * *Omega * P[i].row(2).transpose() > 0.0;
    }
    if (2 * positive < n) {
      *Omega = -*Omega;
    }
    double mean_nu = 0.0;
    for (int i = 0; i < n; ++i) {
      nu[i] = std::abs(P[i].row(2) * *Omega * P[i].row(2).transpose());
      mean_nu += nu[i] / n;
    }
    if (!(mean_nu > 0.0)) {
      return false;
    }
    for (double& v : nu) {
      v = std::max(v, 1e-3 * mean_nu);
    }
  }
  return true;
}

// 3 E / trace(E) - I for E = K^-1 P B B^T P^T K^-T, K = diag(f, f, 1).
// Off-diagonal entries are scaled by sqrt(2), so the squared norm of the
// residual is the squared Frobenius norm of the 3x3 matrix.
struct DualQuadricResidual {
  explicit DualQuadricResidual(const OseCamera& P) : P_(P) {}

  template <typename T>
  bool operator()(const T* B, const T* f, T* residual) const {
    T Q[3][3];
    const T inv_f = T(1.0) / f[0];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T sum = T(0.0);
        for (int k = 0; k < 4; ++k) {
          sum += P_(r, k) * B[k + 4 * c];  // B is 4x3, column-major
        }
        Q[r][c] = r < 2 ? sum * inv_f : sum;
      }
    }
    T E[3][3];
    for (int a = 0; a < 3; ++a) {
      for (int b = a; b < 3; ++b) {
        E[a][b] = Q[a][0] * Q[b][0] + Q[a][1] * Q[b][1] + Q[a][2] * Q[b][2];
      }
    }
    const T s = T(3.0) / (E[0][0] + E[1][1] + E[2][2]);
    const T sqrt2 = T(std::sqrt(2.0));
    residual[0] = s * E[0][0] - T(1.0);
    residual[1] = s * E[1][1] - T(1.0);
    residual[2] = s * E[2][2] - T(1.0);
    residual[3] = sqrt2 * s * E[0][1];
    residual[4] = sqrt2 * s * E[0][2];
    residual[5] = sqrt2 * s * E[1][2];
    return true;
  }

  static ceres::CostFunction* Create(const OseCamera& P) {
    return new ceres::AutoDiffCostFunction<DualQuadricResidual, 6, 12, 1>(
        new DualQuadricResidual(P));
  }

  const OseCamera P_;
};

bool RefineQuadric(const OseCameraVector& cameras,
                   const bool estimate_focal_lengths,
                   const MetricUpgradeOptions& options,
                   Eigen::Matrix4d* H,
                   std::vector<double>* focal_lengths,
                   std::string* error) {
  const int n = static_cast<int>(cameras.size());
  Eigen::Matrix<double, 4, 3> B = H->leftCols<3>();
  B /= B.norm();
  ceres::Problem problem;
  const bool shared = estimate_focal_lengths && options.shared_focal_length;
  for (int i = 0; i < n; ++i) {
    double* f = shared ? &(*focal_lengths)[0] : &(*focal_lengths)[i];
    problem.AddResidualBlock(
        DualQuadricResidual::Create(cameras[i] / cameras[i].norm()),
        nullptr,
        B.data(),
        f);
    if (!estimate_focal_lengths) {
      problem.SetParameterBlockConstant(f);
    } else {
      problem.SetParameterLowerBound(f, 0, 1e-6);
    }
  }
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type =
      estimate_focal_lengths && !shared ? ceres::DENSE_SCHUR : ceres::DENSE_QR;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = 1e-14;
  solver_options.parameter_tolerance = 1e-12;
  solver_options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  if (!summary.IsSolutionUsable() || !B.allFinite()) {
    *error = "Nonlinear metric refinement failed: " + summary.message;
    return false;
  }
  if (shared) {
    std::fill(focal_lengths->begin(), focal_lengths->end(),
              (*focal_lengths)[0]);
  }
  // Fourth column: the null vector of B^T (Omega's null space, the plane at
  // infinity). Any vector outside span(B) only changes the similarity gauge.
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 4>> svd(B.transpose(),
                                                    Eigen::ComputeFullV);
  H->leftCols<3>() = B;
  H->col(3) = svd.matrixV().col(3);
  if (std::abs(H->determinant()) < 1e-14) {
    *error = "Nonlinear metric refinement gave a rank-deficient quadric.";
    return false;
  }
  return true;
}

double MeanOrthogonalityError(const OseCameraVector& cameras,
                              const Eigen::Matrix4d& H,
                              const std::vector<double>& focal_lengths) {
  double sum = 0.0;
  for (size_t i = 0; i < cameras.size(); ++i) {
    Eigen::Matrix3d M = cameras[i] * H.leftCols<3>();
    M.topRows<2>() /= focal_lengths[i];
    const double s2 = std::pow(std::abs(M.determinant()), 2.0 / 3.0);
    sum += (M * M.transpose() / s2 - Eigen::Matrix3d::Identity()).norm();
  }
  return sum / cameras.size();
}

double FractionInFront(const OseProblem& problem,
                       const MetricUpgradeResult& result) {
  int in_front = 0;
  int total = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int j = problem.observation_point[k];
    if (!result.point_valid[j]) continue;
    const int c = problem.observation_camera[k];
    const double depth =
        (result.rotations[c] * result.points.col(j) + result.translations[c])
            .z();
    in_front += depth > 0.0;
    ++total;
  }
  return total > 0 ? static_cast<double>(in_front) / total : 0.0;
}

// Metric cameras and points from H and the focal lengths.
bool ApplyUpgrade(const OseProblem& problem,
                  const OseCameraVector& cameras,
                  const Eigen::Matrix4Xd& points,
                  const Eigen::Matrix4d& H,
                  MetricUpgradeResult* result,
                  std::string* error) {
  const int n = static_cast<int>(cameras.size());
  const Eigen::Matrix4d H_inv = H.inverse();
  result->H = H;
  result->rotations.resize(n);
  result->translations.resize(n);
  for (int i = 0; i < n; ++i) {
    OseCamera Q = cameras[i] * H;
    Q.topRows<2>() /= result->focal_lengths[i];
    const Eigen::Matrix3d M = Q.leftCols<3>();
    const double s = std::cbrt(M.determinant());
    if (!std::isfinite(s) || std::abs(s) < 1e-15) {
      *error = "Upgraded camera " + std::to_string(i) + " is degenerate.";
      return false;
    }
    result->rotations[i] = ProjectToRotation(M / s);
    result->translations[i] = Q.col(3) / s;
  }
  result->mean_orthogonality_error =
      MeanOrthogonalityError(cameras, H, result->focal_lengths);

  result->points.resize(3, points.cols());
  result->point_valid.assign(points.cols(), 0);
  for (int j = 0; j < points.cols(); ++j) {
    const Eigen::Vector4d X = H_inv * points.col(j);
    if (std::abs(X[3]) > 1e-12 * X.head<3>().norm()) {
      result->points.col(j) = X.head<3>() / X[3];
      result->point_valid[j] = result->points.col(j).allFinite();
    } else {
      result->points.col(j).setZero();
    }
  }

  // Cheirality: (R, t, X) and (R, -t, -X) give the same projections. Pick the
  // one with most points in front of the cameras.
  result->fraction_in_front = FractionInFront(problem, *result);
  if (result->fraction_in_front < 0.5) {
    for (auto& t : result->translations) t = -t;
    result->points = -result->points;
    Eigen::Matrix4d flip = Eigen::Matrix4d::Identity();
    flip(3, 3) = -1.0;
    result->H = H * flip;
    result->fraction_in_front = FractionInFront(problem, *result);
  }
  return true;
}

// Radial cameras (RPOSE): only rows 0 and 1 of P are used.

// Rows 0 and 1 of P, normalized to unit norm, with a zero third row (so that
// QuadricRow applies).
OseCamera RadialCamera(const OseCamera& P) {
  OseCamera normalized = OseCamera::Zero();
  normalized.topRows<2>() = P.topRows<2>() / P.topRows<2>().norm();
  return normalized;
}

// P_12 Omega P_12^T ~ I_2: equal diagonal entries and zero off-diagonal entry,
// two equations per camera.
bool LinearQuadricRadial(const OseCameraVector& cameras,
                         Eigen::Matrix4d* Omega) {
  const int n = static_cast<int>(cameras.size());
  Eigen::MatrixXd A(2 * n, 10);
  for (int i = 0; i < n; ++i) {
    const OseCamera P = RadialCamera(cameras[i]);
    A.row(2 * i + 0) = QuadricRow(P, 0, 1);
    A.row(2 * i + 1) = QuadricRow(P, 0, 0) - QuadricRow(P, 1, 1);
  }
  *Omega = QuadricFromVector(SmallestRightSingularVector(A));
  // Sign: the trace of P_12 Omega P_12^T is positive for most cameras.
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const OseCamera P = RadialCamera(cameras[i]);
    sum += (P.row(0) * *Omega * P.row(0).transpose()).value() +
           (P.row(1) * *Omega * P.row(1).transpose()).value();
  }
  if (sum < 0.0) {
    *Omega = -*Omega;
  }
  return true;
}

// With Q = P_12 B (B is 4x3, column-major) and E = Q Q^T, the residuals are
// 2 E / trace(E) - I_2 (the off-diagonal entry is scaled by sqrt(2)). The
// residuals depend on Omega = B B^T only, so no focal length is estimated.
struct RadialQuadricResidual {
  explicit RadialQuadricResidual(const OseCamera& P) : P_(P) {}

  template <typename T>
  bool operator()(const T* B, T* residual) const {
    T Q[2][3];
    for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 3; ++c) {
        T sum = T(0.0);
        for (int k = 0; k < 4; ++k) {
          sum += P_(r, k) * B[k + 4 * c];
        }
        Q[r][c] = sum;
      }
    }
    const T E00 = Q[0][0] * Q[0][0] + Q[0][1] * Q[0][1] + Q[0][2] * Q[0][2];
    const T E11 = Q[1][0] * Q[1][0] + Q[1][1] * Q[1][1] + Q[1][2] * Q[1][2];
    const T E01 = Q[0][0] * Q[1][0] + Q[0][1] * Q[1][1] + Q[0][2] * Q[1][2];
    const T s = T(2.0) / (E00 + E11);
    residual[0] = s * E00 - T(1.0);
    residual[1] = s * E11 - T(1.0);
    residual[2] = T(std::sqrt(2.0)) * s * E01;
    return true;
  }

  static ceres::CostFunction* Create(const OseCamera& P) {
    return new ceres::AutoDiffCostFunction<RadialQuadricResidual, 3, 12>(
        new RadialQuadricResidual(P));
  }

  const OseCamera P_;
};

// Same structure as RefineQuadric, with the radial residuals.
bool RefineRadialQuadric(const OseCameraVector& cameras,
                         const MetricUpgradeOptions& options,
                         Eigen::Matrix4d* H,
                         std::string* error) {
  Eigen::Matrix<double, 4, 3> B = H->leftCols<3>();
  B /= B.norm();
  ceres::Problem problem;
  for (const OseCamera& P : cameras) {
    problem.AddResidualBlock(RadialQuadricResidual::Create(RadialCamera(P)),
                             nullptr,
                             B.data());
  }
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = 1e-14;
  solver_options.parameter_tolerance = 1e-12;
  solver_options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  if (!summary.IsSolutionUsable() || !B.allFinite()) {
    *error = "Nonlinear radial metric refinement failed: " + summary.message;
    return false;
  }
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 4>> svd(B.transpose(),
                                                    Eigen::ComputeFullV);
  H->leftCols<3>() = B;
  H->col(3) = svd.matrixV().col(3);
  if (std::abs(H->determinant()) < 1e-14) {
    *error = "Nonlinear radial metric refinement gave a rank-deficient quadric.";
    return false;
  }
  return true;
}

// Radial analogue of MeanOrthogonalityError: the rows of M = P_12 H (left 2x3
// block), scaled to unit average norm, should be orthonormal.
double RadialMeanOrthogonalityError(const OseCameraVector& cameras,
                                    const Eigen::Matrix4d& H) {
  double sum = 0.0;
  for (const OseCamera& P : cameras) {
    const Eigen::Matrix<double, 2, 4> Q = P.topRows<2>() * H;
    const Eigen::Matrix<double, 2, 3> M = Q.leftCols<3>();
    const double s2 = 0.5 * (M.row(0).squaredNorm() + M.row(1).squaredNorm());
    sum += (M * M.transpose() / s2 - Eigen::Matrix2d::Identity()).norm();
  }
  return sum / cameras.size();
}

// Metric radial reconstruction for the upgrade H. Returns false (and sets
// `error`) if the candidate is degenerate. `residual` is the sum of squared
// residuals |m_k (r3 . X_k + t3) - f a_k|^2 over the valid observations.
bool BuildRadialCandidate(const OseProblem& problem,
                          const OseCameraVector& cameras,
                          const Eigen::Matrix4Xd& points,
                          const Eigen::Matrix4d& H,
                          const bool calibrated,
                          MetricUpgradeResult* result,
                          double* residual,
                          std::string* error) {
  const int n = static_cast<int>(cameras.size());
  *result = MetricUpgradeResult();
  result->H = H;
  result->rotations.assign(n, Eigen::Matrix3d::Identity());
  result->translations.assign(n, Eigen::Vector3d::Zero());
  result->focal_lengths.assign(n, 1.0);
  for (int i = 0; i < n; ++i) {
    const Eigen::Matrix<double, 2, 4> Q = cameras[i].topRows<2>() * H;
    const Eigen::Matrix<double, 2, 3> M = Q.leftCols<3>();
    const double s =
        std::sqrt(0.5 * (M.row(0).squaredNorm() + M.row(1).squaredNorm()));
    if (!(s > 0.0) || !std::isfinite(s)) {
      *error = "Upgraded camera " + std::to_string(i) + " is degenerate.";
      return false;
    }
    // Nearest matrix with orthonormal rows.
    Eigen::JacobiSVD<Eigen::Matrix<double, 2, 3>> svd(
        M / s, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix<double, 2, 3> R12 =
        svd.matrixU() * svd.matrixV().leftCols<2>().transpose();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.topRows<2>() = R12;
    const Eigen::Vector3d r1 = R12.row(0).transpose();
    const Eigen::Vector3d r2 = R12.row(1).transpose();
    R.row(2) = r1.cross(r2).transpose();
    result->rotations[i] = R;
    result->translations[i] = Eigen::Vector3d(Q(0, 3) / s, Q(1, 3) / s, 0.0);
  }

  const Eigen::Matrix4d H_inv = H.inverse();
  result->points.resize(3, points.cols());
  result->point_valid.assign(points.cols(), 0);
  for (int j = 0; j < points.cols(); ++j) {
    const Eigen::Vector4d X = H_inv * points.col(j);
    if (std::abs(X[3]) > 1e-12 * X.head<3>().norm()) {
      result->points.col(j) = X.head<3>() / X[3];
      result->point_valid[j] = result->points.col(j).allFinite();
    } else {
      result->points.col(j).setZero();
    }
  }

  // Per camera: 2D directions R12 X + t12 must agree with the observations
  // (a 180 degree rotation about the optical axis flips them). Fewer than half
  // positive: flip the first two rows, which keeps r3 and the depths.
  std::vector<int> positive(n, 0);
  std::vector<int> total(n, 0);
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int j = problem.observation_point[k];
    if (!result->point_valid[j]) continue;
    const int c = problem.observation_camera[k];
    const Eigen::Vector2d a =
        result->rotations[c].topRows<2>() * result->points.col(j) +
        result->translations[c].head<2>();
    ++total[c];
    positive[c] += problem.observations.col(k).dot(a) > 0.0;
  }
  for (int c = 0; c < n; ++c) {
    if (2 * positive[c] < total[c]) {
      result->rotations[c].topRows<2>() *= -1.0;
      result->translations[c].head<2>() *= -1.0;
    }
  }

  // Forward translation t3 and focal length f from m (r3 . X + t3) = f a.
  // Calibrated: f = 1 and m t3 = a - m c, least squares over the
  // observations. PIXEL_ISOTROPIC: unknowns (t3, f); each observation gives
  // two rows [m_p, -a_p] (t3, f)^T = -m_p c for p = 0, 1.
  std::vector<double> num(n, 0.0);
  std::vector<double> den(n, 0.0);
  std::vector<Eigen::Matrix2d> normal(n, Eigen::Matrix2d::Zero());
  std::vector<Eigen::Vector2d> rhs(n, Eigen::Vector2d::Zero());
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int j = problem.observation_point[k];
    if (!result->point_valid[j]) continue;
    const int c = problem.observation_camera[k];
    const Eigen::Vector3d X = result->points.col(j);
    const Eigen::Vector3d r3 = result->rotations[c].row(2).transpose();
    const Eigen::Vector2d a =
        result->rotations[c].topRows<2>() * X + result->translations[c].head<2>();
    const Eigen::Vector2d m = problem.observations.col(k);
    const double c_k = r3.dot(X);
    if (calibrated) {
      num[c] += m.dot(a - m * c_k);
      den[c] += m.squaredNorm();
    } else {
      for (int p = 0; p < 2; ++p) {
        const Eigen::Vector2d row(m[p], -a[p]);
        normal[c] += row * row.transpose();
        rhs[c] += row * (-m[p] * c_k);
      }
    }
  }
  for (int c = 0; c < n; ++c) {
    double t3 = 0.0;
    double f = 1.0;
    if (calibrated) {
      if (!(den[c] > 0.0)) {
        *error = "Radial metric upgrade: camera " + std::to_string(c) +
                 " has no valid observations.";
        return false;
      }
      t3 = num[c] / den[c];
    } else {
      const Eigen::Matrix2d& A = normal[c];
      const double det = A(0, 0) * A(1, 1) - A(0, 1) * A(0, 1);
      if (!(det > 1e-12 * A.trace() * A.trace())) {
        *error = "Radial metric upgrade: singular focal length fit for camera " +
                 std::to_string(c) + ".";
        return false;
      }
      const Eigen::Vector2d u = A.ldlt().solve(rhs[c]);
      t3 = u[0];
      f = u[1];
      if (!(f > 0.0) || !std::isfinite(f)) {
        *error = "Radial metric upgrade: non-positive focal length for camera " +
                 std::to_string(c) + ".";
        return false;
      }
    }
    result->translations[c][2] = t3;
    result->focal_lengths[c] = f;
  }

  double sum = 0.0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int j = problem.observation_point[k];
    if (!result->point_valid[j]) continue;
    const int c = problem.observation_camera[k];
    const Eigen::Vector3d X = result->points.col(j);
    const Eigen::Vector3d r3 = result->rotations[c].row(2).transpose();
    const Eigen::Vector2d a =
        result->rotations[c].topRows<2>() * X + result->translations[c].head<2>();
    const Eigen::Vector2d m = problem.observations.col(k);
    const double depth = r3.dot(X) + result->translations[c][2];
    sum += (m * depth - result->focal_lengths[c] * a).squaredNorm();
  }
  *residual = sum;
  result->fraction_in_front = FractionInFront(problem, *result);
  return true;
}

// Radial metric upgrade (UpgradeToMetric with options.radial_cameras).
bool UpgradeToMetricRadial(const OseProblem& problem,
                           const OseCameraVector& cameras,
                           const Eigen::Matrix4Xd& points,
                           const MetricUpgradeOptions& options,
                           MetricUpgradeResult* result,
                           std::string* error) {
  CHECK_NOTNULL(result);
  CHECK_NOTNULL(error);
  *result = MetricUpgradeResult();
  const int n = static_cast<int>(cameras.size());
  if (n < 5) {
    *error = "Radial metric upgrade needs at least 5 cameras (two equations "
             "each for 9 degrees of freedom), got " +
             std::to_string(n) + ".";
    return false;
  }
  if (points.cols() != problem.NumPoints() || n != problem.NumCameras()) {
    *error = "Cameras or points do not match the problem.";
    return false;
  }
  const bool calibrated =
      problem.normalization == OseObservationNormalization::CALIBRATED_RAYS;

  Eigen::Matrix4d Omega;
  LinearQuadricRadial(cameras, &Omega);
  Eigen::Matrix4d H;
  if (!UpgradeFromQuadric(Omega, &H, error)) {
    return false;
  }
  const double linear_error = RadialMeanOrthogonalityError(cameras, H);
  bool nonlinear_run = false;
  if (options.nonlinear_refinement) {
    if (!RefineRadialQuadric(cameras, options, &H, error)) {
      return false;
    }
    nonlinear_run = true;
  }
  const double final_error = RadialMeanOrthogonalityError(cameras, H);

  // H diag(1, 1, -1, 1) gives the same 2D directions with all depths flipped.
  // Keep the candidate with more points in front, ties by the fit residual.
  Eigen::Matrix4d mirror = Eigen::Matrix4d::Identity();
  mirror(2, 2) = -1.0;
  const Eigen::Matrix4d candidate_H[2] = {H, H * mirror};
  MetricUpgradeResult candidates[2];
  double residuals[2] = {0.0, 0.0};
  bool valid[2] = {false, false};
  std::string errors[2];
  for (int c = 0; c < 2; ++c) {
    valid[c] = BuildRadialCandidate(problem,
                                    cameras,
                                    points,
                                    candidate_H[c],
                                    calibrated,
                                    &candidates[c],
                                    &residuals[c],
                                    &errors[c]);
  }
  int best = -1;
  for (int c = 0; c < 2; ++c) {
    if (!valid[c]) continue;
    if (best < 0 ||
        candidates[c].fraction_in_front > candidates[best].fraction_in_front ||
        (candidates[c].fraction_in_front ==
             candidates[best].fraction_in_front &&
         residuals[c] < residuals[best])) {
      best = c;
    }
  }
  if (best < 0) {
    *error = errors[0];
    return false;
  }
  *result = std::move(candidates[best]);
  result->mean_orthogonality_error = final_error;
  result->linear_mean_orthogonality_error = linear_error;
  result->nonlinear_refinement_run = nonlinear_run;
  return true;
}

bool UpgradeToMetricImpl(const OseProblem& problem,
                         const OseCameraVector& cameras,
                         const Eigen::Matrix4Xd& points,
                         const MetricUpgradeOptions& options,
                         const bool calibrated,
                         MetricUpgradeResult* result,
                         std::string* error) {
  CHECK_NOTNULL(result);
  CHECK_NOTNULL(error);
  *result = MetricUpgradeResult();
  const int n = static_cast<int>(cameras.size());
  const int min_cameras = calibrated ? 2 : 3;
  if (n < min_cameras) {
    *error = "Metric upgrade needs at least " + std::to_string(min_cameras) +
             " cameras.";
    return false;
  }
  if (points.cols() != problem.NumPoints() ||
      n != problem.NumCameras()) {
    *error = "Cameras or points do not match the problem.";
    return false;
  }

  Eigen::Matrix4d Omega;
  if (calibrated) {
    LinearQuadricCalibrated(cameras, &Omega);
  } else if (!LinearQuadricUncalibrated(cameras, &Omega)) {
    *error = "Linear self-calibration failed (degenerate quadric).";
    return false;
  }
  Eigen::Matrix4d H;
  if (!UpgradeFromQuadric(Omega, &H, error)) {
    return false;
  }
  if (calibrated) {
    result->focal_lengths.assign(n, 1.0);
  } else if (!FocalLengthsFromUpgrade(cameras,
                                      H,
                                      options.shared_focal_length,
                                      &result->focal_lengths,
                                      error)) {
    return false;
  }
  result->linear_mean_orthogonality_error =
      MeanOrthogonalityError(cameras, H, result->focal_lengths);

  if (options.nonlinear_refinement) {
    if (!RefineQuadric(cameras,
                       !calibrated,
                       options,
                       &H,
                       &result->focal_lengths,
                       error)) {
      return false;
    }
    result->nonlinear_refinement_run = true;
  }
  return ApplyUpgrade(problem, cameras, points, H, result, error);
}

}  // namespace

bool UpgradeToMetric(const OseProblem& problem,
                     const OseCameraVector& cameras,
                     const Eigen::Matrix4Xd& points,
                     const MetricUpgradeOptions& options,
                     MetricUpgradeResult* result,
                     std::string* error) {
  if (options.radial_cameras) {
    return UpgradeToMetricRadial(problem, cameras, points, options, result, error);
  }
  return UpgradeToMetricImpl(
      problem,
      cameras,
      points,
      options,
      problem.normalization == OseObservationNormalization::CALIBRATED_RAYS,
      result,
      error);
}

bool UpgradeToMetricCalibrated(const OseProblem& problem,
                               const OseCameraVector& cameras,
                               const Eigen::Matrix4Xd& points,
                               MetricUpgradeResult* result,
                               std::string* error) {
  return UpgradeToMetricImpl(problem,
                             cameras,
                             points,
                             MetricUpgradeOptions(),
                             /*calibrated=*/true,
                             result,
                             error);
}

}  // namespace theia
