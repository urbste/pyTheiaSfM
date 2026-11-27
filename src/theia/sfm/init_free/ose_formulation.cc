// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/ose_formulation.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>

namespace theia {

namespace {

// Rows 0 and 1 of every projective formulation: the object space error
//   sqrt(1 - alpha) * (p_r^T X_h - m_r p_3^T X_h),  r = 0, 1.
void OseRows(const double w,
             const OseCamera& P,
             const Eigen::Vector2d& m,
             OseObservationModel* model) {
  for (int r = 0; r < 2; ++r) {
    model->G.row(r) = w * (P.block<1, 3>(r, 0) - m[r] * P.block<1, 3>(2, 0));
    model->z[r] = -w * (P(r, 3) - m[r] * P(2, 3));
  }
}

void OseJacobianRows(const double w,
                     const Eigen::Vector2d& m,
                     const Eigen::Vector4d& Xh,
                     OseCameraJacobian* jacobian) {
  for (int r = 0; r < 2; ++r) {
    // Row r of P is camera parameters [4r, 4r + 4); row 3 is [8, 12).
    jacobian->block<1, 4>(r, 4 * r) = w * Xh.transpose();
    jacobian->block<1, 4>(r, 8) = -w * m[r] * Xh.transpose();
  }
}

// pOSE (Hong & Zach, CVPR 2018) with X_h = [X; 1]:
//   r_ose = sqrt(1 - alpha) * (P_{1:2} X_h - (p_3^T X_h) m)
//   r_aff = sqrt(alpha)     * (P_{1:2} X_h - m)
// Residual rows: [r_ose(0), r_ose(1), r_aff(0), r_aff(1)].
class PoseFormulation : public OseFormulation {
 public:
  PoseFormulation(const double alpha, const bool use_rotation_terms)
      : w_ose_(std::sqrt(1.0 - alpha)),
        w_aff_(std::sqrt(alpha)),
        use_rotation_terms_(use_rotation_terms) {}

  OseFormulationType Type() const override {
    return use_rotation_terms_ ? OseFormulationType::POSE_ROT
                               : OseFormulationType::POSE;
  }

  int NumResidualsPerObservation() const override { return 4; }

  bool UsesRotationTerms() const override { return use_rotation_terms_; }

  void ObservationModel(const OseCamera& P,
                        const Eigen::Vector2d& m,
                        double /*depth*/,
                        double /*state*/,
                        OseObservationModel* model) const override {
    model->G.resize(4, 3);
    model->z.resize(4);
    OseRows(w_ose_, P, m, model);
    for (int r = 0; r < 2; ++r) {
      model->G.row(2 + r) = w_aff_ * P.block<1, 3>(r, 0);
      model->z[2 + r] = w_aff_ * (m[r] - P(r, 3));
    }
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      double /*depth*/,
                      const Eigen::Vector3d& X,
                      double /*state*/,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(4, kOseCameraParameters);
    OseJacobianRows(w_ose_, m, Xh, jacobian);
    for (int r = 0; r < 2; ++r) {
      jacobian->block<1, 4>(2 + r, 4 * r) = w_aff_ * Xh.transpose();
    }
  }

 private:
  const double w_ose_;
  const double w_aff_;
  const bool use_rotation_terms_;
};

// OSE with a depth regularizer (rOSE in InitFreeBA):
//   r_dep = sqrt(alpha) * (p_3^T X_h - 1).
class RoseFormulation : public OseFormulation {
 public:
  explicit RoseFormulation(const double alpha)
      : w_ose_(std::sqrt(1.0 - alpha)), w_dep_(std::sqrt(alpha)) {}

  OseFormulationType Type() const override { return OseFormulationType::ROSE; }

  int NumResidualsPerObservation() const override { return 3; }

  void ObservationModel(const OseCamera& P,
                        const Eigen::Vector2d& m,
                        double /*depth*/,
                        double /*state*/,
                        OseObservationModel* model) const override {
    model->G.resize(3, 3);
    model->z.resize(3);
    OseRows(w_ose_, P, m, model);
    model->G.row(2) = w_dep_ * P.block<1, 3>(2, 0);
    model->z[2] = w_dep_ * (1.0 - P(2, 3));
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      double /*depth*/,
                      const Eigen::Vector3d& X,
                      double /*state*/,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(3, kOseCameraParameters);
    OseJacobianRows(w_ose_, m, Xh, jacobian);
    jacobian->block<1, 4>(2, 8) = w_dep_ * Xh.transpose();
  }

 private:
  const double w_ose_;
  const double w_dep_;
};

// expOSE (Iglesias, Nilsson & Olsson, CVPR 2023), in the form of InitFreeBA:
// with m~ = [m; 1], n = |m~| and the depth s = m~^T P X_h / n, the regularizer
// alpha / 2 * exp(-s) is replaced by its second-order expansion around s0,
//
//   r_exp = sqrt(alpha) * sqrt(exp(-s0) / 2) * (s - 1 - s0),
//
// whose value and first two derivatives in s match alpha/2 exp(-s) (up to a
// constant) at s = s0. s0 is the per-observation state: n (unit depth along
// the ray) during the warm-up, then the current depth.
class ExposeFormulation : public OseFormulation {
 public:
  ExposeFormulation(const double alpha, const int warmup_iterations)
      : w_ose_(std::sqrt(1.0 - alpha)),
        sqrt_alpha_(std::sqrt(alpha)),
        warmup_iterations_(std::max(0, warmup_iterations)) {}

  OseFormulationType Type() const override {
    return OseFormulationType::EXPOSE;
  }

  int NumResidualsPerObservation() const override { return 3; }

  bool HasState() const override { return true; }

  double InitialState(const Eigen::Vector2d& m) const override {
    return m.homogeneous().norm();
  }

  double UpdatedState(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      double /*depth*/,
                      const Eigen::Vector3d& X) const override {
    const Eigen::Vector3d mh = m.homogeneous();
    return mh.dot(P * X.homogeneous()) / mh.norm();
  }

  int UpdateStateFromIteration() const override { return warmup_iterations_; }

  void NormalizeInitialCamera(OseCamera* P) const override {
    if (P->leftCols<3>().determinant() < 0.0) {
      *P = -*P;
    }
  }

  void ObservationModel(const OseCamera& P,
                        const Eigen::Vector2d& m,
                        double /*depth*/,
                        const double state,
                        OseObservationModel* model) const override {
    model->G.resize(3, 3);
    model->z.resize(3);
    OseRows(w_ose_, P, m, model);
    const Eigen::Vector3d mh = m.homogeneous();
    const double n = mh.norm();
    const double k = Weight(state);
    model->G.row(2) = (k / n) * (mh.transpose() * P.leftCols<3>());
    model->z[2] = k * (1.0 + state) - (k / n) * mh.dot(P.col(3));
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      double /*depth*/,
                      const Eigen::Vector3d& X,
                      const double state,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(3, kOseCameraParameters);
    OseJacobianRows(w_ose_, m, Xh, jacobian);
    const Eigen::Vector3d mh = m.homogeneous();
    const double kn = Weight(state) / mh.norm();
    for (int r = 0; r < 3; ++r) {
      jacobian->block<1, 4>(2, 4 * r) = kn * mh[r] * Xh.transpose();
    }
  }

 private:
  // sqrt(alpha) * sqrt(exp(-s0) / 2). The exponent is clamped so that points
  // far behind a camera (s0 << 0) cannot overflow the weight.
  double Weight(const double s0) const {
    return sqrt_alpha_ * std::sqrt(0.5 * std::exp(std::min(-s0, 30.0)));
  }

  const double w_ose_;
  const double sqrt_alpha_;
  const int warmup_iterations_;
};

// Radial pOSE (RpOSE, Iglesias & Olsson, ICCV 2021) for 1D radial cameras.
// Only the first two rows of P enter; with m_perp = (-m_2, m_1):
//   r_rad = sqrt(1 - alpha) * m_perp^T P_{1:2} X_h / |m|
//   r_aff = sqrt(alpha)     * (P_{1:2} X_h - m)
// For |m| ~ 0 the radial row has no defined direction and is dropped.
class RposeFormulation : public OseFormulation {
 public:
  explicit RposeFormulation(const double alpha)
      : w_rad_(std::sqrt(1.0 - alpha)), w_aff_(std::sqrt(alpha)) {}

  OseFormulationType Type() const override {
    return OseFormulationType::RPOSE;
  }

  int NumResidualsPerObservation() const override { return 3; }

  bool IsRadial() const override { return true; }

  void ObservationModel(const OseCamera& P,
                        const Eigen::Vector2d& m,
                        double /*depth*/,
                        double /*state*/,
                        OseObservationModel* model) const override {
    model->G.resize(3, 3);
    model->z.resize(3);
    const Eigen::Vector2d d = RadialNormal(m);
    model->G.row(0) =
        d[0] * P.block<1, 3>(0, 0) + d[1] * P.block<1, 3>(1, 0);
    model->z[0] = -(d[0] * P(0, 3) + d[1] * P(1, 3));
    for (int r = 0; r < 2; ++r) {
      model->G.row(1 + r) = w_aff_ * P.block<1, 3>(r, 0);
      model->z[1 + r] = w_aff_ * (m[r] - P(r, 3));
    }
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      double /*depth*/,
                      const Eigen::Vector3d& X,
                      double /*state*/,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(3, kOseCameraParameters);
    const Eigen::Vector2d d = RadialNormal(m);
    jacobian->block<1, 4>(0, 0) = d[0] * Xh.transpose();
    jacobian->block<1, 4>(0, 4) = d[1] * Xh.transpose();
    for (int r = 0; r < 2; ++r) {
      jacobian->block<1, 4>(1 + r, 4 * r) = w_aff_ * Xh.transpose();
    }
  }

 private:
  // sqrt(1 - alpha) * m_perp / |m|, or zero for |m| ~ 0.
  Eigen::Vector2d RadialNormal(const Eigen::Vector2d& m) const {
    const double norm = m.norm();
    if (norm < 1e-12) {
      return Eigen::Vector2d::Zero();
    }
    return (w_rad_ / norm) * Eigen::Vector2d(-m[1], m[0]);
  }

  const double w_rad_;
  const double w_aff_;
};

// Cross-product matrix [v]_x, so that [v]_x u = v x u.
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v[2], v[1],
       v[2], 0.0, -v[0],
       -v[1], v[0], 0.0;
  return S;
}

// DOSE (arXiv:2611.31516) on Euclidean cameras P = [R | t]. With y = R X + t,
// m~ = [m; 1], q = m~ / |m~|^2 and the depth prior d of the observation:
//   r_0,1 = y_{0,1} - m_{0,1} y_2
//   r_2   = sqrt(lambda_d * w) * (q^T y / d - 1)
// w is the Huber IRLS weight of the depth residual (the state, if robust), and
// r_2 is absent (zero rows) if the depth prior is not positive and finite.
class DoseFormulation : public OseFormulation {
 public:
  DoseFormulation(const double depth_weight,
                  const double huber_width,
                  const bool robust)
      : depth_weight_(depth_weight),
        huber_width_(huber_width),
        robust_(robust) {}

  OseFormulationType Type() const override { return OseFormulationType::DOSE; }

  int NumResidualsPerObservation() const override { return 3; }

  bool IsEuclidean() const override { return true; }

  bool UsesRotationTerms() const override { return false; }

  bool HasState() const override { return robust_; }

  double InitialState(const Eigen::Vector2d& /*m*/) const override {
    return 1.0;
  }

  int UpdateStateFromIteration() const override { return 0; }

  void ObservationModel(const OseCamera& P,
                        const Eigen::Vector2d& m,
                        const double depth,
                        const double state,
                        OseObservationModel* model) const override {
    model->G.resize(3, 3);
    model->z.resize(3);
    OseRows(1.0, P, m, model);
    model->G.row(2).setZero();
    model->z[2] = 0.0;
    if (DepthValid(depth)) {
      const double s = DepthScale(depth, state);
      const Eigen::Vector3d q = QVector(m);
      model->G.row(2) = s * q.transpose() * P.leftCols<3>();
      model->z[2] = s * (depth - q.dot(P.col(3)));
    }
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      const double depth,
                      const Eigen::Vector3d& X,
                      const double state,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(3, kOseCameraParameters);
    OseJacobianRows(1.0, m, Xh, jacobian);
    if (DepthValid(depth)) {
      const double s = DepthScale(depth, state);
      const Eigen::Vector3d q = QVector(m);
      for (int r = 0; r < 3; ++r) {
        jacobian->block<1, 4>(2, 4 * r) = s * q[r] * Xh.transpose();
      }
    }
  }

  double UpdatedState(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      const double depth,
                      const Eigen::Vector3d& X) const override {
    if (!robust_ || !DepthValid(depth)) {
      return 1.0;
    }
    const Eigen::Vector3d y = P.leftCols<3>() * X + P.col(3);
    const double e = std::abs(QVector(m).dot(y) / depth - 1.0);
    return e <= huber_width_ ? 1.0 : huber_width_ / e;
  }

  void NormalizeInitialCamera(OseCamera* P) const override {
    // Nearest rotation to the left 3x3 block (Procrustes via SVD).
    const Eigen::Matrix3d A = P->leftCols<3>();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (A.allFinite() && !A.isZero()) {
      const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
          A, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U = svd.matrixU();
      const Eigen::Matrix3d V = svd.matrixV();
      R = U * V.transpose();
      if (R.determinant() < 0.0) {
        U.col(2) *= -1.0;
        R = U * V.transpose();
      }
    }
    P->leftCols<3>() = R;
  }

  void CameraTangentBasis(const OseCamera& P,
                          Eigen::Matrix<double, 12, 12>* B) const override {
    // Left perturbation dR = [e_k]_x R for the rotation part; translation
    // perturbations dt = e_k. Columns 6..11 are unused and stay zero.
    B->setZero();
    const Eigen::Matrix3d R = P.leftCols<3>();
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e[k] = 1.0;
      const Eigen::Matrix3d E = SkewSymmetric(e) * R;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          (*B)(4 * r + c, k) = E(r, c);
        }
      }
    }
    for (int k = 0; k < 3; ++k) {
      (*B)(4 * k + 3, 3 + k) = 1.0;
    }
  }

  OseCamera RetractCamera(
      const OseCamera& P,
      const Eigen::Matrix<double, 12, 1>& delta) const override {
    const Eigen::Vector3d omega = delta.head<3>();
    const Eigen::Vector3d dt = delta.segment<3>(3);
    const double angle = omega.norm();
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if (angle > 0.0) {
      dR = Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
    }
    OseCamera out;
    out.leftCols<3>() = dR * P.leftCols<3>();
    out.col(3) = P.col(3) + dt;
    return out;
  }

 private:
  static bool DepthValid(const double depth) {
    return std::isfinite(depth) && depth > 0.0;
  }

  // q = m~ / |m~|^2 with m~ = [m; 1].
  static Eigen::Vector3d QVector(const Eigen::Vector2d& m) {
    const Eigen::Vector3d mt = m.homogeneous();
    return mt / mt.squaredNorm();
  }

  // s = sqrt(lambda_d * w) / depth, the scale of the depth row (G and z).
  double DepthScale(const double depth, const double state) const {
    const double w = robust_ ? state : 1.0;
    return std::sqrt(depth_weight_ * w) / depth;
  }

  const double depth_weight_;
  const double huber_width_;
  const bool robust_;
};

}  // namespace

void OseFormulation::EvaluateRotationTerm(const OseRotationTerm& term,
                                          const OseCamera& Pa,
                                          const OseCamera& Pb,
                                          Eigen::Matrix<double, 9, 1>* residual,
                                          OseRotationJacobian* jacobian_a,
                                          OseRotationJacobian* jacobian_b) {
  const Eigen::Matrix3d A = Pa.leftCols<3>();
  const Eigen::Matrix3d B = Pb.leftCols<3>();
  const Eigen::Matrix3d E = A * B.transpose() - term.relative_rotation;
  Eigen::Matrix<double, 9, 1> e;
  // Residual index e = 3 * r + c for entry (r, c).
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      e[3 * r + c] = E(r, c);
    }
  }
  OseRotationJacobian Ja = OseRotationJacobian::Zero();
  OseRotationJacobian Jb = OseRotationJacobian::Zero();
  // (A B^T)_{rc} = sum_k A_{rk} B_{ck}:
  //   d/dA_{rk} = B_{ck},  d/dB_{ck} = A_{rk}.
  // A_{rk} is camera parameter 4r + k (row-major vec of P).
  const bool same = term.camera_a == term.camera_b;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      const int i = 3 * r + c;
      for (int k = 0; k < 3; ++k) {
        Ja(i, 4 * r + k) += B(c, k);
        if (same) {
          Ja(i, 4 * c + k) += A(r, k);
        } else {
          Jb(i, 4 * c + k) += A(r, k);
        }
      }
    }
  }
  const double w = std::sqrt(term.weight);
  *residual = w * (term.sqrt_weight_matrix * e);
  jacobian_a->noalias() = w * (term.sqrt_weight_matrix * Ja);
  if (same) {
    jacobian_b->setZero();
  } else {
    jacobian_b->noalias() = w * (term.sqrt_weight_matrix * Jb);
  }
}

std::unique_ptr<OseFormulation> CreateOseFormulation(
    const InitFreeOptions& options, std::string* error) {
  if (options.alpha < 0.0 || options.alpha > 1.0) {
    *error = "alpha must be in [0, 1].";
    return nullptr;
  }
  switch (options.formulation) {
    case OseFormulationType::POSE:
      return std::make_unique<PoseFormulation>(options.alpha, false);
    case OseFormulationType::POSE_ROT:
      return std::make_unique<PoseFormulation>(options.alpha, true);
    case OseFormulationType::ROSE:
      return std::make_unique<RoseFormulation>(options.alpha);
    case OseFormulationType::RPOSE:
      return std::make_unique<RposeFormulation>(options.alpha);
    case OseFormulationType::EXPOSE:
      return std::make_unique<ExposeFormulation>(
          options.alpha, options.expose_warmup_iterations);
    case OseFormulationType::DOSE:
      if (!(options.dose_depth_weight >= 0.0)) {
        *error = "dose_depth_weight must be non-negative.";
        return nullptr;
      }
      if (!(options.dose_depth_huber_width > 0.0)) {
        *error = "dose_depth_huber_width must be positive.";
        return nullptr;
      }
      return CreateDoseFormulation(options.dose_depth_weight,
                                   options.dose_depth_huber_width,
                                   true);
  }
  *error = "Unknown OSE formulation.";
  return nullptr;
}

std::unique_ptr<OseFormulation> CreateDoseFormulation(double depth_weight,
                                                      double huber_width,
                                                      bool robust) {
  return std::make_unique<DoseFormulation>(depth_weight, huber_width, robust);
}

}  // namespace theia
