// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/ose_formulation.h"

#include <Eigen/Geometry>
#include <glog/logging.h>

#include <cmath>

namespace theia {

namespace {

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
                        OseObservationModel* model) const override {
    model->G.resize(4, 3);
    model->z.resize(4);
    for (int r = 0; r < 2; ++r) {
      model->G.row(r) =
          w_ose_ * (P.block<1, 3>(r, 0) - m[r] * P.block<1, 3>(2, 0));
      model->z[r] = -w_ose_ * (P(r, 3) - m[r] * P(2, 3));
      model->G.row(2 + r) = w_aff_ * P.block<1, 3>(r, 0);
      model->z[2 + r] = w_aff_ * (m[r] - P(r, 3));
    }
  }

  void CameraJacobian(const OseCamera& P,
                      const Eigen::Vector2d& m,
                      const Eigen::Vector3d& X,
                      OseCameraJacobian* jacobian) const override {
    const Eigen::Vector4d Xh = X.homogeneous();
    jacobian->setZero(4, kOseCameraParameters);
    for (int r = 0; r < 2; ++r) {
      // Row r of P is camera parameters [4r, 4r + 4); row 3 is [8, 12).
      jacobian->block<1, 4>(r, 4 * r) = w_ose_ * Xh.transpose();
      jacobian->block<1, 4>(r, 8) = -w_ose_ * m[r] * Xh.transpose();
      jacobian->block<1, 4>(2 + r, 4 * r) = w_aff_ * Xh.transpose();
    }
  }

 private:
  const double w_ose_;
  const double w_aff_;
  const bool use_rotation_terms_;
};

}  // namespace

void OseFormulation::EvaluateRotationTerm(
    const OseRotationTerm& term,
    const OseCamera& Pa,
    const OseCamera& Pb,
    Eigen::Matrix<double, 9, 1>* residual,
    Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>*
        jacobian_a,
    Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>*
        jacobian_b) {
  const double w = std::sqrt(term.weight);
  const Eigen::Matrix3d A = Pa.leftCols<3>();
  const Eigen::Matrix3d B = Pb.leftCols<3>();
  const Eigen::Matrix3d E = A * B.transpose() - term.relative_rotation;
  // Residual index e = 3 * r + c for entry (r, c).
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      (*residual)[3 * r + c] = w * E(r, c);
    }
  }
  jacobian_a->setZero();
  jacobian_b->setZero();
  // (A B^T)_{rc} = sum_k A_{rk} B_{ck}:
  //   d/dA_{rk} = B_{ck},  d/dB_{ck} = A_{rk}.
  // A_{rk} is camera parameter 4r + k (row-major vec of P).
  const bool same = term.camera_a == term.camera_b;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      const int e = 3 * r + c;
      for (int k = 0; k < 3; ++k) {
        (*jacobian_a)(e, 4 * r + k) += w * B(c, k);
        if (same) {
          (*jacobian_a)(e, 4 * c + k) += w * A(r, k);
        } else {
          (*jacobian_b)(e, 4 * c + k) += w * A(r, k);
        }
      }
    }
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
      *error = "OSE formulation ROSE is not implemented yet.";
      return nullptr;
    case OseFormulationType::RPOSE:
      *error = "OSE formulation RPOSE is not implemented yet.";
      return nullptr;
    case OseFormulationType::EXPOSE:
      *error = "OSE formulation EXPOSE is not implemented yet.";
      return nullptr;
  }
  *error = "Unknown OSE formulation.";
  return nullptr;
}

}  // namespace theia
