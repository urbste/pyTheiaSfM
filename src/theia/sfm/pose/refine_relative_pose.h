// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// robust/optim/relative.h (PinholeRelativePoseRefiner), commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// 5-DoF Sampson refinement of a calibrated relative pose for RANSAC LO.

#ifndef THEIA_SFM_POSE_REFINE_RELATIVE_POSE_H_
#define THEIA_SFM_POSE_REFINE_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "theia/math/lmlsq/lm_optimizer.h"
#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/util.h"

namespace theia {

// Internal LM state: E = [t]_× R (PoseLib essential_from_motion convention).
struct RelativePoseLMState {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::UnitX();
};

inline Eigen::Matrix3d EssentialFromMotion(const RelativePoseLMState& pose) {
  return CrossProductMatrix(pose.translation) * pose.rotation;
}

inline Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + CrossProductMatrix(w);
  }
  return Eigen::AngleAxisd(theta, w / theta).toRotationMatrix();
}

inline void SetupTangentBasis(const Eigen::Vector3d& t,
                              Eigen::Matrix<double, 3, 2>* tangent_basis) {
  if (std::abs(t.x()) < std::abs(t.y())) {
    if (std::abs(t.x()) < std::abs(t.z())) {
      tangent_basis->col(0) = t.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
      tangent_basis->col(0) = t.cross(Eigen::Vector3d::UnitZ()).normalized();
    }
  } else {
    if (std::abs(t.y()) < std::abs(t.z())) {
      tangent_basis->col(0) = t.cross(Eigen::Vector3d::UnitY()).normalized();
    } else {
      tangent_basis->col(0) = t.cross(Eigen::Vector3d::UnitZ()).normalized();
    }
  }
  tangent_basis->col(1) = tangent_basis->col(0).cross(t).normalized();
}

inline void DerivEssentialWrtPose(const Eigen::Matrix3d& E,
                                  const Eigen::Matrix3d& R,
                                  const Eigen::Matrix<double, 3, 2>& tangent_basis,
                                  Eigen::Matrix<double, 9, 3>* dR,
                                  Eigen::Matrix<double, 9, 2>* dt) {
  dR->block<3, 1>(0, 0).setZero();
  dR->block<3, 1>(0, 1) = -E.col(2);
  dR->block<3, 1>(0, 2) = E.col(1);
  dR->block<3, 1>(3, 0) = E.col(2);
  dR->block<3, 1>(3, 1).setZero();
  dR->block<3, 1>(3, 2) = -E.col(0);
  dR->block<3, 1>(6, 0) = -E.col(1);
  dR->block<3, 1>(6, 1) = E.col(0);
  dR->block<3, 1>(6, 2).setZero();

  dt->block<3, 1>(0, 0) = tangent_basis.col(0).cross(R.col(0));
  dt->block<3, 1>(0, 1) = tangent_basis.col(1).cross(R.col(0));
  dt->block<3, 1>(3, 0) = tangent_basis.col(0).cross(R.col(1));
  dt->block<3, 1>(3, 1) = tangent_basis.col(1).cross(R.col(1));
  dt->block<3, 1>(6, 0) = tangent_basis.col(0).cross(R.col(2));
  dt->block<3, 1>(6, 1) = tangent_basis.col(1).cross(R.col(2));
}

// Minimizes Sampson error on normalized image points. 5 parameters:
// right-multiplicative rotation update (3) + translation tangent (2).
class RelativePoseSampsonRefiner {
 public:
  using Model = RelativePoseLMState;
  static constexpr int kNumParams = 5;

  RelativePoseSampsonRefiner(const std::vector<Eigen::Vector2d>& x1,
                             const std::vector<Eigen::Vector2d>& x2)
      : x1_(x1), x2_(x2) {}

  int NumParams() const { return kNumParams; }

  double ComputeResidual(NormalAccumulator& acc, const Model& pose) {
    const Eigen::Matrix3d E = EssentialFromMotion(pose);
    for (size_t k = 0; k < x1_.size(); ++k) {
      const Eigen::Vector3d x1h = x1_[k].homogeneous();
      const Eigen::Vector3d x2h = x2_[k].homogeneous();
      const double C = x2h.dot(E * x1h);
      const double nJc_sq =
          (E.block<2, 3>(0, 0) * x1h).squaredNorm() +
          (E.block<3, 2>(0, 0).transpose() * x2h).squaredNorm();
      acc.AddResidual(C / std::sqrt(nJc_sq));
    }
    return acc.Cost();
  }

  void ComputeJacobian(NormalAccumulator& acc, const Model& pose) {
    const Eigen::Matrix3d R = pose.rotation;
    const Eigen::Matrix3d E = EssentialFromMotion(pose);
    SetupTangentBasis(pose.translation, &tangent_basis_);

    Eigen::Matrix<double, 9, 3> dR;
    Eigen::Matrix<double, 9, 2> dt;
    DerivEssentialWrtPose(E, R, tangent_basis_, &dR, &dt);

    for (size_t k = 0; k < x1_.size(); ++k) {
      const Eigen::Vector3d x1h = x1_[k].homogeneous();
      const Eigen::Vector3d x2h = x2_[k].homogeneous();
      const double C = x2h.dot(E * x1h);

      Eigen::Vector4d J_C;
      J_C << E.block<3, 2>(0, 0).transpose() * x2h, E.block<2, 3>(0, 0) * x1h;
      const double nJ_C = J_C.norm();
      const double inv_nJ_C = 1.0 / nJ_C;
      const double r = C * inv_nJ_C;

      Eigen::Matrix<double, 1, 9> dF;
      dF << x1_[k](0) * x2_[k](0), x1_[k](0) * x2_[k](1), x1_[k](0),
          x1_[k](1) * x2_[k](0), x1_[k](1) * x2_[k](1), x1_[k](1), x2_[k](0),
          x2_[k](1), 1.0;
      const double s = C * inv_nJ_C * inv_nJ_C;
      dF(0) -= s * (J_C(2) * x1_[k](0) + J_C(0) * x2_[k](0));
      dF(1) -= s * (J_C(3) * x1_[k](0) + J_C(0) * x2_[k](1));
      dF(2) -= s * (J_C(0));
      dF(3) -= s * (J_C(2) * x1_[k](1) + J_C(1) * x2_[k](0));
      dF(4) -= s * (J_C(3) * x1_[k](1) + J_C(1) * x2_[k](1));
      dF(5) -= s * (J_C(1));
      dF(6) -= s * (J_C(2));
      dF(7) -= s * (J_C(3));
      dF *= inv_nJ_C;

      Eigen::Matrix<double, 1, 5> J;
      J.block<1, 3>(0, 0) = dF * dR;
      J.block<1, 2>(0, 3) = dF * dt;
      acc.AddJacobian(r, J);
    }
  }

  Model Step(const Eigen::VectorXd& dp, const Model& pose) const {
    Model pose_new;
    pose_new.rotation = pose.rotation * ExpSO3(dp.head<3>());
    pose_new.translation = pose.translation + tangent_basis_ * dp.tail<2>();
    return pose_new;
  }

  // Exposed for finite-difference Jacobian tests.
  Eigen::Matrix<double, 3, 2> tangent_basis_;

 private:
  const std::vector<Eigen::Vector2d>& x1_;
  const std::vector<Eigen::Vector2d>& x2_;
};

// Refines Theia (rotation, position) in place. position is the camera-2 center
// in camera-1 coordinates (unit). Returns true if the LM cost decreased.
// `squared_error_thresh` is the RANSAC squared-Sampson truncation threshold.
bool RefineRelativePoseSampson(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    LmStats* stats = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_POSE_REFINE_RELATIVE_POSE_H_
