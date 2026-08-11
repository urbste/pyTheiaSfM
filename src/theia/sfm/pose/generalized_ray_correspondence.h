// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Generalized (non-central) ray correspondence between two rig / camera frames.
// Model (PoseLib / Sweeney convention):
//   R * (origin1 + lambda1 * direction1) + translation
//       = origin2 + lambda2 * direction2

#ifndef THEIA_SFM_POSE_GENERALIZED_RAY_CORRESPONDENCE_H_
#define THEIA_SFM_POSE_GENERALIZED_RAY_CORRESPONDENCE_H_

#include <Eigen/Core>
#include <limits>

namespace theia {

struct GeneralizedRayCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d origin1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction1 = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d origin2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction2 = Eigen::Vector3d::UnitZ();
};

// Algebraic generalized epipolar residual for X2 = R * X1 + t:
//   (direction2 × (R * direction1)) · (origin2 - R * origin1 - t)
inline double GeneralizedEpipolarResidual(
    const GeneralizedRayCorrespondence& corr,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation) {
  const Eigen::Vector3d w =
      corr.direction2.cross(rotation * corr.direction1);
  const double w_norm = w.norm();
  if (w_norm < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  return w.dot(corr.origin2 - rotation * corr.origin1 - translation) / w_norm;
}

}  // namespace theia

#endif  // THEIA_SFM_POSE_GENERALIZED_RAY_CORRESPONDENCE_H_
