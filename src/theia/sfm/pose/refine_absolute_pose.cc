// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib robust/optim/absolute.h (PinholeAbsolutePoseRefiner),
// commit fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.

#include "theia/sfm/pose/refine_absolute_pose.h"

#include <glog/logging.h>

#include "theia/math/lmlsq/lm_optimizer.h"

namespace theia {

double AbsolutePoseReprojectionRefiner::ComputeResidual(
    NormalAccumulator& acc, const Model& pose) {
  const Eigen::Matrix3d R = pose.rotation;
  for (size_t i = 0; i < features_.size(); ++i) {
    const Eigen::Vector3d Z = R * world_points_[i] + pose.translation;
    if (Z(2) < 0.0) {
      continue;
    }
    const Eigen::Vector2d res = Z.hnormalized() - features_[i];
    acc.AddResidual(res);
  }
  return acc.Cost();
}

void AbsolutePoseReprojectionRefiner::ComputeJacobian(NormalAccumulator& acc,
                                                      const Model& pose) {
  const Eigen::Matrix3d R = pose.rotation;
  Eigen::Matrix<double, 2, 3> Jproj;
  Eigen::Matrix<double, 2, 6> J;

  for (size_t i = 0; i < features_.size(); ++i) {
    const Eigen::Vector3d& Xi = world_points_[i];
    const Eigen::Vector3d Z = R * Xi + pose.translation;
    if (Z(2) < 0.0) {
      continue;
    }

    const Eigen::Vector2d zp = Z.hnormalized();
    Jproj << 1.0 / Z(2), 0.0, -zp(0) / Z(2), 0.0, 1.0 / Z(2), -zp(1) / Z(2);

    const Eigen::Vector2d res = zp - features_[i];

    // Jacobian w.r.t. right-multiplicative rotation and body-frame translation.
    const Eigen::Matrix<double, 2, 3> dZ = Jproj * R;
    J.col(0) = -Xi(2) * dZ.col(1) + Xi(1) * dZ.col(2);
    J.col(1) = Xi(2) * dZ.col(0) - Xi(0) * dZ.col(2);
    J.col(2) = -Xi(1) * dZ.col(0) + Xi(0) * dZ.col(1);
    J.block<2, 3>(0, 3) = dZ;
    acc.AddJacobian(res, J);
  }
}

AbsolutePoseReprojectionRefiner::Model AbsolutePoseReprojectionRefiner::Step(
    const Eigen::VectorXd& dp, const Model& pose) const {
  Model pose_new;
  // R(δ) = R * Exp(δω); t ← t + R δt  (PoseLib post-multiply convention).
  pose_new.rotation = pose.rotation * ExpSO3(dp.head<3>());
  pose_new.translation = pose.translation + pose.rotation * dp.tail<3>();
  return pose_new;
}

bool RefineAbsolutePoseReprojection(
    const std::vector<Eigen::Vector2d>& features,
    const std::vector<Eigen::Vector3d>& world_points,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(position);
  CHECK_EQ(features.size(), world_points.size());
  if (features.size() < 3) {
    return false;
  }

  AbsolutePoseLMState state;
  state.rotation = *rotation;
  // Theia position c with Z = R (X - c)  ⇒  t = -R c.
  state.translation = -(*rotation) * (*position);

  AbsolutePoseReprojectionRefiner refiner(features, world_points);
  LmOptions opt;
  opt.max_iterations = 25;
  opt.loss_scale = squared_error_thresh;

  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *position = -state.rotation.transpose() * state.translation;
  return local_stats.final_cost < local_stats.initial_cost;
}

}  // namespace theia
