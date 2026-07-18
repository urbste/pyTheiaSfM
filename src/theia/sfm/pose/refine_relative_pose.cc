// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib robust/optim/relative.h, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.

#include "theia/sfm/pose/refine_relative_pose.h"

#include <glog/logging.h>

namespace theia {

bool RefineRelativePoseSampson(const std::vector<Eigen::Vector2d>& x1,
                               const std::vector<Eigen::Vector2d>& x2,
                               double squared_error_thresh,
                               Eigen::Matrix3d* rotation,
                               Eigen::Vector3d* position,
                               LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(position);
  CHECK_EQ(x1.size(), x2.size());
  if (x1.size() < 5) {
    return false;
  }

  RelativePoseLMState state;
  state.rotation = *rotation;
  // Theia position = -Rᵀ t  ⇒  t = -R position.
  state.translation = -(*rotation) * (*position);
  if (state.translation.norm() < 1e-12) {
    return false;
  }

  RelativePoseSampsonRefiner refiner(x1, x2);
  LmOptions opt;
  opt.max_iterations = 15;
  opt.loss_scale = squared_error_thresh;

  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *position = -state.rotation.transpose() * state.translation;
  const double pos_norm = position->norm();
  if (pos_norm < 1e-12) {
    return false;
  }
  *position /= pos_norm;

  return local_stats.final_cost < local_stats.initial_cost;
}

}  // namespace theia
