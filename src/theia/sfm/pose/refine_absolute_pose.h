// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// robust/optim/absolute.h (PinholeAbsolutePoseRefiner), commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// 6-DoF reprojection refinement of a calibrated absolute pose for RANSAC LO.

#ifndef THEIA_SFM_POSE_REFINE_ABSOLUTE_POSE_H_
#define THEIA_SFM_POSE_REFINE_ABSOLUTE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_relative_pose.h"

namespace theia {

// Internal LM state: Z = R * X + t (PoseLib CameraPose convention).
struct AbsolutePoseLMState {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

// Minimizes reprojection error on normalized image points. 6 parameters:
// right-multiplicative rotation (3) + body-frame translation (3).
// Translation step is t ← t + R δt (matches the analytic Jacobian).
class AbsolutePoseReprojectionRefiner {
 public:
  using Model = AbsolutePoseLMState;
  static constexpr int kNumParams = 6;

  AbsolutePoseReprojectionRefiner(const std::vector<Eigen::Vector2d>& features,
                                  const std::vector<Eigen::Vector3d>& world_points)
      : features_(features), world_points_(world_points) {}

  int NumParams() const { return kNumParams; }

  double ComputeResidual(NormalAccumulator& acc, const Model& pose);
  void ComputeJacobian(NormalAccumulator& acc, const Model& pose);
  Model Step(const Eigen::VectorXd& dp, const Model& pose) const;

 private:
  const std::vector<Eigen::Vector2d>& features_;
  const std::vector<Eigen::Vector3d>& world_points_;
};

// Refines Theia (rotation, position) in place. position is the camera center
// in world coordinates. Returns true if the LM cost decreased.
// `squared_error_thresh` is the RANSAC squared-reprojection truncation
// threshold (normalized image units).
bool RefineAbsolutePoseReprojection(
    const std::vector<Eigen::Vector2d>& features,
    const std::vector<Eigen::Vector3d>& world_points,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    LmStats* stats = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_POSE_REFINE_ABSOLUTE_POSE_H_
