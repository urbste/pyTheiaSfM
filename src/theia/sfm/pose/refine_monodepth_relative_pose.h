// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// robust/optim/monodepth_relpose.h, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// Reference: Y. Ding et al., "RePoseD: Efficient Relative Pose Estimation
// With Known Depth Information", ICCV 2025.
//
// Dense LM local optimization for monodepth relative-pose RANSAC estimators.

#ifndef THEIA_SFM_POSE_REFINE_MONODEPTH_RELATIVE_POSE_H_
#define THEIA_SFM_POSE_REFINE_MONODEPTH_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_relative_pose.h"

namespace theia {

// Internal LM state (PoseLib metric translation convention).
struct MonoDepthLMState {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::UnitX();
  double scale = 1.0;
  double shift1 = 0.0;
  double shift2 = 0.0;
  double focal1 = 1.0;
  double focal2 = 1.0;
};

// Calibrated: params = [δω(3), δt(3), δs(1)] or + δshift1/2 → 7 or 9.
class MonoDepthRelativePoseRefiner {
 public:
  using Model = MonoDepthLMState;

  MonoDepthRelativePoseRefiner(const std::vector<Eigen::Vector2d>& x1,
                               const std::vector<Eigen::Vector2d>& x2,
                               const std::vector<double>& depth1,
                               const std::vector<double>& depth2,
                               double scale_reproj,
                               double weight_sampson,
                               bool refine_shift);

  int NumParams() const { return num_params_; }
  double ComputeResidual(NormalAccumulator& acc, const Model& geometry);
  void ComputeJacobian(NormalAccumulator& acc, const Model& geometry);
  Model Step(const Eigen::VectorXd& dp, const Model& geometry) const;

 private:
  const std::vector<Eigen::Vector2d>& x1_;
  const std::vector<Eigen::Vector2d>& x2_;
  const std::vector<double>& depth1_;
  const std::vector<double>& depth2_;
  double scale_reproj_;
  double weight_sampson_;
  bool refine_shift_;
  int num_params_;
};

// Shared focal (pp-centered pixels): params = [δω(3), δt(3), δs(1), δf(1)] = 8.
class MonoDepthSharedFocalRelativePoseRefiner {
 public:
  using Model = MonoDepthLMState;

  MonoDepthSharedFocalRelativePoseRefiner(
      const std::vector<Eigen::Vector2d>& x1,
      const std::vector<Eigen::Vector2d>& x2,
      const std::vector<double>& depth1,
      const std::vector<double>& depth2,
      double scale_reproj,
      double weight_sampson);

  int NumParams() const { return 8; }
  double ComputeResidual(NormalAccumulator& acc, const Model& geometry);
  void ComputeJacobian(NormalAccumulator& acc, const Model& geometry);
  Model Step(const Eigen::VectorXd& dp, const Model& geometry) const;

 private:
  const std::vector<Eigen::Vector2d>& x1_;
  const std::vector<Eigen::Vector2d>& x2_;
  const std::vector<double>& depth1_;
  const std::vector<double>& depth2_;
  double scale_reproj_;
  double weight_sampson_;
};

// Varying focals: params = [δω(3), δt(3), δs(1), δf1(1), δf2(1)] = 9.
class MonoDepthVaryingFocalRelativePoseRefiner {
 public:
  using Model = MonoDepthLMState;

  MonoDepthVaryingFocalRelativePoseRefiner(
      const std::vector<Eigen::Vector2d>& x1,
      const std::vector<Eigen::Vector2d>& x2,
      const std::vector<double>& depth1,
      const std::vector<double>& depth2,
      double scale_reproj,
      double weight_sampson);

  int NumParams() const { return 9; }
  double ComputeResidual(NormalAccumulator& acc, const Model& geometry);
  void ComputeJacobian(NormalAccumulator& acc, const Model& geometry);
  Model Step(const Eigen::VectorXd& dp, const Model& geometry) const;

 private:
  const std::vector<Eigen::Vector2d>& x1_;
  const std::vector<Eigen::Vector2d>& x2_;
  const std::vector<double>& depth1_;
  const std::vector<double>& depth2_;
  double scale_reproj_;
  double weight_sampson_;
};

// Free functions used by estimators. Updates Theia (rotation, unit position)
// plus monodepth extras in place. squared_error_thresh is the RANSAC Sampson
// truncation threshold (squared residual units).
bool RefineMonoDepthRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* shift1,
    double* shift2,
    LmStats* stats = nullptr);

bool RefineMonoDepthSharedFocalRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* focal_length,
    LmStats* stats = nullptr);

bool RefineMonoDepthVaryingFocalRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* focal_length1,
    double* focal_length2,
    LmStats* stats = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_POSE_REFINE_MONODEPTH_RELATIVE_POSE_H_
