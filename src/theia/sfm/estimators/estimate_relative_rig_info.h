// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// RANSAC estimation of metric relative pose between two calibrated rig poses
// using the 5+1 generalized relative pose solver (central 5-pt + 1 scale ray).

#ifndef THEIA_SFM_ESTIMATORS_ESTIMATE_RELATIVE_RIG_INFO_H_
#define THEIA_SFM_ESTIMATORS_ESTIMATE_RELATIVE_RIG_INFO_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/pose/generalized_ray_correspondence.h"
#include "theia/sfm/twoview_info.h"
#include "theia/solvers/sample_consensus_estimator.h"

namespace theia {

struct RansacParameters;

// Relative pose of rig/capture 2 w.r.t. rig/capture 1 at identity.
// Convention: X2 = rotation * X1 + translation (PoseLib / Sweeney).
// position = -rotation^T * translation is the second rig origin in the first
// rig frame (TwoViewInfo-compatible camera-center convention).
struct RelativeRigInfo {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  // Fill a TwoViewInfo with metric (non-normalized) position_2.
  void ToTwoViewInfo(TwoViewInfo* info) const;
};

// Estimates metric relative rig pose.
//
// |central_matches| must share a common (origin1, origin2) pair — typically
// same-sensor temporal matches (e.g. left↔left) expressed in the rig frame.
// |generalized_matches| provide the scale-fixing 6th ray (often cross-sensor
// or the other camera); they are also used for scoring. If empty, scale cannot
// be recovered and the function returns false.
//
// RANSAC error threshold in |ransac_params.error_thresh| is on the squared
// generalized epipolar residual (see GeneralizedEpipolarResidual).
bool EstimateRelativeRigInfo(
    const RansacParameters& ransac_params,
    const std::vector<GeneralizedRayCorrespondence>& central_matches,
    const std::vector<GeneralizedRayCorrespondence>& generalized_matches,
    RelativeRigInfo* relative_rig_info,
    RansacSummary* ransac_summary);

// Upright variant: rotation about |gravity_axis| only, using four generalized
// correspondences (Sweeney / FourPointRelativePosePartialRotation).
bool EstimateRelativeRigInfoUpright(
    const RansacParameters& ransac_params,
    const Eigen::Vector3d& gravity_axis,
    const std::vector<GeneralizedRayCorrespondence>& matches,
    RelativeRigInfo* relative_rig_info,
    RansacSummary* ransac_summary);

}  // namespace theia

#endif  // THEIA_SFM_ESTIMATORS_ESTIMATE_RELATIVE_RIG_INFO_H_
