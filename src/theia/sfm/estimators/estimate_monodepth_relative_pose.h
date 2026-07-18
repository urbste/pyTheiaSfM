// Copyright (C) 2024 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// RANSAC estimators wrapping the ported PoseLib monodepth 3-point solvers
// (theia/sfm/pose/relative_pose_monodepth_3pt.h) in theia's own
// SampleConsensusEstimator framework -- mirrors estimate_relative_pose.h /
// estimate_uncalibrated_relative_pose.h.

#ifndef THEIA_SFM_ESTIMATORS_ESTIMATE_MONODEPTH_RELATIVE_POSE_H_
#define THEIA_SFM_ESTIMATORS_ESTIMATE_MONODEPTH_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/create_and_initialize_ransac_variant.h"

namespace theia {

struct FeatureCorrespondence;
struct RansacParameters;
struct RansacSummary;

// Relative pose computed from monocular-depth-assisted 3-point minimal
// samples. Follows the same rotation/position convention as RelativePose
// (position = -rotation^T * unit_translation), so it can be dropped directly
// into TwoViewInfo::rotation_2/position_2.
struct MonoDepthRelativePoseResult {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  // Relative scale of the two (raw) depth maps: metric_depth2 = scale *
  // depth2_raw, holding depth1 as the metric reference. Valuable standalone
  // information (surfaced via TwoViewInfo::scale_estimate), independent of
  // the recovered rotation/position.
  double scale = 1.0;

  // Depth-map shift terms (calibrated solver only; always 0 for the
  // shared/varying-focal solvers, which assume shift-free depth maps).
  double shift1 = 0.0;
  double shift2 = 0.0;

  // Recovered focal length(s); only meaningful for the uncalibrated
  // (shared/varying-focal) variants. 1.0 for the calibrated variant, which
  // assumes intrinsics are already known/applied to the input points.
  double focal_length1 = 1.0;
  double focal_length2 = 1.0;
};

// Calibrated variant: normalized_correspondences must be normalized by known
// camera intrinsics (principal point at origin, divided by focal length),
// exactly like EstimateRelativePose(). Each correspondence's Feature must
// carry a valid depth_prior_ (> 0) from a monocular depth estimator.
bool EstimateMonoDepthRelativePose(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary);

// Uncalibrated variant assuming both cameras share one unknown focal length.
// centered_correspondences must have the principal point subtracted (pixel
// coordinates, NOT divided by any focal guess), exactly like
// EstimateUncalibratedRelativePose(). result->focal_length1 ==
// result->focal_length2 on success.
bool EstimateMonoDepthRelativePoseSharedFocal(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary);

// Uncalibrated variant with two independent unknown focal lengths. Same
// input convention as the shared-focal variant.
bool EstimateMonoDepthRelativePoseVaryingFocal(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary);

}  // namespace theia

#endif  // THEIA_SFM_ESTIMATORS_ESTIMATE_MONODEPTH_RELATIVE_POSE_H_
