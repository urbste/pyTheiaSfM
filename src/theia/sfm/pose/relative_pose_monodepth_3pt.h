// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// This file is adapted from PoseLib (https://github.com/PoseLib/PoseLib),
// solvers/relpose_monodepth_3pt{,_shared_focal,_varying_focal}.{h,cc},
// commit fa7280fee27f97aff31ae7f98bab7f583fac7d08, and is distributed under
// the BSD-3-Clause license. See docs/licenses/POSELIB_LICENSE.txt for the
// full PoseLib license text.
//
// Reference: Y. Ding, V. Larsson, et al., "RePoseD: Efficient Relative Pose
// Estimation With Known Depth Information", ICCV 2025.

#ifndef THEIA_SFM_POSE_RELATIVE_POSE_MONODEPTH_3PT_H_
#define THEIA_SFM_POSE_RELATIVE_POSE_MONODEPTH_3PT_H_

#include <Eigen/Core>
#include <vector>

namespace theia {

// Output of the monodepth 3-point relative pose solvers. Internally the
// solvers use PoseLib's convention x2 ~ rotation * x1 + translation.
struct MonoDepthRelativePose {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  // |translation| encodes the relative scale of the two depth maps; it is
  // NOT a unit vector like theia's other relative-pose solvers.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  // Relative scale of the two depth maps: depth2_metric = scale * depth2_raw
  // (with depth1 held fixed), following the ported solver's convention.
  double scale = 1.0;
  // Estimated additive depth shifts (calibrated 3pt solver only; 0 for the
  // shared/varying-focal solvers, which assume the depth maps are shift-free).
  double shift1 = 0.0;
  double shift2 = 0.0;
  // Shared/varying-focal solvers only; 1.0 (i.e. unused) for the calibrated
  // solver, which assumes both cameras' intrinsics are already known/applied.
  double focal_length1 = 1.0;
  double focal_length2 = 1.0;
};

// Calibrated monodepth 3-point solver. x1h/x2h are homogeneous (or bearing
// vector) coordinates of exactly 3 correspondences, normalized by known
// camera intrinsics; depth1/depth2 are the corresponding monocular depth
// estimates (e.g. from a metric or affine-invariant depth network) for each
// point in image 1 / image 2 respectively. The solver also recovers a shift
// for each depth map (pose.shift1/shift2).
int MonoDepthRelativePose3pt(const std::vector<Eigen::Vector3d>& x1h,
                             const std::vector<Eigen::Vector3d>& x2h,
                             const std::vector<double>& depth1,
                             const std::vector<double>& depth2,
                             std::vector<MonoDepthRelativePose>* poses);

// Uncalibrated variant assuming both cameras share one unknown focal length.
// x1h/x2h should be pixel coordinates with the principal point subtracted
// (NOT divided by any focal length guess); the recovered shared focal length
// is written to pose.focal_length1 == pose.focal_length2. Depth shifts are
// not estimated (shift1 == shift2 == 0).
int MonoDepthRelativePose3ptSharedFocal(
    const std::vector<Eigen::Vector3d>& x1h,
    const std::vector<Eigen::Vector3d>& x2h,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    std::vector<MonoDepthRelativePose>* poses);

// Uncalibrated variant with two independent unknown focal lengths (one per
// camera). Same input convention as the shared-focal variant. Depth shifts
// are not estimated (shift1 == shift2 == 0). Unlike the other two solvers,
// this one returns at most one solution.
int MonoDepthRelativePose3ptVaryingFocal(
    const std::vector<Eigen::Vector3d>& x1h,
    const std::vector<Eigen::Vector3d>& x2h,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    std::vector<MonoDepthRelativePose>* poses);

}  // namespace theia

#endif  // THEIA_SFM_POSE_RELATIVE_POSE_MONODEPTH_3PT_H_
