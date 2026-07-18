// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// This file is adapted from PoseLib (https://github.com/PoseLib/PoseLib),
// solvers/relpose_5pt.{h,cc} (essential-matrix-only overload), commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08, and is distributed under the
// BSD-3-Clause license. See docs/licenses/POSELIB_LICENSE.txt for the full
// PoseLib license text.
//
// Reference: D. Nistér, "An Efficient Solution to the Five-Point Relative
// Pose Problem", IEEE TPAMI, 2004.

#ifndef THEIA_SFM_POSE_FIVE_POINT_RELATIVE_POSE_STURM_H_
#define THEIA_SFM_POSE_FIVE_POINT_RELATIVE_POSE_STURM_H_

#include <Eigen/Core>
#include <vector>

namespace theia {

// Computes the essential matrix (matrices) between two cameras from exactly
// 5 point correspondences, following Nistér's original polynomial-elimination
// route: QR nullspace -> explicit polynomial elimination -> degree-10
// polynomial in z -> Sturm-sequence root bracketing (theia/math/sturm.h) ->
// back-substitution. This never forms the 10x10 action-matrix
// eigendecomposition that theia's existing FivePointRelativePose() (a
// Stewenius-style solver) uses, which is what makes it faster.
//
// Unlike FivePointRelativePose(), this solver is STRICTLY minimal: it always
// consumes exactly 5 correspondences (fixed 9x5 nullspace computation) and
// does not support non-minimal (n > 5) estimation. It is intended for use
// inside the RANSAC inner loop, where it is called thousands of times per
// image pair; FivePointRelativePose() remains the solver of record for the
// public API and for n > 5 samples.
//
// Params:
//   x1h: Homogeneous (or bearing-vector) coordinates of 5 features in image 1.
//   x2h: Homogeneous (or bearing-vector) coordinates of 5 features in image 2.
// Returns: the number of real solutions found (0 to 10); essential_matrices
//   is resized to match.
int FivePointRelativePoseSturm(const std::vector<Eigen::Vector3d>& x1h,
                               const std::vector<Eigen::Vector3d>& x2h,
                               std::vector<Eigen::Matrix3d>* essential_matrices);

}  // namespace theia

#endif  // THEIA_SFM_POSE_FIVE_POINT_RELATIVE_POSE_STURM_H_
