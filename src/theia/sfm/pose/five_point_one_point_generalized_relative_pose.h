// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// 5+1 generalized relative pose: central five-point essential matrix on the
// first five correspondences (shared ray origins), then one additional
// (possibly non-central) correspondence to recover metric translation scale.
//
// Algorithm adapted from PoseLib gen_relpose_5p1pt (BSD-3-Clause), Viktor
// Larsson et al. https://github.com/PoseLib/PoseLib — reimplemented against
// Theia's five-point solvers (no PoseLib dependency).
// See docs/licenses/POSELIB_LICENSE.txt.

#ifndef THEIA_SFM_POSE_FIVE_POINT_ONE_POINT_GENERALIZED_RELATIVE_POSE_H_
#define THEIA_SFM_POSE_FIVE_POINT_ONE_POINT_GENERALIZED_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/pose/generalized_ray_correspondence.h"
#include "theia/sfm/rigid_transformation.h"

namespace theia {

// Estimates relative pose between two generalized cameras / rig frames.
//
// Requirements:
//   - correspondences.size() == 6
//   - The first 5 share the same origin1 and the same origin2 (central pair)
//   - The 6th may use different origins (e.g. another rig sensor) to fix scale
//
// Output RigidTransformation uses:
//   X2 = rotation * X1 + translation
// and is metric when the 6th correspondence provides a known baseline offset.
//
// Returns the number of pose hypotheses written to |solutions|.
int FivePointOnePointGeneralizedRelativePose(
    const std::vector<GeneralizedRayCorrespondence>& correspondences,
    std::vector<RigidTransformation>* solutions);

}  // namespace theia

#endif  // THEIA_SFM_POSE_FIVE_POINT_ONE_POINT_GENERALIZED_RELATIVE_POSE_H_
