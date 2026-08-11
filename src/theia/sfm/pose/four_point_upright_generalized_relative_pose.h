// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Upright generalized relative pose from four ray correspondences.
// Thin wrapper around Theia's FourPointRelativePosePartialRotation (Sweeney
// et al., 3DV 2014) — the same algorithm PoseLib exposes as
// gen_relpose_upright_4pt.

#ifndef THEIA_SFM_POSE_FOUR_POINT_UPRIGHT_GENERALIZED_RELATIVE_POSE_H_
#define THEIA_SFM_POSE_FOUR_POINT_UPRIGHT_GENERALIZED_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/pose/generalized_ray_correspondence.h"
#include "theia/sfm/rigid_transformation.h"

namespace theia {

// Solves for relative pose with rotation constrained to |gravity_axis|
// (typically world up). Correspondences may be non-central (different ray
// origins). Output uses X2 = rotation * X1 + translation.
//
// Returns the number of solutions (at most 6).
int FourPointUprightGeneralizedRelativePose(
    const Eigen::Vector3d& gravity_axis,
    const std::vector<GeneralizedRayCorrespondence>& correspondences,
    std::vector<RigidTransformation>* solutions);

}  // namespace theia

#endif  // THEIA_SFM_POSE_FOUR_POINT_UPRIGHT_GENERALIZED_RELATIVE_POSE_H_
