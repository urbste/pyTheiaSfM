// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_TRANSFORMATION_SIM3_POSE_FROM_VIEW_H_
#define THEIA_SFM_TRANSFORMATION_SIM3_POSE_FROM_VIEW_H_

#include <Eigen/Core>
#include <Sophus/sophus/sim3.hpp>
#include <map>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/transformation/cross_reconstruction_pose_graph_types.h"
#include "theia/sfm/view.h"

namespace theia {

// Camera pose as Sim3(R_c_w, t_c_w) with t_c_w = -R_c_w * camera_position.
Sophus::Sim3d GetSim3PoseFromView(const View& view);

Eigen::Matrix<double, 7, 1> GetSim3LieFromView(const View& view);

void SetViewCameraFromSim3Lie(View* view, const Eigen::Matrix<double, 7, 1>& lie);

void GetSim3LiesFromReconstruction(const Reconstruction& reconstruction,
                                 const std::vector<ViewId>& view_ids,
                                 Sim3LieMap* lies);

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_SIM3_POSE_FROM_VIEW_H_
