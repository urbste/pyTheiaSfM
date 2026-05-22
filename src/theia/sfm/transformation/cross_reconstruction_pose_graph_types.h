// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_TYPES_H_
#define THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Sophus/sophus/sim3.hpp>
#include <vector>

#include "theia/sfm/types.h"

namespace theia {

using Sim3LieMap = aligned_unordered_map<ViewId, Eigen::Matrix<double, 7, 1>>;

struct CrossReconstructionPoseGraphOptions {
  double sequential_weight = 1.0;
  double anchor_weight = 1.0;
  double scale_smooth_weight = 0.1;
  double huber_delta_anchor = 1.0;
  bool auto_scale_smoothness = true;
  int max_num_iterations = 50;
  bool verbose = false;
  // Log per-block costs and pose finiteness before/after Ceres.
  bool debug_cost_breakdown = false;
};

struct CrossReconstructionPoseGraphSummary {
  bool success = false;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int num_iterations = 0;
  bool poses_finite_before = true;
  bool poses_finite_after = true;
  double sequential_residual_cost = 0.0;
  double anchor_residual_cost = 0.0;
  double scale_smooth_residual_cost = 0.0;
};

struct SequentialSim3Edge {
  ViewId view_id_i = kInvalidViewId;
  ViewId view_id_j = kInvalidViewId;
  Sophus::Sim3d measured_S_ji;
  Eigen::Matrix<double, 7, 7> sqrt_information =
      Eigen::Matrix<double, 7, 7>::Identity();
};

struct CrossViewAnchorEdge {
  ViewId variable_view_id = kInvalidViewId;  // run keyframe
  Sophus::Sim3d measured_S_run_in_seg;       // PnP target in segment world
  double weight = 1.0;
};

struct CrossReconstructionConstraints {
  std::vector<ViewId> variable_keyframe_view_ids;
  std::vector<ViewId> fixed_anchor_view_ids;
  std::vector<SequentialSim3Edge> sequential_edges;
  std::vector<CrossViewAnchorEdge> cross_view_edges;
};

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_TYPES_H_
