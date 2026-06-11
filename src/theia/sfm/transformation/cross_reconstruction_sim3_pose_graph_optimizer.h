// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_SIM3_POSE_GRAPH_OPTIMIZER_H_
#define THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_SIM3_POSE_GRAPH_OPTIMIZER_H_

#include <map>
#include <memory>
#include <vector>

#include <ceres/ceres.h>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/transformation/cross_reconstruction_pose_graph_types.h"
#include "theia/sfm/types.h"

namespace theia {

// Standalone Ceres pose-graph alignment between two reconstructions.
// Does not modify BundleAdjuster or core SfM estimators.
class CrossReconstructionSim3PoseGraphOptimizer {
 public:
  explicit CrossReconstructionSim3PoseGraphOptimizer(
      const CrossReconstructionPoseGraphOptions& options =
          CrossReconstructionPoseGraphOptions());

  void SetFixedReconstruction(
      const Reconstruction& fixed_reconstruction,
      const std::vector<ViewId>& anchor_view_ids);

  void SetVariableReconstruction(
      const Reconstruction& variable_reconstruction,
      const std::vector<ViewId>& keyframe_view_ids);

  // Override initial optimizer poses without mutating the reconstruction.
  void SetInitialVariablePoses(const Sim3LieMap& initial_poses);

  void AddSequentialEdge(const SequentialSim3Edge& edge);
  void AddCrossViewEdge(const CrossViewAnchorEdge& edge);
  void AddScaleSmoothnessEdge(ViewId view_i, ViewId view_j, double weight);

  void SetConstraints(const CrossReconstructionConstraints& constraints);

  bool Optimize(CrossReconstructionPoseGraphSummary* summary);

  const Sim3LieMap& variable_poses() const { return variable_lies_; }

  void ApplyToVariableReconstruction(Reconstruction* variable_reconstruction,
                                     bool transform_tracks = false);

 private:
  void ClearProblem();
  bool BuildProblem();
  void AddAutoScaleSmoothnessEdges();
  bool PosesAreFinite(const Sim3LieMap& poses, const char* label) const;
  void LogCostBreakdown(const char* label) const;
  void LogJacobianBreakdown(const char* label) const;
  void FillResidualCostSummary(CrossReconstructionPoseGraphSummary* summary) const;

  CrossReconstructionPoseGraphOptions options_;
  std::vector<SequentialSim3Edge> sequential_edges_;
  std::vector<CrossViewAnchorEdge> cross_view_edges_;
  std::vector<std::tuple<ViewId, ViewId, double>> scale_smooth_edges_;

  Sim3LieMap fixed_lies_;
  Sim3LieMap variable_lies_;
  std::vector<ViewId> variable_keyframe_order_;

  std::unique_ptr<ceres::Problem> problem_;
  // Non-owning; ceres::Problem deletes these in ~Problem.
  std::vector<ceres::LossFunction*> anchor_losses_;

  int num_sequential_residuals_ = 0;
  int num_anchor_residuals_ = 0;
  int num_scale_smooth_residuals_ = 0;
};

bool AlignReconstructionsWithPoseGraph(
    const Reconstruction& fixed_reconstruction,
    Reconstruction* variable_reconstruction,
    const CrossReconstructionConstraints& constraints,
    const CrossReconstructionPoseGraphOptions& options,
    CrossReconstructionPoseGraphSummary* summary,
    bool apply_to_variable_reconstruction = true);

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_SIM3_POSE_GRAPH_OPTIMIZER_H_
