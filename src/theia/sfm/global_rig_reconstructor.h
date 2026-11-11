// Copyright (C) 2026 The pyTheiaSfM Authors.
//
// Global multi-camera SfM for calibrated rigs (MGSfM-inspired). Builds a
// capture–capture view graph (metric 5+1 when tracks allow), then runs
// Theia's selectable rotation / position averaging stack on capture nodes.

#ifndef THEIA_SFM_GLOBAL_RIG_RECONSTRUCTOR_H_
#define THEIA_SFM_GLOBAL_RIG_RECONSTRUCTOR_H_

#include <unordered_map>

#include <Eigen/Core>

#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/rig/capture_view_graph.h"
#include "theia/sfm/types.h"

namespace theia {

class Reconstruction;
class ViewGraph;

// Options reuse ReconstructionEstimatorOptions so callers can evaluate any
// GlobalRotationEstimatorType / GlobalPositionEstimatorType already exposed
// for monocular global SfM. Rig extrinsics are assumed calibrated (not
// optimized during averaging).
struct GlobalRigReconstructorOptions {
  ReconstructionEstimatorOptions sfm_options;
  BuildCaptureViewGraphOptions capture_graph_options;

  // After direction-based position averaging, rescale capture positions so
  // median edge lengths match metric TwoViewInfo.position_2 norms (from 5+1).
  bool rescale_positions_to_metric_edges = true;

  // Tie the camera centers of the views of each capture with the fixed rig
  // extrinsics in the view-graph position estimators (GLOMAP, nonlinear,
  // LiGT, ...). Fixes the metric scale; the median rescale is then skipped.
  bool use_rig_constraints_in_position_estimation = true;

  // Fill TwoViewInfo::scale_estimate on same-sensor capture edges from the
  // rig geometry (see EstimateMetricEdgeScales) before position estimation,
  // so direction-only estimators get metric spacing along straight motion.
  // Only used together with use_rig_constraints_in_position_estimation.
  bool use_rig_edge_scales = true;

  // Bundle adjust with one pose per capture (rig constraint enforced in the
  // optimizer) instead of per-view BA followed by re-snapping to the rig.
  bool use_rig_constrained_ba = true;
};

class GlobalRigReconstructor {
 public:
  explicit GlobalRigReconstructor(const GlobalRigReconstructorOptions& options);

  // |view_graph| is the View–View match graph. Captures / rig membership must
  // already exist on |reconstruction|. Extrinsics are assumed calibrated.
  // Tracks should already be built so metric capture edges can be estimated.
  ReconstructionEstimatorSummary Estimate(ViewGraph* view_graph,
                                          Reconstruction* reconstruction);

 private:
  bool FilterCaptureGraph();
  bool EstimateCaptureRotations();
  void FilterCaptureRotations();
  bool EstimateCapturePositions();
  bool EstimateCapturePositionsFromViewGraph(ViewGraph* view_graph);
  void RescaleCapturePositionsToMetricEdges();
  void SetCapturePosesAndPropagate();
  void EstimateStructure();
  void BundleAdjustAndPropagate();

  GlobalRigReconstructorOptions options_;
  ViewGraph* capture_view_graph_ = nullptr;
  Reconstruction* reconstruction_ = nullptr;

  std::unordered_map<ViewId, Eigen::Vector3d> orientations_;
  std::unordered_map<ViewId, Eigen::Vector3d> positions_;
  bool rig_constraints_applied_ = false;
};

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_RIG_RECONSTRUCTOR_H_
