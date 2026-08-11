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
};

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_RIG_RECONSTRUCTOR_H_
