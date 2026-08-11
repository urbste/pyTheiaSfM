// Copyright (C) 2026 The pyTheiaSfM Authors.
#ifndef THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_
#define THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_

#include "theia/sfm/types.h"
#include "theia/solvers/sample_consensus_estimator.h"

namespace theia {

class Reconstruction;
class ViewGraph;

// Options for building the capture–capture view graph.
struct BuildCaptureViewGraphOptions {
  // If true, estimate metric capture–capture TwoViewInfo via
  // EstimateRelativeRigInfo (5+1) from tracks when possible. Falls back to
  // stripping unit-scale View–View essentials when metric estimation fails
  // (if |fallback_to_twoview_strip| is true).
  bool use_metric_relative_rig_pose = true;

  // When metric estimation fails for a capture pair, use the legacy
  // ViewEdgeToCaptureEdge strip of ViewGraph essentials.
  bool fallback_to_twoview_strip = true;

  // RANSAC knobs for EstimateRelativeRigInfo.
  RansacParameters relative_rig_ransac;

  BuildCaptureViewGraphOptions() {
    // Normalized generalized epipolar residual; loose enough for pixel noise
    // after lifting to bearings.
    relative_rig_ransac.error_thresh = 1e-4;
    relative_rig_ransac.min_iterations = 100;
    relative_rig_ransac.max_iterations = 2000;
    relative_rig_ransac.failure_probability = 0.01;
  }
};

// Builds a view graph whose vertices are CaptureIds.
//
// Default path (|options.use_metric_relative_rig_pose|): for each capture pair
// that shares tracks, lift observations into the abstract rig frame and run
// EstimateRelativeRigInfo (central same-sensor 5-pt + cross-sensor scale ray).
// Edges are metric. Pairs that cannot be estimated metrically optionally fall
// back to stripping calibrated RigSensor extrinsics from View–View TwoViewInfo
// edges (unit-scale). Intra-capture edges are never motion edges.
//
// Returns false if no inter-capture edges could be created.
bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph);

bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph,
                           const BuildCaptureViewGraphOptions& options);

}  // namespace theia

#endif  // THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_
