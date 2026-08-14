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
  // If true, estimate metric capture–capture TwoViewInfo. Rotation and unit
  // translation come from the stripped View–View essential; metric scale is
  // recovered from stereo triangulation (3D–3D, then 3D–bearing). 5+1
  // EstimateRelativeRigInfo is only the fallback when that fails.
  bool use_metric_relative_rig_pose = true;

  // When metric estimation fails for a capture pair, use the legacy
  // ViewEdgeToCaptureEdge strip of ViewGraph essentials.
  bool fallback_to_twoview_strip = true;

  // Only estimate metric pose on capture pairs that already have a View–View
  // edge. Avoids O(pairs × tracks) scans over transitive track overlap.
  bool metric_only_for_viewgraph_pairs = true;

  // Skip 5+1 when the (stripped) motion direction is nearly parallel to a
  // sensor baseline — the scale ray is degenerate.
  bool skip_metric_if_baseline_degenerate = true;
  double max_baseline_translation_alignment = 0.95;

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
// with a View–View essential, take that stripped rotation and unit translation
// and recover metric scale from stereo triangulation in the two captures
// (3D–3D, then 3D–bearing, then generalized-ray γ). 5+1 is the last fallback.
// Pairs that cannot be estimated metrically optionally fall back to the
// unit-scale stripped essential. Intra-capture edges are never motion edges.
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
