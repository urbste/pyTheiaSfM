// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Metric per-edge scale estimation for same-sensor view pairs of a rig
// reconstruction. Used to fill TwoViewInfo::scale_estimate before global
// position averaging.

#ifndef THEIA_SFM_RIG_RIG_EDGE_SCALES_H_
#define THEIA_SFM_RIG_RIG_EDGE_SCALES_H_

namespace theia {

class Reconstruction;
class ViewGraph;

// For every edge (v1, v2) of |view_graph| where v1 and v2 are the same rig
// sensor in two different captures, estimates the metric distance |c2 - c1|
// between the two camera centres. The camera centres of the other sensors of
// v1's capture are used to triangulate points (rays from the posed cameras of
// that capture), and the scale is the median over those points of the
// distance along the edge direction that best explains v2's bearings.
//
// The reconstruction must have its capture orientations set and its view
// poses propagated (e.g. with capture positions at zero); the camera centres
// of the views of one capture must be metric relative to each other.
//
// Edges that cannot be estimated (missing rig membership, no second sensor
// in the capture, too few points, unstable estimates) are left unchanged.
// Returns the number of edges whose scale_estimate was set.
int EstimateMetricEdgeScales(const Reconstruction& reconstruction,
                             ViewGraph* view_graph,
                             int min_points = 10);

}  // namespace theia

#endif  // THEIA_SFM_RIG_RIG_EDGE_SCALES_H_
