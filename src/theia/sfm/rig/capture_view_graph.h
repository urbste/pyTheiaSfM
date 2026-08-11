// Copyright (C) 2026 The pyTheiaSfM Authors.
#ifndef THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_
#define THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_

#include "theia/sfm/types.h"

namespace theia {

class Reconstruction;
class ViewGraph;

// Builds a view graph whose vertices are CaptureIds. Each edge is the relative
// pose of capture 2 w.r.t. capture 1 (capture 1 at identity), obtained by
// stripping known calibrated RigSensor extrinsics from View–View TwoViewInfo
// edges. Intra-capture edges are ignored. When several View edges map to the
// same CaptureId pair, the edge with the most verified matches is kept.
//
// Returns false if no inter-capture edges could be created.
bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph);

}  // namespace theia

#endif  // THEIA_SFM_RIG_CAPTURE_VIEW_GRAPH_H_
