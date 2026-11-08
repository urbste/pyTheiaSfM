#include "theia/sfm/filter_view_graph_by_relative_inliers.h"

#include <glog/logging.h>

#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "theia/sfm/filter_view_graph_cycles_by_rotation.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace theia {

int FilterViewGraphByRelativeInliers(const double min_relative_inliers,
                                     ViewGraph* view_graph) {
  if (min_relative_inliers <= 0.0 || view_graph == nullptr) {
    return 0;
  }
  const auto& edges = view_graph->GetAllEdges();
  std::unordered_map<ViewId, int> max_inliers;
  for (const auto& edge : edges) {
    const int n = edge.second.num_verified_matches;
    int& m1 = max_inliers[edge.first.first];
    m1 = std::max(m1, n);
    int& m2 = max_inliers[edge.first.second];
    m2 = std::max(m2, n);
  }
  std::vector<ViewIdPair> to_remove;
  for (const auto& edge : edges) {
    const double reference = std::min(max_inliers[edge.first.first],
                                      max_inliers[edge.first.second]);
    if (edge.second.num_verified_matches < min_relative_inliers * reference) {
      to_remove.push_back(edge.first);
    }
  }
  for (const ViewIdPair& pair : to_remove) {
    view_graph->RemoveEdge(pair.first, pair.second);
  }
  return static_cast<int>(to_remove.size());
}

int PruneTracksToViewGraph(const ViewGraph& view_graph,
                           Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  int num_removed_observations = 0;
  for (const TrackId track_id : reconstruction->TrackIds()) {
    const Track* track = reconstruction->Track(track_id);
    if (track == nullptr) {
      continue;
    }
    const std::unordered_set<ViewId> track_views = track->ViewIds();
    std::vector<ViewId> unsupported;
    for (const ViewId v : track_views) {
      bool supported = false;
      for (const ViewId w : track_views) {
        if (w != v && view_graph.GetEdge(v, w) != nullptr) {
          supported = true;
          break;
        }
      }
      if (!supported) {
        unsupported.push_back(v);
      }
    }
    if (unsupported.empty()) {
      continue;
    }
    if (track_views.size() - unsupported.size() < 2) {
      num_removed_observations += static_cast<int>(track_views.size());
      reconstruction->RemoveTrack(track_id);
      continue;
    }
    for (const ViewId v : unsupported) {
      reconstruction->MutableView(v)->RemoveFeature(track_id);
      reconstruction->MutableTrack(track_id)->RemoveView(v);
      ++num_removed_observations;
    }
  }
  return num_removed_observations;
}

void FilterViewGraphForEstimation(
    const double min_relative_inliers,
    const double max_cycle_rotation_error_degrees,
    ViewGraph* view_graph,
    Reconstruction* reconstruction) {
  if (view_graph == nullptr) {
    return;
  }
  const int num_edges_before = view_graph->NumEdges();
  FilterViewGraphByRelativeInliers(min_relative_inliers, view_graph);
  if (max_cycle_rotation_error_degrees > 0.0) {
    FilterViewGraphCyclesByRotation(max_cycle_rotation_error_degrees,
                                    view_graph);
  }
  if (reconstruction != nullptr && view_graph->NumEdges() != num_edges_before) {
    PruneTracksToViewGraph(*view_graph, reconstruction);
  }
}

}  // namespace theia
