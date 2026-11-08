#ifndef THEIA_SFM_FILTER_VIEW_GRAPH_BY_RELATIVE_INLIERS_H_
#define THEIA_SFM_FILTER_VIEW_GRAPH_BY_RELATIVE_INLIERS_H_

namespace theia {
class ViewGraph;
class Reconstruction;

// Removes every edge (view pair) whose number of verified matches is smaller
// than `min_relative_inliers` times the number of verified matches of the
// strongest edge of either endpoint view (the smaller of the two maxima is
// used). An absolute inlier threshold is meaningless when the matcher returns
// thousands of matches per pair (e.g. learned features with 8k keypoints); a
// relative one adapts to the match density of each view. A value <= 0 disables
// the filter. Returns the number of removed edges.
int FilterViewGraphByRelativeInliers(const double min_relative_inliers,
                                     ViewGraph* view_graph);

// Removes the observations from the tracks that are no longer supported by the
// view graph: an observation in view v of a track is kept only if v has an edge
// in the view graph to at least one other view of the same track. Tracks left
// with fewer than two observations are removed. Call this after removing edges
// from the view graph so the matches of removed (wrong) view pairs do not keep
// influencing the track based steps (triangulation, LiGT, bundle adjustment).
// Returns the number of removed observations.
int PruneTracksToViewGraph(const ViewGraph& view_graph,
                           Reconstruction* reconstruction);

// Convenience used by the reconstruction estimators: applies the optional
// view graph consistency filters configured in ReconstructionEstimatorOptions
// (relative inlier support first, then the triplet rotation-loop filter). If a
// reconstruction is given and any edge was removed, the tracks are pruned to the
// remaining edges (see PruneTracksToViewGraph). Both
// filters are disabled when their parameter is <= 0.
void FilterViewGraphForEstimation(const double min_relative_inliers,
                                  const double max_cycle_rotation_error_degrees,
                                  ViewGraph* view_graph,
                                  Reconstruction* reconstruction = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_FILTER_VIEW_GRAPH_BY_RELATIVE_INLIERS_H_
