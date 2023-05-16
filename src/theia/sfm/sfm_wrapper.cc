
#include "theia/sfm/sfm_wrapper.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view_graph/view_graph.h"
//#include "theia/image/image.h"
#include "theia/sfm/camera/camera.h"

namespace theia {

std::tuple<bool, std::unordered_set<TrackId>>
SelectGoodTracksForBundleAdjustmentWrapper(
    const Reconstruction& reconstruction,
    const std::unordered_set<ViewId>& view_ids,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view) {
  std::unordered_set<TrackId> tracks_to_optimize;
  const bool success =
      SelectGoodTracksForBundleAdjustment(reconstruction,
                                          view_ids,
                                          long_track_length_threshold,
                                          image_grid_cell_size_pixels,
                                          min_num_optimized_tracks_per_view,
                                          &tracks_to_optimize);
  return std::make_tuple(success, tracks_to_optimize);
}

int SetOutlierTracksToUnestimatedWrapper(
    const std::unordered_set<TrackId>& tracks,
    const double max_inlier_reprojection_error,
    const double min_triangulation_angle_degrees,
    Reconstruction& reconstruction) {
  int num_features_rm =
      SetOutlierTracksToUnestimated(tracks,
                                    max_inlier_reprojection_error,
                                    min_triangulation_angle_degrees,
                                    &reconstruction);
  return num_features_rm;
}

}  // namespace theia
