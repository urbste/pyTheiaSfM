
#include "theia/sfm/sfm_wrapper.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view_graph/view_graph.h"
//#include "theia/image/image.h"
#include "theia/sfm/camera/camera.h"

namespace theia {

std::tuple<bool, TwoViewInfo, std::vector<int>> EstimateTwoViewInfoWrapper(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences) {
  TwoViewInfo twoview_info;
  std::vector<int> inlier_indices;
  const bool success = EstimateTwoViewInfo(options,
                                           intrinsics1,
                                           intrinsics2,
                                           correspondences,
                                           &twoview_info,
                                           &inlier_indices);
  return std::make_tuple(success, twoview_info, inlier_indices);
}

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

// Outputs the ViewId of all estimated views in the reconstruction.
std::unordered_set<ViewId> GetEstimatedViewsFromReconstructionWrapper(
    const Reconstruction& reconstruction) {
  std::unordered_set<ViewId> estimated_views;
  theia::GetEstimatedViewsFromReconstruction(reconstruction, &estimated_views);
  return estimated_views;
}

// Outputs the TrackId of all estimated tracks in the reconstruction.
std::unordered_set<TrackId> GetEstimatedTracksFromReconstructionWrapper(
    const Reconstruction& reconstruction) {
  std::unordered_set<TrackId> estimated_tracks;
  theia::GetEstimatedTracksFromReconstruction(reconstruction, &estimated_tracks);
  return estimated_tracks;
}

void AddFeatureCorrespondencesToTrackBuilderWrapper(
    const ViewId view_id1,
    const std::vector<Eigen::Vector2d>& features1,
    const ViewId view_id2,
    const std::vector<Eigen::Vector2d>& features2,
    TrackBuilder& track_builder) {
  CHECK_EQ(features1.size(), features2.size())
      << "The number of features in each view must be the same.";
  for (int i = 0; i < features1.size(); i++) {
    Feature feature1(features1[i].x(), features1[i].y());
    Feature feature2(features2[i].x(), features2[i].y());
    track_builder.AddFeatureCorrespondence(view_id1, feature1, view_id2, feature2);
  }
}

}  // namespace theia
