
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

std::vector<std::vector<Eigen::Vector2f>> ComputeUndistortionMap(
    const Camera& distorted_camera,
    const Camera& undistorted_camera) {

  const int width = undistorted_camera.ImageWidth();
  const int height = undistorted_camera.ImageHeight();
  std::vector<std::vector<Eigen::Vector2f>> map(
    height, std::vector<Eigen::Vector2f>(width));

  const CameraIntrinsicsModel& undistorted_intrinsics =
      *undistorted_camera.CameraIntrinsics();
  const CameraIntrinsicsModel& distorted_intrinsics =
      *distorted_camera.CameraIntrinsics();

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Eigen::Vector2d image_point(x, y);
      const Eigen::Vector3d undistorted_point =
        undistorted_intrinsics.ImageToCameraCoordinates(image_point);
      const Eigen::Vector2d distorted_pixel =
         distorted_intrinsics.CameraToImageCoordinates(undistorted_point);
         
      map[y][x] = Eigen::Vector2f(distorted_pixel.x(), distorted_pixel.y());
    }
  }

  return map;
}

Camera UndistortCameraWrapper(const Camera& distorted_camera,
  const bool scale_intr_to_new_image_bounds) {
  Camera undistorted_camera;
  UndistortCamera(distorted_camera, 
    scale_intr_to_new_image_bounds, &undistorted_camera);
  return undistorted_camera;
}

}  // namespace theia
