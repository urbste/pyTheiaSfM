
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
    const std::vector<FeatureCorrespondence>& correspondences){
    TwoViewInfo twoview_info;
    std::vector<int> inlier_indices;
    const bool success = EstimateTwoViewInfo(options, intrinsics1, intrinsics2, correspondences, &twoview_info, &inlier_indices);
    return std::make_tuple(success, twoview_info, inlier_indices);
}

Reconstruction ColorizeReconstructionWrapper(const std::string& image_directory,
                                             const int num_threads){
    Reconstruction reconstruction;
    ColorizeReconstruction(image_directory, num_threads, &reconstruction);
    return reconstruction;
}

ViewGraph ExtractMaximallyParallelRigidSubgraphWrapper(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations){
    ViewGraph view_graph;
    ExtractMaximallyParallelRigidSubgraph(orientations, &view_graph);
    return view_graph;
}

ViewGraph FilterViewGraphCyclesByRotationWrapper(const double max_loop_error_degrees){
    ViewGraph view_pairs;
    FilterViewGraphCyclesByRotation(max_loop_error_degrees, &view_pairs);
    return view_pairs;
}

ViewGraph FilterViewPairsFromOrientationWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees){
    ViewGraph view_pairs;
    FilterViewPairsFromOrientation(orientations, max_relative_rotation_difference_degrees, &view_pairs);
    return view_pairs;
}

ViewGraph FilterViewPairsFromRelativeTranslationWrapper(
    const FilterViewPairsFromRelativeTranslationOptions& options,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations){
    ViewGraph view_graph;
    FilterViewPairsFromRelativeTranslation(options, orientations, &view_graph);
    return view_graph;
}

std::tuple<bool, Reconstruction, RansacSummary> LocalizeViewToReconstructionWrapper(
    const ViewId view_to_localize,
    const LocalizeViewToReconstructionOptions options){
    Reconstruction reconstruction;
    RansacSummary summary;
    const bool success = LocalizeViewToReconstruction(view_to_localize, options, &reconstruction, &summary);
    return std::make_tuple(success, reconstruction, summary);
}

std::tuple<bool, std::unordered_set<TrackId>> SelectGoodTracksForBundleAdjustmentAllWrapper(
    const Reconstruction& reconstruction,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view){
    std::unordered_set<TrackId> tracks_to_optimize;
    const bool success = SelectGoodTracksForBundleAdjustment(reconstruction, long_track_length_threshold, image_grid_cell_size_pixels, min_num_optimized_tracks_per_view, &tracks_to_optimize);
    return std::make_tuple(success, tracks_to_optimize);
}

std::tuple<bool, std::unordered_set<TrackId>> SelectGoodTracksForBundleAdjustmentWrapper(
    const Reconstruction& reconstruction,
    const std::unordered_set<ViewId>& view_ids,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view){
    std::unordered_set<TrackId> tracks_to_optimize;
    const bool success = SelectGoodTracksForBundleAdjustment(reconstruction, view_ids, long_track_length_threshold, image_grid_cell_size_pixels, min_num_optimized_tracks_per_view, &tracks_to_optimize);
    return std::make_tuple(success, tracks_to_optimize);
}

Reconstruction SetCameraIntrinsicsFromPriorsWrapper(){
    Reconstruction reconstruction;
    SetCameraIntrinsicsFromPriors(&reconstruction);
    return reconstruction;
}

std::tuple<int, Reconstruction> SetOutlierTracksToUnestimatedWrapper(const std::unordered_set<TrackId>& tracks,
                                  const double max_inlier_reprojection_error,
                                  const double min_triangulation_angle_degrees){
    Reconstruction reconstruction;
    int num_features_rm = SetOutlierTracksToUnestimated(tracks, max_inlier_reprojection_error, min_triangulation_angle_degrees, &reconstruction);
    return std::make_tuple(num_features_rm, reconstruction);
}

std::tuple<int, Reconstruction> SetOutlierTracksToUnestimatedAllWrapper(const double max_inlier_reprojection_error,
                                                                        const double min_triangulation_angle_degrees){
    Reconstruction reconstruction;
    int num_features_rm = SetOutlierTracksToUnestimated(max_inlier_reprojection_error, min_triangulation_angle_degrees, &reconstruction);
    return std::make_tuple(num_features_rm, reconstruction);
}


//std::tuple<bool, FloatImage> UndistortImageWrapper(const Camera& distorted_camera,
//                    const FloatImage& distorted_image,
//                    const Camera& undistorted_camera){
//    FloatImage undistorted_image;
//    const bool success = UndistortImage(distorted_camera, distorted_image, undistorted_camera,&undistorted_image);
//    return std::make_tuple(success, undistorted_image);
//}


//std::tuple<bool, Camera> UndistortCameraWrapper(const Camera& distorted_camera){
//    Camera undistorted_camera;
//    const bool success = UndistortCamera(distorted_camera, &undistorted_camera);
//    return std::make_tuple(success, undistorted_camera);
//}



//std::tuple<bool, Reconstruction> UndistortReconstructionWrapper(){
//    Reconstruction reconstruction;
//    const bool success = UndistortReconstruction(&reconstruction);
//    return std::make_tuple(success, reconstruction);
//}

} // namespace theia
