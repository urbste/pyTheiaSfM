#include "theia/sfm/estimate_twoview_info.h"
#include "theia/sfm/colorize_reconstruction.h"
#include "theia/sfm/extract_maximally_parallel_rigid_subgraph.h"

#include "theia/sfm/filter_view_graph_cycles_by_rotation.h"
#include "theia/sfm/filter_view_pairs_from_orientation.h"
#include "theia/sfm/filter_view_pairs_from_relative_translation.h"
#include "theia/sfm/find_common_tracks_in_views.h"
#include "theia/sfm/find_common_views_by_name.h"
#include "theia/sfm/localize_view_to_reconstruction.h"
#include "theia/sfm/select_good_tracks_for_bundle_adjustment.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/undistort_image.h"

namespace theia {

class FloatImage;
class Reconstruction;
class TwoViewInfo;
class ViewGraph;
class Camera;


std::tuple<bool, TwoViewInfo, std::vector<int>> EstimateTwoViewInfoWrapper(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences);

Reconstruction ColorizeReconstructionWrapper(const std::string& image_directory,
                            const int num_threads);

ViewGraph ExtractMaximallyParallelRigidSubgraphWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations);

ViewGraph FilterViewGraphCyclesByRotationWrapper(const double max_loop_error_degrees);

ViewGraph FilterViewPairsFromOrientationWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const double max_relative_rotation_difference_degrees);

ViewGraph FilterViewPairsFromRelativeTranslationWrapper(
    const FilterViewPairsFromRelativeTranslationOptions& options,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations);

std::tuple<bool, Reconstruction, RansacSummary> LocalizeViewToReconstructionWrapper(
    const ViewId view_to_localize,
    const LocalizeViewToReconstructionOptions options);

std::tuple<bool, std::unordered_set<TrackId>> SelectGoodTracksForBundleAdjustmentAllWrapper(
    const Reconstruction& reconstruction,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view);

std::tuple<bool, std::unordered_set<TrackId>> SelectGoodTracksForBundleAdjustmentWrapper(
    const Reconstruction& reconstruction,
    const std::unordered_set<ViewId>& view_ids,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view);

Reconstruction SetCameraIntrinsicsFromPriorsWrapper();

std::tuple<int, Reconstruction> SetOutlierTracksToUnestimatedWrapper(const std::unordered_set<TrackId>& tracks,
                                  const double max_inlier_reprojection_error,
                                  const double min_triangulation_angle_degrees);

std::tuple<int, Reconstruction> SetOutlierTracksToUnestimatedAllWrapper(const double max_inlier_reprojection_error,
                                  const double min_triangulation_angle_degrees);


std::tuple<bool, FloatImage> UndistortImageWrapper(const Camera& distorted_camera,
                    const FloatImage& distorted_image,
                    const Camera& undistorted_camera);


std::tuple<bool, Camera> UndistortCameraWrapper(const Camera& distorted_camera);



std::tuple<bool, Reconstruction> UndistortReconstructionWrapper();

} // namespace theia
