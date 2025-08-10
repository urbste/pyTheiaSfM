#pragma once

#include "theia/sfm/colorize_reconstruction.h"
#include "theia/sfm/estimate_twoview_info.h"
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
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/track_builder.h"
#include "theia/sfm/find_common_tracks_by_feature_in_reconstructions.h"

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

std::tuple<bool, std::unordered_set<TrackId>>
SelectGoodTracksForBundleAdjustmentWrapper(
    const Reconstruction& reconstruction,
    const std::unordered_set<ViewId>& view_ids,
    const int long_track_length_threshold,
    const int image_grid_cell_size_pixels,
    const int min_num_optimized_tracks_per_view);

int SetOutlierTracksToUnestimatedWrapper(
    const std::unordered_set<TrackId>& tracks,
    const double max_inlier_reprojection_error,
    const double min_triangulation_angle_degrees,
    Reconstruction& reconstruction);

// Outputs the ViewId of all estimated views in the reconstruction.
std::unordered_set<ViewId> GetEstimatedViewsFromReconstructionWrapper(
    const Reconstruction& reconstruction);

// Outputs the TrackId of all estimated tracks in the reconstruction.
std::unordered_set<TrackId> GetEstimatedTracksFromReconstructionWrapper(
    const Reconstruction& reconstruction);

// Let's us fill the track builder with a vector of features.
void AddFeatureCorrespondencesToTrackBuilderWrapper(
    const ViewId view_id1,
    const std::vector<Eigen::Vector2d>& features1,
    const ViewId view_id2,
    const std::vector<Eigen::Vector2d>& features2,
    TrackBuilder& track_builder);

void UpdateFeaturesInViewWrapper(
    const ViewId& view_id,
    const std::vector<TrackId>& track_ids,
    const std::vector<Eigen::Vector2d>& new_features,
    const std::vector<Eigen::Matrix2d>& covariances,
    Reconstruction& reconstruction);

// Find common tracks between two reconstructions based on ffeature similiarity
// Feature need to be exactly identical in terms of 2D location
std::tuple<std::vector<Eigen::Vector3d>,
           std::vector<Eigen::Vector3d>,
           std::vector<std::pair<TrackId, TrackId>>>
FindCommonTracksByFeatureInReconstructionsWrapper(
    const Reconstruction& reconstruction_ref,   
    const Reconstruction& reconstruction_qry,
    const std::vector<std::pair<ViewId, ViewId>>& view_graph_matches_ref_qry);

}  // namespace theia
