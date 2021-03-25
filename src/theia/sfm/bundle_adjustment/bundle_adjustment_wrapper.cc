#include "theia/sfm/bundle_adjustment/bundle_adjustment_wrapper.h"


namespace theia {


std::tuple<BundleAdjustmentSummary, Reconstruction> BundleAdjustReconstructionWrapper(
        const BundleAdjustmentOptions& options){
    Reconstruction reconstruction;
    BundleAdjustmentSummary ba_summary = BundleAdjustReconstruction(options, &reconstruction);
    return std::make_tuple(ba_summary, reconstruction);
}

std::tuple<BundleAdjustmentSummary, Reconstruction> BundleAdjustPartialReconstructionWrapper(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& views_to_optimize,
        const std::unordered_set<TrackId>& tracks_to_optimize){
    Reconstruction reconstruction;
    BundleAdjustmentSummary ba_summary = BundleAdjustPartialReconstruction(options, views_to_optimize, tracks_to_optimize, &reconstruction);
    return std::make_tuple(ba_summary, reconstruction);

}


std::tuple<BundleAdjustmentSummary, Reconstruction> BundleAdjustViewWrapper(const BundleAdjustmentOptions& options,
                                                const ViewId view_id){
    Reconstruction reconstruction;
    BundleAdjustmentSummary ba_summary = BundleAdjustView(options, view_id, &reconstruction);
    return std::make_tuple(ba_summary, reconstruction);
}


std::tuple<BundleAdjustmentSummary, Reconstruction> BundleAdjustTrackWrapper(
    const BundleAdjustmentOptions& options,
        const TrackId track_id){
    Reconstruction reconstruction;
    BundleAdjustmentSummary ba_summary = BundleAdjustTrack(options, track_id, &reconstruction);
    return std::make_tuple(ba_summary, reconstruction);
}

std::tuple<BundleAdjustmentSummary, Camera, Camera, std::vector<Eigen::Vector4d>> BundleAdjustTwoViewsWrapper(
    const TwoViewBundleAdjustmentOptions& options,
        const std::vector<FeatureCorrespondence>& correspondences){
    Camera camera1;
    Camera camera2;
    std::vector<Eigen::Vector4d> points_3d;
    BundleAdjustmentSummary ba_summary = BundleAdjustTwoViews(options, correspondences, &camera1, &camera2, &points_3d);
    return std::make_tuple(ba_summary, camera1, camera2, points_3d);
}

std::tuple<BundleAdjustmentSummary, TwoViewInfo> BundleAdjustTwoViewsAngularWrapper(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& correspondences){
    TwoViewInfo info;
    BundleAdjustmentSummary ba_summary = BundleAdjustTwoViewsAngular(options, correspondences, &info);
    return std::make_tuple(ba_summary, info);
}

std::tuple<bool, Eigen::Vector3d> OptimizeRelativePositionWithKnownRotationWrapper(
    const std::vector<FeatureCorrespondence>& correspondences,
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2){
    Eigen::Vector3d relative_position;
    const bool success = OptimizeRelativePositionWithKnownRotation(correspondences, rotation1, rotation2, &relative_position);
    return std::make_tuple(success, relative_position);
}

}  // namespace theia
