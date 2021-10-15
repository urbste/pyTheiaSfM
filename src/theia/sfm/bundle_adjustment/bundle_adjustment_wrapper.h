
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/bundle_adjust_two_views.h"
#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"

namespace theia {

using Matrix3d = Eigen::Matrix<double,3,3>;
using Matrix6d = Eigen::Matrix<double,6,6>;

BundleAdjustmentSummary BundleAdjustTwoViewsAngularWrapper(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo& two_view_info_prior);

BundleAdjustmentSummary BundleAdjustViewWrapper(
    Reconstruction& reconstruction, 
    const BundleAdjustmentOptions& options,
    const ViewId view_id);

BundleAdjustmentSummary BundleAdjustTrackWrapper(
    Reconstruction& reconstruction, 
    const BundleAdjustmentOptions& options,
    const TrackId track_id);

// with covariance information
std::tuple<BundleAdjustmentSummary, Matrix6d, double> BundleAdjustViewWithCovWrapper(
    Reconstruction& reconstruction, const BundleAdjustmentOptions& options, const ViewId view_id);

std::tuple<BundleAdjustmentSummary, Matrix3d, double> BundleAdjustTrackWithCovWrapper(
    Reconstruction& reconstruction, const BundleAdjustmentOptions& options, const TrackId track_id);

std::tuple<BundleAdjustmentSummary, std::map<ViewId, Matrix6d>, double> BundleAdjustViewsWithCovWrapper(
    Reconstruction& reconstruction, const BundleAdjustmentOptions& options, const std::vector<ViewId>& view_id);

std::tuple<BundleAdjustmentSummary, std::map<TrackId, Matrix3d>, double> BundleAdjustTracksWithCovWrapper(
    Reconstruction& reconstruction, const BundleAdjustmentOptions& options, const std::vector<TrackId>& track_id);

BundleAdjustmentSummary BundleAdjustReconstructionWrapper(
        const BundleAdjustmentOptions& options,
        Reconstruction& reconstruction);

BundleAdjustmentSummary BundleAdjustPartialReconstructionWrapper(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& views_to_optimize,
    const std::unordered_set<TrackId>& tracks_to_optimize,
    Reconstruction& reconstruction);

BundleAdjustmentSummary BundleAdjustPartialViewsConstantWrapper(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& views_to_optimize,
    const std::vector<TrackId>& tracks_to_optimize,
    Reconstruction& reconstruction);

// std::tuple<BundleAdjustmentSummary, Camera, Camera, std::vector<Eigen::Vector4d>> BundleAdjustTwoViewsWrapper(
//     const TwoViewBundleAdjustmentOptions& options,
//     const std::vector<FeatureCorrespondence>& correspondences);

// std::tuple<bool, Eigen::Vector3d> OptimizeRelativePositionWithKnownRotationWrapper(
//     const std::vector<FeatureCorrespondence>& correspondences,
//     const Eigen::Vector3d& rotation1,
//     const Eigen::Vector3d& rotation2);

}  // namespace theia
