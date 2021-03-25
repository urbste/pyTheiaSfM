#include <Eigen/Core>
#include <vector>

#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/alignment/alignment.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/types.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/pose/six_point_radial_distortion_homography.h"

#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_dominant_plane_from_points.h"
#include "theia/sfm/estimators/estimate_radial_distortion_homography.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_relative_pose.h"

#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/estimators/camera_and_feature_correspondence_2d_3d.h"
#include "theia/sfm/similarity_transformation.h"
#include "theia/sfm/rigid_transformation.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/matching/feature_correspondence.h"

namespace theia {

std::tuple<bool, Eigen::Vector3d, RansacSummary> EstimateAbsolutePoseWithKnownOrientationWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const Eigen::Vector3d& camera_orientation,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences);

std::tuple<bool,CalibratedAbsolutePose, RansacSummary> EstimateCalibratedAbsolutePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences);

std::tuple<bool, Plane, RansacSummary> EstimateDominantPlaneFromPointsWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<Eigen::Vector3d>& points);

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateEssentialMatrixWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences);

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateFundamentalMatrixWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& correspondences);

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateHomographyWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& correspondences);

std::tuple<bool, RadialHomographyResult, RansacSummary> EstimateRadialHomographyMatrixWrapper(
    const RansacParameters& ransac_params, const RansacType& ransac_type,
    const std::vector<RadialDistortionFeatureCorrespondence>&
        normalized_correspondences);

std::tuple<bool, RelativePose, RansacSummary> EstimateRelativePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences);

std::tuple<bool, Eigen::Vector3d, RansacSummary>  EstimateRelativePoseWithKnownOrientationWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& rotated_correspondences);

std::tuple<bool, RigidTransformation, RansacSummary>  EstimateRigidTransformation2D3DWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<CameraAndFeatureCorrespondence2D3D>& correspondences);

std::tuple<bool, RigidTransformation, RansacSummary>  EstimateRigidTransformation2D3DNormalizedWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences);

std::tuple<bool, SimilarityTransformation, RansacSummary> EstimateSimilarityTransformation2D3DWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<CameraAndFeatureCorrespondence2D3D>& correspondences);


std::tuple<bool, Eigen::Vector4d, RansacSummary> EstimateTriangulationWrapper(const RansacParameters& ransac_params,
                           const std::vector<Camera>& cameras,
                           const std::vector<Eigen::Vector2d>& features);

std::tuple<bool, UncalibratedAbsolutePose, RansacSummary> EstimateUncalibratedAbsolutePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences);

std::tuple<bool, UncalibratedRelativePose, RansacSummary> EstimateUncalibratedRelativePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences);


} // namespace theia
