#include "theia/sfm/estimators/estimators_wrapper.h"

#include "theia/sfm/estimators/estimate_absolute_pose_with_known_orientation.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_dominant_plane_from_points.h"
#include "theia/sfm/estimators/estimate_essential_matrix.h"
#include "theia/sfm/estimators/estimate_fundamental_matrix.h"
#include "theia/sfm/estimators/estimate_homography.h"
#include "theia/sfm/estimators/estimate_radial_distortion_homography.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_relative_pose_with_known_orientation.h"
#include "theia/sfm/estimators/estimate_rigid_transformation_2d_3d.h"
#include "theia/sfm/estimators/estimate_similarity_transformation_2d_3d.h"
#include "theia/sfm/estimators/estimate_triangulation.h"
#include "theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_relative_pose.h"
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/estimators/camera_and_feature_correspondence_2d_3d.h"



namespace theia {


std::tuple<bool, Eigen::Vector3d, RansacSummary> EstimateAbsolutePoseWithKnownOrientationWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const Eigen::Vector3d& camera_orientation,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences){
    Eigen::Vector3d camera_position;
    RansacSummary ransac_summary;
    const bool success = EstimateAbsolutePoseWithKnownOrientation(ransac_params, ransac_type, camera_orientation, normalized_correspondences, &camera_position, &ransac_summary);
    return std::make_tuple(success, camera_position, ransac_summary);
}

std::tuple<bool,CalibratedAbsolutePose, RansacSummary> EstimateCalibratedAbsolutePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences){
    CalibratedAbsolutePose absolute_pose;
    RansacSummary ransac_summary;
    const bool success = EstimateCalibratedAbsolutePose(ransac_params, ransac_type, normalized_correspondences, &absolute_pose, &ransac_summary);
    return std::make_tuple(success, absolute_pose, ransac_summary);

}

std::tuple<bool, Plane, RansacSummary> EstimateDominantPlaneFromPointsWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<Eigen::Vector3d>& points){
    Plane plane;
    RansacSummary ransac_summary;
    const bool success = EstimateDominantPlaneFromPoints(ransac_params, ransac_type, points, &plane, &ransac_summary);
    return std::make_tuple(success, plane, ransac_summary);

}

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateEssentialMatrixWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences){
    Eigen::Matrix3d essential_matrix;
    RansacSummary ransac_summary;
    const bool success = EstimateEssentialMatrix(ransac_params, ransac_type, normalized_correspondences, &essential_matrix, &ransac_summary);
    return std::make_tuple(success, essential_matrix, ransac_summary);

}

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateFundamentalMatrixWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& correspondences){
    Eigen::Matrix3d fundamental_matrix;
    RansacSummary ransac_summary;
    const bool success = EstimateFundamentalMatrix(ransac_params, ransac_type, correspondences, &fundamental_matrix, &ransac_summary);
    return std::make_tuple(success, fundamental_matrix, ransac_summary);

}

std::tuple<bool, Eigen::Matrix3d, RansacSummary> EstimateHomographyWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& correspondences){
    Eigen::Matrix3d homography;
    RansacSummary ransac_summary;
    const bool success = EstimateHomography(ransac_params, ransac_type, correspondences, &homography, &ransac_summary);
    return std::make_tuple(success, homography, ransac_summary);

}

std::tuple<bool, RadialHomographyResult, RansacSummary> EstimateRadialHomographyMatrixWrapper(
    const RansacParameters& ransac_params, const RansacType& ransac_type,
    const std::vector<RadialDistortionFeatureCorrespondence>&
        normalized_correspondences){
    RadialHomographyResult result;
    RansacSummary ransac_summary;
    const bool success = EstimateRadialHomographyMatrix(ransac_params, ransac_type, normalized_correspondences, &result, &ransac_summary);
    return std::make_tuple(success, result, ransac_summary);

}

std::tuple<bool, RelativePose, RansacSummary> EstimateRelativePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences){
    RelativePose relative_pose;
    RansacSummary ransac_summary;
    const bool success = EstimateRelativePose(ransac_params, ransac_type, normalized_correspondences, &relative_pose, &ransac_summary);
    return std::make_tuple(success, relative_pose, ransac_summary);

}

std::tuple<bool, Eigen::Vector3d, RansacSummary>  EstimateRelativePoseWithKnownOrientationWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& rotated_correspondences){
    Eigen::Vector3d relative_camera2_position;
    RansacSummary ransac_summary;
    const bool success = EstimateRelativePoseWithKnownOrientation(ransac_params, ransac_type, rotated_correspondences, &relative_camera2_position, &ransac_summary);
    return std::make_tuple(success, relative_camera2_position, ransac_summary);

}

std::tuple<bool, RigidTransformation, RansacSummary>  EstimateRigidTransformation2D3DWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<CameraAndFeatureCorrespondence2D3D>& correspondences){
    RigidTransformation estimated_transformation;
    RansacSummary ransac_summary;
    const bool success = EstimateRigidTransformation2D3D(ransac_params, ransac_type, correspondences, &estimated_transformation, &ransac_summary);
    return std::make_tuple(success, estimated_transformation, ransac_summary);

}

std::tuple<bool, RigidTransformation, RansacSummary>  EstimateRigidTransformation2D3DNormalizedWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences){
    RigidTransformation estimated_transformation;
    RansacSummary ransac_summary;
    const bool success = EstimateRigidTransformation2D3D(ransac_params, ransac_type, normalized_correspondences, &estimated_transformation, &ransac_summary);
    return std::make_tuple(success, estimated_transformation, ransac_summary);

}

std::tuple<bool, SimilarityTransformation, RansacSummary> EstimateSimilarityTransformation2D3DWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<CameraAndFeatureCorrespondence2D3D>& correspondences){
    SimilarityTransformation similarity_transformation;
    RansacSummary ransac_summary;
    const bool success = EstimateSimilarityTransformation2D3D(ransac_params, ransac_type, correspondences, &similarity_transformation, &ransac_summary);
    return std::make_tuple(success, similarity_transformation, ransac_summary);

}


std::tuple<bool, Eigen::Vector4d, RansacSummary> EstimateTriangulationWrapper(const RansacParameters& ransac_params,
                           const std::vector<Camera>& cameras,
                           const std::vector<Eigen::Vector2d>& features){
    Eigen::Vector4d triangulated_point;
    RansacSummary summary;
    const bool success = EstimateTriangulation(ransac_params, cameras, features, &triangulated_point, &summary);
    return std::make_tuple(success, triangulated_point, summary);

}

std::tuple<bool, UncalibratedAbsolutePose, RansacSummary> EstimateUncalibratedAbsolutePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences){
    UncalibratedAbsolutePose absolute_pose;
    RansacSummary ransac_summary;
    const bool success = EstimateUncalibratedAbsolutePose(ransac_params, ransac_type, normalized_correspondences, &absolute_pose, &ransac_summary);
    return std::make_tuple(success, absolute_pose, ransac_summary);

}

std::tuple<bool, UncalibratedRelativePose, RansacSummary> EstimateUncalibratedRelativePoseWrapper(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences){
    UncalibratedRelativePose relative_pose;
    RansacSummary ransac_summary;
    const bool success = EstimateUncalibratedRelativePose(ransac_params, ransac_type, centered_correspondences, &relative_pose, &ransac_summary);
    return std::make_tuple(success, relative_pose, ransac_summary);

}



} // namespace theia
