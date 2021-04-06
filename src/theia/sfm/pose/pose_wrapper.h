#include <tuple>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/types.h"

namespace theia {
std::tuple<std::vector<Eigen::Matrix<double,4,1>>, std::vector<Eigen::Vector3d>> DlsPnpWrapper(const std::vector<Eigen::Vector2d>& feature_positions,
            const std::vector<Eigen::Vector3d>& world_point);

std::tuple<bool, Eigen::Matrix3d> NormalizedEightPointFundamentalMatrixWrapper(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points);

std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d> DecomposeEssentialMatrixWrapper(const Eigen::Matrix3d& essential_matrix);

Eigen::Matrix3d EssentialMatrixFromTwoProjectionMatricesWrapper(const Matrix3x4d& pose1,    const Matrix3x4d& pose2);



std::tuple<int, Eigen::Matrix3d, Eigen::Vector3d> GetBestPoseFromEssentialMatrixWrapper(
    const Eigen::Matrix3d& essential_matrix,
    const std::vector<FeatureCorrespondence>& normalized_correspondences);


std::tuple<bool, std::vector<Eigen::Matrix<double, 3, 4> >, std::vector<std::vector<double> >> FivePointFocalLengthRadialDistortionWrapper(
    const std::vector<Eigen::Vector2d>& feature_positions,
    const std::vector<Eigen::Vector3d>& world_points,
    const int num_radial_distortion_params);

std::tuple<bool, std::vector<Eigen::Matrix3d>> FivePointRelativePoseWrapper(const std::vector<Eigen::Vector2d>& image1_points,
                           const std::vector<Eigen::Vector2d>& image2_points);

std::tuple<int, std::vector<Eigen::Matrix<double, 3, 4>>> FourPointPoseAndFocalLengthWrapper(const std::vector<Eigen::Vector2d>& feature_vectors,
                                       const std::vector<Eigen::Vector3d>& world_points);

std::tuple<bool, std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>, std::vector<double>, std::vector<double>> FourPointsPoseFocalLengthRadialDistortionWrapper(
    const std::vector<Eigen::Vector2d>& feature_vectors,
    const std::vector<Eigen::Vector3d>& world_points);


std::tuple<bool, Eigen::Matrix3d> FourPointHomographyWrapper(const std::vector<Eigen::Vector2d>& image_1_points,
                         const std::vector<Eigen::Vector2d>& image_2_points);




std::tuple<std::vector<Eigen::Matrix<double,4,1>>, std::vector<Eigen::Vector3d>> FourPointRelativePosePartialRotationWrapper(
    const Eigen::Vector3d& rotation_axis,
    const std::vector<Eigen::Vector3d> image_one_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_one_ray_origins_in,
    const std::vector<Eigen::Vector3d> image_two_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_two_ray_origins_in);

std::tuple<bool, double, double> FocalLengthsFromFundamentalMatrixWrapper(const Eigen::Matrix3d fmatrix);

std::tuple<bool, double> SharedFocalLengthsFromFundamentalMatrixWrapper(const Eigen::Matrix3d fmatrix);

std::tuple<Eigen::Matrix<double, 3, 4>, Eigen::Matrix<double, 3, 4>> ProjectionMatricesFromFundamentalMatrixWrapper(const Eigen::Matrix3d fmatrix);

Eigen::Matrix3d FundamentalMatrixFromProjectionMatricesWrapper(const Eigen::Matrix<double, 3, 4> pmatrix1,
                                             const Eigen::Matrix<double, 3, 4> pmatrix2);

Eigen::Matrix3d EssentialMatrixFromFundamentalMatrixWrapper(const Eigen::Matrix3d fmatrix,
                                          const double focal_length1,
                                          const double focal_length2);


Eigen::Matrix3d ComposeFundamentalMatrixWrapper(const double focal_length1,
                              const double focal_length2,
                              const Eigen::Matrix3d rotation,
                              const Eigen::Vector3d translation);

std::tuple<bool, std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>> PoseFromThreePointsWrapper(const std::vector<Eigen::Vector2d>& feature_points_in,
                           const std::vector<Eigen::Vector3d>& points_3d_in);

std::tuple<bool, Eigen::Vector3d> PositionFromTwoRaysWrapper(const Eigen::Vector2d& rotated_feature1,
                         const Eigen::Vector3d& point1,
                         const Eigen::Vector2d& rotated_feature2,
                         const Eigen::Vector3d& point2);

std::tuple<bool, Eigen::Vector3d> RelativePoseFromTwoPointsWithKnownRotationWrapper(
    const std::vector<Eigen::Vector2d> rotated_features1_in,
    const std::vector<Eigen::Vector2d> rotated_features2_in);

std::tuple<bool, std::vector<Eigen::Matrix3d>> SevenPointFundamentalMatrixWrapper(
    const std::vector<Eigen::Vector2d>& image1_points,
    const std::vector<Eigen::Vector2d>& image2_points);


std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>, std::vector<Eigen::Vector3d>, std::vector<double>> SimTransformPartialRotationWrapper(
    const Eigen::Vector3d& rotation_axis,
    const std::vector<Eigen::Vector3d> image_one_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_one_ray_origins_in,
    const std::vector<Eigen::Vector3d> image_two_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_two_ray_origins_in);

std::tuple<std::vector<Eigen::Matrix<double,4,1>>, std::vector<Eigen::Vector3d>> ThreePointRelativePosePartialRotationWrapper(
    const Eigen::Vector3d& rotation_axis,
    const std::vector<Eigen::Vector3d> image_1_rays_in,
    const std::vector<Eigen::Vector3d> image_2_rays_in);

std::tuple<int, std::vector<Eigen::Matrix<double,4,1>>, std::vector<Eigen::Vector3d>> TwoPointPosePartialRotationWrapper(const Eigen::Vector3d& axis,
                                const Eigen::Vector3d& model_point_1,
                                const Eigen::Vector3d& model_point_2,
                                const Eigen::Vector3d& image_ray_1,
                                const Eigen::Vector3d& image_ray_2);


}  // namespace theia
