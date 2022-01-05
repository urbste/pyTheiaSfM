// header files
#include "theia/sfm/pose/pose_wrapper.h"

#include "theia/sfm/pose/eight_point_fundamental_matrix.h"
#include "theia/sfm/pose/five_point_focal_length_radial_distortion.h"
#include "theia/sfm/pose/five_point_relative_pose.h"
#include "theia/sfm/pose/four_point_focal_length.h"
#include "theia/sfm/pose/four_point_focal_length_radial_distortion.h"
#include "theia/sfm/pose/four_point_homography.h"
#include "theia/sfm/pose/four_point_relative_pose_partial_rotation.h"
#include "theia/sfm/pose/perspective_three_point.h"
#include "theia/sfm/pose/three_point_relative_pose_partial_rotation.h"
#include "theia/sfm/pose/two_point_pose_partial_rotation.h"

#include "theia/sfm/pose/dls_pnp.h"
#include "theia/sfm/pose/sqpnp.h"
#include "theia/sfm/pose/essential_matrix_utils.h"
#include "theia/sfm/pose/fundamental_matrix_util.h"
#include "theia/sfm/pose/position_from_two_rays.h"
#include "theia/sfm/pose/relative_pose_from_two_points_with_known_rotation.h"
#include "theia/sfm/pose/seven_point_fundamental_matrix.h"
#include "theia/sfm/pose/sim_transform_partial_rotation.h"
#include "theia/sfm/types.h"

using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace theia {

std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>, std::vector<Vector3d>>
DlsPnpWrapper(const std::vector<Vector2d>& feature_positions,
              const std::vector<Vector3d>& world_point) {
  std::vector<Quaterniond> solution_rotation_q;
  std::vector<Vector3d> solution_translation;
  DlsPnp(feature_positions,
         world_point,
         &solution_rotation_q,
         &solution_translation);

  std::vector<Eigen::Matrix<double, 4, 1>> solution_rotation;
  for (int i = 0; i < solution_rotation_q.size(); ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = solution_rotation_q[i].w();
    tmp(1, 0) = solution_rotation_q[i].x();
    tmp(2, 0) = solution_rotation_q[i].y();
    tmp(3, 0) = solution_rotation_q[i].z();
    solution_rotation.push_back(tmp);
  }
  return std::make_tuple(solution_rotation, solution_translation);
}

std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>,
           std::vector<Eigen::Vector3d>>
SQPnPWrapper(const std::vector<Eigen::Vector2d>& feature_positions,
              const std::vector<Eigen::Vector3d>& world_point) {
  std::vector<Quaterniond> solution_rotation_q;
  std::vector<Vector3d> solution_translation;
  SQPnP(feature_positions,
         world_point,
         &solution_rotation_q,
         &solution_translation);

  std::vector<Eigen::Matrix<double, 4, 1>> solution_rotation;
  for (int i = 0; i < solution_rotation_q.size(); ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = solution_rotation_q[i].w();
    tmp(1, 0) = solution_rotation_q[i].x();
    tmp(2, 0) = solution_rotation_q[i].y();
    tmp(3, 0) = solution_rotation_q[i].z();
    solution_rotation.push_back(tmp);
  }
  return std::make_tuple(solution_rotation, solution_translation);            
}

std::tuple<bool, Eigen::Matrix3d> NormalizedEightPointFundamentalMatrixWrapper(
    const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points) {
  Matrix3d fundamental_matrix;
  const bool success = NormalizedEightPointFundamentalMatrix(
      image_1_points, image_2_points, &fundamental_matrix);
  return std::make_tuple(success, fundamental_matrix);
}

std::tuple<Matrix3d, Matrix3d, Vector3d> DecomposeEssentialMatrixWrapper(
    const Eigen::Matrix3d& essential_matrix) {
  Matrix3d rotation1;
  Matrix3d rotation2;
  Vector3d translation;
  DecomposeEssentialMatrix(
      essential_matrix, &rotation1, &rotation2, &translation);
  return std::make_tuple(rotation1, rotation2, translation);
}

Eigen::Matrix3d EssentialMatrixFromTwoProjectionMatricesWrapper(
    const Matrix3x4d& pose1, const Matrix3x4d& pose2) {
  Eigen::Matrix3d essential_matrix;
  EssentialMatrixFromTwoProjectionMatrices(pose1, pose2, &essential_matrix);
  return essential_matrix;
}

std::tuple<int, Matrix3d, Vector3d> GetBestPoseFromEssentialMatrixWrapper(
    const Eigen::Matrix3d& essential_matrix,
    const std::vector<FeatureCorrespondence>& normalized_correspondences) {
  Matrix3d rotation;
  Vector3d position;
  const int num_solutions = GetBestPoseFromEssentialMatrix(
      essential_matrix, normalized_correspondences, &rotation, &position);
  return std::make_tuple(num_solutions, rotation, position);
}

std::tuple<bool,
           std::vector<Matrix<double, 3, 4>>,
           std::vector<std::vector<double>>>
FivePointFocalLengthRadialDistortionWrapper(
    const std::vector<Vector2d>& feature_positions,
    const std::vector<Vector3d>& world_points,
    const int num_radial_distortion_params) {
  std::vector<Matrix<double, 3, 4>> projection_matrices;
  std::vector<std::vector<double>> radial_distortions;
  const bool success =
      FivePointFocalLengthRadialDistortion(feature_positions,
                                           world_points,
                                           num_radial_distortion_params,
                                           &projection_matrices,
                                           &radial_distortions);
  return std::make_tuple(success, projection_matrices, radial_distortions);
}

std::tuple<bool, std::vector<Matrix3d>> FivePointRelativePoseWrapper(
    const std::vector<Vector2d>& image1_points,
    const std::vector<Vector2d>& image2_points) {
  std::vector<Matrix3d> essential_matrices;
  const bool success =
      FivePointRelativePose(image1_points, image2_points, &essential_matrices);
  return std::make_tuple(success, essential_matrices);
}

std::tuple<int, std::vector<Matrix<double, 3, 4>>>
FourPointPoseAndFocalLengthWrapper(const std::vector<Vector2d>& feature_vectors,
                                   const std::vector<Vector3d>& world_points) {
  std::vector<Matrix<double, 3, 4>> projection_matrices;
  int num_solutions = FourPointPoseAndFocalLength(
      feature_vectors, world_points, &projection_matrices);
  return std::make_tuple(num_solutions, projection_matrices);
}

std::tuple<bool,
           std::vector<Matrix3d>,
           std::vector<Vector3d>,
           std::vector<double>,
           std::vector<double>>
FourPointsPoseFocalLengthRadialDistortionWrapper(
    const std::vector<Vector2d>& feature_vectors,
    const std::vector<Vector3d>& world_points) {
  std::vector<Matrix3d> rotations;
  std::vector<Vector3d> translations;
  std::vector<double> radial_distortions;
  std::vector<double> focal_lengths;
  //    const bool success =
  //    FourPointsPoseFocalLengthRadialDistortion(feature_vectors, world_points,
  //    &rotations, &translations, &radial_distortions, &focal_lengths); return
  //    std::make_tuple(success, rotations, translations, radial_distortions,
  //    focal_lengths);
}

std::tuple<bool, Matrix3d> FourPointHomographyWrapper(
    const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points) {
  Matrix3d homography;
  const bool success =
      FourPointHomography(image_1_points, image_2_points, &homography);
  return std::make_tuple(success, homography);
}

std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>,
           std::vector<Eigen::Vector3d>>
FourPointRelativePosePartialRotationWrapper(
    const Vector3d& rotation_axis,
    const std::vector<Vector3d> image_one_ray_directions_in,
    const std::vector<Vector3d> image_one_ray_origins_in,
    const std::vector<Vector3d> image_two_ray_directions_in,
    const std::vector<Vector3d> image_two_ray_origins_in) {
  std::vector<Quaterniond> soln_rotations_q;
  std::vector<Vector3d> soln_translations;
  const Vector3d image_one_ray_directions[4] = {image_one_ray_directions_in[0],
                                                image_one_ray_directions_in[1],
                                                image_one_ray_directions_in[2],
                                                image_one_ray_directions_in[3]};
  const Vector3d image_one_ray_origins[4] = {image_one_ray_origins_in[0],
                                             image_one_ray_origins_in[1],
                                             image_one_ray_origins_in[2],
                                             image_one_ray_origins_in[3]};
  const Vector3d image_two_ray_directions[4] = {image_two_ray_directions_in[0],
                                                image_two_ray_directions_in[1],
                                                image_two_ray_directions_in[2],
                                                image_two_ray_directions_in[3]};
  const Vector3d image_two_ray_origins[4] = {image_two_ray_origins_in[0],
                                             image_two_ray_origins_in[1],
                                             image_two_ray_origins_in[2],
                                             image_two_ray_origins_in[3]};
  FourPointRelativePosePartialRotation(rotation_axis,
                                       image_one_ray_directions,
                                       image_one_ray_origins,
                                       image_two_ray_directions,
                                       image_two_ray_origins,
                                       &soln_rotations_q,
                                       &soln_translations);

  std::vector<Eigen::Matrix<double, 4, 1>> soln_rotations;
  for (int i = 0; i < soln_rotations_q.size(); ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = soln_rotations_q[i].w();
    tmp(1, 0) = soln_rotations_q[i].x();
    tmp(2, 0) = soln_rotations_q[i].y();
    tmp(3, 0) = soln_rotations_q[i].z();
    soln_rotations.push_back(tmp);
  }

  return std::make_tuple(soln_rotations, soln_translations);
}

Eigen::Matrix3d FundamentalMatrixFromProjectionMatricesWrapper(
    const Eigen::Matrix<double, 3, 4> pmatrix1,
    const Eigen::Matrix<double, 3, 4> pmatrix2) {
  Eigen::Matrix3d fmatrix;
  FundamentalMatrixFromProjectionMatrices(
      pmatrix1.data(), pmatrix2.data(), fmatrix.data());
  return fmatrix;
}
Eigen::Matrix3d EssentialMatrixFromFundamentalMatrixWrapper(
    const Eigen::Matrix3d fmatrix,
    const double focal_length1,
    const double focal_length2) {
  Eigen::Matrix3d ematrix;
  EssentialMatrixFromFundamentalMatrix(
      fmatrix.data(), focal_length1, focal_length2, ematrix.data());
  return ematrix;
}
Eigen::Matrix3d ComposeFundamentalMatrixWrapper(
    const double focal_length1,
    const double focal_length2,
    const Eigen::Matrix3d rotation,
    const Eigen::Vector3d translation) {
  Eigen::Matrix3d fmatrix;
  ComposeFundamentalMatrix(focal_length1,
                           focal_length2,
                           rotation.data(),
                           translation.data(),
                           fmatrix.data());
  return fmatrix;
}

std::tuple<bool, double, double> FocalLengthsFromFundamentalMatrixWrapper(
    const Eigen::Matrix3d fmatrix) {
  double focal_length1;
  double focal_length2;
  const bool success = FocalLengthsFromFundamentalMatrix(
      fmatrix.data(), &focal_length1, &focal_length2);
  return std::make_tuple(success, focal_length1, focal_length2);
}

std::tuple<bool, double> SharedFocalLengthsFromFundamentalMatrixWrapper(
    const Eigen::Matrix3d fmatrix) {
  double focal_length;
  const bool success =
      SharedFocalLengthsFromFundamentalMatrix(fmatrix.data(), &focal_length);
  return std::make_tuple(success, focal_length);
}

std::tuple<Eigen::Matrix<double, 3, 4>, Eigen::Matrix<double, 3, 4>>
ProjectionMatricesFromFundamentalMatrixWrapper(const Eigen::Matrix3d fmatrix) {
  Eigen::Matrix<double, 3, 4> pmatrix1;
  Eigen::Matrix<double, 3, 4> pmatrix2;
  ProjectionMatricesFromFundamentalMatrix(
      fmatrix.data(), pmatrix1.data(), pmatrix2.data());
  return std::make_tuple(pmatrix1, pmatrix2);
}

std::tuple<bool, std::vector<Matrix3d>, std::vector<Vector3d>>
PoseFromThreePointsWrapper(const std::vector<Vector2d>& feature_points_in,
                           const std::vector<Vector3d>& points_3d_in) {
  std::vector<Matrix3d> solution_rotations;
  std::vector<Vector3d> solution_translations;
  const bool success = PoseFromThreePoints(
      feature_points_in, points_3d_in, &solution_rotations, &solution_translations);
  return std::make_tuple(success, solution_rotations, solution_translations);
}

std::tuple<bool, Eigen::Vector3d> PositionFromTwoRaysWrapper(
    const Eigen::Vector2d& rotated_feature1,
    const Eigen::Vector3d& point1,
    const Eigen::Vector2d& rotated_feature2,
    const Eigen::Vector3d& point2) {
  Eigen::Vector3d position;
  const bool success = PositionFromTwoRays(
      rotated_feature1, point1, rotated_feature2, point2, &position);
  return std::make_tuple(success, position);
}

std::tuple<bool, Eigen::Vector3d>
RelativePoseFromTwoPointsWithKnownRotationWrapper(
    const std::vector<Eigen::Vector2d> rotated_features1_in,
    const std::vector<Eigen::Vector2d> rotated_features2_in) {
  Eigen::Vector3d relative_position2;
  Eigen::Vector2d rotated_features1[2] = {rotated_features1_in[0],
                                          rotated_features1_in[1]};
  Eigen::Vector2d rotated_features2[2] = {rotated_features2_in[0],
                                          rotated_features2_in[1]};
  const bool success = RelativePoseFromTwoPointsWithKnownRotation(
      rotated_features1, rotated_features2, &relative_position2);
  return std::make_tuple(success, relative_position2);
}

std::tuple<bool, std::vector<Eigen::Matrix3d>>
SevenPointFundamentalMatrixWrapper(
    const std::vector<Eigen::Vector2d>& image1_points,
    const std::vector<Eigen::Vector2d>& image2_points) {
  std::vector<Eigen::Matrix3d> fundamental_matrices;
  const bool success = SevenPointFundamentalMatrix(
      image1_points, image2_points, &fundamental_matrices);
  return std::make_tuple(success, fundamental_matrices);
}

std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>,
           std::vector<Eigen::Vector3d>,
           std::vector<double>>
SimTransformPartialRotationWrapper(
    const Eigen::Vector3d& rotation_axis,
    const std::vector<Eigen::Vector3d> image_one_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_one_ray_origins_in,
    const std::vector<Eigen::Vector3d> image_two_ray_directions_in,
    const std::vector<Eigen::Vector3d> image_two_ray_origins_in) {
  Eigen::Vector3d image_one_ray_directions[5] = {
      image_one_ray_directions_in[0],
      image_one_ray_directions_in[1],
      image_one_ray_directions_in[2],
      image_one_ray_directions_in[3],
      image_one_ray_directions_in[4]};
  Eigen::Vector3d image_one_ray_origins[5] = {image_one_ray_origins_in[0],
                                              image_one_ray_origins_in[1],
                                              image_one_ray_origins_in[2],
                                              image_one_ray_origins_in[3],
                                              image_one_ray_origins_in[4]};
  Eigen::Vector3d image_two_ray_directions[5] = {
      image_two_ray_directions_in[0],
      image_two_ray_directions_in[1],
      image_two_ray_directions_in[2],
      image_two_ray_directions_in[3],
      image_two_ray_directions_in[4]};
  Eigen::Vector3d image_two_ray_origins[5] = {image_two_ray_origins_in[0],
                                              image_two_ray_origins_in[1],
                                              image_two_ray_origins_in[2],
                                              image_two_ray_origins_in[3],
                                              image_two_ray_origins_in[4]};
  std::vector<Eigen::Quaterniond> soln_rotations_q;
  std::vector<Eigen::Vector3d> soln_translations;
  std::vector<double> soln_scales;
  SimTransformPartialRotation(rotation_axis,
                              image_one_ray_directions,
                              image_one_ray_origins,
                              image_two_ray_directions,
                              image_two_ray_origins,
                              &soln_rotations_q,
                              &soln_translations,
                              &soln_scales);

  std::vector<Eigen::Matrix<double, 4, 1>> soln_rotations;
  for (int i = 0; i < soln_rotations_q.size(); ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = soln_rotations_q[i].w();
    tmp(1, 0) = soln_rotations_q[i].x();
    tmp(2, 0) = soln_rotations_q[i].y();
    tmp(3, 0) = soln_rotations_q[i].z();
    soln_rotations.push_back(tmp);
  }

  return std::make_tuple(soln_rotations, soln_translations, soln_scales);
}

std::tuple<std::vector<Eigen::Matrix<double, 4, 1>>, std::vector<Vector3d>>
ThreePointRelativePosePartialRotationWrapper(
    const Vector3d& rotation_axis,
    const std::vector<Vector3d> image_1_rays_in,
    const std::vector<Vector3d> image_2_rays_in) {

  std::vector<Eigen::Quaterniond> soln_rotations_q;
  std::vector<Vector3d> soln_translations;
  Vector3d image_1_rays[3] = {
      image_1_rays_in[0], image_1_rays_in[1], image_1_rays_in[2]};
  Vector3d image_2_rays[3] = {
      image_2_rays_in[0], image_2_rays_in[1], image_2_rays_in[2]};
  ThreePointRelativePosePartialRotation(rotation_axis,
                                        image_1_rays,
                                        image_2_rays,
                                        &soln_rotations_q,
                                        &soln_translations);

  std::vector<Eigen::Matrix<double, 4, 1>> soln_rotations;
  for (int i = 0; i < soln_rotations_q.size(); ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = soln_rotations_q[i].w();
    tmp(1, 0) = soln_rotations_q[i].x();
    tmp(2, 0) = soln_rotations_q[i].y();
    tmp(3, 0) = soln_rotations_q[i].z();
    soln_rotations.push_back(tmp);
  }

  return std::make_tuple(soln_rotations, soln_translations);
}

std::tuple<int, std::vector<Eigen::Matrix<double, 4, 1>>, std::vector<Vector3d>>
TwoPointPosePartialRotationWrapper(const Vector3d& axis,
                                   const Vector3d& model_point_1,
                                   const Vector3d& model_point_2,
                                   const Vector3d& image_ray_1,
                                   const Vector3d& image_ray_2) {
  Quaterniond soln_rotations_q[2];
  Vector3d soln_translations_in[2];
  const int num_solutions = TwoPointPosePartialRotation(axis,
                                                        model_point_1,
                                                        model_point_2,
                                                        image_ray_1,
                                                        image_ray_2,
                                                        soln_rotations_q,
                                                        soln_translations_in);

  std::vector<Eigen::Matrix<double, 4, 1>> soln_rotations;
  std::vector<Vector3d> soln_translations_out;
  soln_translations_out.push_back(soln_translations_in[0]);
  soln_translations_out.push_back(soln_translations_in[1]);

  for (int i = 0; i < 2; ++i) {
    Eigen::Matrix<double, 4, 1> tmp;
    tmp(0, 0) = soln_rotations_q[i].w();
    tmp(1, 0) = soln_rotations_q[i].x();
    tmp(2, 0) = soln_rotations_q[i].y();
    tmp(3, 0) = soln_rotations_q[i].z();
    soln_rotations.push_back(tmp);
  }

  return std::make_tuple(num_solutions, soln_rotations, soln_translations_out);
}

}  // namespace theia
