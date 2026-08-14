// Copyright (C) 2026, The pyTheiaSfM authors. All rights reserved.

#include "theia/sfm/pose/fast_iterative_five_point.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <vector>

#include "theia/math/util.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/pose/five_point_relative_pose_sturm.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/pose/util.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(73);

void GenerateSyntheticCorrespondences(
    const std::vector<Vector3d>& points_3d,
    const Matrix3d& expected_rotation,
    const Vector3d& expected_translation,
    const double projection_noise_std_dev,
    std::vector<Vector2d>* view_one_points,
    std::vector<Vector2d>* view_two_points,
    std::vector<Vector3d>* x1h,
    std::vector<Vector3d>* x2h) {
  view_one_points->clear();
  view_two_points->clear();
  x1h->clear();
  x2h->clear();

  for (size_t i = 0; i < points_3d.size(); ++i) {
    const Vector3d proj_3d =
        expected_rotation * points_3d[i] + expected_translation;
    view_one_points->emplace_back(points_3d[i].hnormalized());
    view_two_points->emplace_back(proj_3d.hnormalized());
  }

  if (projection_noise_std_dev > 0.0) {
    for (size_t i = 0; i < view_one_points->size(); ++i) {
      AddNoiseToProjection(projection_noise_std_dev, &rng, &(*view_one_points)[i]);
      AddNoiseToProjection(projection_noise_std_dev, &rng, &(*view_two_points)[i]);
    }
  }

  for (size_t i = 0; i < view_one_points->size(); ++i) {
    x1h->emplace_back((*view_one_points)[i].homogeneous());
    x2h->emplace_back((*view_two_points)[i].homogeneous());
  }
}

TEST(FastIterativeFivePoint, ParameterConversions) {
  const Matrix3d R =
      Quaterniond(AngleAxisd(DegToRad(5.0), Vector3d(0.2, 0.4, 0.9).normalized()))
          .toRotationMatrix();
  const Vector3d t(0.05, -0.02, -1.0);

  const Eigen::Matrix<double, 5, 1> w =
      FastIterativeRotationAndTranslationToPoseParameters(R, t);

  Matrix3d R_recovered;
  Vector3d t_recovered;
  FastIterativePoseParametersToRotationAndTranslation(w, &R_recovered, &t_recovered);

  EXPECT_LT((R - R_recovered).norm(), 1e-10);
  EXPECT_LT((t.normalized() - t_recovered).norm(), 1e-10);
}

TEST(FastIterativeFivePoint, BasicMinimalForwardMotion) {
  const std::vector<Vector3d> points_3d = {
      Vector3d(-0.8, 1.2, 4.0),
      Vector3d(1.1, -0.9, 3.5),
      Vector3d(0.5, 0.8, 2.8),
      Vector3d(-1.2, -0.7, 3.2),
      Vector3d(1.4, 1.0, 4.5)};

  // Small rotation (forward motion trajectory)
  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(3.0), Vector3d(0.1, -0.2, 0.05).normalized()))
          .toRotationMatrix();
  const Vector3d soln_translation(0.02, -0.01, -1.0);

  std::vector<Vector2d> view_one_points, view_two_points;
  std::vector<Vector3d> x1h, x2h;
  GenerateSyntheticCorrespondences(
      points_3d, soln_rotation, soln_translation, 0.0,
      &view_one_points, &view_two_points, &x1h, &x2h);

  const Matrix3d gt_ematrix =
      CrossProductMatrix(soln_translation.normalized()) * soln_rotation;

  FastIterativeFivePointOptions options;
  std::vector<Matrix3d> soln_ematrices;
  std::vector<RelativePose> soln_poses;

  const int num_solutions = FastIterativeFivePoint(
      x1h, x2h, options, &soln_ematrices, &soln_poses);

  ASSERT_EQ(num_solutions, 1);
  ASSERT_EQ(soln_ematrices.size(), 1);
  ASSERT_EQ(soln_poses.size(), 1);

  // Check epipolar error on all 5 points
  for (int i = 0; i < 5; ++i) {
    EXPECT_LT(
        SquaredSampsonDistance(soln_ematrices[0], view_one_points[i], view_two_points[i]),
        1e-8);
  }

  // Check essential matrix up to scale
  EXPECT_TRUE(test::ArraysEqualUpToScale(
      9, soln_ematrices[0].data(), gt_ematrix.data(), 1e-4));

  // Check pose rotation and position direction
  EXPECT_LT((soln_poses[0].rotation - soln_rotation).norm(), 1e-3);
  const Vector3d gt_position = -soln_rotation.transpose() * soln_translation.normalized();
  EXPECT_LT((soln_poses[0].position.normalized() - gt_position.normalized()).norm(), 1e-3);
}

TEST(FastIterativeFivePoint, NoiseTestMinimal) {
  const std::vector<Vector3d> points_3d = {
      Vector3d(-1.0, 1.0, 3.0),
      Vector3d(1.2, -0.8, 2.5),
      Vector3d(0.3, 0.9, 3.8),
      Vector3d(-0.9, -1.1, 4.0),
      Vector3d(1.5, 0.7, 3.2)};

  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(2.0), Vector3d(0.0, 1.0, 0.0)))
          .toRotationMatrix();
  const Vector3d soln_translation(0.01, 0.0, -1.0);

  const double kNoiseStdDev = 1.0 / 1024.0;
  std::vector<Vector2d> view_one_points, view_two_points;
  std::vector<Vector3d> x1h, x2h;
  GenerateSyntheticCorrespondences(
      points_3d, soln_rotation, soln_translation, kNoiseStdDev,
      &view_one_points, &view_two_points, &x1h, &x2h);

  FastIterativeFivePointOptions options;
  std::vector<Matrix3d> soln_ematrices;

  const int num_solutions = FastIterativeFivePoint(
      x1h, x2h, options, &soln_ematrices);

  EXPECT_EQ(num_solutions, 1);
  const Matrix3d gt_ematrix =
      CrossProductMatrix(soln_translation.normalized()) * soln_rotation;
  EXPECT_TRUE(test::ArraysEqualUpToScale(
      9, soln_ematrices[0].data(), gt_ematrix.data(), 0.05));
}

TEST(FastIterativeFivePoint, NPointOverdetermined) {
  // Test overdetermined solve (N = 30 correspondences)
  std::vector<Vector3d> points_3d;
  for (int i = 0; i < 30; ++i) {
    points_3d.emplace_back(
        rng.RandDouble(-1.5, 1.5),
        rng.RandDouble(-1.5, 1.5),
        rng.RandDouble(2.0, 6.0));
  }

  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(4.0), Vector3d(0.2, -0.1, 0.1).normalized()))
          .toRotationMatrix();
  const Vector3d soln_translation(0.05, -0.03, -1.0);

  std::vector<Vector2d> view_one_points, view_two_points;
  std::vector<Vector3d> x1h, x2h;
  GenerateSyntheticCorrespondences(
      points_3d, soln_rotation, soln_translation, 0.0,
      &view_one_points, &view_two_points, &x1h, &x2h);

  FastIterativeFivePointOptions options;
  std::vector<Matrix3d> soln_ematrices;
  std::vector<RelativePose> soln_poses;

  const int num_solutions = FastIterativeFivePoint(
      x1h, x2h, options, &soln_ematrices, &soln_poses);

  ASSERT_EQ(num_solutions, 1);
  EXPECT_LT((soln_poses[0].rotation - soln_rotation).norm(), 1e-4);
  const Vector3d gt_position = -soln_rotation.transpose() * soln_translation.normalized();
  EXPECT_LT((soln_poses[0].position.normalized() - gt_position.normalized()).norm(), 1e-4);
}

TEST(FastIterativeFivePoint, EquivalentToSturmOnForwardMotion) {
  const std::vector<Vector3d> points_3d = {
      Vector3d(-1.2, 1.4, 3.5),
      Vector3d(0.8, -1.0, 2.9),
      Vector3d(0.2, 0.5, 4.1),
      Vector3d(-0.7, -0.8, 3.0),
      Vector3d(1.3, 0.9, 3.8)};

  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(2.5), Vector3d(0.3, 0.1, 0.0).normalized()))
          .toRotationMatrix();
  const Vector3d soln_translation(0.02, 0.01, -1.0);

  std::vector<Vector2d> view_one_points, view_two_points;
  std::vector<Vector3d> x1h, x2h;
  GenerateSyntheticCorrespondences(
      points_3d, soln_rotation, soln_translation, 0.0,
      &view_one_points, &view_two_points, &x1h, &x2h);

  std::vector<Matrix3d> sturm_ematrices;
  ASSERT_GT(FivePointRelativePoseSturm(x1h, x2h, &sturm_ematrices), 0);

  FastIterativeFivePointOptions options;
  std::vector<Matrix3d> iterative_ematrices;
  ASSERT_EQ(FastIterativeFivePoint(x1h, x2h, options, &iterative_ematrices), 1);

  bool matched_sturm = false;
  for (const Matrix3d& sturm_E : sturm_ematrices) {
    if (test::ArraysEqualUpToScale(9, iterative_ematrices[0].data(), sturm_E.data(), 1e-3)) {
      matched_sturm = true;
      break;
    }
  }
  EXPECT_TRUE(matched_sturm);
}

}  // namespace
}  // namespace theia
