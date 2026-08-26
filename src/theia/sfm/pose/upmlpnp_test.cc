// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#include "gtest/gtest.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "theia/math/util.h"
#include "theia/sfm/pose/upmlpnp.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/types.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(77);

TEST(UPMLPnP, ExactRecovery) {
  const std::vector<Vector3d> world_points = {
      Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0), Vector3d(-1.0, 1.0, 2.0),
      Vector3d(2.0, 1.0, 3.0), Vector3d(-1.0, -3.0, 2.0)};
  const Matrix3d expected_rotation =
      Eigen::AngleAxisd(DegToRad(7.0), Vector3d(0.1, 1.0, 0.2).normalized())
          .toRotationMatrix();
  const Vector3d expected_translation(0.4, 0.2, 1.1);
  const Vector3d gravity_world(0.0, -1.0, 0.0);
  const Vector3d gravity_camera = expected_rotation * gravity_world;

  Matrix3x4d expected_transform;
  expected_transform << expected_rotation, expected_translation;

  std::vector<Vector2d> feature_points;
  feature_points.reserve(world_points.size());
  for (const Vector3d& point : world_points) {
    feature_points.push_back(
        (expected_transform * point.homogeneous()).eval().hnormalized());
  }

  Matrix3d rotation;
  Vector3d translation;
  ASSERT_TRUE(UPMLPnP(feature_points, {}, world_points, gravity_camera,
                      gravity_world, Eigen::Matrix2d::Identity() * 1e-4,
                      &rotation, &translation));

  Matrix3x4d soln_transform;
  soln_transform << rotation, translation;
  for (size_t i = 0; i < world_points.size(); ++i) {
    const Vector2d reprojected =
        (soln_transform * world_points[i].homogeneous()).eval().hnormalized();
    EXPECT_LT((feature_points[i] - reprojected).squaredNorm(), 1e-4);
  }
}

TEST(UPMLPnP, CovarianceFinite) {
  const std::vector<Vector3d> world_points = {
      Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0), Vector3d(-1.0, 1.0, 2.0),
      Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d expected_rotation =
      Eigen::AngleAxisd(DegToRad(5.0), Vector3d(0.0, 1.0, 0.0)).toRotationMatrix();
  const Vector3d expected_translation(0.2, 0.1, 0.8);
  const Vector3d gravity_world(0.0, -1.0, 0.0);
  const Vector3d gravity_camera = expected_rotation * gravity_world;

  Matrix3x4d expected_transform;
  expected_transform << expected_rotation, expected_translation;

  std::vector<Vector2d> feature_points;
  for (const Vector3d& point : world_points) {
    feature_points.push_back(
        (expected_transform * point.homogeneous()).eval().hnormalized());
  }

  Matrix3d rotation;
  Vector3d translation;
  Eigen::Matrix<double, 6, 6> covariance;
  ASSERT_TRUE(UPMLPnPWithCovariance(
      feature_points, {}, world_points, gravity_camera, gravity_world,
      Eigen::Matrix2d::Identity() * 1e-4, &rotation, &translation,
      &covariance));
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_GT(covariance.trace(), 0.0);
}

TEST(UPMLPnP, RejectsTooFewPoints) {
  const std::vector<Vector3d> world_points = {Vector3d(0.0, 1.0, 2.0),
                                              Vector3d(1.0, 0.0, 3.0)};
  const std::vector<Vector2d> feature_points = {Vector2d(0.1, 0.2),
                                                Vector2d(-0.2, 0.1)};
  Matrix3d rotation;
  Vector3d translation;
  EXPECT_FALSE(UPMLPnP(feature_points, {}, world_points, Vector3d::UnitY(),
                       Vector3d::UnitY(), Eigen::Matrix2d::Identity(), &rotation,
                       &translation));
}

}  // namespace theia
