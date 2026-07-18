// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// This file is adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// test coverage for solvers/relpose_monodepth_3pt*, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08, and is distributed under the
// BSD-3-Clause license. See docs/licenses/POSELIB_LICENSE.txt for the full
// PoseLib license text.
//
// Direct-solver (no RANSAC) round-trip tests on noise-free minimal samples,
// isolating port mistakes (e.g. quartic coefficient order) from estimator
// wiring, per dev/TWO_VIEW_SPEEDUP_PLAN.md Phase 4.1.

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <vector>

#include "theia/math/util.h"
#include "theia/sfm/pose/relative_pose_monodepth_3pt.h"
#include "theia/util/random.h"

namespace theia {
namespace {
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

RandomNumberGenerator rng(92);

// Finds the candidate pose whose rotation is closest to expected_rotation and
// checks it against the ground truth.
void ExpectContainsMatchingPose(
    const std::vector<MonoDepthRelativePose>& poses,
    const Matrix3d& expected_rotation,
    const Vector3d& expected_translation,
    const double expected_scale,
    const double rotation_tolerance,
    const double translation_tolerance,
    const double scale_tolerance) {
  ASSERT_GT(poses.size(), 0);
  double best_rotation_error = std::numeric_limits<double>::max();
  const MonoDepthRelativePose* best_pose = nullptr;
  for (const auto& pose : poses) {
    const double rotation_error =
        (pose.rotation - expected_rotation).norm();
    if (rotation_error < best_rotation_error) {
      best_rotation_error = rotation_error;
      best_pose = &pose;
    }
  }
  ASSERT_NE(best_pose, nullptr);
  EXPECT_LT(best_rotation_error, rotation_tolerance);
  EXPECT_LT((best_pose->translation - expected_translation).norm(),
            translation_tolerance);
  EXPECT_NEAR(best_pose->scale, expected_scale, scale_tolerance);
}

TEST(MonoDepthRelativePose3pt, BasicMinimal) {
  const std::vector<Vector3d> points_cam1 = {Vector3d(-0.3, 0.2, 3.0),
                                             Vector3d(0.4, -0.3, 2.0),
                                             Vector3d(0.1, 0.5, 2.5)};
  const Matrix3d rotation =
      Quaterniond(AngleAxisd(DegToRad(11.0), Vector3d(0.2, 1.0, 0.3).normalized()))
          .toRotationMatrix();
  const Vector3d translation(0.5, -0.2, 0.3);
  const double true_scale = 1.7;

  std::vector<Vector3d> x1h, x2h;
  std::vector<double> depth1, depth2;
  for (const Vector3d& point_cam1 : points_cam1) {
    const Vector3d point_cam2 = rotation * point_cam1 + translation;
    x1h.emplace_back(point_cam1.hnormalized().homogeneous());
    x2h.emplace_back(point_cam2.hnormalized().homogeneous());
    depth1.push_back(point_cam1.z());
    // Camera 2's raw monodepth is on its own scale, off by true_scale from
    // the metric scale shared with camera 1.
    depth2.push_back(point_cam2.z() / true_scale);
  }

  std::vector<MonoDepthRelativePose> poses;
  EXPECT_GT(MonoDepthRelativePose3pt(x1h, x2h, depth1, depth2, &poses), 0);
  ExpectContainsMatchingPose(
      poses, rotation, translation, true_scale, 1e-6, 1e-6, 1e-6);
}

TEST(MonoDepthRelativePose3ptSharedFocal, BasicMinimal) {
  // NOTE: depths are chosen to avoid the degenerate configuration
  // depth1[0] + depth1[1] - 2*depth1[2] == 0 (a coefficient denominator in
  // the shared-focal solver becomes zero exactly when the third depth is the
  // arithmetic mean of the other two).
  const std::vector<Vector3d> points_cam1 = {Vector3d(-0.3, 0.2, 3.1),
                                             Vector3d(0.4, -0.3, 2.0),
                                             Vector3d(0.1, 0.5, 2.7)};
  const Matrix3d rotation =
      Quaterniond(AngleAxisd(DegToRad(9.0), Vector3d(0.1, 0.8, 0.4).normalized()))
          .toRotationMatrix();
  const Vector3d translation(0.4, -0.15, 0.25);
  const double true_scale = 1.3;
  const double true_focal = 800.0;

  std::vector<Vector3d> x1h, x2h;
  std::vector<double> depth1, depth2;
  for (const Vector3d& point_cam1 : points_cam1) {
    const Vector3d point_cam2 = rotation * point_cam1 + translation;
    const Eigen::Vector2d pixel1 = true_focal * point_cam1.hnormalized();
    const Eigen::Vector2d pixel2 = true_focal * point_cam2.hnormalized();
    x1h.emplace_back(pixel1.homogeneous());
    x2h.emplace_back(pixel2.homogeneous());
    depth1.push_back(point_cam1.z());
    depth2.push_back(point_cam2.z() / true_scale);
  }

  std::vector<MonoDepthRelativePose> poses;
  EXPECT_GT(
      MonoDepthRelativePose3ptSharedFocal(x1h, x2h, depth1, depth2, &poses),
      0);
  ASSERT_GT(poses.size(), 0);

  bool found_match = false;
  for (const auto& pose : poses) {
    if ((pose.rotation - rotation).norm() < 1e-4 &&
        (pose.translation - translation).norm() < 1e-3) {
      found_match = true;
      EXPECT_NEAR(pose.scale, true_scale, 1e-3);
      EXPECT_NEAR(pose.focal_length1, true_focal, true_focal * 1e-3);
      EXPECT_NEAR(pose.focal_length2, true_focal, true_focal * 1e-3);
    }
  }
  EXPECT_TRUE(found_match);
}

TEST(MonoDepthRelativePose3ptVaryingFocal, BasicMinimal) {
  const std::vector<Vector3d> points_cam1 = {Vector3d(-0.3, 0.2, 3.0),
                                             Vector3d(0.4, -0.3, 2.0),
                                             Vector3d(0.1, 0.5, 2.5)};
  const Matrix3d rotation =
      Quaterniond(AngleAxisd(DegToRad(7.0), Vector3d(0.3, 0.6, 0.2).normalized()))
          .toRotationMatrix();
  const Vector3d translation(0.3, -0.1, 0.2);
  const double true_scale = 1.1;
  const double true_focal1 = 700.0;
  const double true_focal2 = 900.0;

  std::vector<Vector3d> x1h, x2h;
  std::vector<double> depth1, depth2;
  for (const Vector3d& point_cam1 : points_cam1) {
    const Vector3d point_cam2 = rotation * point_cam1 + translation;
    const Eigen::Vector2d pixel1 = true_focal1 * point_cam1.hnormalized();
    const Eigen::Vector2d pixel2 = true_focal2 * point_cam2.hnormalized();
    x1h.emplace_back(pixel1.homogeneous());
    x2h.emplace_back(pixel2.homogeneous());
    depth1.push_back(point_cam1.z());
    depth2.push_back(point_cam2.z() / true_scale);
  }

  std::vector<MonoDepthRelativePose> poses;
  EXPECT_EQ(
      MonoDepthRelativePose3ptVaryingFocal(x1h, x2h, depth1, depth2, &poses),
      1);
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_LT((poses[0].rotation - rotation).norm(), 1e-4);
  EXPECT_LT((poses[0].translation - translation).norm(), 1e-3);
  EXPECT_NEAR(poses[0].scale, true_scale, 1e-3);
  EXPECT_NEAR(poses[0].focal_length1, true_focal1, true_focal1 * 1e-3);
  EXPECT_NEAR(poses[0].focal_length2, true_focal2, true_focal2 * 1e-3);
}

}  // namespace
}  // namespace theia
