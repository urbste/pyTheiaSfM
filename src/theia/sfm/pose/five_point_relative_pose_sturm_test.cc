// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// This file is adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// test coverage for solvers/relpose_5pt, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08, and is distributed under the
// BSD-3-Clause license. See docs/licenses/POSELIB_LICENSE.txt for the full
// PoseLib license text.

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <vector>

#include "theia/math/util.h"
#include "theia/sfm/pose/five_point_relative_pose.h"
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

RandomNumberGenerator rng(67);

// Mirrors five_point_relative_pose_test.cc's minimal-sample test, but for the
// ported Sturm-based solver, which is strictly minimal (always exactly 5
// points).
void TestFivePointSturmResultWithNoise(const std::vector<Vector3d> points_3d,
                                       const double projection_noise_std_dev,
                                       const Matrix3d& expected_rotation,
                                       const Vector3d& expected_translation,
                                       const double ematrix_tolerance) {
  CHECK_EQ(points_3d.size(), 5);
  std::vector<Vector2d> view_one_points;
  std::vector<Vector2d> view_two_points;
  for (int i = 0; i < points_3d.size(); ++i) {
    const Vector3d proj_3d =
        expected_rotation * points_3d[i] + expected_translation;
    view_one_points.emplace_back(points_3d[i].hnormalized());
    view_two_points.emplace_back(proj_3d.hnormalized());
  }

  if (projection_noise_std_dev) {
    for (int i = 0; i < view_one_points.size(); ++i) {
      AddNoiseToProjection(projection_noise_std_dev, &rng, &view_one_points[i]);
      AddNoiseToProjection(projection_noise_std_dev, &rng, &view_two_points[i]);
    }
  }

  std::vector<Vector3d> x1h, x2h;
  for (int i = 0; i < view_one_points.size(); ++i) {
    x1h.emplace_back(view_one_points[i].homogeneous());
    x2h.emplace_back(view_two_points[i].homogeneous());
  }

  const Matrix3d gt_ematrix =
      CrossProductMatrix(expected_translation) * expected_rotation;
  std::vector<Matrix3d> soln_ematrices;
  EXPECT_GT(FivePointRelativePoseSturm(x1h, x2h, &soln_ematrices), 0);

  bool matched_transform = false;
  const double kEpipolarTolerance = 1e-8;
  for (int n = 0; n < soln_ematrices.size(); ++n) {
    for (int i = 0; i < view_one_points.size(); i++) {
      EXPECT_LT(SquaredSampsonDistance(
                    soln_ematrices[n], view_one_points[i], view_two_points[i]),
                kEpipolarTolerance);
    }
    if (test::ArraysEqualUpToScale(9,
                                   soln_ematrices[n].data(),
                                   gt_ematrix.data(),
                                   ematrix_tolerance)) {
      matched_transform = true;
    }
  }
  EXPECT_TRUE(matched_transform);
}

TEST(FivePointRelativePoseSturm, BasicMinimal) {
  const std::vector<Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(3.0, 1.0, 2.5),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)))
          .toRotationMatrix();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  TestFivePointSturmResultWithNoise(
      points_3d, 0.0, soln_rotation, soln_translation, 1e-4);
}

TEST(FivePointRelativePoseSturm, NoiseTestMinimal) {
  const std::vector<Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(3.0, 1.0, 2.5),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation =
      Quaterniond(AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)))
          .toRotationMatrix();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  TestFivePointSturmResultWithNoise(
      points_3d, 1.0 / 512.0, soln_rotation, soln_translation, 1e-2);
}

TEST(FivePointRelativePoseSturm, NoRotationMinimal) {
  const std::vector<Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(3.0, 1.0, 2.0),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation = Matrix3d::Identity();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  TestFivePointSturmResultWithNoise(
      points_3d, 1.0 / 512.0, soln_rotation, soln_translation, 0.01);
}

// Equivalence test (Phase 4.3 of dev/TWO_VIEW_SPEEDUP_PLAN.md): on the same
// noise-free minimal sample, the ported Sturm-based solver and theia's
// existing Stewenius-style FivePointRelativePose() must produce the same set
// of essential-matrix solutions (up to scale/sign), since both solve the
// same polynomial system via different algorithms.
TEST(FivePointRelativePoseSturm, EquivalentToFivePointRelativePose) {
  const std::vector<Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(3.0, 1.0, 2.5),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d rotation =
      Quaterniond(AngleAxisd(DegToRad(21.0), Vector3d(0.1, 0.4, 1.0).normalized()))
          .toRotationMatrix();
  const Vector3d translation(0.3, -0.2, 1.0);

  std::vector<Vector2d> view_one_points, view_two_points;
  std::vector<Vector3d> x1h, x2h;
  for (const Vector3d& point_3d : points_3d) {
    const Vector3d proj_3d = rotation * point_3d + translation;
    view_one_points.emplace_back(point_3d.hnormalized());
    view_two_points.emplace_back(proj_3d.hnormalized());
    x1h.emplace_back(view_one_points.back().homogeneous());
    x2h.emplace_back(view_two_points.back().homogeneous());
  }

  std::vector<Matrix3d> stewenius_solutions;
  EXPECT_TRUE(FivePointRelativePose(
      view_one_points, view_two_points, &stewenius_solutions));

  std::vector<Matrix3d> sturm_solutions;
  EXPECT_GT(FivePointRelativePoseSturm(x1h, x2h, &sturm_solutions), 0);

  // Every Sturm solution should match (up to scale) one of the Stewenius
  // solutions.
  for (const Matrix3d& sturm_e : sturm_solutions) {
    bool found_match = false;
    for (const Matrix3d& stewenius_e : stewenius_solutions) {
      if (test::ArraysEqualUpToScale(
              9, sturm_e.data(), stewenius_e.data(), 1e-6)) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

}  // namespace
}  // namespace theia
