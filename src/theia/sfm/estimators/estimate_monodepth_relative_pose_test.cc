// Copyright (C) 2024 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// RANSAC-level round-trip tests (synthetic scenes, outliers) for the
// monodepth estimators, per dev/TWO_VIEW_SPEEDUP_PLAN.md Phase 4.1.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <vector>

#include "gtest/gtest.h"

#include "theia/matching/feature_correspondence.h"
#include "theia/math/util.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/estimate_monodepth_relative_pose.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/pose/util.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/test/test_utils.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

RandomNumberGenerator monodepth_rng(71);

void GeneratePoints(std::vector<Vector3d>* points) {
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = 4; k <= 6; k++) {
        points->emplace_back(Vector3d(i, j, k));
      }
    }
  }
}

}  // namespace

TEST(EstimateMonoDepthRelativePose, OutliersNoNoise) {
  std::vector<Vector3d> points3d;
  GeneratePoints(&points3d);

  const Matrix3d rotation =
      AngleAxisd(DegToRad(10.0), Vector3d(0.1, 1.0, -0.2).normalized())
          .toRotationMatrix();
  const Vector3d translation(0.3, -0.1, 0.2);
  const double true_scale = 1.4;
  const double kInlierRatio = 0.7;

  std::vector<FeatureCorrespondence> correspondences;
  for (int i = 0; i < points3d.size(); i++) {
    FeatureCorrespondence correspondence;
    if (i < kInlierRatio * points3d.size()) {
      const Vector3d point_cam2 = rotation * points3d[i] + translation;
      correspondence.feature1 =
          Feature(points3d[i].hnormalized(), points3d[i].z());
      correspondence.feature2 =
          Feature(point_cam2.hnormalized(), point_cam2.z() / true_scale);
    } else {
      correspondence.feature1 = Feature(
          Vector2d(monodepth_rng.RandDouble(-1.0, 1.0),
                  monodepth_rng.RandDouble(-1.0, 1.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
      correspondence.feature2 = Feature(
          Vector2d(monodepth_rng.RandDouble(-1.0, 1.0),
                  monodepth_rng.RandDouble(-1.0, 1.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
    }
    correspondences.emplace_back(correspondence);
  }

  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(monodepth_rng);
  options.use_mle = true;
  options.error_thresh = 1e-4;
  options.failure_probability = 0.0001;

  MonoDepthRelativePoseResult result;
  RansacSummary summary;
  EXPECT_TRUE(EstimateMonoDepthRelativePose(
      options, RansacType::RANSAC, correspondences, &result, &summary));

  EXPECT_GT(static_cast<double>(summary.inliers.size()), 5);

  const Eigen::AngleAxisd rotation_loop(rotation *
                                        result.rotation.transpose());
  EXPECT_LT(RadToDeg(rotation_loop.angle()), 5.0);

  const double translation_diff_rad = std::acos(Clamp(
      (-rotation.transpose() * translation).normalized().dot(result.position),
      -1.0,
      1.0));
  EXPECT_LT(RadToDeg(translation_diff_rad), 5.0);

  EXPECT_NEAR(result.scale, true_scale, 0.1);
}

TEST(EstimateMonoDepthRelativePoseSharedFocal, OutliersNoNoise) {
  std::vector<Vector3d> points3d;
  GeneratePoints(&points3d);

  const Matrix3d rotation =
      AngleAxisd(DegToRad(8.0), Vector3d(0.2, 0.9, -0.1).normalized())
          .toRotationMatrix();
  const Vector3d translation(0.25, -0.05, 0.15);
  const double true_scale = 1.2;
  const double true_focal = 850.0;
  const double kInlierRatio = 0.7;

  std::vector<FeatureCorrespondence> correspondences;
  for (int i = 0; i < points3d.size(); i++) {
    FeatureCorrespondence correspondence;
    if (i < kInlierRatio * points3d.size()) {
      const Vector3d point_cam2 = rotation * points3d[i] + translation;
      correspondence.feature1 =
          Feature(true_focal * points3d[i].hnormalized(), points3d[i].z());
      correspondence.feature2 = Feature(
          true_focal * point_cam2.hnormalized(), point_cam2.z() / true_scale);
    } else {
      correspondence.feature1 = Feature(
          Vector2d(monodepth_rng.RandDouble(-800.0, 800.0),
                  monodepth_rng.RandDouble(-800.0, 800.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
      correspondence.feature2 = Feature(
          Vector2d(monodepth_rng.RandDouble(-800.0, 800.0),
                  monodepth_rng.RandDouble(-800.0, 800.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
    }
    correspondences.emplace_back(correspondence);
  }

  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(monodepth_rng);
  options.use_mle = true;
  options.error_thresh = 4.0;  // pixels^2
  options.failure_probability = 0.0001;

  MonoDepthRelativePoseResult result;
  RansacSummary summary;
  EXPECT_TRUE(EstimateMonoDepthRelativePoseSharedFocal(
      options, RansacType::RANSAC, correspondences, &result, &summary));

  EXPECT_GT(static_cast<double>(summary.inliers.size()), 5);

  const Eigen::AngleAxisd rotation_loop(rotation *
                                        result.rotation.transpose());
  EXPECT_LT(RadToDeg(rotation_loop.angle()), 5.0);
  EXPECT_NEAR(result.focal_length1, true_focal, true_focal * 0.05);
  EXPECT_NEAR(result.focal_length2, true_focal, true_focal * 0.05);
}

TEST(EstimateMonoDepthRelativePoseVaryingFocal, OutliersNoNoise) {
  std::vector<Vector3d> points3d;
  GeneratePoints(&points3d);

  const Matrix3d rotation =
      AngleAxisd(DegToRad(6.0), Vector3d(0.3, 0.7, -0.2).normalized())
          .toRotationMatrix();
  const Vector3d translation(0.2, -0.08, 0.12);
  const double true_scale = 1.05;
  const double true_focal1 = 750.0;
  const double true_focal2 = 950.0;
  const double kInlierRatio = 0.7;

  std::vector<FeatureCorrespondence> correspondences;
  for (int i = 0; i < points3d.size(); i++) {
    FeatureCorrespondence correspondence;
    if (i < kInlierRatio * points3d.size()) {
      const Vector3d point_cam2 = rotation * points3d[i] + translation;
      correspondence.feature1 =
          Feature(true_focal1 * points3d[i].hnormalized(), points3d[i].z());
      correspondence.feature2 = Feature(
          true_focal2 * point_cam2.hnormalized(),
          point_cam2.z() / true_scale);
    } else {
      correspondence.feature1 = Feature(
          Vector2d(monodepth_rng.RandDouble(-800.0, 800.0),
                  monodepth_rng.RandDouble(-800.0, 800.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
      correspondence.feature2 = Feature(
          Vector2d(monodepth_rng.RandDouble(-800.0, 800.0),
                  monodepth_rng.RandDouble(-800.0, 800.0)),
          monodepth_rng.RandDouble(1.0, 5.0));
    }
    correspondences.emplace_back(correspondence);
  }

  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(monodepth_rng);
  options.use_mle = true;
  options.error_thresh = 4.0;  // pixels^2
  options.failure_probability = 0.0001;

  MonoDepthRelativePoseResult result;
  RansacSummary summary;
  EXPECT_TRUE(EstimateMonoDepthRelativePoseVaryingFocal(
      options, RansacType::RANSAC, correspondences, &result, &summary));

  EXPECT_GT(static_cast<double>(summary.inliers.size()), 5);

  const Eigen::AngleAxisd rotation_loop(rotation *
                                        result.rotation.transpose());
  EXPECT_LT(RadToDeg(rotation_loop.angle()), 5.0);
  EXPECT_NEAR(result.focal_length1, true_focal1, true_focal1 * 0.05);
  EXPECT_NEAR(result.focal_length2, true_focal2, true_focal2 * 0.05);
}

}  // namespace theia
