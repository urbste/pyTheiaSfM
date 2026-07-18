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
// End-to-end EstimateTwoViewInfo tests, in particular exercising the
// use_monodepth path (dev/TWO_VIEW_SPEEDUP_PLAN.md Phase 4.2).

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <vector>

#include "gtest/gtest.h"

#include "theia/matching/feature_correspondence.h"
#include "theia/math/util.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/estimate_twoview_info.h"
#include "theia/sfm/twoview_info.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

RandomNumberGenerator twoview_rng(51);

CameraIntrinsicsPrior MakeCalibratedIntrinsics(const double focal_length) {
  CameraIntrinsicsPrior prior;
  prior.image_width = 1024;
  prior.image_height = 768;
  prior.focal_length.is_set = true;
  prior.focal_length.value[0] = focal_length;
  prior.principal_point.is_set = true;
  prior.principal_point.value[0] = 512.0;
  prior.principal_point.value[1] = 384.0;
  return prior;
}

}  // namespace

TEST(EstimateTwoViewInfo, MonodepthCalibratedRecoversPoseAndScale) {
  const double kFocalLength = 1000.0;
  const Matrix3d rotation =
      AngleAxisd(DegToRad(9.0), Vector3d(0.2, 1.0, -0.1).normalized())
          .toRotationMatrix();
  const Vector3d translation(0.3, -0.1, 0.2);
  const double true_scale = 1.3;
  const double kInlierRatio = 0.7;

  std::vector<Vector3d> points3d;
  for (int i = -2; i <= 2; i++) {
    for (int j = -2; j <= 2; j++) {
      for (int k = 4; k <= 6; k++) {
        points3d.emplace_back(0.5 * i, 0.5 * j, k);
      }
    }
  }

  const CameraIntrinsicsPrior intrinsics1 =
      MakeCalibratedIntrinsics(kFocalLength);
  const CameraIntrinsicsPrior intrinsics2 =
      MakeCalibratedIntrinsics(kFocalLength);

  std::vector<FeatureCorrespondence> correspondences;
  for (int i = 0; i < points3d.size(); i++) {
    FeatureCorrespondence correspondence;
    if (i < kInlierRatio * points3d.size()) {
      const Vector3d point_cam2 = rotation * points3d[i] + translation;
      const Vector2d pixel1 =
          kFocalLength * points3d[i].hnormalized() + Vector2d(512.0, 384.0);
      const Vector2d pixel2 =
          kFocalLength * point_cam2.hnormalized() + Vector2d(512.0, 384.0);
      correspondence.feature1 = Feature(pixel1, points3d[i].z());
      correspondence.feature2 =
          Feature(pixel2, point_cam2.z() / true_scale);
    } else {
      correspondence.feature1 =
          Feature(Vector2d(twoview_rng.RandDouble(0.0, 1024.0),
                           twoview_rng.RandDouble(0.0, 768.0)),
                 twoview_rng.RandDouble(1.0, 5.0));
      correspondence.feature2 =
          Feature(Vector2d(twoview_rng.RandDouble(0.0, 1024.0),
                           twoview_rng.RandDouble(0.0, 768.0)),
                 twoview_rng.RandDouble(1.0, 5.0));
    }
    correspondences.emplace_back(correspondence);
  }

  EstimateTwoViewInfoOptions options;
  options.rng = std::make_shared<RandomNumberGenerator>(twoview_rng);
  options.use_mle = true;
  options.use_monodepth = true;
  options.max_sampson_error_pixels = 4.0;
  options.min_ransac_iterations = 20;
  options.max_ransac_iterations = 2000;

  TwoViewInfo twoview_info;
  std::vector<int> inlier_indices;
  ASSERT_TRUE(EstimateTwoViewInfo(options,
                                  intrinsics1,
                                  intrinsics2,
                                  correspondences,
                                  &twoview_info,
                                  &inlier_indices));

  EXPECT_GT(inlier_indices.size(), 5u);

  const Matrix3d estimated_rotation =
      AngleAxisd(twoview_info.rotation_2.norm(),
                twoview_info.rotation_2.normalized())
          .toRotationMatrix();
  const AngleAxisd rotation_loop(rotation * estimated_rotation.transpose());
  EXPECT_LT(RadToDeg(rotation_loop.angle()), 5.0);

  // scale_estimate should be close to the ratio of the two depth-map scales
  // (the plan's Phase 4.1 acceptance criterion).
  EXPECT_GT(twoview_info.scale_estimate, 0.0);
  EXPECT_NEAR(twoview_info.scale_estimate, true_scale, 0.15);
}

TEST(EstimateTwoViewInfo, MonodepthFallsBackWithoutDepthPriors) {
  // Without depth priors, use_monodepth=true must still succeed by falling
  // back to the standard estimator (no regression / no crash).
  const double kFocalLength = 1000.0;
  const Matrix3d rotation =
      AngleAxisd(DegToRad(9.0), Vector3d(0.2, 1.0, -0.1).normalized())
          .toRotationMatrix();
  const Vector3d translation(0.3, -0.1, 0.2);

  std::vector<Vector3d> points3d;
  for (int i = -2; i <= 2; i++) {
    for (int j = -2; j <= 2; j++) {
      for (int k = 4; k <= 6; k++) {
        points3d.emplace_back(0.5 * i, 0.5 * j, k);
      }
    }
  }

  const CameraIntrinsicsPrior intrinsics1 =
      MakeCalibratedIntrinsics(kFocalLength);
  const CameraIntrinsicsPrior intrinsics2 =
      MakeCalibratedIntrinsics(kFocalLength);

  std::vector<FeatureCorrespondence> correspondences;
  for (const Vector3d& point : points3d) {
    FeatureCorrespondence correspondence;
    const Vector3d point_cam2 = rotation * point + translation;
    correspondence.feature1 = Feature(
        kFocalLength * point.hnormalized() + Vector2d(512.0, 384.0));
    correspondence.feature2 = Feature(
        kFocalLength * point_cam2.hnormalized() + Vector2d(512.0, 384.0));
    correspondences.emplace_back(correspondence);
  }

  EstimateTwoViewInfoOptions options;
  options.rng = std::make_shared<RandomNumberGenerator>(twoview_rng);
  options.use_monodepth = true;
  options.max_sampson_error_pixels = 4.0;

  TwoViewInfo twoview_info;
  std::vector<int> inlier_indices;
  EXPECT_TRUE(EstimateTwoViewInfo(options,
                                  intrinsics1,
                                  intrinsics2,
                                  correspondences,
                                  &twoview_info,
                                  &inlier_indices));
  EXPECT_GT(inlier_indices.size(), 5u);
}

}  // namespace theia
