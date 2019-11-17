// Copyright (C) 2019 The Regents of the University of California (Regents).
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
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// This file was created by Steffen Urban (urbste@googlemail.com) October 2019

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

#include "theia/math/util.h"
#include "theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h"
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/pose/util.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void DistortPoint(const Eigen::Vector2d& point2d, const double& distortion,
                  Eigen::Vector2d* distorted_point) {
  const double r_u_sq = point2d[0] * point2d[0] + point2d[1] * point2d[1];

  const double denom = 2.0 * distortion * r_u_sq;
  const double inner_sqrt = 1.0 - 4.0 * distortion * r_u_sq;

  // If the denominator is nearly zero then we can evaluate the distorted
  // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
  if (std::abs(denom) < 1e-15 || inner_sqrt < 0.0) {
    (*distorted_point)[0] = point2d[0];
    (*distorted_point)[1] = point2d[1];
  } else {
    const double scale = (1.0 - std::sqrt(inner_sqrt)) / denom;
    (*distorted_point)[0] = point2d[0] * scale;
    (*distorted_point)[1] = point2d[1] * scale;
  }
}

static const int kNumPoints = 20;
static const double kFocalLength = 1000.0;
static const double kRadialDistortion = -1e-7;
static const double kReprojectionError = 1.0;
static const double kErrorThreshold = kReprojectionError * kReprojectionError;
RandomNumberGenerator rng(64);

void ExecuteRandomTest(const RansacParameters& options,
                       const Matrix3d& gt_rotation,
                       const Vector3d& gt_translation,
                       const double inlier_ratio, const double noise,
                       const double tolerance) {
  // Create feature correspondences (inliers and outliers) and add noise if
  // appropriate.
  std::vector<FeatureCorrespondence2D3D> correspondences;
  for (int i = 0; i < kNumPoints; i++) {
    FeatureCorrespondence2D3D correspondence;
    correspondence.world_point =
        Vector3d(rng.RandDouble(-5.0, 5.0), rng.RandDouble(-5.0, 5.0),
                 rng.RandDouble(3.0, 10.0));

    const Matrix3d camera_matrix =
        Eigen::DiagonalMatrix<double, 3>(kFocalLength, kFocalLength, 1.0);
    Eigen::Matrix<double, 3, 4> gt_transformation, gt_projection;
    gt_transformation << gt_rotation, gt_translation;
    gt_projection = camera_matrix * gt_transformation;

    // Add an inlier or outlier.
    if (i < inlier_ratio * kNumPoints) {
      // Make sure the point is in front of the camera.
      correspondence.feature =
          (gt_projection * correspondence.world_point.homogeneous())
              .hnormalized();
      DistortPoint(correspondence.feature, kRadialDistortion,
                   &correspondence.feature);
    } else {
      correspondence.feature =
          Vector2d(rng.RandDouble(-1.0, 1.0), rng.RandDouble(-1.0, 1.0));
    }


    correspondences.emplace_back(correspondence);
  }

  if (noise) {
    for (int i = 0; i < kNumPoints; i++) {
      AddNoiseToProjection(noise, &rng, &correspondences[i].feature);
    }
  }

  // Estimate the absolute pose.
  RadialDistUncalibratedAbsolutePose pose;
  RadialDistUncalibratedAbsolutePoseMetaData meta_data;
  meta_data.max_focal_length = 2000;
  meta_data.min_focal_length = 100;
  meta_data.min_radial_distortion = -1e-9;
  meta_data.max_radial_distortion = -1e-5;
  RansacSummary ransac_summary;
  EXPECT_TRUE(EstimateRadialDistUncalibratedAbsolutePose(
      options, RansacType::RANSAC, correspondences, meta_data, &pose,
      &ransac_summary));

  // Expect poses are near.
  EXPECT_TRUE(test::ArraysEqualUpToScale(9, gt_rotation.data(),
                                         pose.rotation.data(), tolerance));
  // The position is more noisy than the rotation usually.
  EXPECT_TRUE(test::ArraysEqualUpToScale(
      3, gt_translation.data(), pose.translation.data(), 2.0 * tolerance));

  // Expect focal length is near.
  static const double kFocalLengthTolerance = 0.05;
  EXPECT_NEAR(pose.focal_length, kFocalLength,
              kFocalLengthTolerance * kFocalLength);
  // Expect radial distortion is near.
  EXPECT_NEAR(pose.radial_distortion, kRadialDistortion,
              std::abs(2.0 * kFocalLengthTolerance * kRadialDistortion));
}

 TEST(EstimateRadialDistUncalibratedAbsolutePose, AllInliersNoNoise) {
  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(rng);
  options.use_mle = true;
  options.error_thresh = kErrorThreshold;
  options.failure_probability = 0.001;
  options.min_iterations = 1;
  const double kInlierRatio = 1.0;
  const double kNoise = 0.0;
  const double kPoseTolerance = 1e-4;

  const std::vector<Matrix3d> rotations = {
      Matrix3d::Identity(),
      AngleAxisd(DegToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
      AngleAxisd(DegToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
          .toRotationMatrix()};
  const std::vector<Vector3d> positions = {Vector3d(1.3, 0, 0.0),
                                           Vector3d(1.0, 1.0, 0.1)};
  for (int i = 0; i < rotations.size(); i++) {
    for (int j = 0; j < positions.size(); j++) {
      ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                        kNoise, kPoseTolerance);
    }
  }
}

 TEST(EstimateRadialDistUncalibratedAbsolutePose, AllInliersWithNoise) {
  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(rng);
  options.use_mle = true;
  options.error_thresh = kErrorThreshold;
  options.failure_probability = 0.001;
  options.max_iterations = 1000;
  options.min_iterations = 1;
  const double kInlierRatio = 1.0;
  const double kNoise = 1.0;
  const double kPoseTolerance = 1e-2;

  const std::vector<Matrix3d> rotations = {
      Matrix3d::Identity(),
      AngleAxisd(DegToRad(12.0), Vector3d::UnitY()).toRotationMatrix(),
      AngleAxisd(DegToRad(-9.0), Vector3d(1.0, 0.2, -0.8).normalized())
          .toRotationMatrix()};
  const std::vector<Vector3d> positions = {Vector3d(1.3, 0, 0.0),
                                           Vector3d(1.0, 1.0, 0.1)};

  for (int i = 0; i < rotations.size(); i++) {
    for (int j = 0; j < positions.size(); j++) {
      ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                        kNoise, kPoseTolerance);
    }
  }
}

TEST(EstimateRadialDistUncalibratedAbsolutePose, OutliersNoNoise) {
  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(rng);
  options.use_mle = true;
  options.error_thresh = kErrorThreshold;
  options.failure_probability = 0.001;
  options.max_iterations = 1000;
  options.min_iterations = 10;
  const double kInlierRatio = 0.7;
  const double kNoise = 0.0;
  const double kPoseTolerance = 1e-2;

  const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                           RandomRotation(15.0, &rng)};
  const std::vector<Vector3d> positions = {Vector3d(1.3, 0, 0.0),
                                           Vector3d(1.0, 1.0, 0.1)};

  for (int i = 0; i < rotations.size(); i++) {
    for (int j = 0; j < positions.size(); j++) {
      ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                        kNoise, kPoseTolerance);
    }
  }
}

TEST(EstimateRadialDistUncalibratedAbsolutePose, OutliersWithNoise) {
  RansacParameters options;
  options.rng = std::make_shared<RandomNumberGenerator>(rng);
  options.use_mle = true;
  options.error_thresh = kErrorThreshold;
  options.failure_probability = 0.001;
  options.max_iterations = 1000;
  options.min_iterations = 10;
  const double kInlierRatio = 0.7;
  const double kNoise = 1.0;
  const double kPoseTolerance = 0.1;

  const std::vector<Matrix3d> rotations = {Matrix3d::Identity(),
                                           RandomRotation(15.0, &rng)};
  const std::vector<Vector3d> positions = {Vector3d(1.3, 0, 0.0),
                                           Vector3d(1.0, 1.0, 0.1)};

  for (int i = 0; i < rotations.size(); i++) {
    for (int j = 0; j < positions.size(); j++) {
      ExecuteRandomTest(options, rotations[i], positions[j], kInlierRatio,
                        kNoise, kPoseTolerance);
    }
  }
}

}  // namespace theia
