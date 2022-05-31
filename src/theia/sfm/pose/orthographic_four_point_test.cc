// Copyright (C) 2022
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/util.h"
#include "theia/sfm/pose/orthographic_four_point.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/types.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(55);

void PoseFromFourCalibratedTest(const double noise) {
  // Projection matrix.
  const Matrix3d gt_rotation =
      (Eigen::AngleAxisd(DegToRad(15.0), Vector3d(1.0, 0.0, 0.0)) *
       Eigen::AngleAxisd(DegToRad(-10.0), Vector3d(0.0, 1.0, 0.0)))
          .toRotationMatrix();
  const Vector3d gt_translation(0.3, -1.7, 1.15);
  Matrix3x4d projection_mat;
  projection_mat << gt_rotation, gt_translation;

  // Points in the 3D scene.
  const Vector3d kPoints3d[3] = {Vector3d(-0.3001, -0.5840, 1.2271),
                                 Vector3d(-1.4487, 0.6965, 0.3889),
                                 Vector3d(-0.7815, 0.7642, 0.1257)};

  // Points in the camera view.
  Vector2d kPoints2d[3];
  for (int i = 0; i < 3; i++) {
    kPoints2d[i] =
        (projection_mat * kPoints3d[i].homogeneous()).eval().hnormalized();
    if (noise) {
      AddNoiseToProjection(noise, &rng, &kPoints2d[i]);
    }
  }

  std::vector<Matrix3d> rotations;
  std::vector<Vector3d> translations;
  //CHECK(OrthoPoseFromFourPoints(kPoints2d, kPoints3d, &rotations, &translations));

  bool matched_transform = false;
  for (int i = 0; i < rotations.size(); ++i) {
    // Check that the rotation and translation are close.
    double angular_diff =
        RadToDeg(Eigen::Quaterniond(rotations[i])
                     .angularDistance(Eigen::Quaterniond(gt_rotation)));
    double trans_diff =
        ((-gt_rotation * gt_translation) - (-rotations[i] * translations[i]))
            .norm();
    bool rot_match = angular_diff < 1.0;
    bool trans_match = trans_diff < 0.1;
    if (rot_match && trans_match) {
      matched_transform = true;

      Matrix3x4d soln_proj;
      soln_proj << rotations[i], translations[i];
      // Check the reprojection error.
      for (int j = 0; j < 3; j++) {
        const Vector3d projected_pt = soln_proj * kPoints3d[j].homogeneous();
        EXPECT_LT((kPoints2d[j] - projected_pt.hnormalized()).norm() * 800.0,
                  2.0);
      }
    }
  }
  EXPECT_TRUE(matched_transform);
}

TEST(O4P, PoseFromFourCalibrated) { PoseFromFourCalibratedTest(0.0 / 800.0); }

TEST(O4P, PoseFromFourCalibratedNoise) {
  PoseFromFourCalibratedTest(1.0 / 800.0);
}

}  // namespace theia
