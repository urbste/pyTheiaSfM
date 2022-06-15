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

void OrthoPoseUncalibrated(const double noise) {
  // Projection matrix.
  const Matrix3d gt_rotation =
      (Eigen::AngleAxisd(DegToRad(10.0), Vector3d(1.0, 0.0, 0.0)) *
       Eigen::AngleAxisd(DegToRad(-10.0), Vector3d(0.0, 1.0, 0.0)))
          .toRotationMatrix();
  Vector2d principal_point(500,500);
  double gt_magn = 30000.;

  const Vector3d gt_translation(0.01, -0.01, 0.0);
  Matrix3x4d projection_mat;
  projection_mat << gt_rotation, gt_translation;

  // Points in the 3D scene.
  std::vector<Vector3d> kPoints3d = {Vector3d(-0.01001, -0.001840, 0.),
                                 Vector3d(-0.002487, 0.002965, 0.),
                                 Vector3d(-0.01815, 0.01642, 0.),
                                Vector3d(-0.0022, 0.0011, 0.0)};

  // Points in the camera view.
  std::vector<Vector2d> kPoints2d(4);
  for (int i = 0; i < 4; i++) {
    kPoints2d[i] =
        gt_rotation.block<2,3>(0,0) * kPoints3d[i] + gt_translation.topRows(2);
    kPoints2d[i][0] = kPoints2d[i][0] * gt_magn + principal_point[0];
    kPoints2d[i][1] = kPoints2d[i][1] * gt_magn + principal_point[1];

    if (noise) {
      AddNoiseToProjection(noise, &rng, &kPoints2d[i]);
    }
  }

  std::vector<Matrix3d> rotations;
  std::vector<Vector3d> translations;
  double magn;
  CHECK(PlanarUncalibratedOrthographicPose(kPoints2d, kPoints3d, principal_point, &rotations, &translations, &magn));

  bool matched_transform = false;
  for (size_t i = 0; i < rotations.size(); ++i) {
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

      // Check the reprojection error.
      for (int j = 0; j < 4; j++) {
        const Vector2d projected_pt = magn * (
                    rotations[i].block<2,3>(0,0) * kPoints3d[j] + translations[i].topRows(2))
                    + principal_point;
        EXPECT_LT((kPoints2d[j] - projected_pt).norm(),2.0);
      }
    }
  }
  EXPECT_TRUE(matched_transform);
}

TEST(O4P, PoseFromFourCalibrated) { OrthoPoseUncalibrated(0.0); }

TEST(O4P, PoseFromFourCalibratedNoise) {
  OrthoPoseUncalibrated(0.1);
}

}  // namespace theia
