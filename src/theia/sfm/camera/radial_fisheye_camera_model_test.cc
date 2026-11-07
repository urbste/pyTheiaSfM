// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#include <Eigen/Dense>

#include "gtest/gtest.h"
#include <ceres/jet.h>
#include <math.h>

#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::Vector2d;
using Eigen::Vector3d;

TEST(RadialFisheyeCameraModel, InternalParameterGettersAndSetters) {
  RadialFisheyeCameraModel camera;

  EXPECT_EQ(camera.Type(), CameraIntrinsicsModelType::RADIAL_FISHEYE);
  EXPECT_EQ(camera.NumParameters(), RadialFisheyeCameraModel::kIntrinsicsSize);

  // Check that default values are set.
  EXPECT_EQ(camera.FocalLength(), 1.0);
  EXPECT_EQ(camera.AspectRatio(), 1.0);
  EXPECT_EQ(camera.PrincipalPointX(), 0.0);
  EXPECT_EQ(camera.PrincipalPointY(), 0.0);
  for (int i = RadialFisheyeCameraModel::RADIAL_DISTORTION_1; i < RadialFisheyeCameraModel::kIntrinsicsSize; ++i) {
    EXPECT_EQ(camera.GetParameter(i), 0.0);
  }

  // Set parameters to different values.
  camera.SetFocalLength(600.0);
  camera.SetAspectRatio(0.9);
  camera.SetPrincipalPoint(300.0, 400.0);
  camera.SetRadialDistortion(0.01, 0.001);

  // Check that the values were updated.
  EXPECT_EQ(camera.FocalLength(), 600.0);
  EXPECT_EQ(camera.AspectRatio(), 0.9);
  EXPECT_EQ(camera.PrincipalPointX(), 300.0);
  EXPECT_EQ(camera.PrincipalPointY(), 400.0);
  EXPECT_EQ(camera.RadialDistortion1(), 0.01);
  EXPECT_EQ(camera.RadialDistortion2(), 0.001);
}

// The priors set and read back the same parameters.
TEST(RadialFisheyeCameraModel, SetFromCameraIntrinsicsPriors) {
  CameraIntrinsicsPrior prior;
  prior.focal_length.is_set = true;
  prior.focal_length.value[0] = 1000.0;
  prior.principal_point.is_set = true;
  prior.principal_point.value[0] = 400.0;
  prior.principal_point.value[1] = 300.0;
  prior.aspect_ratio.is_set = true;
  prior.aspect_ratio.value[0] = 1.01;
  prior.radial_distortion.is_set = true;
  prior.radial_distortion.value[0] = 0.01;
  prior.radial_distortion.value[1] = 0.001;

  RadialFisheyeCameraModel camera;
  camera.SetFromCameraIntrinsicsPriors(prior);
  EXPECT_EQ(camera.FocalLength(), 1000.0);
  EXPECT_EQ(camera.PrincipalPointX(), 400.0);
  EXPECT_EQ(camera.PrincipalPointY(), 300.0);
  EXPECT_EQ(camera.AspectRatio(), 1.01);
  EXPECT_EQ(camera.RadialDistortion2(), 0.001);

  // Round trip through CameraIntrinsicsPriorFromIntrinsics.
  const CameraIntrinsicsPrior round_trip =
      camera.CameraIntrinsicsPriorFromIntrinsics();
  EXPECT_EQ(round_trip.camera_intrinsics_model_type, "RADIAL_FISHEYE");
  RadialFisheyeCameraModel copy;
  copy.SetFromCameraIntrinsicsPriors(round_trip);
  for (int i = 0; i < RadialFisheyeCameraModel::kIntrinsicsSize; ++i) {
    EXPECT_EQ(copy.GetParameter(i), camera.GetParameter(i)) << "index " << i;
  }
}

TEST(RadialFisheyeCameraModel, GetSubsetFromOptimizeIntrinsicsType) {
  RadialFisheyeCameraModel camera;
  std::vector<int> constant_subset;

  constant_subset =
      camera.GetSubsetFromOptimizeIntrinsicsType(OptimizeIntrinsicsType::NONE);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters());

  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::ALL);
  EXPECT_EQ(constant_subset.size(), 0);

  // Optimizing the focal length keeps everything else constant.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::FOCAL_LENGTH);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 1);
  for (int i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::FOCAL_LENGTH);
  }

  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::ASPECT_RATIO);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 1);
  for (int i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::ASPECT_RATIO);
  }

  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::PRINCIPAL_POINTS);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 2);
  for (int i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::PRINCIPAL_POINT_X);
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::PRINCIPAL_POINT_Y);
  }

  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::RADIAL_DISTORTION);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 2);
  for (int i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::RADIAL_DISTORTION_1);
    EXPECT_NE(constant_subset[i], RadialFisheyeCameraModel::RADIAL_DISTORTION_2);
  }

  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::TANGENTIAL_DISTORTION);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 0);
  for (int i = 0; i < constant_subset.size(); i++) {
  }
}

// Checks that the distortion inverse (Newton) recovers the undistorted point.
TEST(RadialFisheyeCameraModel, DistortUndistortRoundTrip) {
  RadialFisheyeCameraModel camera;
  camera.SetFocalLength(1200.0);
  camera.SetPrincipalPoint(600.0, 400.0);
  camera.SetRadialDistortion(0.02, -0.003);

  for (double u = -0.4; u <= 0.4; u += 0.05) {
    for (double v = -0.4; v <= 0.4; v += 0.05) {
      const Vector2d undistorted(u, v);
      Vector2d distorted, recovered;
      ASSERT_TRUE(RadialFisheyeCameraModel::DistortPoint(
          camera.parameters(), undistorted.data(), distorted.data()));
      ASSERT_TRUE(RadialFisheyeCameraModel::UndistortPoint(
          camera.parameters(), distorted.data(), recovered.data()));
      EXPECT_LT((undistorted - recovered).norm(), 1e-8)
          << "undistorted: " << undistorted.transpose();
    }
  }
}

// The derivatives of the projection with respect to the intrinsics (used by
// bundle adjustment) must match finite differences.
TEST(RadialFisheyeCameraModel, ProjectionJetMatchesFiniteDifference) {
  RadialFisheyeCameraModel camera;
  camera.SetFocalLength(900.0);
  camera.SetAspectRatio(1.02);
  camera.SetPrincipalPoint(500.0, 350.0);
  camera.SetRadialDistortion(0.02, -0.003);

  using JetT = ceres::Jet<double, RadialFisheyeCameraModel::kIntrinsicsSize>;
  JetT jet_params[RadialFisheyeCameraModel::kIntrinsicsSize];
  for (int i = 0; i < RadialFisheyeCameraModel::kIntrinsicsSize; ++i) {
    jet_params[i] = JetT(camera.GetParameter(i), i);
  }
  const Vector3d point(0.2, -0.15, 2.0);
  JetT jet_point[3] = {JetT(point.x()), JetT(point.y()), JetT(point.z())};
  JetT jet_pixel[2];
  ASSERT_TRUE(RadialFisheyeCameraModel::CameraToPixelCoordinates(jet_params, jet_point, jet_pixel));

  const double kStep = 1e-6;
  for (int i = 0; i < RadialFisheyeCameraModel::kIntrinsicsSize; ++i) {
    RadialFisheyeCameraModel perturbed = camera;
    perturbed.SetParameter(i, camera.GetParameter(i) + kStep);
    const Vector2d plus = perturbed.CameraToImageCoordinates(point);
    perturbed.SetParameter(i, camera.GetParameter(i) - kStep);
    const Vector2d minus = perturbed.CameraToImageCoordinates(point);
    for (int k = 0; k < 2; ++k) {
      const double numeric = (plus[k] - minus[k]) / (2.0 * kStep);
      EXPECT_NEAR(jet_pixel[k].v[i], numeric, 1e-3)
          << "parameter " << i << " output " << k;
    }
  }
}

void ReprojectionTest(const RadialFisheyeCameraModel& camera) {
  static const double kTolerance = 1e-5;
  const double kNormalizedTolerance = kTolerance / camera.FocalLength();
  static const int kImageWidth = 1200;
  static const int kImageHeight = 980;
  static const double kMinDepth = 2.0;
  static const double kMaxDepth = 25.0;

  // Ensure the image -> camera -> image transformation works.
  for (double x = 0.0; x < kImageWidth; x += 10.0) {
    for (double y = 0.0; y < kImageHeight; y += 10.0) {
      const Vector2d pixel(x, y);
      // Get the normalized ray of that pixel.
      const Vector3d normalized_ray = camera.ImageToCameraCoordinates(pixel);

      // Test the reprojection at several depths.
      for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.0) {
        const Vector3d point = normalized_ray * depth;
        const Vector2d reprojected_pixel =
            camera.CameraToImageCoordinates(point);
        EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
            << "gt pixel: " << pixel.transpose()
            << "\nreprojected pixel: " << reprojected_pixel.transpose();
      }
    }
  }

  // Ensure the camera -> image -> camera transformation works.
  for (double x = -0.8; x < 0.8; x += 0.1) {
    for (double y = -0.8; y < 0.8; y += 0.1) {
      for (double depth = kMinDepth; depth < kMaxDepth; depth += 1.0) {
        const Vector3d point(x, y, depth);
        const Vector2d pixel = camera.CameraToImageCoordinates(point);
        const Vector3d normalized_ray = camera.ImageToCameraCoordinates(pixel);
        const Vector3d reprojected_point = normalized_ray * depth;
        EXPECT_LT((point - reprojected_point).norm(), kNormalizedTolerance)
            << "gt point: " << point.transpose()
            << "\nreprojected point: " << reprojected_point.transpose();
      }
    }
  }
}

TEST(RadialFisheyeCameraModel, ReprojectionNoDistortion) {
  RadialFisheyeCameraModel camera;
  camera.SetFocalLength(1200.0);
  camera.SetPrincipalPoint(600.0, 400.0);
  ReprojectionTest(camera);
}

TEST(RadialFisheyeCameraModel, ReprojectionDistortion) {
  RadialFisheyeCameraModel camera;
  camera.SetFocalLength(1200.0);
  camera.SetPrincipalPoint(600.0, 400.0);
  camera.SetRadialDistortion(0.02, -0.003);

  ReprojectionTest(camera);
}

}  // namespace theia
