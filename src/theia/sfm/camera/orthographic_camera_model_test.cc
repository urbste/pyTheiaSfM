// Copyright (C) 2022
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban

#include <Eigen/Dense>

#include "gtest/gtest.h"
#include <ceres/rotation.h>
#include <math.h>

#include "theia/alignment/alignment.h"
#include "theia/sfm/camera/orthographic_camera_model.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

TEST(OrthographicCameraModel, InternalParameterGettersAndSetters) {
  OrthographicCameraModel camera;

  EXPECT_EQ(camera.Type(), CameraIntrinsicsModelType::ORTHOGRAPHIC);

  // Check that default values are set
  EXPECT_EQ(camera.FocalLength(), 1.0);
  EXPECT_EQ(camera.AspectRatio(), 1.0);
  EXPECT_EQ(camera.Skew(), 0.0);
  EXPECT_EQ(camera.PrincipalPointX(), 0.0);
  EXPECT_EQ(camera.PrincipalPointY(), 0.0);
  EXPECT_EQ(camera.RadialDistortion1(), 0.0);
  EXPECT_EQ(camera.RadialDistortion2(), 0.0);

  // Set parameters to different values.
  camera.SetFocalLength(30000);
  camera.SetAspectRatio(1.1);
  camera.SetSkew(0.1);
  camera.SetPrincipalPoint(300.0, 400.0);
  camera.SetRadialDistortion(0.01, 0.001);

  // Check that the values were updated.
  EXPECT_EQ(camera.FocalLength(), 30000);
  EXPECT_EQ(camera.AspectRatio(), 1.1);
  EXPECT_EQ(camera.Skew(),0.1);
  EXPECT_EQ(camera.PrincipalPointX(), 300.0);
  EXPECT_EQ(camera.PrincipalPointY(), 400.0);
  EXPECT_EQ(camera.RadialDistortion1(), 0.01);
  EXPECT_EQ(camera.RadialDistortion2(), 0.001);
}

// Test to ensure that the camera intrinsics are being set appropriately.
void TestSetFromCameraintrinsicsPrior(const CameraIntrinsicsPrior& prior) {
  const OrthographicCameraModel default_camera;
  OrthographicCameraModel camera;
  camera.SetFromCameraIntrinsicsPriors(prior);

  if (prior.focal_length.is_set) {
    EXPECT_EQ(camera.FocalLength(), prior.focal_length.value[0]);
  } else {
    EXPECT_EQ(camera.FocalLength(), default_camera.FocalLength());
  }

  if (prior.principal_point.is_set) {
    EXPECT_EQ(camera.PrincipalPointX(), prior.principal_point.value[0]);
    EXPECT_EQ(camera.PrincipalPointY(), prior.principal_point.value[1]);
  } else {
    EXPECT_EQ(camera.PrincipalPointX(), default_camera.PrincipalPointX());
    EXPECT_EQ(camera.PrincipalPointY(), default_camera.PrincipalPointY());
  }

  if (prior.aspect_ratio.is_set) {
    EXPECT_EQ(camera.AspectRatio(), prior.aspect_ratio.value[0]);
  } else {
    EXPECT_EQ(camera.AspectRatio(), default_camera.AspectRatio());
  }

  if (prior.skew.is_set) {
    EXPECT_EQ(camera.Skew(), prior.skew.value[0]);
  } else {
    EXPECT_EQ(camera.Skew(), default_camera.Skew());
  }

  if (prior.radial_distortion.is_set) {
    EXPECT_EQ(camera.RadialDistortion1(), prior.radial_distortion.value[0]);
    EXPECT_EQ(camera.RadialDistortion2(), prior.radial_distortion.value[1]);
  } else {
    EXPECT_EQ(camera.RadialDistortion1(), default_camera.RadialDistortion1());
    EXPECT_EQ(camera.RadialDistortion2(), default_camera.RadialDistortion2());
  }
}

// Gradually add one prior at a time and ensure that the method still works. We
// test before and after setting the "is_set" member variable to true to ensure
// that setting the value of priors when is_set=false is handled properly.
TEST(OrthographicCameraModel, SetFromCameraIntrinsicsPriors) {
  CameraIntrinsicsPrior prior;
  prior.focal_length.value[0] = 30000;
  prior.principal_point.value[0] = 400.0;
  prior.principal_point.value[1] = 300.0;
  prior.skew.value[0] = 0.0;
  prior.aspect_ratio.value[0] = 1.001;
  prior.radial_distortion.value[0] = 0.01;
  prior.radial_distortion.value[1] = 0.001;

  TestSetFromCameraintrinsicsPrior(prior);

  prior.focal_length.is_set = true;
  TestSetFromCameraintrinsicsPrior(prior);

  prior.principal_point.is_set = true;
  TestSetFromCameraintrinsicsPrior(prior);

  prior.aspect_ratio.is_set = true;
  TestSetFromCameraintrinsicsPrior(prior);

  prior.skew.is_set = true;
  TestSetFromCameraintrinsicsPrior(prior);

  prior.radial_distortion.is_set = true;
  TestSetFromCameraintrinsicsPrior(prior);
}

TEST(OrthographicCameraModel, GetSubsetFromOptimizeIntrinsicsType) {
  OrthographicCameraModel camera;
  std::vector<int> constant_subset;

  constant_subset =
      camera.GetSubsetFromOptimizeIntrinsicsType(OptimizeIntrinsicsType::NONE);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters());

  // Test that optimizing for magnification works correctly.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::FOCAL_LENGTH);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 1);
  for (size_t i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::FOCAL_LENGTH);
  }

  // Test that optimizing for principal_points works correctly.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::PRINCIPAL_POINTS);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 2);
  for (size_t i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::PRINCIPAL_POINT_X);
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::PRINCIPAL_POINT_Y);
  }

  // Test that optimizing for pixel pitches works correctly.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::ASPECT_RATIO);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 1);
  for (size_t i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::ASPECT_RATIO);
  }

  // Test that optimizing for pixel pitches works correctly.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::SKEW);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 1);
  for (size_t i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::SKEW);
  }

  // Test that optimizing for radial distortion works correctly.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::RADIAL_DISTORTION);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters() - 2);
  for (size_t i = 0; i < constant_subset.size(); i++) {
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::RADIAL_DISTORTION_1);
    EXPECT_NE(constant_subset[i], OrthographicCameraModel::RADIAL_DISTORTION_2);
  }

  // Test that optimizing for tangential distortion does not optimize any
  // parameters.
  constant_subset = camera.GetSubsetFromOptimizeIntrinsicsType(
      OptimizeIntrinsicsType::TANGENTIAL_DISTORTION);
  EXPECT_EQ(constant_subset.size(), camera.NumParameters());
}

// Test the projection functions of the camera model by testing pixels on a grid
// in the entire image to ensure that the effects of lens distortion at the
// edges of the image are modeled correctly.
void ReprojectionTest(const OrthographicCameraModel& camera) {
  static const double kTolerance = 1e-5;
  const double kNormalizedTolerance = kTolerance;
  static const int kImageWidth = 2560;
  static const int kImageHeight = 1920;


  // Ensure the camera -> image -> camera transformation works.
  for (double x = -0.01; x < 0.01; x += 0.001) {
    for (double y = -0.01; y < 0.01; y += 0.001) {
        const Eigen::Vector3d point(x, y, 1.0);
        const Vector2d pixel = camera.CameraToImageCoordinates(point);

        // Get the normalized ray of that pixel.
        const Vector3d p_c = camera.ImageToCameraCoordinates(pixel);

        // Expect the reprojection to be close.
        EXPECT_LT((point - p_c).norm(), kNormalizedTolerance)
            << "gt pixel: " << point.transpose()
            << "\nreprojected pixel: " << p_c.transpose();
    }
  }


  // Ensure the image -> camera -> image transformation works.
  for (double x = 0.0; x < kImageWidth; x += 20.0) {
    for (double y = 0.0; y < kImageHeight; y += 20.0) {
      const Eigen::Vector2d pixel(x, y);
      // Get the normalized ray of that pixel.
      const Vector3d p_c = camera.ImageToCameraCoordinates(pixel);

      // Test the reprojection at several depths.
        // Convert it to a full 3D point in the camera coordinate system.
        const Vector2d reprojected_pixel =
            camera.CameraToImageCoordinates(p_c);

        // Expect the reprojection to be close.
        EXPECT_LT((pixel - reprojected_pixel).norm(), kTolerance)
            << "gt pixel: " << pixel.transpose()
            << "\nreprojected pixel: " << reprojected_pixel.transpose();
    }
  }
}

TEST(OrthographicCameraModel, ReprojectionNoDistortion) {
  static const double kPrincipalPoint[2] = {1180.0, 1010.0};
  // this is actually magnification / pixel_pitch, e.g. 0.08/2e-6
  static const double kFocalLength = 40000.0;
  OrthographicCameraModel camera;
  camera.SetFocalLength(kFocalLength);
  camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
  camera.SetRadialDistortion(0, 0);
  ReprojectionTest(camera);
}

TEST(OrthographicCameraModel, ReprojectionOneDistortion) {
  static const double kPrincipalPoint[2] = {1180.0, 1010.0};
  static const double kFocalLength = 40000.0;
  OrthographicCameraModel camera;
  camera.SetFocalLength(kFocalLength);
  camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
  camera.SetRadialDistortion(1e-4, 0);
  ReprojectionTest(camera);
}

TEST(OrthographicCameraModel, ReprojectionTwoDistortion) {
  static const double kPrincipalPoint[2] = {1180.0, 1010.0};
  static const double kFocalLength = 40000.0;
  OrthographicCameraModel camera;
  camera.SetFocalLength(kFocalLength);
  camera.SetPrincipalPoint(kPrincipalPoint[0], kPrincipalPoint[1]);
  camera.SetRadialDistortion(1e-4, 1e-5);
  ReprojectionTest(camera);
}

}  // namespace theia
