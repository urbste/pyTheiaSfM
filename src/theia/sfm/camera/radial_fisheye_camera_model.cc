// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#include "theia/sfm/camera/radial_fisheye_camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <algorithm>

#include "theia/sfm/camera/projection_matrix_utils.h"
#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

using Eigen::Matrix3d;

RadialFisheyeCameraModel::RadialFisheyeCameraModel() {
  parameters_.resize(kIntrinsicsSize);
  SetFocalLength(1.0);
  SetPrincipalPoint(0.0, 0.0);
  SetParameter(ASPECT_RATIO, 1.0);
  SetParameter(RADIAL_DISTORTION_1, 0.0);
  SetParameter(RADIAL_DISTORTION_2, 0.0);
}

int RadialFisheyeCameraModel::NumParameters() const { return kIntrinsicsSize; }

CameraIntrinsicsModelType RadialFisheyeCameraModel::Type() const {
  return CameraIntrinsicsModelType::RADIAL_FISHEYE;
}

void RadialFisheyeCameraModel::SetFromCameraIntrinsicsPriors(
    const CameraIntrinsicsPrior& prior) {
  // Set the focal length.
  if (prior.focal_length.is_set) {
    SetFocalLength(prior.focal_length.value[0]);
  } else if (prior.image_width != 0.0 && prior.image_height != 0.0) {
    // Same initialization as the other fisheye models.
    static const double kFocalLengthScaleFactor = 0.4;
    SetFocalLength(
        kFocalLengthScaleFactor *
        static_cast<double>(std::max(prior.image_width, prior.image_height)));
  }

  // Set the principal point.
  if (prior.principal_point.is_set) {
    SetPrincipalPoint(prior.principal_point.value[0],
                      prior.principal_point.value[1]);
  } else if (prior.image_width != 0.0 && prior.image_height != 0.0) {
    SetPrincipalPoint(prior.image_width / 2.0, prior.image_height / 2.0);
  }

  // Set aspect ratio if available.
  if (prior.aspect_ratio.is_set) {
    SetParameter(ASPECT_RATIO, prior.aspect_ratio.value[0]);
  }

  // Set radial distortion if available.
  if (prior.radial_distortion.is_set) {
    SetParameter(RADIAL_DISTORTION_1, prior.radial_distortion.value[0]);
    SetParameter(RADIAL_DISTORTION_2, prior.radial_distortion.value[1]);
  }
}

CameraIntrinsicsPrior RadialFisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics()
    const {
  CameraIntrinsicsPrior prior;
  prior.camera_intrinsics_model_type =
      CameraIntrinsicsModelTypeToString(Type());
  prior.focal_length.is_set = true;
  prior.focal_length.value[0] = FocalLength();
  prior.principal_point.is_set = true;
  prior.principal_point.value[0] = PrincipalPointX();
  prior.principal_point.value[1] = PrincipalPointY();
  prior.aspect_ratio.is_set = true;
  prior.aspect_ratio.value[0] = AspectRatio();
  prior.radial_distortion.is_set = true;
  prior.radial_distortion.value[0] = RadialDistortion1();
  prior.radial_distortion.value[1] = RadialDistortion2();
  return prior;
}

std::vector<int> RadialFisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType(
    const OptimizeIntrinsicsType& intrinsics_to_optimize) const {
  std::vector<int> constant_intrinsics;
  if (intrinsics_to_optimize == OptimizeIntrinsicsType::ALL) {
    return constant_intrinsics;
  }

  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::FOCAL_LENGTH) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(FOCAL_LENGTH);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::ASPECT_RATIO) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(ASPECT_RATIO);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::PRINCIPAL_POINTS) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(PRINCIPAL_POINT_X);
    constant_intrinsics.emplace_back(PRINCIPAL_POINT_Y);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::RADIAL_DISTORTION) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_1);
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_2);
  }
  return constant_intrinsics;
}

void RadialFisheyeCameraModel::GetCalibrationMatrix(Matrix3d* kmatrix) const {
  IntrinsicsToCalibrationMatrix(parameters_[FOCAL_LENGTH],
                                0.0,
                                parameters_[ASPECT_RATIO],
                                parameters_[PRINCIPAL_POINT_X],
                                parameters_[PRINCIPAL_POINT_Y],
                                kmatrix);
}

void RadialFisheyeCameraModel::PrintIntrinsics() const {
  LOG(INFO) << "Camera model type: "
            << CameraIntrinsicsModelTypeToString(Type())
            << "\nFocal length (pixels): " << FocalLength()
            << "\nPrincipal Point (px, py) = (" << PrincipalPointX() << ", "
            << PrincipalPointY() << ")"
            << "\nAspect Ratio: " << AspectRatio()
            << "\nRadialDistortion: " << RadialDistortion1() << ", "
            << RadialDistortion2();
}

// ----------------------- Getter and Setter methods ---------------------- //

void RadialFisheyeCameraModel::SetAspectRatio(const double aspect_ratio) {
  parameters_[ASPECT_RATIO] = aspect_ratio;
}

double RadialFisheyeCameraModel::AspectRatio() const {
  return parameters_[ASPECT_RATIO];
}

void RadialFisheyeCameraModel::SetRadialDistortion(
    const double radial_distortion_1, const double radial_distortion_2) {
  parameters_[RADIAL_DISTORTION_1] = radial_distortion_1;
  parameters_[RADIAL_DISTORTION_2] = radial_distortion_2;
}

double RadialFisheyeCameraModel::RadialDistortion1() const {
  return parameters_[RADIAL_DISTORTION_1];
}

double RadialFisheyeCameraModel::RadialDistortion2() const {
  return parameters_[RADIAL_DISTORTION_2];
}

}  // namespace theia
