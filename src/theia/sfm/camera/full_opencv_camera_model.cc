// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#include "theia/sfm/camera/full_opencv_camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <algorithm>

#include "theia/sfm/camera/projection_matrix_utils.h"
#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

using Eigen::Matrix3d;

FullOpenCVCameraModel::FullOpenCVCameraModel() {
  parameters_.resize(kIntrinsicsSize);
  SetFocalLength(1.0);
  SetPrincipalPoint(0.0, 0.0);
  SetParameter(ASPECT_RATIO, 1.0);
  for (int i = RADIAL_DISTORTION_1; i < kIntrinsicsSize; ++i) {
    SetParameter(i, 0.0);
  }
}

int FullOpenCVCameraModel::NumParameters() const { return kIntrinsicsSize; }

CameraIntrinsicsModelType FullOpenCVCameraModel::Type() const {
  return CameraIntrinsicsModelType::FULL_OPENCV;
}

void FullOpenCVCameraModel::SetFromCameraIntrinsicsPriors(
    const CameraIntrinsicsPrior& prior) {
  // Set the focal length.
  if (prior.focal_length.is_set) {
    SetFocalLength(prior.focal_length.value[0]);
  } else if (prior.image_width != 0.0 && prior.image_height != 0.0) {
    SetFocalLength(1.2 * static_cast<double>(
                             std::max(prior.image_width, prior.image_height)));
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

  // The first four radial coefficients come from radial_distortion. The full
  // set of six comes from radial_distortion_6 and takes precedence.
  if (prior.radial_distortion.is_set) {
    SetParameter(RADIAL_DISTORTION_1, prior.radial_distortion.value[0]);
    SetParameter(RADIAL_DISTORTION_2, prior.radial_distortion.value[1]);
    SetParameter(RADIAL_DISTORTION_3, prior.radial_distortion.value[2]);
    SetParameter(RADIAL_DISTORTION_4, prior.radial_distortion.value[3]);
  }
  if (prior.radial_distortion_6.is_set) {
    SetRadialDistortion(prior.radial_distortion_6.value[0],
                        prior.radial_distortion_6.value[1],
                        prior.radial_distortion_6.value[2],
                        prior.radial_distortion_6.value[3],
                        prior.radial_distortion_6.value[4],
                        prior.radial_distortion_6.value[5]);
  }

  if (prior.tangential_distortion.is_set) {
    SetTangentialDistortion(prior.tangential_distortion.value[0],
                            prior.tangential_distortion.value[1]);
  }
}

CameraIntrinsicsPrior FullOpenCVCameraModel::CameraIntrinsicsPriorFromIntrinsics()
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
  prior.radial_distortion_6.is_set = true;
  prior.radial_distortion_6.value[0] = RadialDistortion1();
  prior.radial_distortion_6.value[1] = RadialDistortion2();
  prior.radial_distortion_6.value[2] = RadialDistortion3();
  prior.radial_distortion_6.value[3] = RadialDistortion4();
  prior.radial_distortion_6.value[4] = RadialDistortion5();
  prior.radial_distortion_6.value[5] = RadialDistortion6();
  prior.tangential_distortion.is_set = true;
  prior.tangential_distortion.value[0] = TangentialDistortion1();
  prior.tangential_distortion.value[1] = TangentialDistortion2();
  return prior;
}

std::vector<int> FullOpenCVCameraModel::GetSubsetFromOptimizeIntrinsicsType(
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
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_3);
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_4);
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_5);
    constant_intrinsics.emplace_back(RADIAL_DISTORTION_6);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::TANGENTIAL_DISTORTION) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(TANGENTIAL_DISTORTION_1);
    constant_intrinsics.emplace_back(TANGENTIAL_DISTORTION_2);
  }
  return constant_intrinsics;
}

void FullOpenCVCameraModel::GetCalibrationMatrix(Matrix3d* kmatrix) const {
  IntrinsicsToCalibrationMatrix(parameters_[FOCAL_LENGTH],
                                0.0,
                                parameters_[ASPECT_RATIO],
                                parameters_[PRINCIPAL_POINT_X],
                                parameters_[PRINCIPAL_POINT_Y],
                                kmatrix);
}

void FullOpenCVCameraModel::PrintIntrinsics() const {
  LOG(INFO) << "Camera model type: "
            << CameraIntrinsicsModelTypeToString(Type())
            << "\nFocal length (pixels): " << FocalLength()
            << "\nPrincipal Point (px, py) = (" << PrincipalPointX() << ", "
            << PrincipalPointY() << ")"
            << "\nAspect Ratio: " << AspectRatio() << "\nRadialDistortion: "
            << RadialDistortion1() << ", " << RadialDistortion2() << ", "
            << RadialDistortion3() << ", " << RadialDistortion4() << ", "
            << RadialDistortion5() << ", " << RadialDistortion6()
            << "\nTangentialDistortion: " << TangentialDistortion1() << ", "
            << TangentialDistortion2();
}

// ----------------------- Getter and Setter methods ---------------------- //

void FullOpenCVCameraModel::SetAspectRatio(const double aspect_ratio) {
  parameters_[ASPECT_RATIO] = aspect_ratio;
}

double FullOpenCVCameraModel::AspectRatio() const {
  return parameters_[ASPECT_RATIO];
}

void FullOpenCVCameraModel::SetRadialDistortion(
    const double radial_distortion_1,
    const double radial_distortion_2,
    const double radial_distortion_3,
    const double radial_distortion_4,
    const double radial_distortion_5,
    const double radial_distortion_6) {
  parameters_[RADIAL_DISTORTION_1] = radial_distortion_1;
  parameters_[RADIAL_DISTORTION_2] = radial_distortion_2;
  parameters_[RADIAL_DISTORTION_3] = radial_distortion_3;
  parameters_[RADIAL_DISTORTION_4] = radial_distortion_4;
  parameters_[RADIAL_DISTORTION_5] = radial_distortion_5;
  parameters_[RADIAL_DISTORTION_6] = radial_distortion_6;
}

double FullOpenCVCameraModel::RadialDistortion1() const {
  return parameters_[RADIAL_DISTORTION_1];
}

double FullOpenCVCameraModel::RadialDistortion2() const {
  return parameters_[RADIAL_DISTORTION_2];
}

double FullOpenCVCameraModel::RadialDistortion3() const {
  return parameters_[RADIAL_DISTORTION_3];
}

double FullOpenCVCameraModel::RadialDistortion4() const {
  return parameters_[RADIAL_DISTORTION_4];
}

double FullOpenCVCameraModel::RadialDistortion5() const {
  return parameters_[RADIAL_DISTORTION_5];
}

double FullOpenCVCameraModel::RadialDistortion6() const {
  return parameters_[RADIAL_DISTORTION_6];
}

void FullOpenCVCameraModel::SetTangentialDistortion(
    const double tangential_distortion_1,
    const double tangential_distortion_2) {
  parameters_[TANGENTIAL_DISTORTION_1] = tangential_distortion_1;
  parameters_[TANGENTIAL_DISTORTION_2] = tangential_distortion_2;
}

double FullOpenCVCameraModel::TangentialDistortion1() const {
  return parameters_[TANGENTIAL_DISTORTION_1];
}

double FullOpenCVCameraModel::TangentialDistortion2() const {
  return parameters_[TANGENTIAL_DISTORTION_2];
}

}  // namespace theia
