// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <algorithm>

#include "theia/sfm/camera/projection_matrix_utils.h"
#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

using Eigen::Matrix3d;

RadTanThinPrismFisheyeCameraModel::RadTanThinPrismFisheyeCameraModel() {
  parameters_.resize(kIntrinsicsSize);
  SetFocalLength(1.0);
  SetPrincipalPoint(0.0, 0.0);
  SetParameter(ASPECT_RATIO, 1.0);
  for (int i = RADIAL_DISTORTION_1; i < kIntrinsicsSize; ++i) {
    SetParameter(i, 0.0);
  }
}

int RadTanThinPrismFisheyeCameraModel::NumParameters() const {
  return kIntrinsicsSize;
}

CameraIntrinsicsModelType RadTanThinPrismFisheyeCameraModel::Type() const {
  return CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE;
}

void RadTanThinPrismFisheyeCameraModel::SetFromCameraIntrinsicsPriors(
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

  if (prior.thin_prism_distortion.is_set) {
    SetThinPrismDistortion(prior.thin_prism_distortion.value[0],
                           prior.thin_prism_distortion.value[1],
                           prior.thin_prism_distortion.value[2],
                           prior.thin_prism_distortion.value[3]);
  }
}

CameraIntrinsicsPrior
RadTanThinPrismFisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics() const {
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
  prior.thin_prism_distortion.is_set = true;
  prior.thin_prism_distortion.value[0] = ThinPrismDistortion1();
  prior.thin_prism_distortion.value[1] = ThinPrismDistortion2();
  prior.thin_prism_distortion.value[2] = ThinPrismDistortion3();
  prior.thin_prism_distortion.value[3] = ThinPrismDistortion4();
  return prior;
}

std::vector<int>
RadTanThinPrismFisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType(
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
    for (int i = RADIAL_DISTORTION_1; i <= RADIAL_DISTORTION_6; ++i) {
      constant_intrinsics.emplace_back(i);
    }
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::TANGENTIAL_DISTORTION) ==
      OptimizeIntrinsicsType::NONE) {
    for (int i = TANGENTIAL_DISTORTION_1; i <= THIN_PRISM_DISTORTION_4; ++i) {
      constant_intrinsics.emplace_back(i);
    }
  }
  return constant_intrinsics;
}

void RadTanThinPrismFisheyeCameraModel::GetCalibrationMatrix(
    Matrix3d* kmatrix) const {
  IntrinsicsToCalibrationMatrix(parameters_[FOCAL_LENGTH],
                                0.0,
                                parameters_[ASPECT_RATIO],
                                parameters_[PRINCIPAL_POINT_X],
                                parameters_[PRINCIPAL_POINT_Y],
                                kmatrix);
}

void RadTanThinPrismFisheyeCameraModel::PrintIntrinsics() const {
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
            << TangentialDistortion2() << "\nThinPrismDistortion: "
            << ThinPrismDistortion1() << ", " << ThinPrismDistortion2() << ", "
            << ThinPrismDistortion3() << ", " << ThinPrismDistortion4();
}

// ----------------------- Getter and Setter methods ---------------------- //

void RadTanThinPrismFisheyeCameraModel::SetAspectRatio(
    const double aspect_ratio) {
  parameters_[ASPECT_RATIO] = aspect_ratio;
}

double RadTanThinPrismFisheyeCameraModel::AspectRatio() const {
  return parameters_[ASPECT_RATIO];
}

void RadTanThinPrismFisheyeCameraModel::SetRadialDistortion(
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

double RadTanThinPrismFisheyeCameraModel::RadialDistortion1() const {
  return parameters_[RADIAL_DISTORTION_1];
}

double RadTanThinPrismFisheyeCameraModel::RadialDistortion2() const {
  return parameters_[RADIAL_DISTORTION_2];
}

double RadTanThinPrismFisheyeCameraModel::RadialDistortion3() const {
  return parameters_[RADIAL_DISTORTION_3];
}

double RadTanThinPrismFisheyeCameraModel::RadialDistortion4() const {
  return parameters_[RADIAL_DISTORTION_4];
}

double RadTanThinPrismFisheyeCameraModel::RadialDistortion5() const {
  return parameters_[RADIAL_DISTORTION_5];
}

double RadTanThinPrismFisheyeCameraModel::RadialDistortion6() const {
  return parameters_[RADIAL_DISTORTION_6];
}

void RadTanThinPrismFisheyeCameraModel::SetTangentialDistortion(
    const double tangential_distortion_1,
    const double tangential_distortion_2) {
  parameters_[TANGENTIAL_DISTORTION_1] = tangential_distortion_1;
  parameters_[TANGENTIAL_DISTORTION_2] = tangential_distortion_2;
}

double RadTanThinPrismFisheyeCameraModel::TangentialDistortion1() const {
  return parameters_[TANGENTIAL_DISTORTION_1];
}

double RadTanThinPrismFisheyeCameraModel::TangentialDistortion2() const {
  return parameters_[TANGENTIAL_DISTORTION_2];
}

void RadTanThinPrismFisheyeCameraModel::SetThinPrismDistortion(
    const double thin_prism_distortion_1,
    const double thin_prism_distortion_2,
    const double thin_prism_distortion_3,
    const double thin_prism_distortion_4) {
  parameters_[THIN_PRISM_DISTORTION_1] = thin_prism_distortion_1;
  parameters_[THIN_PRISM_DISTORTION_2] = thin_prism_distortion_2;
  parameters_[THIN_PRISM_DISTORTION_3] = thin_prism_distortion_3;
  parameters_[THIN_PRISM_DISTORTION_4] = thin_prism_distortion_4;
}

double RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion1() const {
  return parameters_[THIN_PRISM_DISTORTION_1];
}

double RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion2() const {
  return parameters_[THIN_PRISM_DISTORTION_2];
}

double RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion3() const {
  return parameters_[THIN_PRISM_DISTORTION_3];
}

double RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion4() const {
  return parameters_[THIN_PRISM_DISTORTION_4];
}

}  // namespace theia
