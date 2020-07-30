// Copyright (C) 2016 The Regents of the University of California (Regents).
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

#include "theia/sfm/camera/double_sphere_camera_model.h"

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/camera/projection_matrix_utils.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

DoubleSphereCameraModel::DoubleSphereCameraModel() {
  parameters_.resize(kIntrinsicsSize);
  SetFocalLength(1.0);
  SetPrincipalPoint(0.0, 0.0);
  SetParameter(ASPECT_RATIO, 1.0);
  SetParameter(SKEW, 0.0);
  SetParameter(ALPHA, 0.0);
  SetParameter(XI, 0.5);
}

int DoubleSphereCameraModel::NumParameters() const {return kIntrinsicsSize;}

// Returns the camera model type of the object.
CameraIntrinsicsModelType DoubleSphereCameraModel::Type() const {
  return CameraIntrinsicsModelType::DOUBLE_SPHERE;
}

// Set the intrinsic camera parameters from the priors.
void DoubleSphereCameraModel::SetFromCameraIntrinsicsPriors(
    const CameraIntrinsicsPrior& prior) {
  // Set the focal length.
  if (prior.focal_length.is_set) {
    SetFocalLength(prior.focal_length.value[0]);
  } else if (prior.image_width != 0.0 && prior.image_height != 0.0) {
    SetFocalLength(1.2 * static_cast<double>(std::max(
        prior.image_width, prior.image_height)));
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

  // Set skew if available.
  if (prior.skew.is_set) {
    SetParameter(SKEW, prior.skew.value[0]);
  }

  // Set radial distortion if available.
  if (prior.radial_distortion.is_set) {
    SetParameter(ALPHA, prior.radial_distortion.value[0]);
    SetParameter(XI, prior.radial_distortion.value[1]);
  }
}

CameraIntrinsicsPrior DoubleSphereCameraModel::CameraIntrinsicsPriorFromIntrinsics()
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
  prior.skew.is_set = true;
  prior.skew.value[0] = Skew();
  prior.radial_distortion.is_set = true;
  prior.radial_distortion.value[0] = Alpha();
  prior.radial_distortion.value[1] = Xi();

  return prior;
}

// Returns the indices of the parameters that will be optimized during bundle
// adjustment.
std::vector<int> DoubleSphereCameraModel::GetSubsetFromOptimizeIntrinsicsType(
    const OptimizeIntrinsicsType& intrinsics_to_optimize) const {
  std::vector<int> constant_intrinsics;
  if (intrinsics_to_optimize == OptimizeIntrinsicsType::ALL) {
    return constant_intrinsics;
  }

  if ((intrinsics_to_optimize &
      OptimizeIntrinsicsType::FOCAL_LENGTH) == OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(FOCAL_LENGTH);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::ASPECT_RATIO) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(ASPECT_RATIO);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::SKEW) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(SKEW);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::PRINCIPAL_POINTS) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(PRINCIPAL_POINT_X);
    constant_intrinsics.emplace_back(PRINCIPAL_POINT_Y);
  }
  if ((intrinsics_to_optimize & OptimizeIntrinsicsType::RADIAL_DISTORTION) ==
      OptimizeIntrinsicsType::NONE) {
    constant_intrinsics.emplace_back(ALPHA);
    constant_intrinsics.emplace_back(XI);
  }
  return constant_intrinsics;
}

void DoubleSphereCameraModel::GetCalibrationMatrix(Matrix3d* kmatrix) const {
  IntrinsicsToCalibrationMatrix(parameters_[FOCAL_LENGTH],
                                parameters_[SKEW],
                                parameters_[ASPECT_RATIO],
                                parameters_[PRINCIPAL_POINT_X],
                                parameters_[PRINCIPAL_POINT_Y],
                                kmatrix);
}

void DoubleSphereCameraModel::PrintIntrinsics() const {
  LOG(INFO) << "Camera model type: "
            << CameraIntrinsicsModelTypeToString(Type())
            << "\nFocal length (pixels): " << FocalLength()
            << "\nPrincipal Point (px, py) = (" << PrincipalPointX() << ", "
            << PrincipalPointY() << ")"
            << "\nSkew: " << Skew() << "\nAspect Ratio: " << AspectRatio()
            << "\nAlpha and Xi: " << Alpha() << ", "
            << Xi();
}

// ----------------------- Getter and Setter methods ---------------------- //

void DoubleSphereCameraModel::SetAspectRatio(const double aspect_ratio) {
  CHECK_GT(aspect_ratio, 0.0)
      << "Invalid aspect ratio. Aspect ratio must be greater than 0.0.";
  parameters_[ASPECT_RATIO] = aspect_ratio;
}
double DoubleSphereCameraModel::AspectRatio() const {
  return parameters_[ASPECT_RATIO];
}

void DoubleSphereCameraModel::SetSkew(const double skew) {
  parameters_[SKEW] = skew;
}

double DoubleSphereCameraModel::Skew() const {
  return parameters_[SKEW];
}

void DoubleSphereCameraModel::SetAlphaXiDistortion(
        const double alpha,
        const double xi) {
  parameters_[ALPHA] = alpha;
  parameters_[XI] = xi;
}

double DoubleSphereCameraModel::Alpha() const {
  return parameters_[ALPHA];
}

double DoubleSphereCameraModel::Xi() const {
  return parameters_[XI];
}

}  // namespace theia
