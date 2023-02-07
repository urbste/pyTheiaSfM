// Please contact the author of this library if you have any questions.
// Author: Steffen Urban, (urbste@gmail.com), Feb 2023

#include "theia/sfm/camera/equirectangular_camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/camera/projection_matrix_utils.h"
#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

EquirectangularCameraModel::EquirectangularCameraModel() {
  parameters_.resize(kIntrinsicsSize);
  SetColsRows(1000, 1000);
}

int EquirectangularCameraModel::NumParameters() const { return kIntrinsicsSize; }

// Returns the camera model type of the object.
CameraIntrinsicsModelType EquirectangularCameraModel::Type() const {
  return CameraIntrinsicsModelType::EQUIRECTANGULAR;
}

// Set the intrinsic camera parameters from the priors.
void EquirectangularCameraModel::SetFromCameraIntrinsicsPriors(
    const CameraIntrinsicsPrior& prior) {
  // Set the focal length.
  if (prior.image_width != 0.0 && prior.image_height != 0.0) {
    SetColsRows(prior.image_width,prior.image_height);
  }
}

CameraIntrinsicsPrior EquirectangularCameraModel::CameraIntrinsicsPriorFromIntrinsics()
    const {
  CameraIntrinsicsPrior prior;
  prior.camera_intrinsics_model_type =
      CameraIntrinsicsModelTypeToString(Type());
  prior.image_width = (int)Cols();
  prior.image_height = (int)Rows();

  return prior;
}

// Returns the indices of the parameters that will be optimized during bundle
// adjustment.
std::vector<int> EquirectangularCameraModel::GetSubsetFromOptimizeIntrinsicsType(
    const OptimizeIntrinsicsType& intrinsics_to_optimize) const {
  std::vector<int> constant_intrinsics;
  if (intrinsics_to_optimize == OptimizeIntrinsicsType::ALL) {
    return constant_intrinsics;
  }
  return constant_intrinsics;
}

void EquirectangularCameraModel::GetCalibrationMatrix(Matrix3d* kmatrix) const {
  kmatrix->setIdentity();
}

void EquirectangularCameraModel::PrintIntrinsics() const {
  LOG(INFO) << "Camera model type: "
            << CameraIntrinsicsModelTypeToString(Type())
            << "\nCols: " << Cols() <<" Rows: "<<Rows();
}

// ----------------------- Getter and Setter methods ---------------------- //

void EquirectangularCameraModel::SetColsRows(unsigned int cols, unsigned int rows) {
  parameters_[ROWS] = rows;
  parameters_[COLS] = cols;
}

unsigned int EquirectangularCameraModel::Cols() const {
  return parameters_[COLS];
}

unsigned int EquirectangularCameraModel::Rows() const {
  return parameters_[ROWS];
}

}  // namespace theia
