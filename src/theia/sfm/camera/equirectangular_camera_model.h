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

// Author: Steffen Urban (urbste@googlemail.com), Feb. 2023

#ifndef THEIA_SFM_CAMERA_EQUIRECTANGULAR_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_EQUIRECTANGULAR_CAMERA_MODEL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <ceres/ceres.h>
#include <stdint.h>
#include <vector>

#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/types.h"

namespace theia {

class EquirectangularCameraModel : public CameraIntrinsicsModel {
 public:
  EquirectangularCameraModel();
  ~EquirectangularCameraModel() {}

  static const int kIntrinsicsSize = 2;

  enum InternalParametersIndex {
    ROWS = 0,
    COLS = 1
  };

  int NumParameters() const override;

  // Returns the camera model type of the object.
  CameraIntrinsicsModelType Type() const override;

  // Set the intrinsic camera parameters from the priors.
  void SetFromCameraIntrinsicsPriors(
      const CameraIntrinsicsPrior& prior) override;

  // Return a CameraIntrinsicsPrior that can be used to initialize a camera with
  // the same parameters with the SetFromCameraIntrinsicsPriors method.
  CameraIntrinsicsPrior CameraIntrinsicsPriorFromIntrinsics() const override;

  // Returns the indices of the parameters that will be optimized during bundle
  // adjustment.
  std::vector<int> GetSubsetFromOptimizeIntrinsicsType(
      const OptimizeIntrinsicsType& intrinsics_to_optimize) const override;

  // Returns the calibration matrix in the form specified above.
  void GetCalibrationMatrix(Eigen::Matrix3d* kmatrix) const override;

  // Prints the camera intrinsics in a human-readable format.
  void PrintIntrinsics() const override;

  // Given a point in the camera coordinate system, apply the camera intrinsics
  // (e.g., focal length, principal point, distortion) to transform the point
  // into pixel coordinates.
  template <typename T>
  static bool CameraToPixelCoordinates(const T* intrinsic_parameters,
                                       const T* point,
                                       T* pixel);

  // Given a pixel in the image coordinates, remove the effects of camera
  // intrinsics parameters and lens distortion to produce a point in the camera
  // coordinate system. The point output by this method is effectively a ray in
  // the direction of the pixel in the camera coordinate system.
  template <typename T>
  static bool PixelToCameraCoordinates(const T* intrinsic_parameters,
                                       const T* pixel,
                                       T* point);

  // Given an undistorted point, apply lens distortion to the point to get a
  // distorted point. The type of distortion (i.e. radial, tangential, fisheye,
  // etc.) will depend on the camera intrinsics model.
  template <typename T>
  static bool DistortPoint(const T* intrinsic_parameters,
                           const T* undistorted_point,
                           T* distorted_point);

  // Given a distorted point, apply lens undistortion to the point to get an
  // undistorted point. The type of distortion (i.e. radial, tangential,
  // fisheye, etc.) will depend on the camera intrinsics model.
  template <typename T>
  static bool UndistortPoint(const T* intrinsic_parameters,
                             const T* distorted_point,
                             T* undistorted_point);

  // ----------------------- Getter and Setter methods ---------------------- //

  void SetColsRows(const unsigned int cols, const unsigned int rows);
  unsigned int Rows() const;
  unsigned int Cols() const;

 private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    if (version > 0) {
      ar(cereal::base_class<CameraIntrinsicsModel>(this));
    } else {
      CHECK_EQ(this->parameters_.size(), NumParameters());
      ar(cereal::binary_data(this->parameters_.data(),
                             sizeof(double) * NumParameters()));
    }
  }
};

template <typename T>
bool EquirectangularCameraModel::CameraToPixelCoordinates(const T* intrinsic_parameters,
                                                  const T* point,
                                                  T* pixel) {

    const T& rows =
        intrinsic_parameters[EquirectangularCameraModel::ROWS];
    const T& cols = intrinsic_parameters[EquirectangularCameraModel::COLS];

  // convert to unit polar coordinates
  const T latitude = -ceres::asin(point[1]);
  const T longitude = ceres::atan2(point[0], point[2]);

  // convert to pixel image coordinated
  pixel[0] = cols * (T(0.5) + longitude / (T(2.0) * T(M_PI)));
  pixel[1] = rows * (T(0.5) - latitude / T(M_PI));

  return true;
}

template <typename T>
bool EquirectangularCameraModel::PixelToCameraCoordinates(const T* intrinsic_parameters,
                                                  const T* pixel,
                                                  T* point) {
  const T& rows =
      intrinsic_parameters[EquirectangularCameraModel::ROWS];
  const T& cols = intrinsic_parameters[EquirectangularCameraModel::COLS];
  // convert to unit polar coordinates
  const T longitude = (pixel[0] / rows - T(0.5)) * (T(2.0) * T(M_PI));
  const T latitude = -(pixel[1] / cols - T(0.5)) * T(M_PI);

  // convert to equirectangular coordinates
  point[0] = ceres::cos(latitude) * ceres::sin(longitude);
  point[1] = -ceres::sin(latitude);
  point[2] = ceres::cos(latitude) * ceres::cos(longitude);

  return true;
}

template <typename T>
bool EquirectangularCameraModel::DistortPoint(const T* intrinsic_parameters,
                                      const T* undistorted_point,
                                      T* distorted_point) {
  return true;
}

template <typename T>
bool EquirectangularCameraModel::UndistortPoint(const T* intrinsic_parameters,
                                        const T* distorted_point,
                                        T* undistorted_point) {
  return true;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::EquirectangularCameraModel, 1)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::EquirectangularCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::EquirectangularCameraModel)

#endif  // THEIA_SFM_CAMERA_PINHOLE_CAMERA_MODEL_H_
