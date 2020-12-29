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

// author: Steffen Urban (urbste@googlemail.com), juli 2020

#ifndef THEIA_SFM_CAMERA_DOUBLE_SPHERE_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_DOUBLE_SPHERE_CAMERA_MODEL_H_

#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <ceres/ceres.h>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/types.h"

namespace theia {

// This class implements the double sphere camera model presented in:
// https://arxiv.org/pdf/1807.08957.pdf, "The Double Sphere Camera Model", V.Usenko et al.

class DoubleSphereCameraModel : public CameraIntrinsicsModel {
 public:
  DoubleSphereCameraModel();
  ~DoubleSphereCameraModel() {}

  static const int kIntrinsicsSize = 7;

  enum InternalParametersIndex{
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    SKEW = 2,
    PRINCIPAL_POINT_X = 3,
    PRINCIPAL_POINT_Y = 4,
    XI = 5, // value range -1, 1
    ALPHA = 6 // value range 0, 1
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
  static void CameraToPixelCoordinates(const T* intrinsic_parameters,
                                       const T* point,
                                       T* pixel);

  // Given a pixel in the image coordinates, remove the effects of camera
  // intrinsics parameters and lens distortion to produce a point in the camera
  // coordinate system. The point output by this method is effectively a ray in
  // the direction of the pixel in the camera coordinate system.
  template <typename T>
  static void PixelToCameraCoordinates(const T* intrinsic_parameters,
                                       const T* pixel,
                                       T* point);

  // Given an undistorted point, apply lens distortion to the point to get a
  // distorted point. The type of distortion (i.e. radial, tangential, fisheye,
  // etc.) will depend on the camera intrinsics model.
  template <typename T>
  static void DistortPoint(const T* intrinsic_parameters,
                           const T* undistorted_point,
                           T* distorted_point);

  // Given a distorted point, apply lens undistortion to the point to get an
  // undistorted point. The type of distortion (i.e. radial, tangential,
  // fisheye, etc.) will depend on the camera intrinsics model.
  template <typename T>
  static void UndistortPoint(const T* intrinsic_parameters,
                             const T* distorted_point,
                             T* undistorted_point);

  // ----------------------- Getter and Setter methods ---------------------- //
  void SetAspectRatio(const double aspect_ratio);
  double AspectRatio() const;

  void SetSkew(const double skew);
  double Skew() const;

  void SetAlphaXiDistortion(const double alpha,
                            const double xi);
  double Alpha() const;
  double Xi() const;

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
void DoubleSphereCameraModel::CameraToPixelCoordinates(
    const T* intrinsic_parameters, const T* point, T* pixel) {
  // Get normalized pixel projection at image plane depth = 1.

  // Apply radial distortion.
  T distorted_pixel[2];
  DoubleSphereCameraModel::DistortPoint(intrinsic_parameters,
                                   point,
                                   distorted_pixel);

  // Apply calibration parameters to transform normalized units into pixels.
  const T& focal_length =
      intrinsic_parameters[DoubleSphereCameraModel::FOCAL_LENGTH];
  const T& skew = intrinsic_parameters[DoubleSphereCameraModel::SKEW];
  const T& aspect_ratio =
      intrinsic_parameters[DoubleSphereCameraModel::ASPECT_RATIO];
  const T& principal_point_x =
      intrinsic_parameters[DoubleSphereCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[DoubleSphereCameraModel::PRINCIPAL_POINT_Y];

  pixel[0] = focal_length * distorted_pixel[0] + skew * distorted_pixel[1] +
             principal_point_x;
  pixel[1] = focal_length * aspect_ratio * distorted_pixel[1] +
             principal_point_y;
}

template <typename T>
void DoubleSphereCameraModel::PixelToCameraCoordinates(const T* intrinsic_parameters,
                                                  const T* pixel,
                                                  T* point) {
  const T& focal_length =
      intrinsic_parameters[DoubleSphereCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[DoubleSphereCameraModel::ASPECT_RATIO];
  const T& focal_length_y = focal_length * aspect_ratio;
  const T& skew = intrinsic_parameters[DoubleSphereCameraModel::SKEW];
  const T& principal_point_x =
      intrinsic_parameters[DoubleSphereCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[DoubleSphereCameraModel::PRINCIPAL_POINT_Y];

  // Normalize the y coordinate first.
  T distorted_point[2];
  distorted_point[1] = (pixel[1] - principal_point_y) / focal_length_y;
  distorted_point[0] =
      (pixel[0] - principal_point_x - distorted_point[1] * skew) / focal_length;

  // Undo the distortion.
  DoubleSphereCameraModel::UndistortPoint(intrinsic_parameters,
                                     distorted_point,
                                     point);
}

template <typename T>
void DoubleSphereCameraModel::DistortPoint(const T* intrinsic_parameters,
                                      const T* undistorted_point,
                                      T* distorted_point) {
  const T& alpha =
      intrinsic_parameters[DoubleSphereCameraModel::ALPHA];
  const T& xi =
      intrinsic_parameters[DoubleSphereCameraModel::XI];

  const T xx = undistorted_point[0] * undistorted_point[0];
  const T yy = undistorted_point[1] * undistorted_point[1];
  const T zz = undistorted_point[2] * undistorted_point[2];

  const T r2 = xx + yy;

  const T d1_2 = r2 + zz;
  const T d1 = sqrt(d1_2);

  const T k = xi * d1 + undistorted_point[2];
  const T kk = k * k;

  const T d2_2 = r2 + kk;
  const T d2 = sqrt(d2_2);

  const T norm = alpha * d2 + (T(1) - alpha) * k;

  distorted_point[0] = undistorted_point[0] / norm;
  distorted_point[1] = undistorted_point[1] / norm;
}

template <typename T>
void DoubleSphereCameraModel::UndistortPoint(const T* intrinsic_parameters,
                                        const T* distorted_point,
                                        T* undistorted_point) {
    const T& alpha =
        intrinsic_parameters[DoubleSphereCameraModel::ALPHA];
    const T& xi =
        intrinsic_parameters[DoubleSphereCameraModel::XI];

    const T r2 = distorted_point[0] * distorted_point[0] +
                 distorted_point[1] * distorted_point[1];

    const T xi2_2 = alpha * alpha;
    const T xi1_2 = xi * xi;

    const T sqrt2 = ceres::sqrt(T(1) - (T(2) * alpha - T(1)) * r2);

    const T norm2 = alpha * sqrt2 + T(1) - alpha;

    const T mz = (T(1) - xi2_2 * r2) / norm2;
    const T mz2 = mz * mz;

    const T norm1 = mz2 + r2;
    const T sqrt1 = ceres::sqrt(mz2 + (T(1) - xi1_2) * r2);
    const T k = (mz * xi + sqrt1) / norm1;


    undistorted_point[0] = k * distorted_point[0];
    undistorted_point[1] = k * distorted_point[1];
    undistorted_point[2] = k * mz - xi;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::DoubleSphereCameraModel, 1)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::DoubleSphereCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::DoubleSphereCameraModel)

#endif  // THEIA_SFM_CAMERA_DOUBLE_SPHERE_MODEL_H_
