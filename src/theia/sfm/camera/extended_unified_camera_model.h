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

// author: Steffen Urban (urbste@googlemail.com), December 2020

#ifndef THEIA_SFM_CAMERA_EXTENDED_UNIFIED_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_EXTENDED_UNIFIED_CAMERA_MODEL_H_

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

// This class implements the extended unified camera model:
// https://arxiv.org/pdf/1807.08957.pdf, "The Double Sphere Camera Model",
// V.Usenko et al.

class ExtendedUnifiedCameraModel : public CameraIntrinsicsModel {
public:
  ExtendedUnifiedCameraModel();
  ~ExtendedUnifiedCameraModel() {}

  static const int kIntrinsicsSize = 7;

  enum InternalParametersIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    SKEW = 2,
    PRINCIPAL_POINT_X = 3,
    PRINCIPAL_POINT_Y = 4,
    ALPHA = 5, // value range 0, 1
    BETA = 6   // value range > 0
  };

  int NumParameters() const override;

  // Returns the camera model type of the object.
  CameraIntrinsicsModelType Type() const override;

  // Set the intrinsic camera parameters from the priors.
  void
  SetFromCameraIntrinsicsPriors(const CameraIntrinsicsPrior &prior) override;

  // Return a CameraIntrinsicsPrior that can be used to initialize a camera with
  // the same parameters with the SetFromCameraIntrinsicsPriors method.
  CameraIntrinsicsPrior CameraIntrinsicsPriorFromIntrinsics() const override;

  // Returns the indices of the parameters that will be optimized during bundle
  // adjustment.
  std::vector<int> GetSubsetFromOptimizeIntrinsicsType(
      const OptimizeIntrinsicsType &intrinsics_to_optimize) const override;

  // Returns the calibration matrix in the form specified above.
  void GetCalibrationMatrix(Eigen::Matrix3d *kmatrix) const override;

  // Prints the camera intrinsics in a human-readable format.
  void PrintIntrinsics() const override;

  // Given a point in the camera coordinate system, apply the camera intrinsics
  // (e.g., focal length, principal point, distortion) to transform the point
  // into pixel coordinates.
  template <typename T>
  static bool CameraToPixelCoordinates(const T *intrinsic_parameters,
                                       const T *point, T *pixel);

  // Given a pixel in the image coordinates, remove the effects of camera
  // intrinsics parameters and lens distortion to produce a point in the camera
  // coordinate system. The point output by this method is effectively a ray in
  // the direction of the pixel in the camera coordinate system.
  template <typename T>
  static bool PixelToCameraCoordinates(const T *intrinsic_parameters,
                                       const T *pixel, T *point);

  // Given an undistorted point, apply lens distortion to the point to get a
  // distorted point. The type of distortion (i.e. radial, tangential, fisheye,
  // etc.) will depend on the camera intrinsics model.
  template <typename T>
  static bool DistortPoint(const T *intrinsic_parameters,
                           const T *undistorted_point, T *distorted_point);

  // Given a distorted point, apply lens undistortion to the point to get an
  // undistorted point. The type of distortion (i.e. radial, tangential,
  // fisheye, etc.) will depend on the camera intrinsics model.
  template <typename T>
  static bool UndistortPoint(const T *intrinsic_parameters,
                             const T *distorted_point, T *undistorted_point);

  // ----------------------- Getter and Setter methods ---------------------- //
  void SetAspectRatio(const double aspect_ratio);
  double AspectRatio() const;

  void SetSkew(const double skew);
  double Skew() const;

  void SetAlphaBetaDistortion(const double alpha, const double beta);
  double Alpha() const;
  double Beta() const;

private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive &ar, const std::uint32_t version) { // NOLINT
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
bool ExtendedUnifiedCameraModel::CameraToPixelCoordinates(
    const T *intrinsic_parameters, const T *point, T *pixel) {
  // Get normalized pixel projection at image plane depth = 1.

  // Apply radial distortion.
  T distorted_pixel[2];
  bool result = ExtendedUnifiedCameraModel::DistortPoint(
      intrinsic_parameters, point, distorted_pixel);

  // Apply calibration parameters to transform normalized units into pixels.
  const T &focal_length =
      intrinsic_parameters[ExtendedUnifiedCameraModel::FOCAL_LENGTH];
  const T &skew = intrinsic_parameters[ExtendedUnifiedCameraModel::SKEW];
  const T &aspect_ratio =
      intrinsic_parameters[ExtendedUnifiedCameraModel::ASPECT_RATIO];
  const T &principal_point_x =
      intrinsic_parameters[ExtendedUnifiedCameraModel::PRINCIPAL_POINT_X];
  const T &principal_point_y =
      intrinsic_parameters[ExtendedUnifiedCameraModel::PRINCIPAL_POINT_Y];

  pixel[0] = focal_length * distorted_pixel[0] + skew * distorted_pixel[1] +
             principal_point_x;
  pixel[1] =
      focal_length * aspect_ratio * distorted_pixel[1] + principal_point_y;

  return result;
}

template <typename T>
bool ExtendedUnifiedCameraModel::PixelToCameraCoordinates(
    const T *intrinsic_parameters, const T *pixel, T *point) {
  const T &focal_length =
      intrinsic_parameters[ExtendedUnifiedCameraModel::FOCAL_LENGTH];
  const T &aspect_ratio =
      intrinsic_parameters[ExtendedUnifiedCameraModel::ASPECT_RATIO];
  const T &focal_length_y = focal_length * aspect_ratio;
  const T &skew = intrinsic_parameters[ExtendedUnifiedCameraModel::SKEW];
  const T &principal_point_x =
      intrinsic_parameters[ExtendedUnifiedCameraModel::PRINCIPAL_POINT_X];
  const T &principal_point_y =
      intrinsic_parameters[ExtendedUnifiedCameraModel::PRINCIPAL_POINT_Y];

  // Normalize the y coordinate first.
  T distorted_point[2];
  distorted_point[1] = (pixel[1] - principal_point_y) / focal_length_y;
  distorted_point[0] =
      (pixel[0] - principal_point_x - distorted_point[1] * skew) / focal_length;

  // Undo the distortion.
  return ExtendedUnifiedCameraModel::UndistortPoint(intrinsic_parameters,
                                                    distorted_point, point);
}

template <typename T>
bool ExtendedUnifiedCameraModel::DistortPoint(const T *intrinsic_parameters,
                                              const T *undistorted_point,
                                              T *distorted_point) {
  const T &alpha = intrinsic_parameters[ExtendedUnifiedCameraModel::ALPHA];
  const T &beta = intrinsic_parameters[ExtendedUnifiedCameraModel::BETA];

  const T xx = undistorted_point[0] * undistorted_point[0];
  const T yy = undistorted_point[1] * undistorted_point[1];
  const T zz = undistorted_point[2] * undistorted_point[2];

  const T r2 = xx + yy;
  const T rho2 = beta * r2 + zz;
  const T rho = ceres::sqrt(rho2);

  const T norm = alpha * rho + (T(1) - alpha) * undistorted_point[2];
  distorted_point[0] = T(0);
  distorted_point[1] = T(0);

  if (norm < T(1e-3)) {
      return true;
  }

  // Check that the point is in the upper hemisphere in case of ellipsoid
  if (alpha > T(0.5))
  {
      const T zn = undistorted_point[2] / norm;
      const T C = (alpha - T(1)) / (alpha + alpha - T(1));
      if (zn < C) {
          return true;
      }
  }

  distorted_point[0] = undistorted_point[0] / norm;
  distorted_point[1] = undistorted_point[1] / norm;
  return true;
}

template <typename T>
bool ExtendedUnifiedCameraModel::UndistortPoint(const T *intrinsic_parameters,
                                                const T *distorted_point,
                                                T *undistorted_point) {
  const T &alpha = intrinsic_parameters[ExtendedUnifiedCameraModel::ALPHA];
  const T &beta = intrinsic_parameters[ExtendedUnifiedCameraModel::BETA];

  const T r2 = distorted_point[0] * distorted_point[0] +
               distorted_point[1] * distorted_point[1];
  const T gamma = T(1) - alpha;

  if (alpha > T(0.5)) {
    if (r2 >= T(1) / ((alpha - gamma) * beta)) {
      return false;
    }
  }

  const T tmp1 = (T(1) - alpha * alpha * beta * r2);
  const T tmp_sqrt = ceres::sqrt(T(1) - (alpha - gamma) * beta * r2);
  const T tmp2 = (alpha * tmp_sqrt + gamma);

  const T k = tmp1 / tmp2;

  T norm = ceres::sqrt(r2 + k * k);

  if (norm < T(1e-12)) {
      norm = T(1e-12);
  }

  undistorted_point[0] = distorted_point[0] / norm;
  undistorted_point[1] = distorted_point[1] / norm;
  undistorted_point[2] = k / norm;

  return true;
}

} // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::ExtendedUnifiedCameraModel, 1)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::ExtendedUnifiedCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::ExtendedUnifiedCameraModel)

#endif // THEIA_SFM_CAMERA_EXTENDED_UNIFIED_CAMERA_MODEL_H_
