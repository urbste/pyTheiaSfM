// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_CAMERA_RADIAL_FISHEYE_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_RADIAL_FISHEYE_CAMERA_MODEL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <ceres/jet.h>
#include <stdint.h>
#include <vector>

#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/distortion_inversion.h"
#include "theia/sfm/camera_intrinsics_prior.h"

namespace theia {

// Camera intrinsics for the COLMAP RADIAL_FISHEYE model: an equidistant
// fisheye projection with two radial distortion coefficients. The COLMAP
// SIMPLE_RADIAL_FISHEYE model is the same model with k2 = 0.
//
// A normalized point (u, v) = (x / z, y / z) is first mapped to the angle
// scaled point (uu, vv) = atan(r) / r * (u, v) with r = |(u, v)|. The radial
// distortion is then applied to the angle scaled point with theta^2 = uu^2 +
// vv^2 as
//
//   uu_d = uu * (1 + k1 theta^2 + k2 theta^4)
//
// and the pixel is (f * uu_d + cx, f * aspect * vv_d + cy).
//
// The parameter order matches COLMAP's RADIAL_FISHEYE model (f, cx, cy, k1,
// k2) after the focal length and the aspect ratio.
class RadialFisheyeCameraModel : public CameraIntrinsicsModel {
 public:
  RadialFisheyeCameraModel();
  ~RadialFisheyeCameraModel() {}

  static const int kIntrinsicsSize = 6;

  enum InternalParametersIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    PRINCIPAL_POINT_X = 2,
    PRINCIPAL_POINT_Y = 3,
    RADIAL_DISTORTION_1 = 4,  // k1
    RADIAL_DISTORTION_2 = 5,  // k2
  };

  int NumParameters() const override;

  // Returns the camera model type of the object.
  CameraIntrinsicsModelType Type() const override;

  // Set the intrinsic camera parameters from the priors. The first two entries
  // of radial_distortion are used as k1 and k2. The skew prior is not modeled
  // and is ignored.
  void SetFromCameraIntrinsicsPriors(
      const CameraIntrinsicsPrior& prior) override;

  // Return a CameraIntrinsicsPrior that can be used to initialize a camera with
  // the same parameters with the SetFromCameraIntrinsicsPriors method.
  CameraIntrinsicsPrior CameraIntrinsicsPriorFromIntrinsics() const override;

  // Returns the indices of the parameters that will be optimized during bundle
  // adjustment.
  std::vector<int> GetSubsetFromOptimizeIntrinsicsType(
      const OptimizeIntrinsicsType& intrinsics_to_optimize) const override;

  // Returns the calibration matrix in the form specified above (zero skew).
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

  // Given a normalized undistorted point (u, v), apply the fisheye projection
  // and the radial distortion. The result is in the distorted fisheye plane.
  template <typename T>
  static bool DistortPoint(const T* intrinsic_parameters,
                           const T* undistorted_point,
                           T* distorted_point);

  // Given a point in the distorted fisheye plane, remove the radial distortion
  // and the fisheye projection to get a normalized undistorted point.
  template <typename T>
  static bool UndistortPoint(const T* intrinsic_parameters,
                             const T* distorted_point,
                             T* undistorted_point);

  // Applies the radial distortion to the angle scaled point (u, v). extra_params
  // points to k1, k2.
  template <typename T>
  static void ApplyDistortion(const T* extra_params,
                              const T u,
                              const T v,
                              T* x,
                              T* y);

  // ----------------------- Getter and Setter methods ---------------------- //
  void SetAspectRatio(const double aspect_ratio);
  double AspectRatio() const;

  void SetRadialDistortion(const double radial_distortion_1,
                           const double radial_distortion_2);
  double RadialDistortion1() const;
  double RadialDistortion2() const;

 private:
  // Templated method for disk I/O with cereal. This method tells cereal which
  // data members should be used when reading/writing to/from disk.
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(cereal::base_class<CameraIntrinsicsModel>(this));
  }
};

template <typename T>
bool RadialFisheyeCameraModel::CameraToPixelCoordinates(
    const T* intrinsic_parameters, const T* point, T* pixel) {
  const T normalized_point[2] = {point[0] / point[2], point[1] / point[2]};

  T distorted_point[2];
  RadialFisheyeCameraModel::DistortPoint(
      intrinsic_parameters, normalized_point, distorted_point);

  const T& focal_length =
      intrinsic_parameters[RadialFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[RadialFisheyeCameraModel::ASPECT_RATIO];
  const T& principal_point_x =
      intrinsic_parameters[RadialFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[RadialFisheyeCameraModel::PRINCIPAL_POINT_Y];

  pixel[0] = focal_length * distorted_point[0] + principal_point_x;
  pixel[1] = focal_length * aspect_ratio * distorted_point[1] +
             principal_point_y;
  return true;
}

template <typename T>
bool RadialFisheyeCameraModel::PixelToCameraCoordinates(
    const T* intrinsic_parameters, const T* pixel, T* point) {
  const T& focal_length =
      intrinsic_parameters[RadialFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[RadialFisheyeCameraModel::ASPECT_RATIO];
  const T focal_length_y = focal_length * aspect_ratio;
  const T& principal_point_x =
      intrinsic_parameters[RadialFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[RadialFisheyeCameraModel::PRINCIPAL_POINT_Y];

  T distorted_point[2];
  distorted_point[0] = (pixel[0] - principal_point_x) / focal_length;
  distorted_point[1] = (pixel[1] - principal_point_y) / focal_length_y;

  T undistorted_point[2];
  if (!RadialFisheyeCameraModel::UndistortPoint(
          intrinsic_parameters, distorted_point, undistorted_point)) {
    return false;
  }
  point[0] = undistorted_point[0];
  point[1] = undistorted_point[1];
  point[2] = T(1.0);
  return true;
}

template <typename T>
bool RadialFisheyeCameraModel::DistortPoint(const T* intrinsic_parameters,
                                            const T* undistorted_point,
                                            T* distorted_point) {
  T angle_scaled_point[2];
  internal::FisheyeFromNormal(undistorted_point[0],
                              undistorted_point[1],
                              &angle_scaled_point[0],
                              &angle_scaled_point[1]);
  RadialFisheyeCameraModel::ApplyDistortion(
      &intrinsic_parameters[RadialFisheyeCameraModel::RADIAL_DISTORTION_1],
      angle_scaled_point[0],
      angle_scaled_point[1],
      &distorted_point[0],
      &distorted_point[1]);
  return true;
}

template <typename T>
bool RadialFisheyeCameraModel::UndistortPoint(const T* intrinsic_parameters,
                                              const T* distorted_point,
                                              T* undistorted_point) {
  T angle_scaled_point[2];
  if (!internal::InvertDistortion<RadialFisheyeCameraModel, 2>(
          &intrinsic_parameters[RadialFisheyeCameraModel::RADIAL_DISTORTION_1],
          distorted_point,
          angle_scaled_point)) {
    return false;
  }
  internal::NormalFromFisheye(angle_scaled_point[0],
                              angle_scaled_point[1],
                              &undistorted_point[0],
                              &undistorted_point[1]);
  return true;
}

template <typename T>
void RadialFisheyeCameraModel::ApplyDistortion(const T* extra_params,
                                               const T u,
                                               const T v,
                                               T* x,
                                               T* y) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];

  const T theta2 = u * u + v * v;
  const T theta4 = theta2 * theta2;
  const T radial = k1 * theta2 + k2 * theta4;
  *x = u + u * radial;
  *y = v + v * radial;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::RadialFisheyeCameraModel, 0)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::RadialFisheyeCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::RadialFisheyeCameraModel)

#endif  // THEIA_SFM_CAMERA_RADIAL_FISHEYE_CAMERA_MODEL_H_
