// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_CAMERA_RAD_TAN_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_RAD_TAN_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_

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

// Camera intrinsics for the COLMAP RAD_TAN_THIN_PRISM_FISHEYE model (the
// "FISHEYE624" model of Project Aria). An equidistant fisheye projection with
// a six-term radial polynomial in theta, tangential distortion and four
// thin-prism terms.
//
// A normalized point (u, v) = (x / z, y / z) is mapped to the angle scaled
// point (uu, vv) = atan(r) / r * (u, v). With theta^2 = uu^2 + vv^2 let
//
//   th_radial = 1 + k0 theta^2 + k1 theta^4 + ... + k5 theta^12
//   x = th_radial * uu,  y = th_radial * vv,  r2 = x^2 + y^2
//
// Then the distorted point is
//
//   x_d = x + 2 p1 x y + p0 (r2 + 2 x^2) + s0 r2 + s1 r2^2
//   y_d = y + 2 p0 x y + p1 (r2 + 2 y^2) + s2 r2 + s3 r2^2
//
// and the pixel is (f * x_d + cx, f * aspect * y_d + cy).
//
// The parameter order matches COLMAP's RAD_TAN_THIN_PRISM_FISHEYE model
// (fx, fy, cx, cy, k0, k1, k2, k3, k4, k5, p0, p1, s0, s1, s2, s3).
class RadTanThinPrismFisheyeCameraModel : public CameraIntrinsicsModel {
 public:
  RadTanThinPrismFisheyeCameraModel();
  ~RadTanThinPrismFisheyeCameraModel() {}

  static const int kIntrinsicsSize = 16;

  enum InternalParametersIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    PRINCIPAL_POINT_X = 2,
    PRINCIPAL_POINT_Y = 3,
    RADIAL_DISTORTION_1 = 4,      // k0
    RADIAL_DISTORTION_2 = 5,      // k1
    RADIAL_DISTORTION_3 = 6,      // k2
    RADIAL_DISTORTION_4 = 7,      // k3
    RADIAL_DISTORTION_5 = 8,      // k4
    RADIAL_DISTORTION_6 = 9,      // k5
    TANGENTIAL_DISTORTION_1 = 10,  // p0
    TANGENTIAL_DISTORTION_2 = 11,  // p1
    THIN_PRISM_DISTORTION_1 = 12,  // s0
    THIN_PRISM_DISTORTION_2 = 13,  // s1
    THIN_PRISM_DISTORTION_3 = 14,  // s2
    THIN_PRISM_DISTORTION_4 = 15,  // s3
  };

  int NumParameters() const override;

  // Returns the camera model type of the object.
  CameraIntrinsicsModelType Type() const override;

  // Set the intrinsic camera parameters from the priors. radial_distortion_6
  // holds k0..k5 and takes precedence; the first four entries of
  // radial_distortion are used as k0..k3 if radial_distortion_6 is not set.
  // tangential_distortion is (p0, p1) and thin_prism_distortion is (s0..s3).
  // The skew prior is not modeled and is ignored.
  void SetFromCameraIntrinsicsPriors(
      const CameraIntrinsicsPrior& prior) override;

  // Return a CameraIntrinsicsPrior that can be used to initialize a camera with
  // the same parameters with the SetFromCameraIntrinsicsPriors method.
  CameraIntrinsicsPrior CameraIntrinsicsPriorFromIntrinsics() const override;

  // Returns the indices of the parameters that will be optimized during bundle
  // adjustment. The tangential and thin-prism terms are grouped with the
  // TANGENTIAL_DISTORTION flag.
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
  // and the distortion. The result is in the distorted fisheye plane.
  template <typename T>
  static bool DistortPoint(const T* intrinsic_parameters,
                           const T* undistorted_point,
                           T* distorted_point);

  // Given a point in the distorted fisheye plane, remove the distortion and the
  // fisheye projection to get a normalized undistorted point.
  template <typename T>
  static bool UndistortPoint(const T* intrinsic_parameters,
                             const T* distorted_point,
                             T* undistorted_point);

  // Applies the distortion to the angle scaled point (u, v). extra_params points
  // to k0..k5, p0, p1, s0..s3 (the COLMAP order).
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
                           const double radial_distortion_2,
                           const double radial_distortion_3,
                           const double radial_distortion_4,
                           const double radial_distortion_5,
                           const double radial_distortion_6);
  double RadialDistortion1() const;
  double RadialDistortion2() const;
  double RadialDistortion3() const;
  double RadialDistortion4() const;
  double RadialDistortion5() const;
  double RadialDistortion6() const;

  void SetTangentialDistortion(const double tangential_distortion_1,
                               const double tangential_distortion_2);
  double TangentialDistortion1() const;
  double TangentialDistortion2() const;

  void SetThinPrismDistortion(const double thin_prism_distortion_1,
                              const double thin_prism_distortion_2,
                              const double thin_prism_distortion_3,
                              const double thin_prism_distortion_4);
  double ThinPrismDistortion1() const;
  double ThinPrismDistortion2() const;
  double ThinPrismDistortion3() const;
  double ThinPrismDistortion4() const;

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
bool RadTanThinPrismFisheyeCameraModel::CameraToPixelCoordinates(
    const T* intrinsic_parameters, const T* point, T* pixel) {
  const T normalized_point[2] = {point[0] / point[2], point[1] / point[2]};

  T distorted_point[2];
  RadTanThinPrismFisheyeCameraModel::DistortPoint(
      intrinsic_parameters, normalized_point, distorted_point);

  const T& focal_length =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::ASPECT_RATIO];
  const T& principal_point_x =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::PRINCIPAL_POINT_Y];

  pixel[0] = focal_length * distorted_point[0] + principal_point_x;
  pixel[1] = focal_length * aspect_ratio * distorted_point[1] +
             principal_point_y;
  return true;
}

template <typename T>
bool RadTanThinPrismFisheyeCameraModel::PixelToCameraCoordinates(
    const T* intrinsic_parameters, const T* pixel, T* point) {
  const T& focal_length =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::ASPECT_RATIO];
  const T focal_length_y = focal_length * aspect_ratio;
  const T& principal_point_x =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::PRINCIPAL_POINT_Y];

  T distorted_point[2];
  distorted_point[0] = (pixel[0] - principal_point_x) / focal_length;
  distorted_point[1] = (pixel[1] - principal_point_y) / focal_length_y;

  T undistorted_point[2];
  if (!RadTanThinPrismFisheyeCameraModel::UndistortPoint(
          intrinsic_parameters, distorted_point, undistorted_point)) {
    return false;
  }
  point[0] = undistorted_point[0];
  point[1] = undistorted_point[1];
  point[2] = T(1.0);
  return true;
}

template <typename T>
bool RadTanThinPrismFisheyeCameraModel::DistortPoint(
    const T* intrinsic_parameters, const T* undistorted_point, T* distorted_point) {
  T angle_scaled_point[2];
  internal::FisheyeFromNormal(undistorted_point[0],
                              undistorted_point[1],
                              &angle_scaled_point[0],
                              &angle_scaled_point[1]);
  RadTanThinPrismFisheyeCameraModel::ApplyDistortion(
      &intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::RADIAL_DISTORTION_1],
      angle_scaled_point[0],
      angle_scaled_point[1],
      &distorted_point[0],
      &distorted_point[1]);
  return true;
}

template <typename T>
bool RadTanThinPrismFisheyeCameraModel::UndistortPoint(
    const T* intrinsic_parameters, const T* distorted_point, T* undistorted_point) {
  T angle_scaled_point[2];
  if (!internal::InvertDistortion<RadTanThinPrismFisheyeCameraModel, 12>(
          &intrinsic_parameters[RadTanThinPrismFisheyeCameraModel::RADIAL_DISTORTION_1],
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
void RadTanThinPrismFisheyeCameraModel::ApplyDistortion(const T* extra_params,
                                                        const T u,
                                                        const T v,
                                                        T* x,
                                                        T* y) {
  constexpr int kNumRadialParams = 6;

  const T theta2 = u * u + v * v;
  T th_radial = T(1.0);
  T theta_power = T(1.0);
  for (int i = 0; i < kNumRadialParams; ++i) {
    theta_power *= theta2;
    th_radial += extra_params[i] * theta_power;
  }

  const T p0 = extra_params[6];
  const T p1 = extra_params[7];
  const T s0 = extra_params[8];
  const T s1 = extra_params[9];
  const T s2 = extra_params[10];
  const T s3 = extra_params[11];

  const T xr = th_radial * u;
  const T yr = th_radial * v;

  const T x2 = xr * xr;
  const T y2 = yr * yr;
  const T xy = xr * yr;
  const T r2 = x2 + y2;
  const T r4 = r2 * r2;

  const T dx_tang = T(2.0) * p1 * xy + p0 * (r2 + T(2.0) * x2);
  const T dy_tang = T(2.0) * p0 * xy + p1 * (r2 + T(2.0) * y2);
  const T dx_tp = s0 * r2 + s1 * r4;
  const T dy_tp = s2 * r2 + s3 * r4;

  *x = xr + dx_tang + dx_tp;
  *y = yr + dy_tang + dy_tp;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::RadTanThinPrismFisheyeCameraModel, 0)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::RadTanThinPrismFisheyeCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::RadTanThinPrismFisheyeCameraModel)

#endif  // THEIA_SFM_CAMERA_RAD_TAN_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_
