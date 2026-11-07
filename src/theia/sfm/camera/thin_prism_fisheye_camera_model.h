// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_CAMERA_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_

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

// Camera intrinsics for the COLMAP THIN_PRISM_FISHEYE model: an equidistant
// fisheye projection with radial, tangential and thin-prism distortion.
//
// A normalized point (u, v) = (x / z, y / z) is mapped to the angle scaled
// point (uu, vv) = atan(r) / r * (u, v) with r = |(u, v)|. With
// theta^2 = uu^2 + vv^2 the distorted point is
//
//   radial = k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8
//   uu_d = uu + uu * radial + 2 p1 uu vv + p2 (theta^2 + 2 uu^2) + sx1 theta^2
//   vv_d = vv + vv * radial + 2 p2 uu vv + p1 (theta^2 + 2 vv^2) + sy1 theta^2
//
// and the pixel is (f * uu_d + cx, f * aspect * vv_d + cy).
//
// The parameter order matches COLMAP's THIN_PRISM_FISHEYE model
// (fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1).
class ThinPrismFisheyeCameraModel : public CameraIntrinsicsModel {
 public:
  ThinPrismFisheyeCameraModel();
  ~ThinPrismFisheyeCameraModel() {}

  static const int kIntrinsicsSize = 12;

  enum InternalParametersIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    PRINCIPAL_POINT_X = 2,
    PRINCIPAL_POINT_Y = 3,
    RADIAL_DISTORTION_1 = 4,      // k1
    RADIAL_DISTORTION_2 = 5,      // k2
    TANGENTIAL_DISTORTION_1 = 6,  // p1
    TANGENTIAL_DISTORTION_2 = 7,  // p2
    RADIAL_DISTORTION_3 = 8,      // k3
    RADIAL_DISTORTION_4 = 9,      // k4
    THIN_PRISM_DISTORTION_1 = 10,  // sx1
    THIN_PRISM_DISTORTION_2 = 11,  // sy1
  };

  int NumParameters() const override;

  // Returns the camera model type of the object.
  CameraIntrinsicsModelType Type() const override;

  // Set the intrinsic camera parameters from the priors. The four entries of
  // radial_distortion are k1..k4, tangential_distortion is (p1, p2) and the
  // first two entries of thin_prism_distortion are (sx1, sy1). The skew prior
  // is not modeled and is ignored.
  void SetFromCameraIntrinsicsPriors(
      const CameraIntrinsicsPrior& prior) override;

  // Return a CameraIntrinsicsPrior that can be used to initialize a camera with
  // the same parameters with the SetFromCameraIntrinsicsPriors method.
  CameraIntrinsicsPrior CameraIntrinsicsPriorFromIntrinsics() const override;

  // Returns the indices of the parameters that will be optimized during bundle
  // adjustment. The thin-prism terms are non-radial and are grouped with the
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
  // to k1, k2, p1, p2, k3, k4, sx1, sy1 (the COLMAP order).
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
                           const double radial_distortion_4);
  double RadialDistortion1() const;
  double RadialDistortion2() const;
  double RadialDistortion3() const;
  double RadialDistortion4() const;

  void SetTangentialDistortion(const double tangential_distortion_1,
                               const double tangential_distortion_2);
  double TangentialDistortion1() const;
  double TangentialDistortion2() const;

  void SetThinPrismDistortion(const double thin_prism_distortion_1,
                              const double thin_prism_distortion_2);
  double ThinPrismDistortion1() const;
  double ThinPrismDistortion2() const;

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
bool ThinPrismFisheyeCameraModel::CameraToPixelCoordinates(
    const T* intrinsic_parameters, const T* point, T* pixel) {
  const T normalized_point[2] = {point[0] / point[2], point[1] / point[2]};

  T distorted_point[2];
  ThinPrismFisheyeCameraModel::DistortPoint(
      intrinsic_parameters, normalized_point, distorted_point);

  const T& focal_length =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::ASPECT_RATIO];
  const T& principal_point_x =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::PRINCIPAL_POINT_Y];

  pixel[0] = focal_length * distorted_point[0] + principal_point_x;
  pixel[1] = focal_length * aspect_ratio * distorted_point[1] +
             principal_point_y;
  return true;
}

template <typename T>
bool ThinPrismFisheyeCameraModel::PixelToCameraCoordinates(
    const T* intrinsic_parameters, const T* pixel, T* point) {
  const T& focal_length =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::ASPECT_RATIO];
  const T focal_length_y = focal_length * aspect_ratio;
  const T& principal_point_x =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[ThinPrismFisheyeCameraModel::PRINCIPAL_POINT_Y];

  T distorted_point[2];
  distorted_point[0] = (pixel[0] - principal_point_x) / focal_length;
  distorted_point[1] = (pixel[1] - principal_point_y) / focal_length_y;

  T undistorted_point[2];
  if (!ThinPrismFisheyeCameraModel::UndistortPoint(
          intrinsic_parameters, distorted_point, undistorted_point)) {
    return false;
  }
  point[0] = undistorted_point[0];
  point[1] = undistorted_point[1];
  point[2] = T(1.0);
  return true;
}

template <typename T>
bool ThinPrismFisheyeCameraModel::DistortPoint(const T* intrinsic_parameters,
                                               const T* undistorted_point,
                                               T* distorted_point) {
  T angle_scaled_point[2];
  internal::FisheyeFromNormal(undistorted_point[0],
                              undistorted_point[1],
                              &angle_scaled_point[0],
                              &angle_scaled_point[1]);
  ThinPrismFisheyeCameraModel::ApplyDistortion(
      &intrinsic_parameters[ThinPrismFisheyeCameraModel::RADIAL_DISTORTION_1],
      angle_scaled_point[0],
      angle_scaled_point[1],
      &distorted_point[0],
      &distorted_point[1]);
  return true;
}

template <typename T>
bool ThinPrismFisheyeCameraModel::UndistortPoint(
    const T* intrinsic_parameters, const T* distorted_point, T* undistorted_point) {
  T angle_scaled_point[2];
  if (!internal::InvertDistortion<ThinPrismFisheyeCameraModel, 8>(
          &intrinsic_parameters[ThinPrismFisheyeCameraModel::RADIAL_DISTORTION_1],
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
void ThinPrismFisheyeCameraModel::ApplyDistortion(const T* extra_params,
                                                  const T u,
                                                  const T v,
                                                  T* x,
                                                  T* y) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];
  const T k3 = extra_params[4];
  const T k4 = extra_params[5];
  const T sx1 = extra_params[6];
  const T sy1 = extra_params[7];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T r8 = r6 * r2;
  const T radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
  *x = u + u * radial + T(2.0) * p1 * uv + p2 * (r2 + T(2.0) * u2) + sx1 * r2;
  *y = v + v * radial + T(2.0) * p2 * uv + p1 * (r2 + T(2.0) * v2) + sy1 * r2;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::ThinPrismFisheyeCameraModel, 0)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::ThinPrismFisheyeCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::ThinPrismFisheyeCameraModel)

#endif  // THEIA_SFM_CAMERA_THIN_PRISM_FISHEYE_CAMERA_MODEL_H_
