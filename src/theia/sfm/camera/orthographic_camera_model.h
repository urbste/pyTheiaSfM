// Copyright (C) 2022
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_CAMERA_ORTHOGRAPHIC_CAMERA_MODEL_H_
#define THEIA_SFM_CAMERA_ORTHOGRAPHIC_CAMERA_MODEL_H_

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

// This class contains the camera pose information pertaining to intrinsics
// camera parameters. For an orthographic camera this includes magnification (m), pixel pitches s_x and s_y (in meter), principal
// points, and (up to 2-parameter) radial distortion. Methods are provided for
// common transformations and projections.
//
// Intrinsics of the camera are modeled such that:
// Given a point in camera coordinats point p_c = [x_c,y_c,z_c]^T the undistorted point in the camera is:
//
// x_u = m * x_c 
// y_u = m * y_c 
//
// For an orthographic camera the projection is independent of the translation in z direction, i.e we
// can always set z_c to an arbitrary value, e.g. 0.
// In addition, we can model a distortion. Here a simple 2 parameter radial distortion is used.
// The undistorted point x_u, y_u is given by:
// r = x_u * x_u + x_u * x_u
// d = 1 + k1 * r + k2 * r^2
// x_d = x_u * d
// y_d = y_u * d

// Finally, we need to transform the point to pixel coordinates using the pixel pitches s_x and s_y
// x_i = 1/s_x * x_d + c_x
// y_i = 1/s_y * x_d + c_y

// So given the extrinsics (R and C) and intrinsics, an object p_o = [x_o, y_o, z_o]^T transforms 
// to pixel coordinates like this:

// x_c = R * (p_o - C)
// x_u = m * x_c
// x_d = d * x_u
// x_i = [1/s_x, 1/s_y] * x_d + [c_x,c_y]
// Putting it all together:
// x_i = [1/s_x, 1/s_y,1] * d*m*(R*(p_o -C)) + [c_x,c_y,1]
//  where R = orientation, C = camera position, and k1 k2 are the radial
//  distortion parameters.
class OrthographicCameraModel : public CameraIntrinsicsModel {
 public:
  OrthographicCameraModel();
  ~OrthographicCameraModel() {}

  static const int kIntrinsicsSize = 7;

  enum InternalParametersIndex {
    FOCAL_LENGTH = 0,
    ASPECT_RATIO = 1,
    SKEW = 2,
    PRINCIPAL_POINT_X = 3,
    PRINCIPAL_POINT_Y = 4,
    RADIAL_DISTORTION_1 = 5,
    RADIAL_DISTORTION_2 = 6
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
  void SetAspectRatio(const double aspect_ratio);
  double AspectRatio() const;

  void SetSkew(const double skew);
  double Skew() const;

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
bool OrthographicCameraModel::CameraToPixelCoordinates(const T* intrinsic_parameters,
                                                  const T* point,
                                                  T* pixel) {
  // Apply radial distortion.
  T distorted_pixel[2];
  OrthographicCameraModel::DistortPoint(
      intrinsic_parameters, point, distorted_pixel);

  // Apply calibration parameters to transform normalized units into pixels.
  const T& fx =
      intrinsic_parameters[OrthographicCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[OrthographicCameraModel::ASPECT_RATIO];
  const T& skew = intrinsic_parameters[OrthographicCameraModel::SKEW];
  const T& principal_point_x =
      intrinsic_parameters[OrthographicCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[OrthographicCameraModel::PRINCIPAL_POINT_Y];

  const T fy = fx*aspect_ratio;
  pixel[0] = fx * distorted_pixel[0] + skew * distorted_pixel[1] + principal_point_x;
  pixel[1] = fy * distorted_pixel[1] + principal_point_y;

  return true;
}

template <typename T>
bool OrthographicCameraModel::PixelToCameraCoordinates(const T* intrinsic_parameters,
                                                  const T* pixel,
                                                  T* point) {
  const T& fx =
      intrinsic_parameters[OrthographicCameraModel::FOCAL_LENGTH];
  const T& aspect_ratio =
      intrinsic_parameters[OrthographicCameraModel::ASPECT_RATIO];
  const T& skew = intrinsic_parameters[OrthographicCameraModel::SKEW];
  const T& principal_point_x =
      intrinsic_parameters[OrthographicCameraModel::PRINCIPAL_POINT_X];
  const T& principal_point_y =
      intrinsic_parameters[OrthographicCameraModel::PRINCIPAL_POINT_Y];
  const T fy = fx*aspect_ratio;

  // Normalize the y coordinate first.
  T distorted_point[2];
  distorted_point[1] = pixel[1] - principal_point_y;
  distorted_point[0] = pixel[0] - principal_point_x - distorted_point[1]*skew;

  distorted_point[0] /= fx;
  distorted_point[1] /= fy;
  // Undo the radial distortion.
  OrthographicCameraModel::UndistortPoint(
      intrinsic_parameters, distorted_point, point);
  point[2] = T(1.0);
  return true;
}

template <typename T>
bool OrthographicCameraModel::DistortPoint(const T* intrinsic_parameters,
                                      const T* undistorted_point,
                                      T* distorted_point) {
  const T& radial_distortion1 =
      intrinsic_parameters[OrthographicCameraModel::RADIAL_DISTORTION_1];
  const T& radial_distortion2 =
      intrinsic_parameters[OrthographicCameraModel::RADIAL_DISTORTION_2];

  const T r_sq = undistorted_point[0] * undistorted_point[0] +
                 undistorted_point[1] * undistorted_point[1];
  const T d = 1.0 + r_sq * (radial_distortion1 + radial_distortion2 * r_sq);

  distorted_point[0] = undistorted_point[0] * d;
  distorted_point[1] = undistorted_point[1] * d;

  return true;
}

template <typename T>
bool OrthographicCameraModel::UndistortPoint(const T* intrinsic_parameters,
                                        const T* distorted_point,
                                        T* undistorted_point) {
  const int kNumUndistortionIterations = 100;
  const T kUndistortionEpsilon = T(1e-16);

  T prev_undistorted_point[2];
  undistorted_point[0] = distorted_point[0];
  undistorted_point[1] = distorted_point[1];
  for (size_t i = 0; i < kNumUndistortionIterations; ++i) {
    prev_undistorted_point[0] = undistorted_point[0];
    prev_undistorted_point[1] = undistorted_point[1];

    // Compute an estimate of the radius of the undistorted point from the
    // center of distortion (which is assumed to be the principal point).
    const T r_sq = undistorted_point[0] * undistorted_point[0] +
                   undistorted_point[1] * undistorted_point[1];
    // Compute the distortion factor.
    const T d = 1.0 + r_sq * (intrinsic_parameters[RADIAL_DISTORTION_1] +
                              intrinsic_parameters[RADIAL_DISTORTION_2] * r_sq);

    // We know that the distorted point = d * undistorted point, so we can solve
    // for a better estimate of the undistorted point by taking the inverse of
    // this equation: undistorted_point = distorted_point / d.
    undistorted_point[0] = distorted_point[0] / d;
    undistorted_point[1] = distorted_point[1] / d;

    // Repeat until convergence.
    if (ceres::abs(undistorted_point[0] - prev_undistorted_point[0]) <
            kUndistortionEpsilon &&
        ceres::abs(undistorted_point[1] - prev_undistorted_point[1]) <
            kUndistortionEpsilon) {
      break;
    }
  }

  return true;
}

}  // namespace theia

#include <cereal/archives/portable_binary.hpp>

CEREAL_CLASS_VERSION(theia::OrthographicCameraModel, 1)
// Register the polymorphic relationship for serialization.
CEREAL_REGISTER_TYPE(theia::OrthographicCameraModel)
CEREAL_REGISTER_POLYMORPHIC_RELATION(theia::CameraIntrinsicsModel,
                                     theia::OrthographicCameraModel)

#endif  // THEIA_SFM_CAMERA_PINHOLE_CAMERA_MODEL_H_
