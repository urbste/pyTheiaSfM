// Copyright (C) 2014 The Regents of the University of California (Regents).
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

#ifndef THEIA_SFM_CAMERA_REPROJECTION_ERROR_H_
#define THEIA_SFM_CAMERA_REPROJECTION_ERROR_H_

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/feature.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Sophus/sophus/se3.hpp>
#include <Sophus/sophus/sim3.hpp>

namespace theia {

template <class CameraModel>
struct ReprojectionError {
 public:
  explicit ReprojectionError(const Feature& feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters,
                  const T* intrinsic_parameters,
                  const T* point,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Matrix3T;
    typedef Eigen::Map<const Matrix3T> ConstMap3T;

    static const T kVerySmallNumber(1e-8);

    // Remove the translation.
    Eigen::Matrix<T, 3, 1> adjusted_point =
        ConstMap3T(point) -
        point[3] * ConstMap3T(extrinsic_parameters + Camera::POSITION);

    // If the point is too close to the camera center then the point cannot be
    // constrained by triangulation. This is likely to only occur when a 3d
    // point is seen by 2 views and the camera center of 1 view lies on or neare
    // the optical axis of the other view.
    //
    // Since we do not know the camera model we cannot say that the point must
    // be in front of the camera (e.g., wide angle cameras that have > 180
    // degree FOV). Instead we simply force that the point is not near the
    // camera center.
    if (adjusted_point.squaredNorm() < kVerySmallNumber) {
      return false;
    }

    // Rotate the point to obtain the point in the camera coordinate system.
    T rotated_point[3];
    ceres::AngleAxisRotatePoint(extrinsic_parameters + Camera::ORIENTATION,
                                adjusted_point.data(),
                                rotated_point);

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, rotated_point, reprojection);
    // Compute the reprojection error.
    // if (res) {
    // TODO FULL COVARIANCE WEIGHTING? -> Although most of the time the off
    // diagonal of image point covs are zero
    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());
    //    }
    //    else {
    //       reprojection_error[0] = T(1e4);
    //       reprojection_error[1] = T(1e4);
    //    }
    return res;
  }

 private:
  const Feature feature_;
};

template <class CameraModel>
struct OrthoReprojectionError {
 public:
  explicit OrthoReprojectionError(const Feature& feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters,
                  const T* intrinsic_parameters,
                  const T* point,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Matrix<T, 4, 1> Vector4T;
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;
    typedef Eigen::Matrix<T, 3, 4> Matrix34T;

    typedef Eigen::Map<const Vector3T> ConstMap3T;
    typedef Eigen::Map<const Vector4T> ConstMap4T;

    Matrix3T R;
    ceres::AngleAxisToRotationMatrix(extrinsic_parameters + Camera::ORIENTATION, R.data());
    const Vector3T t = ConstMap3T(extrinsic_parameters + Camera::POSITION);
    const Vector4T pt_w = ConstMap4T(point);

    Matrix34T T_w_c = Matrix34T::Identity();
    T_w_c.template block<3,3>(0,0) = R;
    T_w_c.template block<3,1>(0,3) = t;
    T_w_c(2,3) = T(0);
    Vector3T pt_in_cam = T_w_c * pt_w;
    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, pt_in_cam.data(), reprojection);
    // Compute the reprojection error.
    if (res) {
      // TODO FULL COVARIANCE WEIGHTING? -> Although most of the time the off
      // diagonal of image point covs are zero
      const T sqrt_information_x =
            T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
      const T sqrt_information_y =
            T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
      reprojection_error[0] =
            sqrt_information_x * (reprojection[0] - feature_.point_.x());
      reprojection_error[1] =
            sqrt_information_y * (reprojection[1] - feature_.point_.y());
    }
    else {
       reprojection_error[0] = T(1e4);
       reprojection_error[1] = T(1e4);
    }
    return res;
  }

 private:
  const Feature feature_;
};

template <class CameraModel>
struct InvReprojectionError {
 public:
  explicit InvReprojectionError(
    const Feature& feature,
    const Eigen::Vector3d& bearing_vector_ref) : 
    feature_(feature), bearing_vector_ref_(bearing_vector_ref)  {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters_ref,
                  const T* extrinsic_parameters_other,
                  const T* intrinsic_parameters_other,
                  const T* inverse_depth,
                  T* reprojection_error) const {

    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;

    // first scale the bearing vector 
    Vector3T point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

    // construct transformation matrices
    Sophus::SO3<T> R_ref_world = Sophus::SO3<T>::exp(
      ConstMap3T(extrinsic_parameters_ref + Camera::ORIENTATION));
    Sophus::SO3<T> R_other_world = Sophus::SO3<T>::exp(
      ConstMap3T(extrinsic_parameters_other + Camera::ORIENTATION));

    Vector3T p_world_ref = ConstMap3T(extrinsic_parameters_ref + Camera::POSITION);
    Vector3T p_world_other = ConstMap3T(extrinsic_parameters_other + Camera::POSITION);

    Sophus::SE3<T> T_world_ref(R_ref_world.inverse(), p_world_ref);
    Sophus::SE3<T> T_world_other(R_other_world.inverse(), p_world_other);

    Vector3T point_world = T_world_ref * point_ref;
    // transform the vector to the other camera
    Vector3T point_other = T_world_other.inverse() * point_world;

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters_other, point_other.data(), reprojection);

    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return res;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_vector_ref_;
};

template <class CameraModel>
struct InvReprojectionPoseError {
 public:
  explicit InvReprojectionPoseError(
    const Feature& feature,
    const Eigen::Vector3d& bearing_vector_ref) : 
    feature_(feature), bearing_vector_ref_(bearing_vector_ref)  {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters_ref,
                  const T* intrinsic_parameters_ref,
                  const T* inverse_depth,
                  T* reprojection_error) const {

    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;

    // first scale the bearing vector 
    Vector3T point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

    // construct transformation matrices
    Sophus::SO3<T> R_ref_world = Sophus::SO3<T>::exp(
      ConstMap3T(extrinsic_parameters_ref + Camera::ORIENTATION));

    Vector3T p_world_ref = ConstMap3T(extrinsic_parameters_ref + Camera::POSITION);

    Sophus::SE3<T> T_world_ref(R_ref_world.inverse(), p_world_ref);

    // transform the vector to the other camera
    Vector3T point = T_world_ref.inverse() * T_world_ref * point_ref;

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters_ref, point.data(), reprojection);

    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return res;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_vector_ref_;
};

template <class CameraModel>
struct Sim3InvReprojectionError {
 public:
  explicit Sim3InvReprojectionError(
    const Feature& feature,
    const Eigen::Vector3d& bearing_vector_ref) : 
    feature_(feature), bearing_vector_ref_(bearing_vector_ref)  {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters_ref,
                  const T* extrinsic_parameters_other,
                  const T* intrinsic_parameters_other,
                  const T* inverse_depth,
                  T* reprojection_error) const {

    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;
    typedef Sophus::Sim3<T> Sim3T;

    // get sim3 poses of ref and anchor cam
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_ref(extrinsic_parameters_ref);
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_other(extrinsic_parameters_other);

    Sim3T S_ref_world = Sim3T::exp(lie_ref);
    Sim3T S_other_world = Sim3T::exp(lie_other);

    // first scale the bearing vector 
    Vector3T point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

    Vector3T point_world = S_ref_world.inverse() * point_ref;
    // transform the vector to the other camera
    Vector3T point_other = S_other_world * point_world;

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters_other, point_other.data(), reprojection);

    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return res;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_vector_ref_;
};

template <class CameraModel>
struct Sim3InvReprojectionPoseError {
 public:
  explicit Sim3InvReprojectionPoseError(
    const Feature& feature,
    const Eigen::Vector3d& bearing_vector_ref) : 
    feature_(feature), bearing_vector_ref_(bearing_vector_ref)  {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters_ref,
                  const T* intrinsic_parameters_other,
                  const T* inverse_depth,
                  T* reprojection_error) const {

    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;
    typedef Sophus::Sim3<T> Sim3T;

    // get sim3 poses of ref and anchor cam
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_ref(extrinsic_parameters_ref);

    Sim3T S_ref_world = Sim3T::exp(lie_ref);

    // first scale the bearing vector 
    Vector3T point_ref = bearing_vector_ref_.cast<T>() / *inverse_depth;

    // transform the vector to the other camera
    Vector3T point_other = S_ref_world * S_ref_world.inverse() * point_ref;

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters_other, point_other.data(), reprojection);

    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return res;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_vector_ref_;
};

template <class CameraModel>
struct Sim3ReprojectionError {
 public:
  explicit Sim3ReprojectionError(const Feature& feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters,
                  const T* intrinsic_parameters,
                  const T* point,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;
    typedef Sophus::Sim3<T> Sim3T;

    static const T kVerySmallNumber(1e-8);

    // Get the SIM3 pose parameters (7 parameters: 3 for rotation, 3 for translation, 1 for scale)
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params(extrinsic_parameters);
    Sim3T sim3_pose = Sim3T::exp(sim3_params);

    // Transform the 3D point from world coordinates to camera coordinates
    // using the SIM3 transformation
    Vector3T point_world = ConstMap3T(point);
    Vector3T point_camera = sim3_pose * point_world;

    // If the point is too close to the camera center then the point cannot be
    // constrained by triangulation. This is likely to only occur when a 3d
    // point is seen by 2 views and the camera center of 1 view lies on or near
    // the optical axis of the other view.
    //
    // Since we do not know the camera model we cannot say that the point must
    // be in front of the camera (e.g., wide angle cameras that have > 180
    // degree FOV). Instead we simply force that the point is not near the
    // camera center.
    if (point_camera.squaredNorm() < kVerySmallNumber) {
      return false;
    }

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, point_camera.data(), reprojection);

    // Compute the reprojection error with covariance weighting
    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return res;
  }

 private:
  const Feature feature_;
};

template <class CameraModel>
struct Sim3RelativeReprojectionError {
 public:
  explicit Sim3RelativeReprojectionError(const Feature& feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T* sim3_pose1,
                  const T* sim3_pose2,
                  const T* intrinsic_parameters,
                  const T* point,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;
    typedef Sophus::Sim3<T> Sim3T;

    static const T kVerySmallNumber(1e-8);

    // Get the two SIM3 poses
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params1(sim3_pose1);
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params2(sim3_pose2);
    Sim3T pose1 = Sim3T::exp(sim3_params1);
    Sim3T pose2 = Sim3T::exp(sim3_params2);

    // Transform the 3D point from world coordinates to camera coordinates
    // using the relative SIM3 transformation
    Vector3T point_world = ConstMap3T(point);
    Vector3T point_camera1 = pose1 * point_world;
    Vector3T point_camera2 = pose2 * point_world;

    // Check if points are too close to camera centers
    if (point_camera1.squaredNorm() < kVerySmallNumber ||
        point_camera2.squaredNorm() < kVerySmallNumber) {
      return false;
    }

    // Apply the camera intrinsics to get the reprojected pixels
    T reprojection1[2], reprojection2[2];
    const bool res1 = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, point_camera1.data(), reprojection1);
    const bool res2 = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, point_camera2.data(), reprojection2);

    if (!res1 || !res2) {
      return false;
    }

    // Compute the reprojection errors with covariance weighting
    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));

    // Return errors for both cameras (4-dimensional error)
    reprojection_error[0] =
        sqrt_information_x * (reprojection1[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection1[1] - feature_.point_.y());
    reprojection_error[2] =
        sqrt_information_x * (reprojection2[0] - feature_.point_.x());
    reprojection_error[3] =
        sqrt_information_y * (reprojection2[1] - feature_.point_.y());

    return true;
  }

 private:
  const Feature feature_;
};

template <class CameraModel>
struct Sim3PoseOnlyReprojectionError {
 public:
  explicit Sim3PoseOnlyReprojectionError(const Feature& feature, 
                                        const Eigen::Vector3d& world_point)
      : feature_(feature), world_point_(world_point) {}

  template <typename T>
  bool operator()(const T* extrinsic_parameters,
                  const T* intrinsic_parameters,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Sophus::Sim3<T> Sim3T;

    static const T kVerySmallNumber(1e-8);

    // Get the SIM3 pose parameters
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params(extrinsic_parameters);
    Sim3T sim3_pose = Sim3T::exp(sim3_params);

    // Transform the fixed 3D point from world coordinates to camera coordinates
    Vector3T point_world = world_point_.cast<T>();
    Vector3T point_camera = sim3_pose * point_world;

    // Check if point is too close to camera center
    if (point_camera.squaredNorm() < kVerySmallNumber) {
      return false;
    }

    // Apply the camera intrinsics to get the reprojected pixel
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, point_camera.data(), reprojection);

    if (!res) {
      return false;
    }

    // Compute the reprojection error with covariance weighting
    const T sqrt_information_x =
        T(1. / ceres::sqrt(feature_.covariance_(0, 0)));
    const T sqrt_information_y =
        T(1. / ceres::sqrt(feature_.covariance_(1, 1)));
    reprojection_error[0] =
        sqrt_information_x * (reprojection[0] - feature_.point_.x());
    reprojection_error[1] =
        sqrt_information_y * (reprojection[1] - feature_.point_.y());

    return true;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d world_point_;
};

}  // namespace theia

#endif  // THEIA_SFM_CAMERA_REPROJECTION_ERROR_H_
