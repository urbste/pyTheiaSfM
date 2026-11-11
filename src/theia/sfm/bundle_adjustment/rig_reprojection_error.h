// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Rig-constrained reprojection error. The camera pose of a view that belongs to
// a RigCapture is not a free parameter: it is composed from the shared capture
// (rig body) pose and the constant sensor extrinsics, exactly as
// ComposeCameraPoseFromRig() does in theia/sfm/rig/rig_utils.cc.

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_RIG_REPROJECTION_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_RIG_REPROJECTION_ERROR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model_type.h"
#include "theia/sfm/feature.h"

namespace theia {

// Reprojection error of one observation whose camera belongs to a rig.
//
// Parameter blocks (same layout as the Camera extrinsics, see Camera::POSITION
// and Camera::ORIENTATION):
//   capture_pose[0:3] = c_rig, the rig body center in world coordinates,
//   capture_pose[3:6] = angle-axis of R_world_to_rig (X_rig = R * (X - c_rig)).
//   intrinsic_parameters: intrinsics of the view's camera (CameraModel layout).
//   point: homogeneous 3D point (4 values), as in ReprojectionError.
//
// Constant data (the sensor extrinsics of the rig, Camera convention):
//   rotation_rig_to_cam: R_rig_to_cam (RigSensor::GetOrientationAsRotationMatrix).
//   position_in_rig: o_s, the camera center in the rig frame (RigSensor::position).
//
// The camera pose is composed inside the residual. With w = point[3] and
// X = point[0:3], the camera-frame point is
//   X_cam = R_rig_to_cam * (R_world_to_rig * (X - w * c_rig) - w * o_s),
// which is the homogeneous version of the camera pose given by
// ComposeCameraPoseFromRig: R_wc = R_rig_to_cam * R_wr and
// c_cam = c_rig + R_wr^T * o_s.
template <class CameraModel>
struct RigReprojectionError {
 public:
  RigReprojectionError(const Feature& feature,
                       const Eigen::Matrix3d& rotation_rig_to_cam,
                       const Eigen::Vector3d& position_in_rig)
      : feature_(feature),
        rotation_rig_to_cam_(rotation_rig_to_cam),
        position_in_rig_(position_in_rig) {}

  template <typename T>
  bool operator()(const T* capture_pose,
                  const T* intrinsic_parameters,
                  const T* point,
                  T* reprojection_error) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Map<const Vector3T> ConstMap3T;

    static const T kVerySmallNumber(1e-8);

    // X - w * c_rig.
    const Vector3T rig_center = ConstMap3T(capture_pose + Camera::POSITION);
    const Vector3T point_rel_rig = ConstMap3T(point) - point[3] * rig_center;

    // R_world_to_rig * (X - w * c_rig).
    Vector3T point_in_rig;
    ceres::AngleAxisRotatePoint(capture_pose + Camera::ORIENTATION,
                                point_rel_rig.data(),
                                point_in_rig.data());

    // R_rig_to_cam * (R_world_to_rig * (X - w * c_rig) - w * o_s).
    const Vector3T point_in_rig_frame =
        point_in_rig - point[3] * position_in_rig_.cast<T>();
    const Vector3T point_in_cam =
        rotation_rig_to_cam_.cast<T>() * point_in_rig_frame;

    // Same degenerate-point guard as ReprojectionError.
    if (point_in_cam.squaredNorm() < kVerySmallNumber) {
      return false;
    }

    // Apply the camera intrinsics to get the reprojected pixel.
    T reprojection[2];
    const bool res = CameraModel::CameraToPixelCoordinates(
        intrinsic_parameters, point_in_cam.data(), reprojection);

    // Whiten with the full 2x2 feature covariance.
    const Eigen::Matrix2d sqrt_information = feature_.SqrtInformation();
    const T dx = reprojection[0] - T(feature_.point_.x());
    const T dy = reprojection[1] - T(feature_.point_.y());
    reprojection_error[0] =
        T(sqrt_information(0, 0)) * dx + T(sqrt_information(0, 1)) * dy;
    reprojection_error[1] =
        T(sqrt_information(1, 0)) * dx + T(sqrt_information(1, 1)) * dy;
    return res;
  }

 private:
  const Feature feature_;
  const Eigen::Matrix3d rotation_rig_to_cam_;
  const Eigen::Vector3d position_in_rig_;
};

// Creates the autodiff RigReprojectionError cost function for the intrinsics
// model |camera_model_type|. The cost function has parameter block sizes
// {Camera::kExtrinsicsSize, intrinsics size, 4}. Returns nullptr for camera
// models that are not supported (ORTHOGRAPHIC).
ceres::CostFunction* CreateRigReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature,
    const Eigen::Matrix3d& rotation_rig_to_cam,
    const Eigen::Vector3d& position_in_rig);

// Wraps ANY cost function whose first parameter block is a camera extrinsics
// block (6 values, Camera::POSITION / Camera::ORIENTATION layout) so that this
// block becomes the pose of the rig body of a RigCapture. The camera pose is
// composed from the capture pose and the constant sensor extrinsics as in
// ComposeCameraPoseFromRig; the Jacobian of the first block is chained through
// the (autodiff) 6x6 composition Jacobian. Remaining blocks are passed through.
// This gives rig-constrained versions of the analytic and autodiff
// reprojection errors as well as of the position / orientation / gravity /
// depth priors without duplicating those functors. Takes ownership of |inner|.
ceres::CostFunction* WrapCostFunctionWithRigPose(
    ceres::CostFunction* inner,
    const Eigen::Matrix3d& rotation_rig_to_cam,
    const Eigen::Vector3d& position_in_rig);

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_RIG_REPROJECTION_ERROR_H_
