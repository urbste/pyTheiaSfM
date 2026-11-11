// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/bundle_adjustment/rig_reprojection_error.h"

#include <ceres/ceres.h>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model_type.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/full_opencv_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/sfm/camera/thin_prism_fisheye_camera_model.h"

#include <glog/logging.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace theia {

namespace {

// Camera extrinsics (position, angle-axis) from the rig capture pose.
struct ComposeCameraExtrinsicsFromRig {
  ComposeCameraExtrinsicsFromRig(const Eigen::Matrix3d& R_rig_to_cam,
                                 const Eigen::Vector3d& position_in_rig)
      : R_rig_to_cam_(R_rig_to_cam), position_in_rig_(position_in_rig) {}

  template <typename T>
  bool operator()(const T* capture_pose, T* camera_extrinsics) const {
    T R_wr[9];  // column major
    ceres::AngleAxisToRotationMatrix(capture_pose + Camera::ORIENTATION,
                                     ceres::ColumnMajorAdapter3x3(R_wr));
    Eigen::Map<const Eigen::Matrix<T, 3, 3>> Rwr(R_wr);
    const Eigen::Matrix<T, 3, 1> offset =
        Rwr.transpose() * position_in_rig_.cast<T>();
    for (int i = 0; i < 3; ++i) {
      camera_extrinsics[Camera::POSITION + i] =
          capture_pose[Camera::POSITION + i] + offset[i];
    }
    const Eigen::Matrix<T, 3, 3> Rwc = R_rig_to_cam_.cast<T>() * Rwr;
    T rwc_col_major[9];
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        rwc_col_major[c * 3 + r] = Rwc(r, c);
      }
    }
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(static_cast<const T*>(rwc_col_major)),
        camera_extrinsics + Camera::ORIENTATION);
    return true;
  }

  const Eigen::Matrix3d R_rig_to_cam_;
  const Eigen::Vector3d position_in_rig_;
};

class RigPoseCostFunction : public ceres::CostFunction {
 public:
  RigPoseCostFunction(ceres::CostFunction* inner,
                      const Eigen::Matrix3d& R_rig_to_cam,
                      const Eigen::Vector3d& position_in_rig)
      : inner_(inner),
        compose_(new ceres::AutoDiffCostFunction<ComposeCameraExtrinsicsFromRig,
                                                 6,
                                                 6>(
            new ComposeCameraExtrinsicsFromRig(R_rig_to_cam,
                                               position_in_rig))) {
    set_num_residuals(inner_->num_residuals());
    *mutable_parameter_block_sizes() = inner_->parameter_block_sizes();
    CHECK_EQ(parameter_block_sizes()[0], 6)
        << "First block must be the camera extrinsics.";
  }

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    const int num_blocks = static_cast<int>(parameter_block_sizes().size());
    double camera_extrinsics[6];
    double compose_jacobian[36];  // row major 6x6
    double* compose_jacobians[1] = {compose_jacobian};
    const bool need_jacobian = jacobians != nullptr && jacobians[0] != nullptr;
    const double* capture_pose[1] = {parameters[0]};
    if (!compose_->Evaluate(capture_pose,
                            camera_extrinsics,
                            need_jacobian ? compose_jacobians : nullptr)) {
      return false;
    }

    std::vector<const double*> inner_params(parameters, parameters + num_blocks);
    inner_params[0] = camera_extrinsics;

    const int num_res = num_residuals();
    std::vector<double> inner_jacobian0;
    std::vector<double*> inner_jacobians;
    if (jacobians != nullptr) {
      inner_jacobians.assign(jacobians, jacobians + num_blocks);
      if (need_jacobian) {
        inner_jacobian0.resize(num_res * 6);
        inner_jacobians[0] = inner_jacobian0.data();
      } else {
        inner_jacobians[0] = nullptr;
      }
    }
    if (!inner_->Evaluate(inner_params.data(),
                          residuals,
                          jacobians != nullptr ? inner_jacobians.data()
                                               : nullptr)) {
      return false;
    }
    if (need_jacobian) {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>>(
          jacobians[0], num_res, 6) =
          Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 6,
                                          Eigen::RowMajor>>(
              inner_jacobian0.data(), num_res, 6) *
          Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
              compose_jacobian);
    }
    return true;
  }

 private:
  std::unique_ptr<ceres::CostFunction> inner_;
  std::unique_ptr<ceres::CostFunction> compose_;
};

}  // namespace

ceres::CostFunction* WrapCostFunctionWithRigPose(
    ceres::CostFunction* inner,
    const Eigen::Matrix3d& rotation_rig_to_cam,
    const Eigen::Vector3d& position_in_rig) {
  return new RigPoseCostFunction(inner, rotation_rig_to_cam, position_in_rig);
}

ceres::CostFunction* CreateRigReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature,
    const Eigen::Matrix3d& rotation_rig_to_cam,
    const Eigen::Vector3d& position_in_rig) {
  static const int kResidualSize = 2;
  static const int kPointSize = 4;

#define THEIA_RIG_REPROJECTION_CASE(MODEL)                                  \
  return new ceres::AutoDiffCostFunction<RigReprojectionError<MODEL>,       \
                                         kResidualSize,                     \
                                         Camera::kExtrinsicsSize,           \
                                         MODEL::kIntrinsicsSize,            \
                                         kPointSize>(                       \
      new RigReprojectionError<MODEL>(feature, rotation_rig_to_cam,         \
                                      position_in_rig))

  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      THEIA_RIG_REPROJECTION_CASE(PinholeCameraModel);
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      THEIA_RIG_REPROJECTION_CASE(PinholeRadialTangentialCameraModel);
    case CameraIntrinsicsModelType::FISHEYE:
      THEIA_RIG_REPROJECTION_CASE(FisheyeCameraModel);
    case CameraIntrinsicsModelType::FOV:
      THEIA_RIG_REPROJECTION_CASE(FOVCameraModel);
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      THEIA_RIG_REPROJECTION_CASE(DivisionUndistortionCameraModel);
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      THEIA_RIG_REPROJECTION_CASE(DoubleSphereCameraModel);
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      THEIA_RIG_REPROJECTION_CASE(ExtendedUnifiedCameraModel);
    case CameraIntrinsicsModelType::FULL_OPENCV:
      THEIA_RIG_REPROJECTION_CASE(FullOpenCVCameraModel);
    case CameraIntrinsicsModelType::RADIAL_FISHEYE:
      THEIA_RIG_REPROJECTION_CASE(RadialFisheyeCameraModel);
    case CameraIntrinsicsModelType::THIN_PRISM_FISHEYE:
      THEIA_RIG_REPROJECTION_CASE(ThinPrismFisheyeCameraModel);
    case CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE:
      THEIA_RIG_REPROJECTION_CASE(RadTanThinPrismFisheyeCameraModel);
    case CameraIntrinsicsModelType::ORTHOGRAPHIC:
    default:
      return nullptr;
  }

#undef THEIA_RIG_REPROJECTION_CASE
}

}  // namespace theia
