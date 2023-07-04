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

#ifndef THEIA_SFM_CAMERA_CREATE_REPROJECTION_ERROR_COST_FUNCTION_H_
#define THEIA_SFM_CAMERA_CREATE_REPROJECTION_ERROR_COST_FUNCTION_H_

#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/orthographic_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/camera/reprojection_error.h"

namespace theia {
// Create the appropriate reprojection error cost function based on the camera
// intrinsics model that is passed in. The ReprojectionError struct is templated
// on the camera intrinsics model class and so it will automatically model the
// reprojection error appropriately.
inline ceres::CostFunction* CreateReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature) {
  static const int kResidualSize = 2;
  static const int kPointSize = 4;
  // Return the appropriate reprojection error cost function based on the camera
  // model type.
  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<PinholeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          PinholeCameraModel::kIntrinsicsSize,
          kPointSize>(new ReprojectionError<PinholeCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<PinholeRadialTangentialCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          PinholeRadialTangentialCameraModel::kIntrinsicsSize,
          kPointSize>(
          new ReprojectionError<PinholeRadialTangentialCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::FISHEYE:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<FisheyeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          FisheyeCameraModel::kIntrinsicsSize,
          kPointSize>(new ReprojectionError<FisheyeCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::FOV:
      return new ceres::AutoDiffCostFunction<ReprojectionError<FOVCameraModel>,
                                             kResidualSize,
                                             Camera::kExtrinsicsSize,
                                             FOVCameraModel::kIntrinsicsSize,
                                             kPointSize>(
          new ReprojectionError<FOVCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<DivisionUndistortionCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          DivisionUndistortionCameraModel::kIntrinsicsSize,
          kPointSize>(
          new ReprojectionError<DivisionUndistortionCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<DoubleSphereCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          DoubleSphereCameraModel::kIntrinsicsSize,
          kPointSize>(new ReprojectionError<DoubleSphereCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<ExtendedUnifiedCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          ExtendedUnifiedCameraModel::kIntrinsicsSize,
          kPointSize>(
          new ReprojectionError<ExtendedUnifiedCameraModel>(feature));
      break;
    case CameraIntrinsicsModelType::ORTHOGRAPHIC:
      return new ceres::AutoDiffCostFunction<
          ReprojectionError<OrthographicCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          OrthographicCameraModel::kIntrinsicsSize,
          kPointSize>(
          new ReprojectionError<OrthographicCameraModel>(feature));
      break;
    default:
      LOG(FATAL) << "Invalid camera type. Please see camera_intrinsics_model.h "
                    "for a list of valid camera models.";
      return NULL;
      break;
  }
}

inline ceres::CostFunction* CreateInvReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature, const Eigen::Vector3d& ref_bearing) {
  static const int kResidualSize = 2;
  static const int kPointSize = 1;
  // Return the appropriate reprojection error cost function based on the camera
  // model type.
  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<PinholeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          PinholeCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionError<PinholeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<PinholeRadialTangentialCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          PinholeRadialTangentialCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionError<PinholeRadialTangentialCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FISHEYE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<FisheyeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          FisheyeCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionError<FisheyeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FOV:
      return new ceres::AutoDiffCostFunction<InvReprojectionError<FOVCameraModel>,
                                             kResidualSize,
                                             Camera::kExtrinsicsSize, // anch
                                             Camera::kExtrinsicsSize, // other
                                             FOVCameraModel::kIntrinsicsSize,
                                             kPointSize>(
          new InvReprojectionError<FOVCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<DivisionUndistortionCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          DivisionUndistortionCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionError<DivisionUndistortionCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<DoubleSphereCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          DoubleSphereCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionError<DoubleSphereCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionError<ExtendedUnifiedCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, // anch
          Camera::kExtrinsicsSize, // other
          ExtendedUnifiedCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionError<ExtendedUnifiedCameraModel>(feature, ref_bearing));
      break;
    default:
      LOG(FATAL) << "Invalid camera type. Please see camera_intrinsics_model.h "
                    "for a list of valid camera models.";
      return NULL;
      break;
  }
}

inline ceres::CostFunction* CreateInvReprojectionPoseErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature, const Eigen::Vector3d& ref_bearing) {
  static const int kResidualSize = 2;
  static const int kPointSize = 1;
  // Return the appropriate reprojection error cost function based on the camera
  // model type.
  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<PinholeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, 
          PinholeCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionPoseError<PinholeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<PinholeRadialTangentialCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          PinholeRadialTangentialCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionPoseError<PinholeRadialTangentialCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FISHEYE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<FisheyeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          FisheyeCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionPoseError<FisheyeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FOV:
      return new ceres::AutoDiffCostFunction<InvReprojectionPoseError<FOVCameraModel>,
                                             kResidualSize,
                                             Camera::kExtrinsicsSize,
                                             FOVCameraModel::kIntrinsicsSize,
                                             kPointSize>(
          new InvReprojectionPoseError<FOVCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<DivisionUndistortionCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          DivisionUndistortionCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionPoseError<DivisionUndistortionCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<DoubleSphereCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize,
          DoubleSphereCameraModel::kIntrinsicsSize,
          kPointSize>(new InvReprojectionPoseError<DoubleSphereCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      return new ceres::AutoDiffCostFunction<
          InvReprojectionPoseError<ExtendedUnifiedCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize, 
          ExtendedUnifiedCameraModel::kIntrinsicsSize,
          kPointSize>(
          new InvReprojectionPoseError<ExtendedUnifiedCameraModel>(feature, ref_bearing));
      break;
    default:
      LOG(FATAL) << "Invalid camera type. Please see camera_intrinsics_model.h "
                    "for a list of valid camera models.";
      return NULL;
      break;
  }
}

inline ceres::CostFunction* CreateSim3InvReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature, const Eigen::Vector3d& ref_bearing) {
  static const int kResidualSize = 2;
  static const int kPointSize = 1;
  // Return the appropriate reprojection error cost function based on the camera
  // model type.
  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<PinholeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          PinholeCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionError<PinholeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<PinholeRadialTangentialCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          PinholeRadialTangentialCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionError<PinholeRadialTangentialCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FISHEYE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<FisheyeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          FisheyeCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionError<FisheyeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FOV:
      return new ceres::AutoDiffCostFunction<Sim3InvReprojectionError<FOVCameraModel>,
                                             kResidualSize,
                                             Camera::kExtrinsicsSize + 1, // anch
                                             Camera::kExtrinsicsSize + 1, // other
                                             FOVCameraModel::kIntrinsicsSize,
                                             kPointSize>(
          new Sim3InvReprojectionError<FOVCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<DivisionUndistortionCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          DivisionUndistortionCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionError<DivisionUndistortionCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<DoubleSphereCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          DoubleSphereCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionError<DoubleSphereCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionError<ExtendedUnifiedCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          Camera::kExtrinsicsSize + 1, // other
          ExtendedUnifiedCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionError<ExtendedUnifiedCameraModel>(feature, ref_bearing));
      break;
    default:
      LOG(FATAL) << "Invalid camera type. Please see camera_intrinsics_model.h "
                    "for a list of valid camera models.";
      return NULL;
      break;
  }
}

inline ceres::CostFunction* CreateSim3InvReprojectionPoseErrorCostFunction(
    const CameraIntrinsicsModelType& camera_model_type,
    const Feature& feature, const Eigen::Vector3d& ref_bearing) {
  static const int kResidualSize = 2;
  static const int kPointSize = 1;
  // Return the appropriate reprojection error cost function based on the camera
  // model type.
  switch (camera_model_type) {
    case CameraIntrinsicsModelType::PINHOLE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<PinholeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          PinholeCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionPoseError<PinholeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<PinholeRadialTangentialCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          PinholeRadialTangentialCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionPoseError<PinholeRadialTangentialCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FISHEYE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<FisheyeCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          FisheyeCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionPoseError<FisheyeCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::FOV:
      return new ceres::AutoDiffCostFunction<Sim3InvReprojectionPoseError<FOVCameraModel>,
                                             kResidualSize,
                                             Camera::kExtrinsicsSize + 1, // anch
                                             FOVCameraModel::kIntrinsicsSize,
                                             kPointSize>(
          new Sim3InvReprojectionPoseError<FOVCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DIVISION_UNDISTORTION:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<DivisionUndistortionCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          DivisionUndistortionCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionPoseError<DivisionUndistortionCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::DOUBLE_SPHERE:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<DoubleSphereCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          DoubleSphereCameraModel::kIntrinsicsSize,
          kPointSize>(new Sim3InvReprojectionPoseError<DoubleSphereCameraModel>(feature, ref_bearing));
      break;
    case CameraIntrinsicsModelType::EXTENDED_UNIFIED:
      return new ceres::AutoDiffCostFunction<
          Sim3InvReprojectionPoseError<ExtendedUnifiedCameraModel>,
          kResidualSize,
          Camera::kExtrinsicsSize + 1, // anch
          ExtendedUnifiedCameraModel::kIntrinsicsSize,
          kPointSize>(
          new Sim3InvReprojectionPoseError<ExtendedUnifiedCameraModel>(feature, ref_bearing));
      break;
    default:
      LOG(FATAL) << "Invalid camera type. Please see camera_intrinsics_model.h "
                    "for a list of valid camera models.";
      return NULL;
      break;
  }
}

}  // namespace theia

#endif  // THEIA_SFM_CAMERA_CREATE_REPROJECTION_ERROR_COST_FUNCTION_H_
