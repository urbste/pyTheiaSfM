// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/rig/rig_utils.h"

#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/Dense>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/pose/util.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"

namespace theia {

void ComposeCameraPoseFromRig(const Eigen::Vector3d& rig_position,
                              const Eigen::Matrix3d& rig_orientation_w_to_r,
                              const RigSensor& sensor,
                              Eigen::Vector3d* camera_position,
                              Eigen::Matrix3d* camera_orientation_w_to_c) {
  CHECK_NOTNULL(camera_position);
  CHECK_NOTNULL(camera_orientation_w_to_c);

  const Eigen::Matrix3d R_r_to_c = sensor.GetOrientationAsRotationMatrix();
  // X_cam = R_rc * (X_rig - c_sensor)
  // X_rig = R_wr * (X_world - c_rig)
  // => X_cam = R_rc * R_wr * (X_world - (c_rig + R_wr^T * c_sensor))
  *camera_orientation_w_to_c = R_r_to_c * rig_orientation_w_to_r;
  *camera_position =
      rig_position +
      rig_orientation_w_to_r.transpose() * sensor.position;
}

void ComposeRigPoseFromCamera(const Eigen::Vector3d& camera_position,
                              const Eigen::Matrix3d& camera_orientation_w_to_c,
                              const RigSensor& sensor,
                              Eigen::Vector3d* rig_position,
                              Eigen::Matrix3d* rig_orientation_w_to_r) {
  CHECK_NOTNULL(rig_position);
  CHECK_NOTNULL(rig_orientation_w_to_r);

  const Eigen::Matrix3d R_r_to_c = sensor.GetOrientationAsRotationMatrix();
  // R_wc = R_rc * R_wr  =>  R_wr = R_rc^T * R_wc
  *rig_orientation_w_to_r = R_r_to_c.transpose() * camera_orientation_w_to_c;
  // c_cam = c_rig + R_wr^T * c_sensor
  // => c_rig = c_cam - R_wr^T * c_sensor
  *rig_position = camera_position -
                  rig_orientation_w_to_r->transpose() * sensor.position;
}

void SetCameraPoseFromRig(const RigCapture& capture,
                          const RigSensor& sensor,
                          Camera* camera) {
  CHECK_NOTNULL(camera);
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
  ComposeCameraPoseFromRig(capture.GetPosition(),
                           capture.GetOrientationAsRotationMatrix(),
                           sensor,
                           &position,
                           &orientation);
  camera->SetPosition(position);
  camera->SetOrientationFromRotationMatrix(orientation);
}

bool PropagateCameraPosesForCapture(const CaptureId capture_id,
                                    Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  RigCapture* capture = reconstruction->MutableRigCapture(capture_id);
  if (capture == nullptr) {
    LOG(WARNING) << "Capture " << capture_id << " does not exist.";
    return false;
  }
  const CameraRig* rig = reconstruction->GetCameraRig(capture->GetRigId());
  if (rig == nullptr) {
    LOG(WARNING) << "Rig " << capture->GetRigId() << " does not exist.";
    return false;
  }

  for (const auto& sensor_and_view : capture->ViewIds()) {
    const RigCameraId rig_camera_id = sensor_and_view.first;
    const ViewId view_id = sensor_and_view.second;
    const RigSensor* sensor = rig->GetSensor(rig_camera_id);
    View* view = reconstruction->MutableView(view_id);
    if (sensor == nullptr || view == nullptr) {
      LOG(WARNING) << "Missing sensor or view for capture " << capture_id;
      continue;
    }
    SetCameraPoseFromRig(*capture, *sensor, view->MutableCamera());
    view->SetEstimated(capture->IsEstimated());
  }
  return true;
}

void PropagateAllEstimatedCapturePoses(Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  for (const CaptureId capture_id : reconstruction->CaptureIds()) {
    const RigCapture* capture = reconstruction->GetRigCapture(capture_id);
    if (capture != nullptr && capture->IsEstimated()) {
      PropagateCameraPosesForCapture(capture_id, reconstruction);
    }
  }
}

bool SetCapturePoseFromMemberViews(const CaptureId capture_id,
                                   Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);
  RigCapture* capture = reconstruction->MutableRigCapture(capture_id);
  if (capture == nullptr || capture->ViewIds().empty()) {
    return false;
  }
  const CameraRig* rig = reconstruction->GetCameraRig(capture->GetRigId());
  if (rig == nullptr) {
    return false;
  }

  Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_sum = Eigen::Matrix3d::Zero();
  int count = 0;
  for (const auto& sensor_and_view : capture->ViewIds()) {
    const View* view = reconstruction->View(sensor_and_view.second);
    const RigSensor* sensor = rig->GetSensor(sensor_and_view.first);
    if (view == nullptr || sensor == nullptr || !view->IsEstimated()) {
      continue;
    }
    Eigen::Vector3d rig_position;
    Eigen::Matrix3d rig_orientation;
    ComposeRigPoseFromCamera(view->Camera().GetPosition(),
                             view->Camera().GetOrientationAsRotationMatrix(),
                             *sensor,
                             &rig_position,
                             &rig_orientation);
    mean_position += rig_position;
    rotation_sum += rig_orientation;
    ++count;
  }
  if (count == 0) {
    return false;
  }
  mean_position /= static_cast<double>(count);
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      rotation_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d mean_rotation = svd.matrixU() * svd.matrixV().transpose();
  if (mean_rotation.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    mean_rotation = U * svd.matrixV().transpose();
  }
  capture->SetPosition(mean_position);
  capture->SetOrientationFromRotationMatrix(mean_rotation);
  capture->SetEstimated(true);
  return true;
}

Eigen::Matrix3d EssentialMatrixFromRigSensors(const RigSensor& from_sensor,
                                              const RigSensor& to_sensor) {
  const Eigen::Matrix3d R_from = from_sensor.GetOrientationAsRotationMatrix();
  const Eigen::Matrix3d R_to = to_sensor.GetOrientationAsRotationMatrix();
  const Eigen::Matrix3d R = R_to * R_from.transpose();
  const Eigen::Vector3d t =
      R_to * (from_sensor.position - to_sensor.position);
  Eigen::Matrix3d t_cross;
  t_cross << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  return t_cross * R;
}

bool FilterCorrespondencesWithEssential(
    const Eigen::Matrix3d& essential_matrix,
    const Camera& camera1,
    const Camera& camera2,
    const std::vector<FeatureCorrespondence>& correspondences,
    double max_sampson_error_pixels,
    std::vector<int>* inlier_indices) {
  CHECK_NOTNULL(inlier_indices)->clear();
  if (correspondences.empty()) {
    return false;
  }
  const double focal = std::max(1.0, 0.5 * (camera1.FocalLength() +
                                            camera2.FocalLength()));
  const double sq_thresh =
      (max_sampson_error_pixels / focal) * (max_sampson_error_pixels / focal);
  inlier_indices->reserve(correspondences.size());
  for (int i = 0; i < static_cast<int>(correspondences.size()); ++i) {
    const Eigen::Vector3d n1 =
        camera1.PixelToNormalizedCoordinates(correspondences[i].feature1.point_);
    const Eigen::Vector3d n2 =
        camera2.PixelToNormalizedCoordinates(correspondences[i].feature2.point_);
    if (std::abs(n1.z()) < 1e-12 || std::abs(n2.z()) < 1e-12) {
      continue;
    }
    const Eigen::Vector2d x1 = n1.hnormalized();
    const Eigen::Vector2d x2 = n2.hnormalized();
    if (SquaredSampsonDistance(essential_matrix, x1, x2) < sq_thresh) {
      inlier_indices->push_back(i);
    }
  }
  return !inlier_indices->empty();
}

}  // namespace theia
