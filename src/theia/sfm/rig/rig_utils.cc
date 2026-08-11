// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/rig/rig_utils.h"

#include <glog/logging.h>

#include "theia/sfm/camera/camera.h"
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

}  // namespace theia
