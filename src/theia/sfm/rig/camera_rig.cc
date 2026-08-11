// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/rig/camera_rig.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include "theia/util/map_util.h"

namespace theia {

Eigen::Matrix3d RigSensor::GetOrientationAsRotationMatrix() const {
  Eigen::Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(
      orientation.data(),
      ceres::ColumnMajorAdapter3x3(rotation.data()));
  return rotation;
}

void RigSensor::SetOrientationFromRotationMatrix(
    const Eigen::Matrix3d& rotation) {
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation.data()), orientation.data());
}

void RigSensor::SetOrientationFromAngleAxis(
    const Eigen::Vector3d& angle_axis) {
  orientation = angle_axis;
}

void RigSensor::SetPosition(const Eigen::Vector3d& position_in_rig) {
  position = position_in_rig;
}

RigCameraId CameraRig::AddSensor(
    const std::string& sensor_name,
    const Eigen::Vector3d& position_in_rig,
    const Eigen::Vector3d& orientation_rig_to_cam) {
  RigSensor sensor;
  sensor.name = sensor_name;
  sensor.position = position_in_rig;
  sensor.orientation = orientation_rig_to_cam;
  return AddSensor(sensor);
}

RigCameraId CameraRig::AddSensor(const RigSensor& sensor) {
  const RigCameraId id = next_rig_camera_id_++;
  sensors_[id] = sensor;
  return id;
}

bool CameraRig::HasSensor(const RigCameraId rig_camera_id) const {
  return ContainsKey(sensors_, rig_camera_id);
}

const RigSensor* CameraRig::GetSensor(
    const RigCameraId rig_camera_id) const {
  return FindOrNull(sensors_, rig_camera_id);
}

RigSensor* CameraRig::MutableSensor(const RigCameraId rig_camera_id) {
  return FindOrNull(sensors_, rig_camera_id);
}

std::vector<RigCameraId> CameraRig::SensorIds() const {
  std::vector<RigCameraId> ids;
  ids.reserve(sensors_.size());
  for (const auto& sensor : sensors_) {
    ids.push_back(sensor.first);
  }
  return ids;
}

}  // namespace theia
