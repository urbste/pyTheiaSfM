// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/rig/rig_capture.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include "theia/util/map_util.h"

namespace theia {

RigCapture::RigCapture() = default;

RigCapture::RigCapture(const theia::RigId rig_id, const double timestamp)
    : rig_id_(rig_id), timestamp_(timestamp) {}

void RigCapture::SetPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

void RigCapture::SetOrientationFromRotationMatrix(
    const Eigen::Matrix3d& rotation) {
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation.data()), orientation_.data());
}

void RigCapture::SetOrientationFromAngleAxis(
    const Eigen::Vector3d& angle_axis) {
  orientation_ = angle_axis;
}

Eigen::Matrix3d RigCapture::GetOrientationAsRotationMatrix() const {
  Eigen::Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(
      orientation_.data(),
      ceres::ColumnMajorAdapter3x3(rotation.data()));
  return rotation;
}

bool RigCapture::AddView(const RigCameraId rig_camera_id,
                         const ViewId view_id) {
  if (ContainsKey(view_ids_, rig_camera_id)) {
    return false;
  }
  view_ids_[rig_camera_id] = view_id;
  return true;
}

bool RigCapture::RemoveView(const RigCameraId rig_camera_id) {
  return view_ids_.erase(rig_camera_id) > 0;
}

ViewId RigCapture::ViewIdForCamera(const RigCameraId rig_camera_id) const {
  return FindWithDefault(view_ids_, rig_camera_id, kInvalidViewId);
}

std::vector<ViewId> RigCapture::GetViewIds() const {
  std::vector<ViewId> ids;
  ids.reserve(view_ids_.size());
  for (const auto& entry : view_ids_) {
    ids.push_back(entry.second);
  }
  return ids;
}

}  // namespace theia
