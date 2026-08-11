// Copyright (C) 2026 The pyTheiaSfM Authors.
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
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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

#ifndef THEIA_SFM_RIG_CAMERA_RIG_H_
#define THEIA_SFM_RIG_CAMERA_RIG_H_

#include <Eigen/Core>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "theia/io/eigen_serializable.h"
#include "theia/sfm/types.h"

namespace theia {

// A sensor slot mounted on a CameraRig. Pose uses the same convention as
// Camera: |position| is the camera center expressed in the abstract rig /
// body frame, and |orientation| is the angle-axis rotation taking a point
// from the rig frame into the camera frame:
//   X_cam = R_rig_to_cam * (X_rig - position)
struct RigSensor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string name;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation = Eigen::Vector3d::Zero();  // angle-axis
  bool optimize_extrinsics = false;
  CameraIntrinsicsGroupId intrinsics_group_id =
      kInvalidCameraIntrinsicsGroupId;

  Eigen::Matrix3d GetOrientationAsRotationMatrix() const;
  void SetOrientationFromRotationMatrix(const Eigen::Matrix3d& rotation);
  void SetOrientationFromAngleAxis(const Eigen::Vector3d& angle_axis);
  void SetPosition(const Eigen::Vector3d& position_in_rig);

 private:
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(name,
       position,
       orientation,
       optimize_extrinsics,
       intrinsics_group_id);
  }
};

// Defines a multi-camera rig in an abstract body / rig coordinate system.
// The body frame is not required to coincide with any physical camera; sensor
// extrinsics are expressed relative to that frame. The first RigCapture of a
// trajectory typically places this body at the world identity.
class CameraRig {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraRig() = default;
  explicit CameraRig(const std::string& name) : name_(name) {}

  const std::string& Name() const { return name_; }
  void SetName(const std::string& name) { name_ = name; }

  // Adds a sensor and returns its RigCameraId. Ids are dense starting at 0.
  RigCameraId AddSensor(const std::string& sensor_name,
                        const Eigen::Vector3d& position_in_rig,
                        const Eigen::Vector3d& orientation_rig_to_cam);

  RigCameraId AddSensor(const RigSensor& sensor);

  bool HasSensor(const RigCameraId rig_camera_id) const;
  const RigSensor* GetSensor(const RigCameraId rig_camera_id) const;
  RigSensor* MutableSensor(const RigCameraId rig_camera_id);

  std::vector<RigCameraId> SensorIds() const;
  int NumSensors() const { return static_cast<int>(sensors_.size()); }

 private:
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(name_, next_rig_camera_id_, sensors_);
  }

  std::string name_;
  RigCameraId next_rig_camera_id_ = 0;
  // Use aligned map so Eigen members in RigSensor stay safe.
  aligned_unordered_map<RigCameraId, RigSensor> sensors_;
};

}  // namespace theia

CEREAL_CLASS_VERSION(theia::RigSensor, 0);
CEREAL_CLASS_VERSION(theia::CameraRig, 0);

#endif  // THEIA_SFM_RIG_CAMERA_RIG_H_
