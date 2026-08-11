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

#ifndef THEIA_SFM_RIG_RIG_CAPTURE_H_
#define THEIA_SFM_RIG_RIG_CAPTURE_H_

#include <Eigen/Core>
#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/unordered_map.hpp>
#include <stdint.h>
#include <unordered_map>
#include <vector>

#include "theia/io/eigen_serializable.h"

#include "theia/sfm/types.h"

namespace theia {

// One synchronized multi-camera sample: a single abstract-body pose at a
// unique timestamp for a given CameraRig.
class RigCapture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RigCapture();
  RigCapture(const RigId rig_id, const double timestamp);

  theia::RigId GetRigId() const { return rig_id_; }
  void SetRigId(const theia::RigId rig_id) { rig_id_ = rig_id; }

  double GetTimestamp() const { return timestamp_; }
  void SetTimestamp(const double timestamp) { timestamp_ = timestamp; }

  void SetEstimated(const bool is_estimated) { is_estimated_ = is_estimated; }
  bool IsEstimated() const { return is_estimated_; }

  void SetPosition(const Eigen::Vector3d& position);
  Eigen::Vector3d GetPosition() const { return position_; }

  void SetOrientationFromRotationMatrix(const Eigen::Matrix3d& rotation);
  void SetOrientationFromAngleAxis(const Eigen::Vector3d& angle_axis);
  Eigen::Matrix3d GetOrientationAsRotationMatrix() const;
  Eigen::Vector3d GetOrientationAsAngleAxis() const { return orientation_; }

  // View membership for this capture (sensor slot -> ViewId).
  bool AddView(const RigCameraId rig_camera_id, const ViewId view_id);
  bool RemoveView(const RigCameraId rig_camera_id);
  ViewId ViewIdForCamera(const RigCameraId rig_camera_id) const;
  const std::unordered_map<RigCameraId, ViewId>& ViewIds() const {
    return view_ids_;
  }
  std::vector<ViewId> GetViewIds() const;
  int NumViews() const { return static_cast<int>(view_ids_.size()); }

 private:
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(rig_id_,
       timestamp_,
       is_estimated_,
       position_,
       orientation_,
       view_ids_);
  }

  theia::RigId rig_id_ = kInvalidRigId;
  double timestamp_ = 0.0;
  bool is_estimated_ = false;
  // Pose of the abstract body / rig frame in the world, Camera convention:
  // X_rig = R_world_to_rig * (X_world - position).
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation_ = Eigen::Vector3d::Zero();
  std::unordered_map<RigCameraId, ViewId> view_ids_;
};

// Optional membership of a View in a rig capture.
struct ViewRigMembership {
  RigId rig_id = kInvalidRigId;
  RigCameraId rig_camera_id = kInvalidRigCameraId;
  CaptureId capture_id = kInvalidCaptureId;

 private:
  friend class cereal::access;
  template <class Archive>
  void serialize(Archive& ar, const std::uint32_t version) {  // NOLINT
    ar(rig_id, rig_camera_id, capture_id);
  }
};

}  // namespace theia

CEREAL_CLASS_VERSION(theia::RigCapture, 0);
CEREAL_CLASS_VERSION(theia::ViewRigMembership, 0);

#endif  // THEIA_SFM_RIG_RIG_CAPTURE_H_
