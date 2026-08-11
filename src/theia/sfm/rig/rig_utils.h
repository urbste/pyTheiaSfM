// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Helpers to compose View camera extrinsics from an abstract-body RigCapture
// pose and CameraRig sensor extrinsics.

#ifndef THEIA_SFM_RIG_RIG_UTILS_H_
#define THEIA_SFM_RIG_RIG_UTILS_H_

#include <Eigen/Core>

#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/types.h"

namespace theia {

class Camera;
class Reconstruction;

// Compose world camera extrinsics from a rig body pose and a sensor pose in
// the rig frame (both using Camera's convention).
void ComposeCameraPoseFromRig(const Eigen::Vector3d& rig_position,
                              const Eigen::Matrix3d& rig_orientation_w_to_r,
                              const RigSensor& sensor,
                              Eigen::Vector3d* camera_position,
                              Eigen::Matrix3d* camera_orientation_w_to_c);

// Inverse: recover the abstract body pose from a localized camera that
// belongs to |sensor|.
void ComposeRigPoseFromCamera(const Eigen::Vector3d& camera_position,
                              const Eigen::Matrix3d& camera_orientation_w_to_c,
                              const RigSensor& sensor,
                              Eigen::Vector3d* rig_position,
                              Eigen::Matrix3d* rig_orientation_w_to_r);

// Write composed extrinsics into |camera|.
void SetCameraPoseFromRig(const RigCapture& capture,
                          const RigSensor& sensor,
                          Camera* camera);

// For every view in |capture_id|, set Camera extrinsics from the capture pose
// and the parent CameraRig. Returns false if the capture/rig is missing.
bool PropagateCameraPosesForCapture(const CaptureId capture_id,
                                    Reconstruction* reconstruction);

// Propagate poses for all estimated captures in the reconstruction.
void PropagateAllEstimatedCapturePoses(Reconstruction* reconstruction);

}  // namespace theia

#endif  // THEIA_SFM_RIG_RIG_UTILS_H_
