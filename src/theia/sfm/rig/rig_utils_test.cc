// Copyright (C) 2026 The pyTheiaSfM Authors.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "gtest/gtest.h"

#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/rig/rig_utils.h"
#include "theia/sfm/reconstruction.h"

namespace theia {
namespace {

TEST(RigUtils, ComposeAndInvertPose) {
  RigSensor sensor;
  sensor.position = Eigen::Vector3d(0.1, -0.2, 0.05);
  sensor.SetOrientationFromAngleAxis(Eigen::Vector3d(0.01, -0.02, 0.03));

  const Eigen::Vector3d rig_position(1.0, 2.0, 3.0);
  const Eigen::Matrix3d rig_orientation =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();

  Eigen::Vector3d cam_position;
  Eigen::Matrix3d cam_orientation;
  ComposeCameraPoseFromRig(
      rig_position, rig_orientation, sensor, &cam_position, &cam_orientation);

  Eigen::Vector3d recovered_rig_position;
  Eigen::Matrix3d recovered_rig_orientation;
  ComposeRigPoseFromCamera(cam_position,
                           cam_orientation,
                           sensor,
                           &recovered_rig_position,
                           &recovered_rig_orientation);

  EXPECT_TRUE(rig_position.isApprox(recovered_rig_position, 1e-10));
  EXPECT_TRUE(rig_orientation.isApprox(recovered_rig_orientation, 1e-10));
}

TEST(Reconstruction, AddRigCaptureSharesTimestamp) {
  Reconstruction reconstruction;
  CameraRig rig("stereo");
  const RigCameraId left =
      rig.AddSensor("left", Eigen::Vector3d(-0.05, 0, 0), Eigen::Vector3d::Zero());
  const RigCameraId right =
      rig.AddSensor("right", Eigen::Vector3d(0.05, 0, 0), Eigen::Vector3d::Zero());
  const RigId rig_id = reconstruction.AddCameraRig(rig);

  const CaptureId capture_id = reconstruction.AddRigCapture(
      rig_id,
      1.25,
      {{left, "l.png"}, {right, "r.png"}});

  EXPECT_NE(capture_id, kInvalidCaptureId);
  EXPECT_EQ(reconstruction.NumViews(), 2);
  EXPECT_EQ(reconstruction.NumCaptures(), 1);
  EXPECT_EQ(reconstruction.GetRigCapture(capture_id)->GetTimestamp(), 1.25);
  EXPECT_TRUE(reconstruction.ViewHasRigMembership(
      reconstruction.GetRigCapture(capture_id)->ViewIdForCamera(left)));
}

}  // namespace
}  // namespace theia
