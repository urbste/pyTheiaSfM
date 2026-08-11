# Copyright (C) 2026 The pyTheiaSfM Authors.
"""Smoke tests for CameraRig / RigCapture data model and pose composition."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def test_add_rig_capture_shares_timestamp():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rig_id = recon.AddCameraRig(rig)

    cap = recon.AddRigCapture(
        rig_id,
        1.0,
        {left: "frame0_left.png", right: "frame0_right.png"},
    )
    assert cap != pt.sfm.kInvalidCaptureId if hasattr(pt.sfm, "kInvalidCaptureId") else cap is not None
    assert recon.NumCaptures() == 1
    assert recon.NumViews() == 2

    capture = recon.GetRigCapture(cap)
    assert capture.GetTimestamp() == 1.0
    assert capture.NumViews() == 2
    assert not capture.IsEstimated()


def test_propagate_camera_poses_from_abstract_body():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.1, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.1, 0.0, 0.0]), np.zeros(3))
    rig_id = recon.AddCameraRig(rig)
    cap = recon.AddRigCapture(
        rig_id, 0.0, {left: "l.png", right: "r.png"}
    )

    capture = recon.MutableRigCapture(cap)
    capture.SetPosition(np.array([1.0, 2.0, 3.0]))
    capture.SetOrientationFromAngleAxis(np.zeros(3))
    capture.SetEstimated(True)
    assert pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    left_view = recon.View(capture.ViewIdForCamera(left))
    right_view = recon.View(capture.ViewIdForCamera(right))
    np.testing.assert_allclose(
        left_view.Camera().GetPosition(), np.array([0.9, 2.0, 3.0]), atol=1e-9
    )
    np.testing.assert_allclose(
        right_view.Camera().GetPosition(), np.array([1.1, 2.0, 3.0]), atol=1e-9
    )
    assert left_view.IsEstimated()
    assert right_view.IsEstimated()
