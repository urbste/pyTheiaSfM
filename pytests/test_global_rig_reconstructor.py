"""Tests for capture view graph + GlobalRigReconstructor wiring."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def test_build_capture_view_graph_strips_extrinsics():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)

    c0 = recon.AddRigCapture(rid, 0.0, {left: "l0.png", right: "r0.png"})
    c1 = recon.AddRigCapture(rid, 1.0, {left: "l1.png", right: "r1.png"})
    v_l0 = recon.GetRigCapture(c0).ViewIdForCamera(left)
    v_l1 = recon.GetRigCapture(c1).ViewIdForCamera(left)

    vg = pt.sfm.ViewGraph()
    info = pt.sfm.TwoViewInfo()
    info.rotation_2 = np.array([0.0, 0.1, 0.0])
    info.position_2 = np.array([0.2, 0.0, 1.0])
    info.num_verified_matches = 100
    info.focal_length_1 = 500.0
    info.focal_length_2 = 500.0
    vg.AddEdge(v_l0, v_l1, info)

    # Intra-capture stereo edge should be ignored for capture graph motion.
    stereo = pt.sfm.TwoViewInfo()
    stereo.rotation_2 = np.zeros(3)
    stereo.position_2 = np.array([0.1, 0.0, 0.0])
    stereo.num_verified_matches = 200
    v_r0 = recon.GetRigCapture(c0).ViewIdForCamera(right)
    vg.AddEdge(v_l0, v_r0, stereo)

    cg = pt.sfm.ViewGraph()
    assert pt.sfm.BuildCaptureViewGraph(recon, vg, cg)
    assert cg.NumEdges() == 1


def test_global_rig_reconstructor_options_expose_averaging_backends():
    opts = pt.sfm.GlobalRigReconstructorOptions()
    opts.sfm_options.global_rotation_estimator_type = (
        pt.sfm.GlobalRotationEstimatorType.NONLINEAR
    )
    opts.sfm_options.global_position_estimator_type = (
        pt.sfm.GlobalPositionEstimatorType.GLOMAP
    )
    _ = pt.sfm.GlobalRigReconstructor(opts)
