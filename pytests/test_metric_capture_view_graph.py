"""Metric capture view graph via EstimateRelativeRigInfo (5+1)."""

from __future__ import annotations

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def _prior(focal=500.0, cx=320.0, cy=240.0, w=640, h=480):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [float(focal)]
    prior.focal_length.is_set = True
    prior.principal_point.value = [float(cx), float(cy)]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [1.0]
    prior.aspect_ratio.is_set = True
    prior.image_width = int(w)
    prior.image_height = int(h)
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def _project(cam, X):
    depth, uv = cam.ProjectPoint(np.append(X, 1.0))
    return depth, np.asarray(uv).reshape(2)


def test_build_capture_view_graph_metric_from_tracks():
    """Synthetic stereo: metric capture edge recovers known baseline motion."""
    recon = pt.sfm.Reconstruction()
    half_b = 0.06
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-half_b, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([half_b, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)

    prior = _prior()
    # Motion along +Z (forward). Pure translation parallel to the stereo
    # baseline (X) is degenerate for 5+1 scale recovery.
    caps = []
    for i, t_pos in enumerate([0.0, 0.4]):
        cap = recon.AddRigCapture(
            rid, float(i), {left: f"l{i}.png", right: f"r{i}.png"}
        )
        caps.append(cap)
        capture = recon.MutableRigCapture(cap)
        capture.SetPosition(np.array([0.0, 0.0, t_pos]))
        capture.SetOrientationFromAngleAxis(np.zeros(3))
        capture.SetEstimated(True)
        pt.sfm.PropagateCameraPosesForCapture(cap, recon)
        for vid in capture.GetViewIds():
            v = recon.MutableView(vid)
            v.SetCameraIntrinsicsPrior(prior)
            v.MutableCamera().SetFromCameraIntrinsicsPriors(prior)
            # Restore pose after SetFromPriors may reset extrinsics.
        pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

    # Build tracks from synthetic projections (left-left + left-right + right-right).
    rng = np.random.default_rng(0)
    tb = pt.sfm.TrackBuilder(2, 50)
    points = [
        rng.uniform([-1.0, -0.7, 2.0], [1.0, 0.7, 6.0]) for _ in range(60)
    ]

    def view_ids(cap_id):
        c = recon.GetRigCapture(cap_id)
        return c.ViewIdForCamera(left), c.ViewIdForCamera(right)

    vl0, vr0 = view_ids(caps[0])
    vl1, vr1 = view_ids(caps[1])

    for X in points:
        feats = {}
        for vid in (vl0, vr0, vl1, vr1):
            cam = recon.View(vid).Camera()
            depth, uv = _project(cam, X)
            if depth is None or depth < 0.2:
                continue
            if not (0 <= uv[0] < 640 and 0 <= uv[1] < 480):
                continue
            feats[vid] = pt.sfm.Feature(uv)
        vids = list(feats.keys())
        for a in range(len(vids)):
            for b in range(a + 1, len(vids)):
                tb.AddFeatureCorrespondence(
                    vids[a], feats[vids[a]], vids[b], feats[vids[b]]
                )
    tb.BuildTracks(recon)
    assert recon.NumTracks() >= 20

    # Clear estimated poses so graph building doesn't rely on them.
    for cap_id in caps:
        c = recon.MutableRigCapture(cap_id)
        c.SetEstimated(False)
        for vid in c.GetViewIds():
            recon.MutableView(vid).SetIsEstimated(False)

    vg = pt.sfm.ViewGraph()  # unused for metric path when tracks exist
    cg = pt.sfm.ViewGraph()
    opts = pt.sfm.BuildCaptureViewGraphOptions()
    opts.use_metric_relative_rig_pose = True
    opts.fallback_to_twoview_strip = False
    opts.relative_rig_ransac.error_thresh = 1e-4
    opts.relative_rig_ransac.min_iterations = 50
    opts.relative_rig_ransac.max_iterations = 500

    assert pt.sfm.BuildCaptureViewGraph(recon, vg, cg, opts)
    assert cg.NumEdges() == 1
    edge = list(cg.GetAllEdges().values())[0]
    # Ground-truth motion is 0.4 m along +Z (rig centers).
    pos = np.asarray(edge.position_2)
    assert abs(np.linalg.norm(pos) - 0.4) < 0.05
    assert abs(pos[2]) > 0.3


def test_build_capture_view_graph_fallback_without_tracks():
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
    vg.AddEdge(v_l0, v_l1, info)

    cg = pt.sfm.ViewGraph()
    assert pt.sfm.BuildCaptureViewGraph(recon, vg, cg)
    assert cg.NumEdges() == 1
