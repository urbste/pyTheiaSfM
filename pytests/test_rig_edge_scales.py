"""Metric per-edge scale estimation for same-sensor capture edges of a rig."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")

FOCAL = 500.0
PP = (320.0, 240.0)
BASELINE = 0.6


def _matrix_to_angle_axis(R):
    cos_t = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_t)
    if theta < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(theta)) * theta


def _build_scene(num_captures=6, num_points=300, noise_px=0.0, seed=0):
    """Forward-moving stereo rig with known capture poses (same as the BA tests)."""
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-BASELINE / 2, 0, 0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([BASELINE / 2, 0, 0]), np.zeros(3))
    rig_id = recon.AddCameraRig(rig)

    gt_centres = []
    cap_ids = []
    for i in range(num_captures):
        c = np.array([0.1 * i, 0.02 * i, 0.8 * i])
        aa = np.array([0.0, 0.03 * i, 0.0])
        gt_centres.append(c)
        cid = recon.AddRigCapture(
            rig_id, float(i), {left: f"l{i}.png", right: f"r{i}.png"})
        cap_ids.append(cid)
        capture = recon.MutableRigCapture(cid)
        capture.SetPosition(c)
        capture.SetOrientationFromAngleAxis(aa)
        capture.SetEstimated(True)
        pt.sfm.PropagateCameraPosesForCapture(cid, recon)
        for sensor in (left, right):
            vid = capture.ViewIdForCamera(sensor)
            cam = recon.MutableView(vid).MutableCamera()
            cam.SetFocalLength(FOCAL)
            cam.SetPrincipalPoint(*PP)
            cam.SetImageSize(640, 480)

    points = rng.uniform([-4, -3, 5], [4, 3, 12], size=(num_points, 3))
    for X in points:
        obs = []
        for cid in cap_ids:
            capture = recon.GetRigCapture(cid)
            for sensor in (left, right):
                vid = capture.ViewIdForCamera(sensor)
                cam = recon.View(vid).Camera()
                R = cam.GetOrientationAsRotationMatrix()
                Xc = R @ (X - cam.GetPosition())
                if Xc[2] < 1.0:
                    continue
                px = FOCAL * Xc[:2] / Xc[2] + np.array(PP)
                if not (0 <= px[0] < 640 and 0 <= px[1] < 480):
                    continue
                if noise_px > 0:
                    px = px + rng.normal(0.0, noise_px, size=2)
                obs.append((vid, pt.sfm.Feature(px)))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack(obs)
        track = recon.MutableTrack(tid)
        track.SetIsEstimated(True)
        track.SetPoint(np.append(X, 1.0))
    return recon, cap_ids, (left, right), gt_centres


def _edge_info(recon, v1, v2):
    """TwoViewInfo for the pair (v1, v2) with v2 relative to v1 (metric scale unset)."""
    cam1 = recon.View(v1).Camera()
    cam2 = recon.View(v2).Camera()
    R1 = cam1.GetOrientationAsRotationMatrix()
    R2 = cam2.GetOrientationAsRotationMatrix()
    d = R1 @ (cam2.GetPosition() - cam1.GetPosition())
    info = pt.sfm.TwoViewInfo()
    info.rotation_2 = _matrix_to_angle_axis(R2 @ R1.T)
    info.position_2 = d / np.linalg.norm(d)
    info.num_verified_matches = 100
    info.scale_estimate = -1.0
    return info


def _same_sensor_edges(recon, cap_ids, sensors):
    """Consecutive-capture edges for both sensors; returns (graph, list of (v1, v2, true_dist))."""
    graph = pt.sfm.ViewGraph()
    edges = []
    for sensor in sensors:
        for a, b in zip(cap_ids[:-1], cap_ids[1:]):
            v1 = recon.GetRigCapture(a).ViewIdForCamera(sensor)
            v2 = recon.GetRigCapture(b).ViewIdForCamera(sensor)
            c1 = np.array(recon.View(v1).Camera().GetPosition())
            c2 = np.array(recon.View(v2).Camera().GetPosition())
            graph.AddEdge(min(v1, v2), max(v1, v2), _edge_info(recon, min(v1, v2), max(v1, v2)))
            edges.append((min(v1, v2), max(v1, v2), float(np.linalg.norm(c2 - c1))))
    return graph, edges


@pytest.mark.parametrize("noise_px, tol", [(0.0, 0.02), (0.3, 0.05)])
def test_metric_edge_scales_match_true_capture_distance(noise_px, tol):
    recon, cap_ids, sensors, _ = _build_scene(noise_px=noise_px, seed=0)
    graph, edges = _same_sensor_edges(recon, cap_ids, sensors)

    num_set = pt.sfm.EstimateMetricEdgeScales(recon, graph)
    assert num_set == len(edges)

    for v1, v2, d_true in edges:
        s = graph.GetEdge(v1, v2).scale_estimate
        assert s > 0
        assert s == pytest.approx(d_true, rel=tol), (v1, v2)


def test_cross_sensor_edges_are_left_unset():
    recon, cap_ids, (left, right), _ = _build_scene()
    graph = pt.sfm.ViewGraph()
    cap_a = recon.GetRigCapture(cap_ids[0])
    cap_b = recon.GetRigCapture(cap_ids[1])
    v1 = cap_a.ViewIdForCamera(left)
    v2 = cap_b.ViewIdForCamera(right)
    lo, hi = min(v1, v2), max(v1, v2)
    graph.AddEdge(lo, hi, _edge_info(recon, lo, hi))

    num_set = pt.sfm.EstimateMetricEdgeScales(recon, graph)
    assert num_set == 0
    assert graph.GetEdge(lo, hi).scale_estimate == pytest.approx(-1.0)

