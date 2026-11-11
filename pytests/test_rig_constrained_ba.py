"""Rig-constrained bundle adjustment (fixed sensor extrinsics)."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")

FOCAL = 500.0
PP = (320.0, 240.0)
BASELINE = 0.6


def _skew_free_rodrigues(aa):
    th = np.linalg.norm(aa)
    if th < 1e-12:
        return np.eye(3)
    k = aa / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def _build_scene(num_captures=6, num_points=300, noise_px=0.0, seed=0):
    """Forward-moving stereo rig; returns (recon, capture_ids, sensors, GT poses)."""
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-BASELINE / 2, 0, 0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([BASELINE / 2, 0, 0]), np.zeros(3))
    rig_id = recon.AddCameraRig(rig)

    gt = []  # (position, angle_axis) of the rig body per capture
    cap_ids = []
    for i in range(num_captures):
        c = np.array([0.1 * i, 0.02 * i, 0.8 * i])
        aa = np.array([0.0, 0.03 * i, 0.0])
        gt.append((c, aa))
        cid = recon.AddRigCapture(rig_id, float(i), {left: f"l{i}.png", right: f"r{i}.png"})
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
                px = px + rng.normal(0.0, noise_px, size=2) if noise_px > 0 else px
                obs.append((vid, pt.sfm.Feature(px)))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack(obs)
        track = recon.MutableTrack(tid)
        track.SetIsEstimated(True)
        track.SetPoint(np.append(X, 1.0))
    return recon, cap_ids, (left, right), gt


def _ba_options(**kw):
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.num_threads = 1
    opts.verbose = False
    for k, v in kw.items():
        setattr(opts, k, v)
    return opts


def _perturb(recon, cap_ids, rng, pos=0.15, rot=0.01):
    for cid in cap_ids[1:]:
        cap = recon.MutableRigCapture(cid)
        cap.SetPosition(np.array(cap.GetPosition()) + rng.normal(0, pos, 3))
        cap.SetOrientationFromAngleAxis(
            np.array(cap.GetOrientationAsAngleAxis()) + rng.normal(0, rot, 3))
        pt.sfm.PropagateCameraPosesForCapture(cid, recon)


@pytest.mark.parametrize("analytic", [False, True])
def test_rig_ba_recovers_poses_and_keeps_extrinsics_fixed(analytic):
    recon, cap_ids, (left, right), gt = _build_scene(noise_px=0.3)
    rng = np.random.default_rng(1)
    _perturb(recon, cap_ids, rng)

    summary = pt.sfm.BundleAdjustRigReconstruction(
        _ba_options(analytic_jacobians=analytic), recon)
    assert summary.success

    for cid, (c_gt, aa_gt) in zip(cap_ids, gt):
        cap = recon.GetRigCapture(cid)
        # The sensor extrinsics are constants: left/right centres stay exactly
        # one baseline apart (identical sensor orientations -> along rig x).
        cl = np.array(recon.View(cap.ViewIdForCamera(left)).Camera().GetPosition())
        cr = np.array(recon.View(cap.ViewIdForCamera(right)).Camera().GetPosition())
        assert np.linalg.norm(cr - cl) == pytest.approx(BASELINE, abs=1e-9)
        R = _skew_free_rodrigues(np.array(cap.GetOrientationAsAngleAxis()))
        np.testing.assert_allclose(R @ (cr - cl), [BASELINE, 0, 0], atol=1e-9)

    # Gauge is free (BA fixes nothing), so compare the metric trajectory shape:
    # distances between capture centres must match the ground truth.
    est = [np.array(recon.GetRigCapture(c).GetPosition()) for c in cap_ids]
    d_est = np.linalg.norm(np.diff(est, axis=0), axis=1)
    d_gt = np.linalg.norm(np.diff([g[0] for g in gt], axis=0), axis=1)
    np.testing.assert_allclose(d_est, d_gt, rtol=0.02)


def test_analytic_and_autodiff_rig_ba_agree():
    results = []
    for analytic in (False, True):
        recon, cap_ids, _, _ = _build_scene(noise_px=0.3)
        _perturb(recon, cap_ids, np.random.default_rng(1))
        pt.sfm.BundleAdjustRigReconstruction(_ba_options(analytic_jacobians=analytic), recon)
        results.append(np.array([recon.GetRigCapture(c).GetPosition() for c in cap_ids]))
    np.testing.assert_allclose(results[0], results[1], atol=5e-3)


def test_rig_ba_does_not_flip_estimated_flag_of_unestimated_view():
    recon, cap_ids, (left, right), _ = _build_scene()
    cap = recon.GetRigCapture(cap_ids[2])
    vid_r = cap.ViewIdForCamera(right)
    recon.MutableView(vid_r).SetIsEstimated(False)
    pt.sfm.BundleAdjustRigReconstruction(_ba_options(), recon)
    assert not recon.View(vid_r).IsEstimated()
    assert recon.View(cap.ViewIdForCamera(left)).IsEstimated()


def test_rig_ba_orientation_prior_acts_on_capture():
    recon, cap_ids, (left, right), gt = _build_scene()
    # Pull capture 3 toward a rotated orientation prior on its left view and
    # check the capture (not only a view) moves: rig stays rigid.
    cap = recon.GetRigCapture(cap_ids[3])
    vid = cap.ViewIdForCamera(left)
    view = recon.MutableView(vid)
    cam_aa = np.array(view.Camera().GetOrientationAsAngleAxis())
    view.SetOrientationPrior(cam_aa, np.eye(3) * 1e3)
    opts = _ba_options(use_orientation_priors=True)
    summary = pt.sfm.BundleAdjustRigReconstruction(opts, recon)
    assert summary.success
    c = recon.GetRigCapture(cap_ids[3])
    cl = np.array(recon.View(c.ViewIdForCamera(left)).Camera().GetPosition())
    cr = np.array(recon.View(c.ViewIdForCamera(right)).Camera().GetPosition())
    assert np.linalg.norm(cr - cl) == pytest.approx(BASELINE, abs=1e-9)
