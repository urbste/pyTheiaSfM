"""Analytic vs autodiff Jacobians in bundle adjustment.

BundleAdjustmentOptions.analytic_jacobians (default True) replaces the Ceres
autodiff reprojection cost functions by hand-derived Jacobians. The solver
trajectory must be (numerically) the same: these tests run the same noisy
problem with the option on and off for every camera model, for the homogeneous
XYZ and the inverse depth parametrization, and compare the final cost and the
estimated parameters.
"""

import copy
import os
import sys

import numpy as np
import pytest

import pytheia as pt

sys.path.insert(0, os.path.dirname(__file__))
from test_inverse_depth_ba import (  # noqa: E402
    MODELS,
    make_recon,
    options,
    perturb_points,
    perturb_poses,
    reprojection_rms,
)


def run_ba(model, inverse, analytic, intrinsics, seed=0):
    recon, _ = make_recon(model, n_views=6, n_tracks=80, pixel_noise=0.3, seed=seed)
    perturb_points(recon, 0.02, seed=seed + 1)
    perturb_poses(recon, 0.02, 0.002, seed=seed + 2)
    opts = options(inverse, fix_intrinsics=(intrinsics is None))
    if intrinsics is not None:
        opts.intrinsics_to_optimize = intrinsics
    opts.analytic_jacobians = analytic
    summary = pt.sfm.BundleAdjustReconstruction(opts, recon)
    return summary, recon


def collect(recon):
    poses, intr = [], []
    for vid in sorted(recon.ViewIds()):
        cam = recon.View(vid).Camera()
        poses.append(np.concatenate([cam.GetPosition(), cam.GetOrientationAsAngleAxis()]))
        i = cam.CameraIntrinsics()
        intr.append([i.GetParameter(k) for k in range(i.NumParameters())])
    pts = []
    for tid in sorted(recon.TrackIds()):
        p = np.asarray(recon.Track(tid).Point(), dtype=float)
        pts.append(p[:3] / p[3])
    return np.array(poses), np.array(intr), np.array(pts)


def test_analytic_jacobians_default_on_and_alias_removed():
    o = pt.sfm.BundleAdjustmentOptions()
    assert o.analytic_jacobians is True
    o.analytic_jacobians = False
    assert o.analytic_jacobians is False
    assert not hasattr(o, "experimental_analytic_division_jacobian")


@pytest.mark.parametrize("inverse", [False, True], ids=["xyz", "invdepth"])
@pytest.mark.parametrize("model", MODELS)
def test_analytic_matches_autodiff(model, inverse):
    # Extrinsics + focal length only: the other intrinsics of several models
    # (skew, aspect, xi/alpha, ...) are weakly observable and would amplify
    # the 1e-9 Jacobian differences along the trajectory.
    intr = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH
    s_an, r_an = run_ba(model, inverse, True, intr)
    s_ad, r_ad = run_ba(model, inverse, False, intr)
    assert s_an.success and s_ad.success
    assert np.isfinite(s_an.final_cost)
    assert s_an.final_cost == pytest.approx(s_ad.final_cost, rel=1e-5, abs=1e-9)
    assert reprojection_rms(r_an) == pytest.approx(reprojection_rms(r_ad), rel=1e-4, abs=1e-6)
    pa, ia, xa = collect(r_an)
    pd, id_, xd = collect(r_ad)
    np.testing.assert_allclose(pa, pd, rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(ia, id_, rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(xa, xd, rtol=1e-4, atol=1e-4)


@pytest.mark.parametrize("model", ["PINHOLE", "PINHOLE_RADIAL_TANGENTIAL", "FISHEYE", "FULL_OPENCV", "DIVISION_UNDISTORTION"])
def test_analytic_matches_autodiff_all_intrinsics(model):
    """Optimizing all intrinsics exercises every intrinsics column."""
    intr = pt.sfm.OptimizeIntrinsicsType.ALL
    s_an, r_an = run_ba(model, False, True, intr)
    s_ad, r_ad = run_ba(model, False, False, intr)
    assert s_an.final_cost == pytest.approx(s_ad.final_cost, rel=1e-3, abs=1e-8)
    assert reprojection_rms(r_an) == pytest.approx(reprojection_rms(r_ad), rel=1e-2, abs=1e-4)


def test_analytic_jacobians_with_robust_loss():
    """Huber loss, noisy poses, points kept at their (noisy) initial values."""
    for analytic in (True, False):
        recon, _ = make_recon("PINHOLE", pixel_noise=0.5)
        perturb_poses(recon, 0.02, 0.002)
        o = options(False)
        o.loss_function_type = pt.sfm.LossFunctionType.HUBER
        o.robust_loss_width = 1.0
        o.analytic_jacobians = analytic
        s = pt.sfm.BundleAdjustReconstruction(o, recon)
        assert s.success
        if analytic:
            ref = (s.final_cost, collect(recon)[0])
        else:
            assert s.final_cost == pytest.approx(ref[0], rel=1e-5)
            np.testing.assert_allclose(collect(recon)[0], ref[1], rtol=1e-4, atol=1e-5)
