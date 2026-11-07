"""Tests for 3D point (track position) priors in bundle adjustment.

A track may carry a prior position with a 3x3 sqrt information (or a
covariance). With BundleAdjustmentOptions.use_point_priors set, bundle
adjustment adds the residual sqrt_information * (X - prior) for every optimized
track that has a prior. These tests use noise-free synthetic data with fixed
cameras, so the data-only solution is the true point and the effect of the
prior is easy to read off.
"""

import numpy as np
import pytest

import pytheia as pt

WIDTH, HEIGHT = 1200, 900


def make_camera():
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    cam.SetImageSize(WIDTH, HEIGHT)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(700.0)
    if hasattr(intr, "SetAspectRatio"):
        intr.SetAspectRatio(1.0)
    if hasattr(intr, "SetSkew"):
        intr.SetSkew(0.0)
    intr.SetPrincipalPoint(WIDTH / 2.0, HEIGHT / 2.0)
    return cam


def make_recon(n_views=8, n_tracks=60, seed=0):
    """Cameras on a small arc observing random points in front of them."""
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    base = make_camera()
    for i in range(n_views):
        vid = recon.AddView(str(i), i, float(i))
        cam = recon.MutableView(vid).MutableCamera()
        cam.DeepCopy(base)
        cam.SetPosition(np.array([rng.uniform(-1.5, 1.5), rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.0)]))
        cam.SetOrientationFromAngleAxis(rng.uniform(-0.05, 0.05, size=3))
        recon.MutableView(vid).SetIsEstimated(True)
    truth = {}
    for _ in range(n_tracks):
        xyz = np.array([rng.uniform(-2.0, 2.0), rng.uniform(-1.5, 1.5), rng.uniform(4.0, 9.0)])
        obs = []
        for vid in recon.ViewIds():
            depth, pixel = recon.View(vid).Camera().ProjectPoint(np.append(xyz, 1.0))
            if depth > 0 and 0 < pixel[0] < WIDTH and 0 < pixel[1] < HEIGHT:
                obs.append((vid, pixel))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack()
        for vid, pixel in obs:
            recon.AddObservation(vid, tid, pt.sfm.Feature(pixel))
        track = recon.MutableTrack(tid)
        track.SetPoint(np.append(xyz, 1.0))
        track.SetIsEstimated(True)
        truth[tid] = xyz
    return recon, truth


def perturb_points(recon, truth, sigma, seed=1):
    """Perturb every point by Gaussian noise of the given std (meters)."""
    rng = np.random.default_rng(seed)
    for tid in truth:
        p = euclidean(recon.Track(tid).Point()) + rng.normal(size=3) * sigma
        recon.MutableTrack(tid).SetPoint(np.append(p, 1.0))


def euclidean(point4):
    p = np.asarray(point4, dtype=float)
    return p[:3] / p[3]


def make_options(**kwargs):
    o = pt.sfm.BundleAdjustmentOptions()
    o.num_threads = 1
    o.verbose = False
    o.loss_function_type = pt.sfm.LossFunctionType.TRIVIAL
    o.use_inner_iterations = False
    o.max_num_iterations = 200
    o.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    # Only the points move: the cameras are held fixed.
    o.constant_camera_orientation = True
    o.constant_camera_position = True
    for k, v in kwargs.items():
        setattr(o, k, v)
    return o


def sqrt_info_from_sigma(sigmas):
    return np.diag(1.0 / np.asarray(sigmas, dtype=float))


def run_ba(recon, **kwargs):
    summary = pt.sfm.BundleAdjustReconstruction(make_options(**kwargs), recon)
    assert summary.success
    return summary


# ---------------------------------------------------------------------------
# Track accessors
# ---------------------------------------------------------------------------


def test_default_options_have_no_point_priors():
    o = pt.sfm.BundleAdjustmentOptions()
    assert o.use_point_priors is False
    assert o.robust_loss_width_point_prior == 0.0


def test_track_prior_round_trip_through_accessors():
    track = pt.sfm.Track()
    assert track.HasPositionPrior() is False

    pos = np.array([1.0, -2.0, 3.5])
    sqrt_info = np.array([[2.0, 0.1, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]])
    track.SetPositionPrior(pos, sqrt_info)
    assert track.HasPositionPrior() is True
    np.testing.assert_allclose(track.PositionPrior(), pos)
    np.testing.assert_allclose(track.PositionPriorSqrtInformation(), sqrt_info)

    track.RemovePositionPrior()
    assert track.HasPositionPrior() is False


def test_track_prior_from_covariance_gives_inverse_information():
    track = pt.sfm.Track()
    pos = np.array([0.5, 0.25, -1.0])
    cov = np.array([[0.04, 0.01, 0.0], [0.01, 0.09, 0.0], [0.0, 0.0, 1e-4]])
    assert track.SetPositionPriorCovariance(pos, cov) is True
    assert track.HasPositionPrior() is True
    np.testing.assert_allclose(track.PositionPrior(), pos)
    s = np.asarray(track.PositionPriorSqrtInformation())
    # S^T S must equal the information matrix Sigma^-1.
    np.testing.assert_allclose(s.T @ s, np.linalg.inv(cov), rtol=1e-9)
    # The sqrt information is a valid (upper triangular) Cholesky-type factor.
    np.testing.assert_allclose(np.tril(s, -1), 0.0, atol=1e-12)


def test_track_prior_covariance_rejects_non_spd_and_keeps_track_unchanged():
    track = pt.sfm.Track()
    bad = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    assert track.SetPositionPriorCovariance(np.zeros(3), bad) is False
    assert track.HasPositionPrior() is False


def test_track_prior_survives_reconstruction_write_read(tmp_path):
    recon, truth = make_recon(n_views=4, n_tracks=10, seed=3)
    tids = list(truth.keys())
    prior_tid = tids[0]
    pos = truth[prior_tid] + np.array([0.1, 0.2, -0.3])
    sqrt_info = sqrt_info_from_sigma([0.5, 0.25, 2.0])
    recon.MutableTrack(prior_tid).SetPositionPrior(pos, sqrt_info)

    path = str(tmp_path / "priors.recon")
    assert pt.io.WriteReconstruction(recon, path)
    ok, loaded = pt.io.ReadReconstruction(path)
    assert ok
    track = loaded.Track(prior_tid)
    assert track.HasPositionPrior()
    np.testing.assert_allclose(track.PositionPrior(), pos)
    np.testing.assert_allclose(track.PositionPriorSqrtInformation(), sqrt_info)
    # Tracks without a prior still have none after the round trip.
    assert not loaded.Track(tids[1]).HasPositionPrior()


# ---------------------------------------------------------------------------
# Effect on bundle adjustment
# ---------------------------------------------------------------------------


def test_tight_prior_at_true_position_pulls_perturbed_point_to_truth():
    recon, truth = make_recon()
    tid = next(iter(truth))
    perturb_points(recon, truth, sigma=0.05)
    recon.MutableTrack(tid).SetPoint(np.append(truth[tid] + np.array([0.4, -0.3, 0.5]), 1.0))
    recon.MutableTrack(tid).SetPositionPrior(truth[tid], sqrt_info_from_sigma([1e-4] * 3))

    run_ba(recon, use_point_priors=True)
    assert np.linalg.norm(euclidean(recon.Track(tid).Point()) - truth[tid]) < 1e-3


def test_tight_prior_wins_over_consistent_data():
    """The prior is offset from the true point; a tight prior drags the point to it."""
    recon, truth = make_recon()
    tid = next(iter(truth))
    prior = truth[tid] + np.array([0.2, -0.1, 0.15])
    recon.MutableTrack(tid).SetPositionPrior(prior, sqrt_info_from_sigma([1e-4] * 3))

    run_ba(recon, use_point_priors=True)
    est = euclidean(recon.Track(tid).Point())
    assert np.linalg.norm(est - prior) < 1e-3
    assert np.linalg.norm(est - truth[tid]) > 0.1

    # Without the prior flag the same prior is ignored and the data wins.
    recon2, _ = make_recon()
    recon2.MutableTrack(tid).SetPositionPrior(prior, sqrt_info_from_sigma([1e-4] * 3))
    run_ba(recon2, use_point_priors=False)
    np.testing.assert_allclose(euclidean(recon2.Track(tid).Point()), truth[tid], atol=1e-6)


def test_loose_prior_leaves_data_driven_solution():
    recon, truth = make_recon()
    tid = next(iter(truth))
    prior = truth[tid] + np.array([0.2, -0.1, 0.15])
    recon.MutableTrack(tid).SetPositionPrior(prior, sqrt_info_from_sigma([10.0] * 3))

    run_ba(recon, use_point_priors=True)
    est = euclidean(recon.Track(tid).Point())
    # The data constrains the point far more strongly than a 10 m prior.
    assert np.linalg.norm(est - truth[tid]) < 1e-3
    assert np.linalg.norm(est - prior) > 0.2


@pytest.mark.parametrize("tight_axis", [0, 1, 2])
def test_anisotropic_covariance_is_respected_along_tight_axis(tight_axis):
    """Only the tight axis is pulled to the prior; the loose axes are free.

    The prior is offset along the tight axis only. The offset is inconsistent
    with the data, so the loose axes are biased by the coupling to the tight
    axis; this test therefore only checks the tight axis.
    """
    recon, truth = make_recon()
    tid = next(iter(truth))
    offset = np.zeros(3)
    offset[tight_axis] = 0.2
    prior = truth[tid] + offset
    cov = np.full(3, 100.0)
    cov[tight_axis] = 1e-8
    assert recon.MutableTrack(tid).SetPositionPriorCovariance(prior, np.diag(cov))

    run_ba(recon, use_point_priors=True)
    est = euclidean(recon.Track(tid).Point())
    # Tight axis: the estimate sits on the prior (not on the data solution).
    assert abs(est[tight_axis] - prior[tight_axis]) < 1e-3
    assert abs(est[tight_axis] - truth[tid][tight_axis]) > 0.1


def test_default_flag_off_leaves_results_unchanged():
    recon_a, truth = make_recon(seed=5)
    recon_b, _ = make_recon(seed=5)
    tids = list(truth.keys())
    for r in (recon_a, recon_b):
        perturb_points(r, truth, sigma=0.05, seed=7)
    # Attach strong priors to recon_b only; with use_point_priors False they
    # must not change the outcome.
    for tid in tids[:5]:
        recon_b.MutableTrack(tid).SetPositionPrior(
            truth[tid] + np.array([0.5, 0.5, 0.5]), sqrt_info_from_sigma([1e-4] * 3))

    run_ba(recon_a)
    run_ba(recon_b)
    for tid in tids:
        np.testing.assert_allclose(recon_a.Track(tid).Point(),
                                   recon_b.Track(tid).Point(), rtol=0, atol=1e-10)


def test_track_held_constant_gets_no_prior_residual():
    recon, truth = make_recon()
    tid = next(iter(truth))
    perturbed = truth[tid] + np.array([0.3, 0.0, 0.0])
    recon.MutableTrack(tid).SetPoint(np.append(perturbed, 1.0))
    recon.MutableTrack(tid).SetPositionPrior(truth[tid], sqrt_info_from_sigma([1e-4] * 3))

    o = make_options(use_point_priors=True,
                     constant_camera_orientation=False,
                     constant_camera_position=False)
    summary = pt.sfm.BundleAdjustReconstructionWithConstantTracks(o, [tid], recon)
    assert summary.success
    np.testing.assert_allclose(euclidean(recon.Track(tid).Point()), perturbed, atol=1e-12)


def test_inverse_depth_ignores_point_priors():
    """Point priors are unsupported with inverse depth: the result must not change."""
    recon_a, truth = make_recon(seed=9)
    recon_b, _ = make_recon(seed=9)
    tids = list(truth.keys())
    for r in (recon_a, recon_b):
        perturb_points(r, truth, sigma=0.02, seed=11)
    tid = tids[0]
    recon_b.MutableTrack(tid).SetPositionPrior(
        truth[tid] + np.array([0.3, 0.3, 0.3]), sqrt_info_from_sigma([1e-4] * 3))

    run_ba(recon_a, use_inverse_depth_parametrization=True)
    run_ba(recon_b, use_inverse_depth_parametrization=True, use_point_priors=True)
    for t in tids:
        np.testing.assert_allclose(euclidean(recon_a.Track(t).Point()),
                                   euclidean(recon_b.Track(t).Point()),
                                   rtol=0, atol=1e-8)


def test_robust_point_prior_loss_runs():
    recon, truth = make_recon()
    tid = next(iter(truth))
    recon.MutableTrack(tid).SetPositionPrior(
        truth[tid] + np.array([5.0, 0.0, 0.0]), sqrt_info_from_sigma([1.0] * 3))
    run_ba(recon, use_point_priors=True, robust_loss_width_point_prior=0.5,
           loss_function_type=pt.sfm.LossFunctionType.HUBER)
    assert np.all(np.isfinite(recon.Track(tid).Point()))
