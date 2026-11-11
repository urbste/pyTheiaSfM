"""Tests for the ART (Analytical Rotation-guided Translation) global position estimator.

ART is a track-based estimator (Li, Cai, Wu, arXiv:2611.11770) that is wired
like LIGT: ``GlobalPositionEstimatorType.ART`` and
``ReconstructionEstimatorOptions.art_position_estimator_options``.
"""

import os
import sys

import numpy as np
import pytest

import pytheia as pt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from random_recon_gen import RandomReconGenerator  # noqa: E402


def _scene(seed, nr_views, nr_tracks, pixel_noise):
    """Random scene with rotation-only inputs (ground-truth orientations and
    relative translation directions between all view pairs)."""
    np.random.seed(seed)
    gen = RandomReconGenerator(seed=seed)
    rec = gen.generate_random_recon(nr_views=nr_views, nr_tracks=nr_tracks,
                                    pixel_noise=pixel_noise)
    vids = sorted(rec.ViewIds())
    ori = {v: np.asarray(rec.View(v).Camera().GetOrientationAsAngleAxis()) for v in vids}
    pos = {v: np.asarray(rec.View(v).Camera().GetPosition()) for v in vids}
    pairs = {}
    for i, a in enumerate(vids):
        rot_a = rec.View(a).Camera().GetOrientationAsRotationMatrix()
        for b in vids[i + 1:]:
            info = pt.sfm.TwoViewInfo()
            info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(ori[a], ori[b])
            direction = rot_a @ (pos[b] - pos[a])
            info.position_2 = direction / np.linalg.norm(direction)
            info.num_verified_matches = 100
            pairs[(a, b)] = info
    return rec, ori, pos, pairs


def _sim3_errors(est, gt):
    """Per-view distances after the best Sim(3) (Umeyama) alignment of est onto gt."""
    ids = sorted(set(est) & set(gt))
    src = np.array([est[i] for i in ids])
    dst = np.array([gt[i] for i in ids])
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(d0.T @ s0)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    scale = np.trace(np.diag(S) @ D) / (s0 ** 2).sum()
    aligned = (scale * (R @ s0.T)).T + mu_d
    return np.linalg.norm(aligned - dst, axis=1)


def _run_position_estimator(kind, pairs, ori, rec):
    if kind == "art":
        est = pt.sfm.ARTPositionEstimator(pt.sfm.ARTPositionEstimatorOptions(), rec)
    else:
        est = pt.sfm.LiGTPositionEstimator(pt.sfm.LiGTPositionEstimatorOptions(), rec)
    return est.EstimatePositions(pairs, ori)


def _synthetic_pipeline_input(nr_views=8, nr_tracks=200, seed=3):
    """Same conventions as pytests/test_cli.py::_synthetic_scene (ViewGraph
    edges keyed by (min id, max id) with the smaller id as view 1)."""
    gen = RandomReconGenerator(seed=seed)
    gt = gen.generate_random_recon(nr_views=nr_views, nr_tracks=nr_tracks)
    view_ids = sorted(gt.ViewIds())
    graph = pt.sfm.ViewGraph()
    for i, a in enumerate(view_ids):
        cam_a = gt.View(a).Camera()
        rot_a = cam_a.GetOrientationAsRotationMatrix()
        for b in view_ids[i + 1:]:
            cam_b = gt.View(b).Camera()
            info = pt.sfm.TwoViewInfo()
            info.focal_length_1 = cam_a.FocalLength()
            info.focal_length_2 = cam_b.FocalLength()
            info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(
                cam_a.GetOrientationAsAngleAxis(), cam_b.GetOrientationAsAngleAxis())
            direction = rot_a @ (np.asarray(cam_b.GetPosition()) - np.asarray(cam_a.GetPosition()))
            info.position_2 = direction / np.linalg.norm(direction)
            info.num_verified_matches = 150
            graph.AddEdge(a, b, info)
    for tid in gt.TrackIds():
        gt.MutableTrack(tid).SetIsEstimated(False)
    for vid in view_ids:
        view = gt.MutableView(vid)
        view.SetIsEstimated(False)
        view.SetCameraIntrinsicsPrior(view.Camera().CameraIntrinsicsPriorFromIntrinsics())
    return gt, graph, view_ids


# ---------------------------------------------------------------------------
# Options and bindings
# ---------------------------------------------------------------------------

def test_art_options_defaults_are_readable_and_writable():
    opts = pt.sfm.ARTPositionEstimatorOptions()
    assert opts.num_threads == 1
    assert opts.min_tracks_per_triplet == 10
    assert opts.min_tracks_per_triplet_fallback == 3
    assert opts.use_track_covisibility_triplets
    assert opts.max_tracks_per_triplet == 200
    assert opts.max_triplets_per_view == 50
    assert opts.max_eigen_ratio == pytest.approx(0.1)
    assert opts.num_irls_iterations == 10
    assert opts.irls_scale == pytest.approx(0.0)

    opts.max_triplets_per_view = 17
    opts.irls_scale = 0.25
    opts.max_eigen_ratio = 0.05
    assert opts.max_triplets_per_view == 17
    assert opts.irls_scale == pytest.approx(0.25)
    assert opts.max_eigen_ratio == pytest.approx(0.05)


def test_art_enum_and_reconstruction_options_are_bound():
    assert pt.sfm.GlobalPositionEstimatorType.ART is not None
    options = pt.sfm.ReconstructionEstimatorOptions()
    assert options.art_position_estimator_options is not None
    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.ART
    options.art_position_estimator_options.min_tracks_per_triplet = 12
    assert options.global_position_estimator_type == pt.sfm.GlobalPositionEstimatorType.ART
    assert options.art_position_estimator_options.min_tracks_per_triplet == 12


# ---------------------------------------------------------------------------
# Position estimation on synthetic scenes
# ---------------------------------------------------------------------------

def test_art_noise_free_recovers_all_positions():
    rec, ori, pos, pairs = _scene(seed=1, nr_views=10, nr_tracks=300, pixel_noise=0.0)
    # View 1 of this scene shares only 4 tracks with its partners; at the default
    # screening (10 tracks, fallback 3) it may be left out and the registered views are exact.
    est = _run_position_estimator("art", pairs, ori, rec)
    assert len(est) >= 9
    err = _sim3_errors(est, pos)
    assert np.median(err) < 1e-6, err

    # Relaxing the screening to 4 tracks recovers every view, also exactly.
    opts = pt.sfm.ARTPositionEstimatorOptions()
    opts.min_tracks_per_triplet = 4
    est = pt.sfm.ARTPositionEstimator(opts, rec).EstimatePositions(pairs, ori)
    assert len(est) == 10, sorted(set(pos) - set(est))
    err = _sim3_errors(est, pos)
    assert np.median(err) < 1e-6, err


def test_art_noisy_30_views_against_ligt():
    rec, ori, pos, pairs = _scene(seed=3, nr_views=30, nr_tracks=800, pixel_noise=1.0)
    art = _run_position_estimator("art", pairs, ori, rec)
    ligt = _run_position_estimator("ligt", pairs, ori, rec)
    art_err = _sim3_errors(art, pos)
    ligt_err = _sim3_errors(ligt, pos)
    print(f"\nART: views {len(art)}/30 median sim3 err {np.median(art_err):.3e}"
          f" | LiGT: views {len(ligt)}/30 median sim3 err {np.median(ligt_err):.3e}")
    assert len(art) >= 28
    assert np.median(art_err) < 0.02


# ---------------------------------------------------------------------------
# Full global pipeline
# ---------------------------------------------------------------------------

def test_art_global_pipeline_matches_ground_truth():
    gt, graph, view_ids = _synthetic_pipeline_input()
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.ART
    pt.solvers.SetDefaultRandomSeed(1)
    recon = gt  # tracks are kept, poses are re-estimated
    summary = pt.sfm.GlobalReconstructionEstimator(options).Estimate(graph, recon)
    assert summary.success, summary.message

    est_ids = [v for v in view_ids if recon.View(v).IsEstimated()]
    assert len(est_ids) >= 6
    src = np.array([recon.View(v).Camera().GetPosition() for v in est_ids], dtype=np.float64)
    dst = np.array([gt.View(v).Camera().GetPosition() for v in est_ids], dtype=np.float64)
    # Umeyama alignment of the estimated centers onto the ground truth.
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, S, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    scale = np.trace(np.diag(S) @ D) / ((src - mu_s) ** 2).sum()
    residual = np.linalg.norm(scale * (R @ (src - mu_s).T).T + mu_d - dst, axis=1)
    assert np.median(residual) < 1e-3, residual
    assert residual.max() < 1e-2, residual


def test_art_coverage_options_defaults():
    opts = pt.sfm.ARTPositionEstimatorOptions()
    assert opts.num_coverage_rounds == 3
    assert opts.min_bridge_tracks == 10
    assert opts.num_robust_samples == 20
    opts.min_bridge_tracks = 0
    opts.num_robust_samples = 0
    assert opts.min_bridge_tracks == 0
    assert opts.num_robust_samples == 0


def test_art_attaches_weak_view_with_track_constraints():
    # View 1 shares only 4 tracks with the others: no triplet of 10 tracks, and
    # too few tracks to be attached at min_bridge_tracks = 10.
    rec, ori, pos, pairs = _scene(seed=1, nr_views=10, nr_tracks=300, pixel_noise=0.0)
    opts = pt.sfm.ARTPositionEstimatorOptions()
    est = pt.sfm.ARTPositionEstimator(opts, rec).EstimatePositions(pairs, ori)
    assert len(est) == 9
    # With min_bridge_tracks = 3 it is attached with the LiGT constraints of its
    # tracks, exactly in the noise-free case.
    opts.min_bridge_tracks = 3
    est = pt.sfm.ARTPositionEstimator(opts, rec).EstimatePositions(pairs, ori)
    assert len(est) == 10, sorted(set(pos) - set(est))
    err = _sim3_errors(est, pos)
    assert np.max(err) < 1e-6, err


def test_ligt_removes_weakly_constrained_views():
    rec, ori, pos, pairs = _scene(seed=1, nr_views=10, nr_tracks=300, pixel_noise=0.0)
    opts = pt.sfm.LiGTPositionEstimatorOptions()
    assert opts.min_tracks_per_view == 5
    # View 1 is seen in 4 tracks of length >= 3.
    est = pt.sfm.LiGTPositionEstimator(opts, rec).EstimatePositions(pairs, ori)
    assert len(est) == 9 and 1 not in est
    opts.min_tracks_per_view = 0
    est = pt.sfm.LiGTPositionEstimator(opts, rec).EstimatePositions(pairs, ori)
    assert len(est) == 10
