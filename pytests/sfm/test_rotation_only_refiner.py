"""Tests for the rotation-only refinement (MORE, Li, Cai, Wu, arXiv:2611.11770).

``pt.sfm.RefineRotationsWithRotationOnlyError`` refines the global rotations
with the rotation-only reprojection error. It is enabled in the global pipeline
with ``ReconstructionEstimatorOptions.refine_rotations_with_rotation_only_error``
and configured by ``rotation_only_refiner_options``.
"""

import os
import sys

import numpy as np
import pytest

import pytheia as pt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from random_recon_gen import RandomReconGenerator  # noqa: E402
from test_art_position_estimator import (  # noqa: E402
    _scene,
    _synthetic_pipeline_input,
)


def _rotation_matrix(angle_axis):
    """Rodrigues formula: angle-axis vector to rotation matrix."""
    aa = np.asarray(angle_axis, dtype=np.float64)
    theta = np.linalg.norm(aa)
    if theta < 1e-12:
        return np.eye(3)
    k = aa / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _angle_axis(R):
    """Rotation matrix to angle-axis vector."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos)
    if theta < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(theta)) * theta


def _angle_deg(a, b):
    delta = a.T @ b
    cos = np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(cos))


def _aligned_median_error_deg(est, gt):
    """Median rotation error (degrees) after the best global rotation H (E_v H ~ G_v)."""
    ids = sorted(est)
    M = sum(est[v].T @ gt[v] for v in ids)
    U, _, Vt = np.linalg.svd(M)
    V = Vt.T
    H = U @ V.T
    if np.linalg.det(H) < 0:
        V[:, 2] *= -1.0
        H = U @ V.T
    return float(np.median([_angle_deg(est[v] @ H, gt[v]) for v in ids]))


def _perturb(rng, ori, angle_deg):
    out = {}
    for v, aa in ori.items():
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        delta = _rotation_matrix(axis * np.radians(angle_deg))
        out[v] = _angle_axis(delta @ _rotation_matrix(aa))
    return out


# ---------------------------------------------------------------------------
# Options and bindings
# ---------------------------------------------------------------------------

def test_refiner_options_defaults_are_readable_and_writable():
    opts = pt.sfm.RotationOnlyRefinerOptions()
    assert opts.num_outer_iterations == 5
    assert opts.max_num_iterations == 50
    assert opts.loss_scale_px == pytest.approx(2.0)
    assert opts.max_reference_candidates == 8
    assert opts.tuple_gate_px == pytest.approx(3.0)
    assert opts.verify_observations_px == pytest.approx(4.0)
    assert opts.use_two_view_tuples
    assert not opts.require_view_graph_edges
    assert opts.num_threads == 1

    opts.num_outer_iterations = 3
    opts.max_reference_candidates = 4
    opts.tuple_gate_px = 5.0
    opts.use_two_view_tuples = False
    assert opts.num_outer_iterations == 3
    assert opts.max_reference_candidates == 4
    assert opts.tuple_gate_px == pytest.approx(5.0)
    assert not opts.use_two_view_tuples


def test_refiner_reconstruction_options_are_bound():
    options = pt.sfm.ReconstructionEstimatorOptions()
    assert options.refine_rotations_with_rotation_only_error is False
    assert options.rotation_only_refiner_options is not None
    options.refine_rotations_with_rotation_only_error = True
    options.rotation_only_refiner_options.num_outer_iterations = 2
    assert options.refine_rotations_with_rotation_only_error is True
    assert options.rotation_only_refiner_options.num_outer_iterations == 2


# ---------------------------------------------------------------------------
# Function on a synthetic scene
# ---------------------------------------------------------------------------

def test_refiner_reduces_rotation_error_on_synthetic_scene():
    rec, ori, _, pairs = _scene(seed=5, nr_views=20, nr_tracks=300, pixel_noise=0.5)
    rng = np.random.default_rng(7)
    perturbed = _perturb(rng, ori, 1.0)
    gt = {v: _rotation_matrix(aa) for v, aa in ori.items()}
    before = _aligned_median_error_deg({v: _rotation_matrix(aa) for v, aa in perturbed.items()}, gt)

    options = pt.sfm.RotationOnlyRefinerOptions()
    success, refined = pt.sfm.RefineRotationsWithRotationOnlyError(
        options, rec, pairs, perturbed)
    assert success
    after = _aligned_median_error_deg({v: _rotation_matrix(aa) for v, aa in refined.items()}, gt)
    assert after < 0.5 * before, (before, after)


def test_refiner_failure_returns_input_orientations():
    rec, ori, _, pairs = _scene(seed=5, nr_views=6, nr_tracks=50, pixel_noise=0.0)
    options = pt.sfm.RotationOnlyRefinerOptions()
    options.loss_scale_px = -1.0  # invalid: the refiner refuses to run
    success, refined = pt.sfm.RefineRotationsWithRotationOnlyError(
        options, rec, pairs, ori)
    assert not success
    assert set(refined) == set(ori)
    for v in ori:
        np.testing.assert_allclose(refined[v], ori[v])


# ---------------------------------------------------------------------------
# Full global pipeline
# ---------------------------------------------------------------------------

def test_refiner_in_global_pipeline_succeeds():
    gt, graph, view_ids = _synthetic_pipeline_input()
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.ART
    options.refine_rotations_with_rotation_only_error = True
    pt.solvers.SetDefaultRandomSeed(1)
    summary = pt.sfm.GlobalReconstructionEstimator(options).Estimate(graph, gt)
    assert summary.success, summary.message
    assert "Rotation refinement time" in summary.message
    est_ids = [v for v in view_ids if gt.View(v).IsEstimated()]
    assert len(est_ids) >= 6
