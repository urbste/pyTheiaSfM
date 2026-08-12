"""Tests for 5+1 / upright generalized relative pose and EstimateRelativeRigInfo."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def _make_corr(X, o1, o2, R, t):
    c = pt.sfm.GeneralizedRayCorrespondence()
    c.origin1 = np.asarray(o1, dtype=np.float64)
    c.direction1 = (np.asarray(X) - c.origin1)
    c.direction1 = c.direction1 / np.linalg.norm(c.direction1)
    X2 = R @ np.asarray(X) + t
    c.origin2 = np.asarray(o2, dtype=np.float64)
    c.direction2 = X2 - c.origin2
    c.direction2 = c.direction2 / np.linalg.norm(c.direction2)
    return c


def test_five_point_one_point_generalized_relative_pose_metric():
    left = np.array([-0.06, 0.0, 0.0])
    right = np.array([0.06, 0.0, 0.0])
    angle = 0.15
    R = np.array(
        [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ]
    )
    t = np.array([0.35, 0.02, 0.08])

    rng = np.random.default_rng(0)
    origins1, dirs1, origins2, dirs2 = [], [], [], []
    for i in range(5):
        X = rng.uniform([-1, -0.8, 2], [1, 0.8, 6])
        c = _make_corr(X, left, left, R, t)
        origins1.append(c.origin1)
        dirs1.append(c.direction1)
        origins2.append(c.origin2)
        dirs2.append(c.direction2)
    X = rng.uniform([-1, -0.8, 2], [1, 0.8, 6])
    c = _make_corr(X, left, right, R, t)
    origins1.append(c.origin1)
    dirs1.append(c.direction1)
    origins2.append(c.origin2)
    dirs2.append(c.direction2)

    n, rotations, translations = pt.sfm.FivePointOnePointGeneralizedRelativePose(
        origins1, dirs1, origins2, dirs2
    )
    assert n > 0
    found = False
    for Ri, ti in zip(rotations, translations):
        rot_err = np.arccos(
            np.clip((np.trace(Ri.T @ R) - 1.0) * 0.5, -1.0, 1.0)
        )
        if rot_err < 1e-2 and np.linalg.norm(ti - t) < 1e-2:
            found = True
            break
    assert found


def test_estimate_relative_rig_info():
    left = np.array([-0.06, 0.0, 0.0])
    right = np.array([0.06, 0.0, 0.0])
    angle = 0.12
    R = np.array(
        [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ]
    )
    t = np.array([0.4, -0.05, 0.15])
    rng = np.random.default_rng(1)
    central, generalized = [], []
    for _ in range(40):
        X = rng.uniform([-1, -0.7, 2], [1, 0.7, 7])
        central.append(_make_corr(X, left, left, R, t))
    for _ in range(30):
        X = rng.uniform([-1, -0.7, 2], [1, 0.7, 7])
        generalized.append(_make_corr(X, left, right, R, t))

    params = pt.solvers.RansacParameters()
    params.error_thresh = 1e-4
    params.min_iterations = 100
    params.max_iterations = 1000
    ok, info, summary = pt.sfm.EstimateRelativeRigInfo(
        params, central, generalized
    )
    assert ok
    assert len(summary.inliers) >= 50
    rot_err = np.arccos(
        np.clip((np.trace(np.asarray(info.rotation).T @ R) - 1.0) * 0.5, -1.0, 1.0)
    )
    assert rot_err < 1e-2
    assert np.linalg.norm(np.asarray(info.translation) - t) < 5e-2
    tw = info.ToTwoViewInfo()
    assert tw.scale_estimate > 0.1
