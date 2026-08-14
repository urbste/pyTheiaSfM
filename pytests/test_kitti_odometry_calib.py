"""Unit tests for KITTI odometry calib / pose helpers (no dataset required)."""

from __future__ import annotations

import os
import sys

import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_STEREO = os.path.join(os.path.dirname(_THIS), "pyexamples", "stereo")
if _STEREO not in sys.path:
    sys.path.insert(0, _STEREO)

from kitti_rig_benchmark import (  # noqa: E402
    align_se3,
    apply_se3,
    ate_stats,
    kitti_camera_centers,
    load_kitti_poses,
    parse_kitti_calib,
)


def test_parse_kitti_calib_gray(tmp_path):
    fx, b = 718.856, 0.537165
    tx = -fx * b
    calib = tmp_path / "calib.txt"
    calib.write_text(
        "P0: {0} 0 607.1928 0 0 {0} 185.2157 0 0 0 1 0\n"
        "P1: {0} 0 607.1928 {1} 0 {0} 185.2157 0 0 0 1 0\n".format(fx, tx)
    )
    c = parse_kitti_calib(str(calib), cameras="gray")
    assert abs(c.focal - fx) < 1e-6
    assert abs(c.cx - 607.1928) < 1e-6
    assert abs(c.cy - 185.2157) < 1e-6
    assert abs(c.baseline - b) < 1e-6
    np.testing.assert_allclose(c.left_in_rig, [0, 0, 0])
    np.testing.assert_allclose(c.right_in_rig, [b, 0, 0], atol=1e-6)


def test_load_kitti_poses_and_se3_ate(tmp_path):
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 3.0]
    T2 = np.eye(4)
    T2[:3, 3] = [2.0, 2.0, 3.0]
    path = tmp_path / "00.txt"
    np.savetxt(path, np.vstack([T[:3].ravel(), T2[:3].ravel()]))
    loaded = load_kitti_poses(str(path))
    assert loaded.shape == (2, 4, 4)
    centers = kitti_camera_centers(loaded)
    np.testing.assert_allclose(centers[0], [1, 2, 3])

    src = centers + np.array([0.5, -0.25, 0.1])
    r, t = align_se3(src, centers)
    aligned = apply_se3(src, r, t)
    stats = ate_stats(aligned, centers)
    assert stats["rmse"] < 1e-9
