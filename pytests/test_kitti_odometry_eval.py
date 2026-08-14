"""Unit tests for KITTI odometry MGSfM-style evaluation."""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

_THIS = os.path.dirname(os.path.abspath(__file__))
_STEREO = os.path.join(os.path.dirname(_THIS), "pyexamples", "stereo")
if _STEREO not in sys.path:
    sys.path.insert(0, _STEREO)

from kitti_odometry_eval import (  # noqa: E402
    apply_sim3_to_pose,
    angle_axis_to_matrix,
    mgsfm_metrics_from_poses,
    rotation_angle_deg,
    summarize_errors,
)
from calibrated_stereo_rig import (  # noqa: E402
    default_match_cache_dir,
    match_pair_cache_key,
    MatchPairDiskCache,
    resolve_match_cache_dir,
    StereoRigRunOptions,
)


def _pose_at(x: float, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, 3] = [x, y, z]
    return t


def test_summarize_errors_median_mean():
    stats = summarize_errors(np.array([1.0, 3.0, 2.0]), np.array([0.1, 0.3, 0.2]))
    assert stats["er_median"] == 2.0
    assert stats["er_mean"] == 2.0
    assert stats["et_median"] == 0.2
    assert stats["n"] == 3


def test_rotation_angle_deg_identity():
    r = np.eye(3)
    assert rotation_angle_deg(r, r) < 1e-9


def test_rotation_angle_deg_yaw():
    aa = np.array([0.0, 0.0, np.deg2rad(1.0)])
    r = angle_axis_to_matrix(aa).T  # cam→world yaw
    r_id = np.eye(3)
    err = rotation_angle_deg(r_id, r)
    assert abs(err - 1.0) < 0.05


def test_match_pair_cache_key_order(tmp_path):
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    a.write_bytes(b"x")
    b.write_bytes(b"y")
    k1 = match_pair_cache_key(str(a), str(b), "disk-lightglue", 960, 2048, "width")
    k2 = match_pair_cache_key(str(b), str(a), "disk-lightglue", 960, 2048, "width")
    assert k1 != k2


def test_mgsfm_metrics_identity_after_alignment():
    pytest.importorskip("pytheia")
    gt = [_pose_at(i) for i in range(5)]
    est = [_pose_at(i + 0.01) for i in range(5)]
    stats = mgsfm_metrics_from_poses(est, gt)
    assert stats["n"] == 5
    assert stats["et_mean"] < 0.05
    assert stats["er_mean"] < 1.0


def test_apply_sim3_identity():
    p = _pose_at(1.0, 2.0, 3.0)
    out = apply_sim3_to_pose(p, np.eye(3), np.zeros(3), 1.0)
    assert np.allclose(out[:3, 3], p[:3, 3])


def test_match_pair_cache_roundtrip(tmp_path):
    cache = MatchPairDiskCache(
        str(tmp_path), "disk-lightglue", 960, 2048, "width"
    )
    pa = tmp_path / "000000.png"
    pb = tmp_path / "000001.png"
    pa.write_bytes(b"x")
    pb.write_bytes(b"y")
    k0 = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float64)
    k1 = np.array([[5.0, 6.0], [7.0, 8.0]], dtype=np.float64)
    assert cache.try_load(str(pa), str(pb)) is None
    cache.save(str(pa), str(pb), k0, k1)
    hit = cache.try_load(str(pa), str(pb))
    assert hit is not None
    assert np.allclose(hit[0], k0)
    assert np.allclose(hit[1], k1)
    assert cache.disk_hits == 1


def test_default_match_cache_dir(tmp_path):
    a = tmp_path / "seq" / "image_2" / "000000.png"
    a.parent.mkdir(parents=True)
    a.write_bytes(b"x")
    d = default_match_cache_dir(
        [str(a)], "disk-lightglue", 960, 2048, "width"
    )
    assert ".pytheia_matches" in d
    opts = StereoRigRunOptions(match_cache=False)
    assert resolve_match_cache_dir(opts, [str(a)]) is None
