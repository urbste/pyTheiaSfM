"""Unit tests for calibrated stereo matching schedules (no matcher required)."""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

_THIS = os.path.dirname(os.path.abspath(__file__))
_STEREO = os.path.join(os.path.dirname(_THIS), "pyexamples", "stereo")
if _STEREO not in sys.path:
    sys.path.insert(0, _STEREO)

from calibrated_stereo_rig import (  # noqa: E402
    DetectOnceCache,
    StereoRigRunOptions,
    default_cosplace_cache_path,
    default_feature_cache_dir,
    feature_cache_key,
    frame_id_from_left_path,
    left_motion_pairs,
    load_cosplace_cache,
    loop_pairs_from_global_descriptors,
    lookup_cosplace_cache,
    merge_cosplace_cache,
    require_lightglue_matcher,
    resolve_cosplace_min_frame_gap,
    resolve_feature_cache_dir,
    save_cosplace_cache,
)


class _Extractor:
    def extract(self, img):
        return {}


class _LightGlueLike:
    extractor = _Extractor()
    matcher = object()


class _XfeatLike:
    mode = "sparse"
    model = type("M", (), {"detectAndCompute": lambda self, x, top_k=None: []})()


class _PairOnly:
    matcher = object()


def test_detect_once_backend_inference():
    assert DetectOnceCache._infer_backend(_LightGlueLike()) == "lightglue"
    assert DetectOnceCache._infer_backend(_XfeatLike()) is None
    assert DetectOnceCache._infer_backend(_PairOnly()) is None


def test_require_lightglue_matcher():
    assert require_lightglue_matcher("disk-lightglue") == "disk-lightglue"
    try:
        require_lightglue_matcher("xfeat")
    except ValueError as exc:
        assert "LightGlue" in str(exc)
    else:
        raise AssertionError("xfeat should be rejected")


def test_cascade_unlimited_is_upper_triangle():
    opts = StereoRigRunOptions(
        left_match_mode="cascade", left_match_max_gap=0, loop_stride=20
    )
    pairs = left_motion_pairs(5, opts)
    assert [(i, j) for i, j, _ in pairs] == [
        (0, 1),
        (0, 2),
        (0, 3),
        (0, 4),
        (1, 2),
        (1, 3),
        (1, 4),
        (2, 3),
        (2, 4),
        (3, 4),
    ]
    assert all(kind == "cascade" for _, _, kind in pairs)


def test_cascade_max_gap_caps_and_loops_fill():
    opts = StereoRigRunOptions(
        left_match_mode="cascade", left_match_max_gap=2, loop_stride=4
    )
    pairs = left_motion_pairs(6, opts)
    keys = {(i, j): kind for i, j, kind in pairs}
    assert keys[(0, 1)] == "cascade"
    assert keys[(0, 2)] == "cascade"
    assert (0, 3) not in keys
    assert keys[(0, 4)] == "loop"
    assert keys[(1, 5)] == "loop"


def test_window_temporal_and_loop():
    opts = StereoRigRunOptions(
        left_match_mode="window", temporal_window=2, loop_stride=4
    )
    pairs = left_motion_pairs(6, opts)
    keys = {(i, j) for i, j, _ in pairs}
    assert (0, 1) in keys and (0, 2) in keys
    assert (0, 3) not in keys
    assert (0, 4) in keys
    assert (1, 3) in keys


def test_feature_cache_key_changes_with_mtime(tmp_path):
    img = tmp_path / "a.png"
    img.write_bytes(b"x")
    k1 = feature_cache_key(str(img), "disk-lightglue", 1024, 2048, "lightglue")
    os.utime(img, (1, 2))
    k2 = feature_cache_key(str(img), "disk-lightglue", 1024, 2048, "lightglue")
    k3 = feature_cache_key(str(img), "superpoint-lightglue", 1024, 2048, "lightglue")
    assert k1 != k2
    assert k2 != k3


def test_default_feature_cache_dir(tmp_path):
    left = tmp_path / "left"
    right = tmp_path / "right"
    left.mkdir()
    right.mkdir()
    a = left / "0.png"
    b = right / "0.png"
    a.write_bytes(b"a")
    b.write_bytes(b"b")
    d = default_feature_cache_dir(
        [str(a), str(b)], "disk-lightglue", 960, 2048
    )
    assert d == str(
        tmp_path / ".pytheia_features" / "disk-lightglue_w960_k2048"
    )
    d_max = default_feature_cache_dir(
        [str(a), str(b)], "disk-lightglue", 1024, 2048, resize_mode="max"
    )
    assert d_max == str(
        tmp_path / ".pytheia_features" / "disk-lightglue_r1024_k2048"
    )
    opts = StereoRigRunOptions(feature_cache=False)
    assert resolve_feature_cache_dir(opts, [str(a)]) is None
    opts = StereoRigRunOptions(feature_cache=True, feature_cache_dir=str(tmp_path / "c"))
    assert resolve_feature_cache_dir(opts, [str(a)]) == str(tmp_path / "c")


def test_cosplace_min_frame_gap_auto():
    window = StereoRigRunOptions(left_match_mode="window", temporal_window=4)
    assert resolve_cosplace_min_frame_gap(window) == 15
    window.temporal_window = 20
    assert resolve_cosplace_min_frame_gap(window) == 21
    cascade = StereoRigRunOptions(left_match_mode="cascade", left_match_max_gap=8)
    assert resolve_cosplace_min_frame_gap(cascade) == 9
    override = StereoRigRunOptions(cosplace_min_frame_gap=30)
    assert resolve_cosplace_min_frame_gap(override) == 30


def test_loop_pairs_from_global_descriptors_skips_short_gap():
    pytest.importorskip("pytheia")
    n = 24
    desc = np.zeros((n, 8), dtype=np.float32)
    desc[:3, 0] = 1.0
    desc[20:23, 0] = 1.0
    desc[10:13, 1] = 1.0
    pairs = loop_pairs_from_global_descriptors(
        desc, k=3, min_frame_gap=10, already=None
    )
    keys = {(i, j) for i, j, kind in pairs}
    assert all(kind == "loop" for _, _, kind in pairs)
    assert all(j - i >= 10 for i, j in keys)
    assert any(i < 3 and j >= 20 for i, j in keys)
    assert (0, 1) not in keys
    assert (0, 2) not in keys
    already = set(keys)
    pairs2 = loop_pairs_from_global_descriptors(
        desc, k=3, min_frame_gap=10, already=already
    )
    assert pairs2 == []


def test_frame_id_from_left_path():
    assert frame_id_from_left_path("/data/image_2/000123.png") == 123
    assert frame_id_from_left_path("left_000007.jpg") == 7
    try:
        frame_id_from_left_path("no_digits.png")
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError")


def test_cosplace_dataset_npz_roundtrip(tmp_path):
    left = tmp_path / "image_2"
    left.mkdir()
    paths = []
    for fid in (0, 2, 5):
        p = left / f"{fid:06d}.png"
        p.write_bytes(b"img" + bytes([fid]))
        paths.append(str(p))
    cache_path = default_cosplace_cache_path(paths)
    assert cache_path.endswith("cosplace_resnet18_d128.npz")
    assert cache_path.startswith(str(left))

    desc = np.zeros((3, 128), dtype=np.float32)
    desc[0, 0] = 1.0
    desc[1, 1] = 2.0
    desc[2, 2] = 3.0
    mtime = [os.stat(p).st_mtime_ns for p in paths]
    size = [os.stat(p).st_size for p in paths]
    cache = merge_cosplace_cache(
        load_cosplace_cache(cache_path),
        [0, 2, 5],
        desc,
        mtime,
        size,
    )
    save_cosplace_cache(cache_path, cache)
    loaded = load_cosplace_cache(cache_path)
    np.testing.assert_array_equal(loaded["frame_ids"], [0, 2, 5])
    np.testing.assert_allclose(loaded["descriptors"][0, 0], 1.0)
    np.testing.assert_allclose(loaded["descriptors"][2, 2], 3.0)

    got, pending, fids, _, _ = lookup_cosplace_cache(loaded, paths)
    assert pending == []
    np.testing.assert_array_equal(fids, [0, 2, 5])
    np.testing.assert_allclose(got[1, 1], 2.0)

    extra = left / "000009.png"
    extra.write_bytes(b"img9")
    got2, pending2, fids2, _, _ = lookup_cosplace_cache(loaded, paths + [str(extra)])
    assert pending2 == [3]
    np.testing.assert_array_equal(fids2, [0, 2, 5, 9])
    np.testing.assert_allclose(got2[0, 0], 1.0)
