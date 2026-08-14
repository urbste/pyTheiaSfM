# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Shared calibrated stereo-rig matching + reconstruction (vismatch + pyTheia)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import os
import re


@dataclass
class StereoCalib:
    """Pinhole stereo, sensors along +X in the abstract rig frame."""

    focal: float
    cx: float
    cy: float
    baseline: float
    width: int = 0
    height: int = 0
    aspect_ratio: float = 1.0
    # If None, sensors are placed symmetrically about the body origin.
    left_in_rig: Optional[object] = None
    right_in_rig: Optional[object] = None

    def sensor_positions(self):
        import numpy as np

        half = 0.5 * float(self.baseline)
        left = (
            np.asarray(self.left_in_rig, dtype=np.float64).reshape(3)
            if self.left_in_rig is not None
            else np.array([-half, 0.0, 0.0])
        )
        right = (
            np.asarray(self.right_in_rig, dtype=np.float64).reshape(3)
            if self.right_in_rig is not None
            else np.array([half, 0.0, 0.0])
        )
        return left, right


RESIZE_MODES = ("width", "max")


def matcher_resize_arg(
    resize: int, resize_mode: str, width: int, height: int
) -> int | tuple[int, int]:
    """Build the ``resize`` argument for vismatch ``load_image``."""
    mode = str(resize_mode).lower()
    if mode == "width":
        if width <= 0:
            raise ValueError(f"Invalid image width {width} for width resize")
        scale = int(resize) / float(width)
        return (int(round(height * scale)), int(resize))
    if mode == "max":
        return int(resize)
    raise ValueError(
        f"Unknown resize_mode {resize_mode!r}; expected one of {RESIZE_MODES}"
    )


@dataclass
class StereoRigRunOptions:
    matcher: str = "disk-lightglue"
    device: str = "cuda"
    resize: int = 960
    resize_mode: str = "width"  # "width" = target px width; "max" = longest side
    match_thresh: float = 0.5
    max_keypoints: int = 2048
    min_matches: int = 40
    max_sampson_error: float = 2.0
    # "cascade": left_i vs later left_j (optionally capped by left_match_max_gap).
    # "window": left_i vs i+1..i+temporal_window, plus loop_stride pairs.
    left_match_mode: str = "cascade"
    left_match_max_gap: int = 0  # 0 = all later frames (true cascade)
    temporal_window: int = 5
    loop_stride: int = 20
    # Appearance loop closures (CosPlace + GraphMatch) on left keyframes.
    # Off by default: forward odometry (KITTI 04) has no revisits.
    trajectory_has_loops: bool = False
    cosplace_neighbors: int = 5
    # 0 = auto: just beyond the temporal/cascade window so kNN does not
    # propose already-scheduled neighbours as loops.
    cosplace_min_frame_gap: int = 0
    match_right_temporal: bool = False
    cross_sensor_temporal: bool = False
    # Snap per-pair matcher keypoints onto existing features so tracks chain.
    stereo_snap_pixels: float = 3.0
    # After left–left tracks exist, match left_i↔right_i but only keep hits on
    # tracks that already span several left views.
    guided_stereo: bool = True
    min_track_length: int = 3
    min_track_captures: int = 3
    retriangulation_iterations: int = 0
    method: str = "global"
    rotation_estimator: str = "ROBUST_L1L2"
    position_estimator: str = "LEAST_UNSQUARED_DEVIATION"
    verbose_matches: bool = False
    # Extract keypoints/descriptors once per image, then match cached features.
    # Falls back to per-pair detection for LoFTR / EDM / RoMa / MINIMA pair models.
    detect_once: bool = True
    # Persist detect-once features on disk (auto dir next to the images).
    feature_cache: bool = True
    feature_cache_dir: str = ""
    # Persist raw LightGlue pair matches on disk (keypoints in full-res pixels).
    match_cache: bool = True
    match_cache_dir: str = ""
    match_cache_only: bool = False  # all matching, skip SfM/BA
    min_triangulation_angle_degrees: float = 0.5
    max_reprojection_error_in_pixels: float = 6.0
    # Bundle adjustment robust kernel. Default Huber (was Trivial / plain L2).
    ba_loss: str = "huber"
    ba_robust_width: float = 2.0
    # Reconstruct every Nth capture, then PnP-localize the skipped ones.
    # 1 = all frames in SfM. First and last frames are always keyframes.
    recon_stride: int = 1
    pnp_min_inliers: int = 30
    # Write side-by-side match PNGs + JSON summary for left–left motion pairs.
    match_debug_dir: str = ""
    match_debug_max_pairs: int = 0  # 0 = all scheduled left–left pairs
    match_debug_only: bool = False  # stop after left motion matching (+ debug dump)


@dataclass
class StereoRigRunResult:
    recon: object
    summary: object
    view_graph: object
    left_view_ids: list
    right_view_ids: list
    capture_ids: list
    left_sensor_id: object
    right_sensor_id: object
    match_counts: dict = field(default_factory=dict)


def make_pinhole_prior(pt, calib: StereoCalib, width: int, height: int):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [float(calib.focal)]
    prior.focal_length.is_set = True
    prior.principal_point.value = [float(calib.cx), float(calib.cy)]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [float(calib.aspect_ratio)]
    prior.aspect_ratio.is_set = True
    prior.image_width = int(width)
    prior.image_height = int(height)
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def _scale_kpts(kpts, full_wh, matched_hw):
    import numpy as np

    k = np.asarray(kpts, dtype=np.float64).reshape(-1, 2)
    fw, fh = full_wh
    mh, mw = matched_hw
    if mw <= 0 or mh <= 0:
        return k
    out = k.copy()
    out[:, 0] *= fw / float(mw)
    out[:, 1] *= fh / float(mh)
    return out


def correspondences_from_result(
    pt, result, full_wh_a, full_wh_b, matched_hw_a, matched_hw_b, min_n
):
    import numpy as np

    k0 = result.get("matched_kpts0")
    k1 = result.get("matched_kpts1")
    if k0 is None or k1 is None or np.asarray(k0).size == 0:
        k0 = result.get("inlier_kpts0")
        k1 = result.get("inlier_kpts1")
    if k0 is None or k1 is None:
        return False, []
    k0 = _scale_kpts(k0, full_wh_a, matched_hw_a)
    k1 = _scale_kpts(k1, full_wh_b, matched_hw_b)
    n = min(len(k0), len(k1))
    if n < min_n:
        return False, []
    cors = []
    for i in range(n):
        cors.append(
            pt.matching.FeatureCorrespondence(
                pt.sfm.Feature(k0[i]), pt.sfm.Feature(k1[i])
            )
        )
    return True, cors


LIGHTGLUE_MATCHERS = (
    "superpoint-lightglue",
    "disk-lightglue",
    "aliked-lightglue",
    "sift-lightglue",
    "doghardnet-lightglue",
)

BA_LOSS_TYPES = (
    "trivial",
    "huber",
    "softlone",
    "cauchy",
    "arctan",
    "tukey",
)


def require_lightglue_matcher(name: str) -> str:
    key = (name or "").strip().lower()
    if key in LIGHTGLUE_MATCHERS:
        return key
    raise ValueError(
        f"Stereo rig matching is detect-once + LightGlue only (got {name!r}). "
        f"Choose one of: {', '.join(LIGHTGLUE_MATCHERS)}"
    )


def ba_loss_function_type(pt, name: str):
    key = (name or "huber").strip().lower()
    if key not in BA_LOSS_TYPES:
        raise ValueError(
            f"Unknown ba_loss {name!r}. Choose one of: {', '.join(BA_LOSS_TYPES)}"
        )
    return getattr(pt.sfm.LossFunctionType, key.upper())


def reconstruction_keyframe_indices(n: int, stride: int) -> list[int]:
    """Frames used for matching + SfM. Always includes 0 and n-1."""
    n = int(n)
    stride = max(1, int(stride))
    if n <= 0:
        return []
    if stride == 1 or n <= 2:
        return list(range(n))
    keys = list(range(0, n, stride))
    if keys[-1] != n - 1:
        keys.append(n - 1)
    return keys


def _tensor_hw(tensor, fallback_hw):
    h, w = fallback_hw
    if hasattr(tensor, "shape") and len(tensor.shape) >= 2:
        sh = tuple(int(x) for x in tensor.shape)
        if sh[0] in (1, 3) and len(sh) == 3:
            return sh[1], sh[2]
        return sh[0], sh[1]
    return h, w


def _to_numpy(x):
    import numpy as np

    if x is None:
        return None
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x)


_FEATURE_CACHE_VERSION = 1


def _tree_map(obj, fn):
    import torch

    if isinstance(obj, torch.Tensor):
        return fn(obj)
    if isinstance(obj, dict):
        return {k: _tree_map(v, fn) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_tree_map(v, fn) for v in obj]
    if isinstance(obj, tuple):
        return tuple(_tree_map(v, fn) for v in obj)
    return obj


def _tensors_cpu(obj):
    return _tree_map(obj, lambda t: t.detach().cpu().contiguous())


def _tensors_to(obj, device: str):
    return _tree_map(obj, lambda t: t.to(device, non_blocking=True))


def default_feature_cache_dir(
    paths: list[str],
    matcher: str,
    resize: int,
    max_keypoints: int,
    resize_mode: str = "width",
) -> str:
    abs_paths = [os.path.abspath(p) for p in paths]
    try:
        root = os.path.commonpath(abs_paths)
    except ValueError:
        root = os.path.dirname(abs_paths[0])
    if os.path.isfile(root):
        root = os.path.dirname(root)
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in matcher)
    rm = str(resize_mode).lower()
    prefix = "w" if rm == "width" else "r"
    tag = f"{safe}_{prefix}{int(resize)}_k{int(max_keypoints)}"
    return os.path.join(root, ".pytheia_features", tag)


def feature_cache_key(
    path: str,
    matcher: str,
    resize: int,
    max_keypoints: int,
    backend: str,
    resize_mode: str = "width",
) -> str:
    import hashlib

    ap = os.path.abspath(path)
    st = os.stat(ap)
    payload = (
        f"{ap}|{st.st_mtime_ns}|{st.st_size}|{matcher}|"
        f"{int(resize)}|{resize_mode}|{int(max_keypoints)}|{backend}|"
        f"{_FEATURE_CACHE_VERSION}"
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()


def resolve_feature_cache_dir(
    options: StereoRigRunOptions, paths: list[str]
) -> str | None:
    if not options.feature_cache:
        return None
    if options.feature_cache_dir:
        return os.path.abspath(options.feature_cache_dir)
    return default_feature_cache_dir(
        paths,
        options.matcher,
        options.resize,
        options.max_keypoints,
        options.resize_mode,
    )


_MATCH_CACHE_VERSION = 1


def default_match_cache_dir(
    paths: list[str],
    matcher: str,
    resize: int,
    max_keypoints: int,
    resize_mode: str = "width",
) -> str:
    abs_paths = [os.path.abspath(p) for p in paths]
    try:
        root = os.path.commonpath(abs_paths)
    except ValueError:
        root = os.path.dirname(abs_paths[0])
    if os.path.isfile(root):
        root = os.path.dirname(root)
    safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in matcher)
    rm = str(resize_mode).lower()
    prefix = "w" if rm == "width" else "r"
    tag = f"{safe}_{prefix}{int(resize)}_k{int(max_keypoints)}"
    return os.path.join(root, ".pytheia_matches", tag)


def match_pair_cache_key(
    path_a: str,
    path_b: str,
    matcher: str,
    resize: int,
    max_keypoints: int,
    resize_mode: str = "width",
) -> str:
    import hashlib

    ap = os.path.abspath(path_a)
    bp = os.path.abspath(path_b)
    sta = os.stat(ap)
    stb = os.stat(bp)
    payload = (
        f"{ap}|{bp}|{sta.st_mtime_ns}|{sta.st_size}|{stb.st_mtime_ns}|{stb.st_size}|"
        f"{matcher}|{int(resize)}|{resize_mode}|{int(max_keypoints)}|"
        f"{_MATCH_CACHE_VERSION}"
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()


def resolve_match_cache_dir(
    options: StereoRigRunOptions, paths: list[str]
) -> str | None:
    if not options.match_cache:
        return None
    if options.match_cache_dir:
        return os.path.abspath(options.match_cache_dir)
    return default_match_cache_dir(
        paths,
        options.matcher,
        options.resize,
        options.max_keypoints,
        options.resize_mode,
    )


class MatchPairDiskCache:
    """On-disk cache of raw LightGlue matched keypoints per ordered image pair."""

    def __init__(
        self,
        disk_dir: str,
        matcher_name: str,
        resize: int,
        max_keypoints: int,
        resize_mode: str = "width",
    ):
        self.disk_dir = disk_dir
        self.matcher_name = matcher_name
        self.resize = int(resize)
        self.max_keypoints = int(max_keypoints)
        self.resize_mode = str(resize_mode)
        self.disk_hits = 0
        self.disk_misses = 0

    def _disk_path(self, path_a: str, path_b: str) -> str:
        key = match_pair_cache_key(
            path_a,
            path_b,
            self.matcher_name,
            self.resize,
            self.max_keypoints,
            self.resize_mode,
        )
        stem_a = os.path.splitext(os.path.basename(path_a))[0]
        stem_b = os.path.splitext(os.path.basename(path_b))[0]
        return os.path.join(self.disk_dir, f"{stem_a}_{stem_b}_{key[:16]}.npz")

    def try_load(self, path_a: str, path_b: str):
        import numpy as np

        path = self._disk_path(path_a, path_b)
        if not os.path.isfile(path):
            return None
        try:
            blob = np.load(path, allow_pickle=False)
        except Exception as exc:
            print(f"  warning: could not read match cache {path}: {exc}")
            return None
        if int(blob.get("version", -1)) != _MATCH_CACHE_VERSION:
            return None
        if blob.get("matcher") != self.matcher_name:
            return None
        if int(blob.get("resize", -1)) != self.resize:
            return None
        if blob.get("resize_mode", "max") != self.resize_mode:
            return None
        if int(blob.get("max_keypoints", -1)) != self.max_keypoints:
            return None
        if str(blob.get("path_a", "")) != os.path.abspath(path_a):
            return None
        if str(blob.get("path_b", "")) != os.path.abspath(path_b):
            return None
        k0 = np.asarray(blob["matched_kpts0"], dtype=np.float64)
        k1 = np.asarray(blob["matched_kpts1"], dtype=np.float64)
        if k0.ndim != 2 or k1.ndim != 2 or k0.shape[1] != 2 or k1.shape[1] != 2:
            return None
        if k0.shape[0] != k1.shape[0]:
            return None
        self.disk_hits += 1
        return k0, k1

    def save(self, path_a: str, path_b: str, kpts0, kpts1) -> None:
        import numpy as np

        os.makedirs(self.disk_dir, exist_ok=True)
        path = self._disk_path(path_a, path_b)
        # np.savez_compressed appends ".npz" when the path does not end with it.
        tmp_base = path[:-4] + "_tmp" if path.endswith(".npz") else path + "_tmp"
        k0 = np.asarray(kpts0, dtype=np.float64).reshape(-1, 2)
        k1 = np.asarray(kpts1, dtype=np.float64).reshape(-1, 2)
        np.savez_compressed(
            tmp_base,
            version=_MATCH_CACHE_VERSION,
            matcher=self.matcher_name,
            resize=self.resize,
            resize_mode=self.resize_mode,
            max_keypoints=self.max_keypoints,
            path_a=os.path.abspath(path_a),
            path_b=os.path.abspath(path_b),
            matched_kpts0=k0,
            matched_kpts1=k1,
        )
        os.replace(tmp_base + ".npz", path)
        self.disk_misses += 1


class DetectOnceCache:
    """Detect keypoints once per image; pair matching only runs LightGlue."""

    def __init__(
        self,
        matcher,
        resize: int,
        device: str,
        matcher_name: str = "",
        max_keypoints: int = 2048,
        disk_dir: str | None = None,
        resize_mode: str = "width",
    ):
        self.matcher = matcher
        self.resize = int(resize)
        self.resize_mode = str(resize_mode)
        self.device = device
        self.matcher_name = matcher_name or getattr(matcher, "name", "") or "matcher"
        self.max_keypoints = int(max_keypoints)
        self.disk_dir = disk_dir
        self.backend = self._infer_backend(matcher)
        self._cache: dict = {}
        self.disk_hits = 0
        self.disk_misses = 0

    @staticmethod
    def _infer_backend(matcher) -> str | None:
        ext = getattr(matcher, "extractor", None)
        lg = getattr(matcher, "matcher", None)
        if ext is not None and lg is not None and callable(getattr(ext, "extract", None)):
            return "lightglue"
        return None

    @property
    def supported(self) -> bool:
        return self.backend is not None

    def extract_paths(self, paths: list[str]) -> None:
        import cv2

        unique = []
        seen = set()
        for p in paths:
            ap = os.path.abspath(p)
            if ap not in seen:
                seen.add(ap)
                unique.append(ap)
        n = len(unique)
        self.disk_hits = 0
        self.disk_misses = 0
        where = self.disk_dir or "memory-only"
        print(f"Detect-once ({self.backend}): extracting {n} images  [cache={where}]")
        if self.disk_dir:
            os.makedirs(self.disk_dir, exist_ok=True)
        for i, ap in enumerate(unique, 1):
            self._extract_one(ap, cv2)
            if i == 1 or i == n or i % 20 == 0:
                print(
                    f"  extract {i}/{n}  "
                    f"(disk hit {self.disk_hits}, compute {self.disk_misses})"
                )
        if self.disk_dir:
            print(
                f"Feature cache: {self.disk_hits} loaded, "
                f"{self.disk_misses} written under {self.disk_dir}"
            )

    def _disk_path(self, ap: str) -> str | None:
        if not self.disk_dir or not self.backend:
            return None
        key = feature_cache_key(
            ap,
            self.matcher_name,
            self.resize,
            self.max_keypoints,
            self.backend,
            self.resize_mode,
        )
        stem = os.path.splitext(os.path.basename(ap))[0]
        return os.path.join(self.disk_dir, f"{stem}_{key[:16]}.pt")

    def _try_load_disk(self, ap: str):
        import torch

        path = self._disk_path(ap)
        if path is None or not os.path.isfile(path):
            return None
        try:
            try:
                blob = torch.load(path, map_location="cpu", weights_only=False)
            except TypeError:
                blob = torch.load(path, map_location="cpu")
        except Exception as exc:
            print(f"  warning: could not read feature cache {path}: {exc}")
            return None
        if not isinstance(blob, dict) or blob.get("version") != _FEATURE_CACHE_VERSION:
            return None
        if blob.get("backend") != self.backend:
            return None
        if blob.get("resize_mode", "max") != self.resize_mode:
            return None
        rec = blob.get("record")
        if not isinstance(rec, dict):
            return None
        return _tensors_to(rec, self.device)

    def _save_disk(self, ap: str, rec: dict) -> None:
        import torch

        path = self._disk_path(ap)
        if path is None:
            return
        tmp = path + ".tmp"
        blob = {
            "version": _FEATURE_CACHE_VERSION,
            "backend": self.backend,
            "matcher": self.matcher_name,
            "resize": self.resize,
            "resize_mode": self.resize_mode,
            "max_keypoints": self.max_keypoints,
            "path": ap,
            "record": _tensors_cpu(rec),
        }
        torch.save(blob, tmp)
        os.replace(tmp, path)

    def kpts_fullres(self, path: str):
        rec = self._cache.get(os.path.abspath(path))
        if rec is None:
            return None
        return rec["kpts_full"]

    def match(self, path_a: str, path_b: str):
        """Return (ok, result_dict, full_wh_a, full_wh_b, matched_hw_a, matched_hw_b).

        Keypoints in the result are already in full-resolution pixels; matched_hw
        is set to the full image size so callers can still run ``_scale_kpts``.
        """
        a = self._cache[os.path.abspath(path_a)]
        b = self._cache[os.path.abspath(path_b)]
        mk0, mk1 = self._match_lightglue(a, b)
        if mk0 is None or len(mk0) == 0:
            return False, {}, a["full_wh"], b["full_wh"], a["full_hw"], b["full_hw"]
        result = {"matched_kpts0": mk0, "matched_kpts1": mk1}
        return True, result, a["full_wh"], b["full_wh"], a["full_hw"], b["full_hw"]

    def _extract_one(self, ap: str, cv2_mod) -> None:
        import numpy as np
        import torch

        loaded = self._try_load_disk(ap)
        if loaded is not None:
            self._cache[ap] = loaded
            self.disk_hits += 1
            return

        img = cv2_mod.imread(ap)
        if img is None:
            raise FileNotFoundError(f"Failed to read {ap}")
        h, w = img.shape[:2]
        full_wh = (w, h)
        resize_arg = matcher_resize_arg(self.resize, self.resize_mode, w, h)
        tensor = self.matcher.load_image(ap, resize=resize_arg)
        mh, mw = _tensor_hw(tensor, (h, w))
        with torch.inference_mode():
            rec = self._extract_lightglue(tensor, full_wh, (mh, mw))
        rec["full_wh"] = full_wh
        rec["full_hw"] = (h, w)
        rec["matched_hw"] = (mh, mw)
        rec["kpts_full"] = _scale_kpts(rec["kpts_matched"], full_wh, (mh, mw))
        if np.asarray(rec["kpts_full"]).size == 0:
            rec["kpts_full"] = np.zeros((0, 2), dtype=np.float64)
        self._cache[ap] = rec
        del tensor
        self._save_disk(ap, rec)
        self.disk_misses += 1

    def _extract_lightglue(self, tensor, full_wh, matched_hw):
        img = tensor.to(self.device)
        feats = self.matcher.extractor.extract(img)
        kpts = _to_numpy(feats["keypoints"])
        if kpts is not None and kpts.ndim == 3:
            kpts = kpts[0]
        return {"feats": feats, "kpts_matched": kpts}

    def _match_lightglue(self, a, b):
        import torch

        lg = self.matcher.matcher
        with torch.inference_mode():
            pred = lg({"image0": a["feats"], "image1": b["feats"]})
        matches = pred.get("matches")
        if matches is None:
            m0 = pred.get("matches0")
            if m0 is None:
                return None, None
            m0 = m0[0] if getattr(m0, "ndim", 1) > 1 else m0
            m0 = _to_numpy(m0).reshape(-1)
            i0 = [i for i, j in enumerate(m0) if int(j) >= 0]
            i1 = [int(m0[i]) for i in i0]
            if not i0:
                return None, None
            return a["kpts_full"][i0], b["kpts_full"][i1]
        if isinstance(matches, (list, tuple)):
            matches = matches[0]
        matches = _to_numpy(matches)
        if matches is None or matches.size == 0:
            return None, None
        matches = matches.reshape(-1, 2).astype("int64")
        return a["kpts_full"][matches[:, 0]], b["kpts_full"][matches[:, 1]]


def release_torch_cuda(matcher=None, detect_cache=None, img_cache=None) -> None:
    """Drop GPU matcher state before Ceres BA (Torch CUDA + OpenMP otherwise SIGSEGV)."""
    if detect_cache is not None:
        detect_cache._cache.clear()
        detect_cache.matcher = None
    if img_cache is not None:
        img_cache.clear()
    _ = matcher
    try:
        import gc

        gc.collect()
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
    except Exception:
        pass


def left_motion_pairs(n: int, options: StereoRigRunOptions) -> list[tuple[int, int, str]]:
    """Left image pairs to match. Cascade is i vs every later j (optionally capped)."""
    pairs: list[tuple[int, int, str]] = []
    seen: set[tuple[int, int]] = set()

    def add(i: int, j: int, kind: str) -> None:
        if j <= i or j >= n:
            return
        key = (i, j)
        if key in seen:
            return
        seen.add(key)
        pairs.append((i, j, kind))

    mode = (options.left_match_mode or "cascade").lower()
    if mode not in ("cascade", "window"):
        raise ValueError(
            f"left_match_mode must be 'cascade' or 'window', got {options.left_match_mode!r}"
        )
    if mode == "window":
        for i in range(n):
            for dt in range(1, max(int(options.temporal_window), 0) + 1):
                add(i, i + dt, "temporal")
            if options.loop_stride > 0:
                j = i + options.loop_stride
                while j < n:
                    add(i, j, "loop")
                    j += options.loop_stride
        return pairs

    max_gap = int(options.left_match_max_gap)
    for i in range(n):
        j_end = n if max_gap <= 0 else min(n, i + 1 + max_gap)
        for j in range(i + 1, j_end):
            add(i, j, "cascade")
        if options.loop_stride > 0 and max_gap > 0:
            j = i + options.loop_stride
            while j < n:
                add(i, j, "loop")
                j += options.loop_stride
    return pairs


_COSPLACE_BACKBONE = "ResNet18"
_COSPLACE_DIM = 128
_COSPLACE_INPUT = 512
_COSPLACE_CACHE_VERSION = 1
_COSPLACE_FRAME_ID_RE = re.compile(r"(\d+)$")


def resolve_cosplace_min_frame_gap(options: StereoRigRunOptions) -> int:
    """Skip GraphMatch pairs already covered by the geometric schedule."""
    if int(options.cosplace_min_frame_gap) > 0:
        return int(options.cosplace_min_frame_gap)
    mode = (options.left_match_mode or "cascade").lower()
    if mode == "window":
        return max(int(options.temporal_window) + 1, 15)
    max_gap = int(options.left_match_max_gap)
    if max_gap > 0:
        return max_gap + 1
    return 15


def frame_id_from_left_path(path: str) -> int:
    """Numeric frame id from a left image name (`000123.png`, `left_000123`)."""
    stem = os.path.splitext(os.path.basename(path))[0]
    try:
        return int(stem)
    except ValueError:
        match = _COSPLACE_FRAME_ID_RE.search(stem)
        if match is None:
            raise ValueError(f"Cannot parse CosPlace frame id from {path}")
        return int(match.group(1))


def default_cosplace_cache_path(left_paths: list[str]) -> str:
    """One npz next to the left-image folder: descriptors[frame_id] rows."""
    abs_paths = [os.path.abspath(p) for p in left_paths]
    try:
        root = os.path.commonpath(abs_paths)
    except ValueError:
        root = os.path.dirname(abs_paths[0])
    if os.path.isfile(root):
        root = os.path.dirname(root)
    tag = f"cosplace_{_COSPLACE_BACKBONE.lower()}_d{_COSPLACE_DIM}.npz"
    return os.path.join(root, ".pytheia_features", tag)


def _empty_cosplace_cache():
    import numpy as np

    return {
        "frame_ids": np.zeros((0,), dtype=np.int64),
        "descriptors": np.zeros((0, _COSPLACE_DIM), dtype=np.float32),
        "mtime_ns": np.zeros((0,), dtype=np.int64),
        "size": np.zeros((0,), dtype=np.int64),
    }


def load_cosplace_cache(path: str):
    import numpy as np

    if not path or not os.path.isfile(path):
        return _empty_cosplace_cache()
    try:
        with np.load(path, allow_pickle=False) as blob:
            version = int(np.asarray(blob["version"]).reshape(-1)[0])
            dim = int(np.asarray(blob["dim"]).reshape(-1)[0])
            backbone_raw = np.asarray(blob["backbone"]).reshape(-1)[0]
            backbone = (
                backbone_raw.decode("utf-8")
                if isinstance(backbone_raw, (bytes, np.bytes_))
                else str(backbone_raw)
            )
            if (
                version != _COSPLACE_CACHE_VERSION
                or dim != _COSPLACE_DIM
                or backbone != _COSPLACE_BACKBONE
            ):
                return _empty_cosplace_cache()
            frame_ids = np.asarray(blob["frame_ids"], dtype=np.int64).reshape(-1)
            descriptors = np.asarray(blob["descriptors"], dtype=np.float32)
            if descriptors.ndim != 2 or descriptors.shape[1] != _COSPLACE_DIM:
                return _empty_cosplace_cache()
            if descriptors.shape[0] != frame_ids.shape[0]:
                return _empty_cosplace_cache()
            mtime_ns = np.asarray(blob["mtime_ns"], dtype=np.int64).reshape(-1)
            size = np.asarray(blob["size"], dtype=np.int64).reshape(-1)
            if mtime_ns.shape[0] != frame_ids.shape[0]:
                mtime_ns = np.zeros_like(frame_ids)
            if size.shape[0] != frame_ids.shape[0]:
                size = np.zeros_like(frame_ids)
            order = np.argsort(frame_ids, kind="stable")
            return {
                "frame_ids": frame_ids[order],
                "descriptors": descriptors[order],
                "mtime_ns": mtime_ns[order],
                "size": size[order],
            }
    except Exception as exc:
        print(f"  warning: could not read CosPlace cache {path}: {exc}")
        return _empty_cosplace_cache()


def save_cosplace_cache(path: str, cache) -> None:
    import numpy as np

    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        np.savez_compressed(
            f,
            frame_ids=np.asarray(cache["frame_ids"], dtype=np.int64),
            descriptors=np.asarray(cache["descriptors"], dtype=np.float32),
            mtime_ns=np.asarray(cache["mtime_ns"], dtype=np.int64),
            size=np.asarray(cache["size"], dtype=np.int64),
            version=np.int32(_COSPLACE_CACHE_VERSION),
            dim=np.int32(_COSPLACE_DIM),
            input=np.int32(_COSPLACE_INPUT),
            backbone=np.asarray(_COSPLACE_BACKBONE),
        )
    os.replace(tmp, path)


def merge_cosplace_cache(cache, frame_ids, descriptors, mtime_ns, size):
    """Upsert rows keyed by frame id. Returns a sorted cache dict."""
    import numpy as np

    ids = np.asarray(frame_ids, dtype=np.int64).reshape(-1)
    desc = np.asarray(descriptors, dtype=np.float32).reshape(-1, _COSPLACE_DIM)
    mt = np.asarray(mtime_ns, dtype=np.int64).reshape(-1)
    sz = np.asarray(size, dtype=np.int64).reshape(-1)
    old_ids = np.asarray(cache["frame_ids"], dtype=np.int64).reshape(-1)
    rows = {
        int(old_ids[i]): (
            cache["descriptors"][i],
            int(cache["mtime_ns"][i]),
            int(cache["size"][i]),
        )
        for i in range(old_ids.shape[0])
    }
    for i, fid in enumerate(ids):
        rows[int(fid)] = (desc[i], int(mt[i]), int(sz[i]))
    sorted_ids = np.array(sorted(rows), dtype=np.int64)
    n = sorted_ids.shape[0]
    out_desc = np.zeros((n, _COSPLACE_DIM), dtype=np.float32)
    out_mt = np.zeros((n,), dtype=np.int64)
    out_sz = np.zeros((n,), dtype=np.int64)
    for i, fid in enumerate(sorted_ids):
        vec, mti, szi = rows[int(fid)]
        out_desc[i] = np.asarray(vec, dtype=np.float32).reshape(_COSPLACE_DIM)
        out_mt[i] = mti
        out_sz[i] = szi
    return {
        "frame_ids": sorted_ids,
        "descriptors": out_desc,
        "mtime_ns": out_mt,
        "size": out_sz,
    }


def lookup_cosplace_cache(cache, left_paths: list[str]):
    """Return (N, dim) descriptors, pending indices, and per-path frame ids."""
    import numpy as np

    n = len(left_paths)
    desc = np.zeros((n, _COSPLACE_DIM), dtype=np.float32)
    pending: list[int] = []
    frame_ids = np.zeros((n,), dtype=np.int64)
    mtime_ns = np.zeros((n,), dtype=np.int64)
    sizes = np.zeros((n,), dtype=np.int64)
    cached_ids = np.asarray(cache["frame_ids"], dtype=np.int64).reshape(-1)
    id_to_row = {int(fid): i for i, fid in enumerate(cached_ids)}
    for i, path in enumerate(left_paths):
        try:
            fid = frame_id_from_left_path(path)
        except ValueError:
            fid = i
        frame_ids[i] = fid
        st = os.stat(path)
        mtime_ns[i] = int(st.st_mtime_ns)
        sizes[i] = int(st.st_size)
        row = id_to_row.get(int(fid))
        if (
            row is not None
            and int(cache["mtime_ns"][row]) == mtime_ns[i]
            and int(cache["size"][row]) == sizes[i]
        ):
            desc[i] = cache["descriptors"][row]
            continue
        pending.append(i)
    return desc, pending, frame_ids, mtime_ns, sizes


def _load_cosplace_model(device: str):
    import torch

    model = torch.hub.load(
        "gmberton/cosplace",
        "get_trained_model",
        backbone=_COSPLACE_BACKBONE,
        fc_output_dim=_COSPLACE_DIM,
    )
    model.to(device)
    model.eval()
    return model


def _cosplace_image_tensor(bgr, device: str):
    import numpy as np
    import torch

    if bgr is None or bgr.size == 0:
        raise ValueError("empty image for CosPlace")
    if bgr.ndim == 2:
        rgb = np.stack([bgr, bgr, bgr], axis=-1)
    else:
        rgb = bgr[:, :, :3][:, :, ::-1].copy()
    t = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float() / 255.0
    t = torch.nn.functional.interpolate(
        t.unsqueeze(0),
        size=(_COSPLACE_INPUT, _COSPLACE_INPUT),
        mode="bilinear",
        align_corners=False,
    )[0]
    mean = torch.tensor([0.485, 0.456, 0.406], dtype=t.dtype).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], dtype=t.dtype).view(3, 1, 1)
    return ((t - mean) / std).to(device)


def extract_cosplace_descriptors(
    paths: list[str],
    device: str = "cuda",
    cache_path: str | None = None,
    batch_size: int = 8,
):
    """(N, 128) CosPlace vectors for left images |paths|.

    Cached as one npz per left-image folder. Rows are keyed by numeric frame id
    (``000123.png`` → 123), so a later run with more/fewer frames reuses hits.
    """
    import numpy as np
    import torch

    n = len(paths)
    cache = load_cosplace_cache(cache_path) if cache_path else _empty_cosplace_cache()
    desc, pending, frame_ids, mtime_ns, sizes = lookup_cosplace_cache(cache, paths)
    print(
        f"CosPlace ({_COSPLACE_BACKBONE} dim={_COSPLACE_DIM}): "
        f"{n} left frames  [cache={cache_path or 'memory-only'}]  "
        f"disk hit {n - len(pending)}, compute {len(pending)}"
    )
    if not pending:
        return desc

    import cv2

    model = _load_cosplace_model(device)
    try:
        with torch.no_grad():
            for start in range(0, len(pending), max(int(batch_size), 1)):
                chunk = pending[start : start + max(int(batch_size), 1)]
                batch = torch.stack(
                    [
                        _cosplace_image_tensor(cv2.imread(paths[i]), device)
                        for i in chunk
                    ],
                    dim=0,
                )
                out = model(batch).detach().cpu().numpy().astype(np.float32)
                if out.ndim == 3:
                    out = out.reshape(out.shape[0], -1)
                for row, idx in enumerate(chunk):
                    vec = np.asarray(out[row], dtype=np.float32).reshape(-1)
                    if vec.size != _COSPLACE_DIM:
                        raise RuntimeError(
                            f"CosPlace returned dim {vec.size}, expected {_COSPLACE_DIM}"
                        )
                    desc[idx] = vec
                done = min(start + len(chunk), len(pending))
                if done == len(pending) or start == 0 or done % 32 == 0:
                    print(f"  CosPlace extract {done}/{len(pending)}")
    finally:
        del model
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    if cache_path:
        cache = merge_cosplace_cache(
            cache,
            frame_ids[pending],
            desc[pending],
            mtime_ns[pending],
            sizes[pending],
        )
        save_cosplace_cache(cache_path, cache)
        print(
            f"  wrote {len(pending)} new rows "
            f"({cache['frame_ids'].shape[0]} frames in cache)"
        )
    return desc


def loop_pairs_from_global_descriptors(
    descriptors,
    k: int,
    min_frame_gap: int,
    already: set[tuple[int, int]] | None = None,
) -> list[tuple[int, int, str]]:
    """GraphMatch on global descriptors, then drop short-gap / already-scheduled pairs."""
    import numpy as np
    import pytheia as pt

    desc = np.asarray(descriptors, dtype=np.float32)
    if desc.ndim != 2 or desc.shape[0] < 2:
        return []
    n = int(desc.shape[0])
    k_nn = max(int(k), 1)
    names = [f"{i:06d}" for i in range(n)]
    name_pairs = pt.matching.GraphMatch(
        names, [desc[i] for i in range(n)], k_nn
    )
    skip = already or set()
    gap = max(int(min_frame_gap), 1)
    out: list[tuple[int, int, str]] = []
    seen: set[tuple[int, int]] = set()
    for a_name, b_name in name_pairs:
        i, j = int(a_name), int(b_name)
        if j < i:
            i, j = j, i
        if j - i < gap or (i, j) in skip or (i, j) in seen:
            continue
        seen.add((i, j))
        out.append((i, j, "loop"))
    return out


def cosplace_loop_pairs(
    left_paths: list[str],
    options: StereoRigRunOptions,
    already: set[tuple[int, int]] | None = None,
) -> list[tuple[int, int, str]]:
    """Precompute CosPlace on |left_paths| and return GraphMatch loop candidates."""
    if not left_paths:
        return []
    cache_path = None
    if options.feature_cache:
        cache_path = default_cosplace_cache_path(left_paths)
    device = options.device or "cpu"
    try:
        import torch

        if str(device).startswith("cuda") and not torch.cuda.is_available():
            device = "cpu"
    except ImportError as exc:
        raise ImportError(
            "trajectory_has_loops requires torch (CosPlace via torch.hub)"
        ) from exc
    desc = extract_cosplace_descriptors(
        left_paths, device=device, cache_path=cache_path
    )
    gap = resolve_cosplace_min_frame_gap(options)
    pairs = loop_pairs_from_global_descriptors(
        desc,
        k=options.cosplace_neighbors,
        min_frame_gap=gap,
        already=already,
    )
    print(
        f"CosPlace GraphMatch: k={options.cosplace_neighbors} "
        f"min_gap={gap}  loop candidates={len(pairs)}"
    )
    return pairs


class _EstimatedTrackIndex:
    """Snap a 2D point onto estimated tracks already in a reconstructed view."""

    def __init__(self, recon, view_id, radius_px: float):
        import numpy as np

        self._tids: list = []
        xy: list = []
        view = recon.View(view_id)
        if view is not None:
            for tid in view.TrackIds():
                track = recon.Track(tid)
                if track is None or not track.IsEstimated():
                    continue
                feat = view.GetFeature(tid)
                if feat is None:
                    continue
                p = np.asarray(feat.point, dtype=np.float64).reshape(-1)[:2]
                xy.append([float(p[0]), float(p[1])])
                self._tids.append(tid)
        self._arr = (
            np.asarray(xy, dtype=np.float64).reshape(-1, 2)
            if xy
            else np.zeros((0, 2), dtype=np.float64)
        )
        self._r2 = float(radius_px) * float(radius_px)

    def snap_tid(self, xy):
        import numpy as np

        if self._arr.shape[0] == 0:
            return None
        d = self._arr - np.asarray(xy, dtype=np.float64).reshape(2)
        dist2 = np.einsum("ij,ij->i", d, d)
        j = int(np.argmin(dist2))
        if dist2[j] > self._r2:
            return None
        return self._tids[j]


def _neighboring_keyframes(i: int, keyframes: list[int]) -> list[int]:
    import bisect

    pos = bisect.bisect_left(keyframes, i)
    out: list[int] = []
    if pos > 0:
        out.append(keyframes[pos - 1])
    if pos < len(keyframes):
        out.append(keyframes[pos])
    return out


def _query_camera_in_rig_frame(pt, recon, view_id, sensor):
    query = pt.sfm.Camera(recon.View(view_id).Camera())
    query.SetPosition(sensor.position)
    query.SetOrientationFromAngleAxis(sensor.orientation)
    return query


def localize_skipped_rig_captures(
    pt,
    recon,
    *,
    skipped: list[int],
    keyframes: list[int],
    left_paths: list[str],
    left_views: list,
    capture_ids: list,
    left_sensor_id,
    rig_id,
    match_pair,
    options: StereoRigRunOptions,
) -> dict:
    """PnP each skipped capture against 3D tracks seen in neighboring keyframes."""
    import numpy as np

    stats = {"pnp_localized": 0, "pnp_failed": 0, "pnp_correspondences": 0}
    if not skipped:
        return stats

    rig = recon.GetCameraRig(rig_id)
    sensor = rig.GetSensor(left_sensor_id)
    track_index = {
        k: _EstimatedTrackIndex(
            recon, left_views[k], options.stereo_snap_pixels
        )
        for k in keyframes
    }
    query_cam = _query_camera_in_rig_frame(pt, recon, left_views[keyframes[0]], sensor)
    thresh = float(options.max_reprojection_error_in_pixels)
    ransac = pt.solvers.RansacParameters()
    ransac.error_thresh = thresh * thresh
    ransac.max_iterations = 1000
    ransac.min_iterations = 50
    min_inliers = max(int(options.pnp_min_inliers), 4)
    # Per-neighbor LightGlue can be sparse; accumulate then require min_inliers.

    print(
        f"PnP-localizing {len(skipped)} skipped captures "
        f"(match to neighboring keyframes, min_inliers={min_inliers})..."
    )
    for si, i in enumerate(skipped, 1):
        neighbors = _neighboring_keyframes(i, keyframes)
        correspondences = []
        obs_feats = []
        obs_tids = []
        seen_tids: set = set()
        for k in neighbors:
            ok, cors = match_pair(left_paths[i], left_paths[k], 8)
            if not ok:
                continue
            snap = track_index[k]
            for c in cors:
                tid = snap.snap_tid(_feature_xy(c.feature2))
                if tid is None or tid in seen_tids:
                    continue
                track = recon.Track(tid)
                if track is None or not track.IsEstimated():
                    continue
                seen_tids.add(tid)
                xy = np.asarray(c.feature1.point, dtype=np.float64).reshape(-1)[:2]
                corr = pt.sfm.CameraAndFeatureCorrespondence2D3D()
                corr.camera = query_cam
                corr.observation = pt.sfm.Feature(xy)
                corr.point3d = np.asarray(track.Point(), dtype=np.float64).reshape(4)
                correspondences.append(corr)
                obs_feats.append(pt.sfm.Feature(xy))
                obs_tids.append(tid)

        if len(correspondences) < min_inliers:
            stats["pnp_failed"] += 1
            if options.verbose_matches:
                print(
                    f"  skip t={i}: {len(correspondences)} 2D-3D "
                    f"(need {min_inliers})"
                )
            continue

        ok, transform, summary = pt.sfm.EstimateRigidTransformation2D3D(
            ransac, pt.sfm.RansacType.RANSAC, correspondences
        )
        n_inliers = len(summary.inliers) if ok else 0
        if not ok or n_inliers < min_inliers:
            stats["pnp_failed"] += 1
            if options.verbose_matches:
                print(f"  skip t={i}: PnP failed (inliers={n_inliers})")
            continue

        R = np.asarray(transform.rotation, dtype=np.float64)
        t = np.asarray(transform.translation, dtype=np.float64).reshape(3)
        cap = recon.MutableRigCapture(capture_ids[i])
        cap.SetOrientationFromRotationMatrix(R)
        cap.SetPosition(-R.T @ t)
        cap.SetEstimated(True)
        pt.sfm.PropagateCameraPosesForCapture(capture_ids[i], recon)

        query_view = left_views[i]
        for k in summary.inliers:
            recon.AddObservation(query_view, obs_tids[k], obs_feats[k])

        stats["pnp_localized"] += 1
        stats["pnp_correspondences"] += n_inliers
        if si == 1 or si == len(skipped) or si % 20 == 0:
            print(
                f"  pnp {si}/{len(skipped)}  "
                f"ok={stats['pnp_localized']} fail={stats['pnp_failed']}"
            )

    print(
        f"PnP fill: localized {stats['pnp_localized']}/{len(skipped)} "
        f"(failed {stats['pnp_failed']})"
    )
    return stats


class _FeatureIndex:
    """Nearest-neighbour snap so per-pair matcher keypoints chain into tracks."""

    def __init__(self, radius_px: float):
        import numpy as np

        self.radius_px = float(radius_px)
        self._r2 = self.radius_px * self.radius_px
        self._xy: list = []
        self._feats: list = []
        self._arr = np.zeros((0, 2), dtype=np.float64)

    def __len__(self) -> int:
        return len(self._feats)

    def snap(self, xy):
        import numpy as np

        if self._arr.shape[0] == 0:
            return None
        d = self._arr - np.asarray(xy, dtype=np.float64).reshape(2)
        dist2 = np.einsum("ij,ij->i", d, d)
        j = int(np.argmin(dist2))
        if dist2[j] > self._r2:
            return None
        return self._feats[j]

    def snap_or_add(self, xy, pt_mod):
        hit = self.snap(xy)
        if hit is not None:
            return hit
        import numpy as np

        xy = np.asarray(xy, dtype=np.float64).reshape(2)
        feat = pt_mod.sfm.Feature(xy)
        self._xy.append([float(xy[0]), float(xy[1])])
        self._feats.append(feat)
        self._arr = np.asarray(self._xy, dtype=np.float64).reshape(-1, 2)
        return feat

    def add_existing(self, feat) -> None:
        import numpy as np

        xy = np.asarray(feat.point, dtype=np.float64).reshape(-1)[:2]
        self._xy.append([float(xy[0]), float(xy[1])])
        self._feats.append(feat)
        self._arr = np.asarray(self._xy, dtype=np.float64).reshape(-1, 2)

    def seed_from_kpts(self, kpts, pt_mod) -> None:
        """Register a fixed keypoint set without nearest-neighbour rebuilds per point."""
        import numpy as np

        k = np.asarray(kpts, dtype=np.float64).reshape(-1, 2)
        for xy in k:
            feat = pt_mod.sfm.Feature(xy)
            self._xy.append([float(xy[0]), float(xy[1])])
            self._feats.append(feat)
        if self._xy:
            self._arr = np.asarray(self._xy, dtype=np.float64).reshape(-1, 2)


def _feature_xy(feat) -> tuple[float, float]:
    import numpy as np

    xy = np.asarray(feat.point, dtype=np.float64).reshape(-1)[:2]
    return float(xy[0]), float(xy[1])


@dataclass
class _MotionMatchEval:
    n_matched: int = 0
    n_ransac_inliers: int = 0
    n_snapped: int = 0
    ransac_ok: bool = False
    edge_added: bool = False


def _correspondence_xy(cors) -> tuple[list, list]:
    k0 = [_feature_xy(c.feature1) for c in cors]
    k1 = [_feature_xy(c.feature2) for c in cors]
    return k0, k1


def draw_side_by_side_matches(
    img_a,
    img_b,
    kpts_a,
    kpts_b,
    inlier_mask=None,
    max_display_width: int = 1400,
):
    """Side-by-side match visualization (green = RANSAC inlier, gray = raw)."""
    import cv2
    import numpy as np

    ha, wa = img_a.shape[:2]
    hb, wb = img_b.shape[:2]
    h = max(ha, hb)
    canvas = np.zeros((h, wa + wb, 3), dtype=np.uint8)
    canvas[:ha, :wa] = img_a
    canvas[:hb, wa : wa + wb] = img_b
    k0 = np.asarray(kpts_a, dtype=np.float64).reshape(-1, 2)
    k1 = np.asarray(kpts_b, dtype=np.float64).reshape(-1, 2)
    n = min(len(k0), len(k1))
    for i in range(n):
        inl = inlier_mask is None or bool(inlier_mask[i])
        color = (0, 220, 0) if inl else (70, 70, 70)
        p0 = (int(round(k0[i, 0])), int(round(k0[i, 1])))
        p1 = (int(round(k1[i, 0])) + wa, int(round(k1[i, 1])))
        cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
    total_w = canvas.shape[1]
    if total_w > max_display_width:
        scale = max_display_width / float(total_w)
        canvas = cv2.resize(
            canvas,
            (int(round(total_w * scale)), int(round(h * scale))),
            interpolation=cv2.INTER_AREA,
        )
    return canvas


def dump_left_match_debug(
    debug_dir: str,
    path_a: str,
    path_b: str,
    frame_a: int,
    frame_b: int,
    kind: str,
    cors,
    inlier_idx,
    stats: _MotionMatchEval,
) -> str:
    """Save one left–left match panel; return output path."""
    import cv2
    import json
    import os

    os.makedirs(debug_dir, exist_ok=True)
    img_a = cv2.imread(path_a)
    img_b = cv2.imread(path_b)
    if img_a is None or img_b is None:
        raise FileNotFoundError(f"Could not read {path_a!r} or {path_b!r}")
    k0, k1 = _correspondence_xy(cors)
    mask = [False] * len(k0)
    for j in inlier_idx:
        if 0 <= int(j) < len(mask):
            mask[int(j)] = True
    panel = draw_side_by_side_matches(img_a, img_b, k0, k1, mask)
    title = (
        f"left {frame_a:04d}-{frame_b:04d} ({kind})  "
        f"matched={stats.n_matched}  ransac={stats.n_ransac_inliers}  "
        f"snapped={stats.n_snapped}  edge={'yes' if stats.edge_added else 'no'}"
    )
    cv2.putText(
        panel,
        title,
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    out_png = os.path.join(
        debug_dir, f"left_{frame_a:04d}_{frame_b:04d}_{kind}.png"
    )
    cv2.imwrite(out_png, panel)
    meta = {
        "frame_a": int(frame_a),
        "frame_b": int(frame_b),
        "kind": kind,
        "path_a": os.path.abspath(path_a),
        "path_b": os.path.abspath(path_b),
        "png": out_png,
        "n_matched": stats.n_matched,
        "n_ransac_inliers": stats.n_ransac_inliers,
        "n_snapped": stats.n_snapped,
        "ransac_ok": stats.ransac_ok,
        "edge_added": stats.edge_added,
    }
    jsonl = os.path.join(debug_dir, "left_matches.jsonl")
    with open(jsonl, "a", encoding="utf-8") as f:
        f.write(json.dumps(meta) + "\n")
    return out_png


def unique_track_captures(recon, track) -> set[int]:
    caps: set[int] = set()
    for vid in track.ViewIds():
        if recon.ViewHasRigMembership(vid):
            caps.add(int(recon.GetViewRigMembership(vid).capture_id))
        else:
            caps.add(int(vid))
    return caps


def prune_tracks_by_capture_count(recon, min_captures: int) -> int:
    if min_captures <= 1:
        return 0
    removed = 0
    for tid in list(recon.TrackIds()):
        track = recon.Track(tid)
        if track is None:
            continue
        if len(unique_track_captures(recon, track)) < min_captures:
            recon.RemoveTrack(tid)
            removed += 1
    return removed


def long_track_feature_index(recon, view_id, motion_index: _FeatureIndex, min_views: int, min_captures: int):
    """Index of features already in long tracks, keyed by the motion-match Feature."""
    idx = _FeatureIndex(motion_index.radius_px)
    view = recon.View(view_id)
    if view is None:
        return idx
    for tid in view.TrackIds():
        track = recon.Track(tid)
        if track is None or track.NumViews() < min_views:
            continue
        if len(unique_track_captures(recon, track)) < min_captures:
            continue
        feat = view.GetFeature(tid)
        if feat is None:
            continue
        src = motion_index.snap(_feature_xy(feat))
        idx.add_existing(src if src is not None else feat)
    return idx


def print_reprojection_stats(recon) -> None:
    import collections

    import numpy as np

    per_cam_sum: dict[int, float] = collections.defaultdict(float)
    per_cam_n: dict[int, int] = collections.defaultdict(int)
    cam_names: dict[int, str] = {}
    total_sum = 0.0
    total_n = 0
    for vid in recon.ViewIds():
        view = recon.View(vid)
        if view is None or not view.IsEstimated():
            continue
        cam_id = None
        if recon.ViewHasRigMembership(vid):
            memb = recon.GetViewRigMembership(vid)
            cam_id = int(memb.rig_camera_id)
            if cam_id not in cam_names:
                rig = recon.GetCameraRig(memb.rig_id)
                sensor = rig.GetSensor(memb.rig_camera_id) if rig is not None else None
                cam_names[cam_id] = (
                    sensor.name if sensor is not None else f"camera_{cam_id}"
                )
        for tid in view.TrackIds():
            track = recon.Track(tid)
            if track is None or not track.IsEstimated():
                continue
            feat = view.GetFeature(tid)
            if feat is None:
                continue
            proj = view.Camera().ProjectPoint(track.Point())[1]
            err = float(np.linalg.norm(np.asarray(proj) - np.asarray(feat.point)))
            total_sum += err
            total_n += 1
            if cam_id is not None:
                per_cam_sum[cam_id] += err
                per_cam_n[cam_id] += 1
    if total_n == 0:
        print("Reprojection error: no estimated observations")
        return
    print(
        f"Mean reprojection error (all): {total_sum / total_n:.4f} px  "
        f"({total_n} observations)"
    )
    for cam_id in sorted(per_cam_n.keys()):
        n = per_cam_n[cam_id]
        name = cam_names.get(cam_id, f"camera_{cam_id}")
        print(f"  {name}: {per_cam_sum[cam_id] / n:.4f} px  ({n} observations)")


def _length_summary(values: list[int]) -> str:
    if not values:
        return "none"
    import numpy as np

    a = np.asarray(values, dtype=np.int32)
    return (
        f"n={len(a)}  min={int(a.min())}  median={float(np.median(a)):.1f}  "
        f"mean={float(a.mean()):.2f}  max={int(a.max())}"
    )


def _print_length_histogram(title: str, values: list[int]) -> None:
    import collections

    print(f"{title}: {_length_summary(values)}")
    if not values:
        return
    counts = collections.Counter(values)
    keys = sorted(counts)
    # Compact tail so a 90-frame run does not print 90 lines.
    head = [k for k in keys if k < 15]
    tail = sum(counts[k] for k in keys if k >= 15)
    parts = [f"{k}:{counts[k]}" for k in head]
    if tail:
        parts.append(f">=15:{tail}")
    print("  histogram  " + "  ".join(parts))


def print_track_length_stats(recon) -> None:
    """Debug: views-per-track and unique-capture lengths, all vs estimated."""
    all_views: list[int] = []
    all_caps: list[int] = []
    est_views: list[int] = []
    est_caps: list[int] = []
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        if track is None:
            continue
        n_views = int(track.NumViews())
        n_caps = len(unique_track_captures(recon, track))
        all_views.append(n_views)
        all_caps.append(n_caps)
        if track.IsEstimated():
            est_views.append(n_views)
            est_caps.append(n_caps)
    print("Track length debug")
    _print_length_histogram("  all tracks (views)", all_views)
    _print_length_histogram("  all tracks (rig captures)", all_caps)
    _print_length_histogram("  estimated tracks (views)", est_views)
    _print_length_histogram("  estimated tracks (rig captures)", est_caps)


def mean_stereo_baseline(recon, left_view_ids, right_view_ids) -> float:
    import numpy as np

    lengths = []
    for vl, vr in zip(left_view_ids, right_view_ids):
        if vl is None or vr is None:
            continue
        left = recon.View(vl)
        right = recon.View(vr)
        if (
            left is None
            or right is None
            or not left.IsEstimated()
            or not right.IsEstimated()
        ):
            continue
        lengths.append(
            np.linalg.norm(
                np.asarray(right.Camera().GetPosition())
                - np.asarray(left.Camera().GetPosition())
            )
        )
    if not lengths:
        return float("nan")
    return float(np.mean(lengths))


def left_camera_trajectory(recon, left_view_ids):
    """Estimated left-camera centers and the frame indices they correspond to."""
    import numpy as np

    xyz = []
    indices = []
    for i, vid in enumerate(left_view_ids):
        if vid is None:
            continue
        view = recon.View(vid)
        if view is None or not view.IsEstimated():
            continue
        xyz.append(np.asarray(view.Camera().GetPosition(), dtype=np.float64).reshape(3))
        indices.append(i)
    if not xyz:
        return np.zeros((0, 3)), []
    return np.vstack(xyz), indices


def add_calibrated_stereo_capture(
    pt,
    recon,
    rig_id,
    left_id,
    right_id,
    index: int,
    left_path: str,
    right_path: str,
    prior,
):
    cap = recon.AddRigCapture(
        rig_id,
        float(index),
        {
            left_id: f"L_{index:05d}_{os.path.basename(left_path)}",
            right_id: f"R_{index:05d}_{os.path.basename(right_path)}",
        },
    )
    capture = recon.GetRigCapture(cap)
    vl = capture.ViewIdForCamera(left_id)
    vr = capture.ViewIdForCamera(right_id)
    for vid in (vl, vr):
        view = recon.MutableView(vid)
        view.SetCameraIntrinsicsPrior(prior)
        view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)
    return cap, vl, vr


def run_calibrated_stereo_rig(
    left_paths: list[str],
    right_paths: list[str],
    calib: StereoCalib,
    options: StereoRigRunOptions,
) -> StereoRigRunResult:
    """Match + reconstruct a calibrated stereo sequence. Caller owns GPU teardown."""
    from vismatch import get_matcher

    import cv2
    import numpy as np
    import pytheia as pt

    n = min(len(left_paths), len(right_paths))
    if n < 2:
        raise ValueError(
            f"Need at least 2 stereo pairs (left={len(left_paths)} right={len(right_paths)})"
        )
    options.matcher = require_lightglue_matcher(options.matcher)
    left_paths, right_paths = left_paths[:n], right_paths[:n]

    im0 = cv2.imread(left_paths[0])
    if im0 is None:
        raise FileNotFoundError(f"Failed to read {left_paths[0]}")
    h0, w0 = im0.shape[:2]
    width = calib.width or w0
    height = calib.height or h0
    prior = make_pinhole_prior(pt, calib, width, height)
    left_pos, right_pos = calib.sensor_positions()

    keyframes = reconstruction_keyframe_indices(n, options.recon_stride)
    skipped = [i for i in range(n) if i not in set(keyframes)]
    print(
        f"Using {n} pairs\n  left:  {os.path.dirname(left_paths[0])}\n"
        f"  right: {os.path.dirname(right_paths[0])}\n"
        f"  baseline={calib.baseline:.6f} m  focal={calib.focal:.3f}  "
        f"cx={calib.cx:.3f} cy={calib.cy:.3f}  {width}x{height}\n"
        f"  SfM keyframes={len(keyframes)}/{n}  recon_stride={max(1, int(options.recon_stride))}  "
        f"ba_loss={options.ba_loss} width={options.ba_robust_width}"
    )

    rig = pt.sfm.CameraRig("stereo")
    left_id = rig.AddSensor("left", left_pos, np.zeros(3))
    right_id = rig.AddSensor("right", right_pos, np.zeros(3))
    recon = pt.sfm.Reconstruction()
    rig_id = recon.AddCameraRig(rig)
    view_graph = pt.sfm.ViewGraph()
    track_builder = pt.sfm.TrackBuilder(int(options.min_track_length), 30)

    capture_ids = [None] * n
    left_views = [None] * n
    right_views = [None] * n
    for i in keyframes:
        cap, vl, vr = add_calibrated_stereo_capture(
            pt,
            recon,
            rig_id,
            left_id,
            right_id,
            i,
            left_paths[i],
            right_paths[i],
            prior,
        )
        capture_ids[i] = cap
        left_views[i] = vl
        right_views[i] = vr

    compact_pairs = left_motion_pairs(len(keyframes), options)
    if options.trajectory_has_loops:
        already = {(a, b) for a, b, _ in compact_pairs}
        extra = cosplace_loop_pairs(
            [left_paths[i] for i in keyframes],
            options,
            already=already,
        )
        compact_pairs.extend(extra)
        print(
            f"Loop schedule: {len(extra)} CosPlace pairs "
            f"(total left–left {len(compact_pairs)})"
        )

    print(
        f"Added {len(keyframes)} keyframe captures "
        f"({2 * len(keyframes)} views; {len(skipped)} deferred for PnP). "
        f"Loading matcher={options.matcher} resize={options.resize} "
        f"({options.resize_mode}) "
        f"thresh={options.match_thresh} max_keypoints={options.max_keypoints}..."
    )
    matcher = get_matcher(
        options.matcher,
        device=options.device,
        max_num_keypoints=options.max_keypoints,
        thresh=options.match_thresh,
    )
    detect_cache = DetectOnceCache(
        matcher,
        options.resize,
        options.device,
        matcher_name=options.matcher,
        max_keypoints=options.max_keypoints,
        disk_dir=resolve_feature_cache_dir(
            options, list(left_paths) + list(right_paths)
        ),
        resize_mode=options.resize_mode,
    )
    use_detect_once = bool(options.detect_once) and detect_cache.supported
    if not detect_cache.supported:
        raise ValueError(
            f"Matcher {options.matcher!r} did not expose an extractor+LightGlue head. "
            f"Choose one of: {', '.join(LIGHTGLUE_MATCHERS)}"
        )
    img_cache: dict = {}
    counts = {
        "stereo": 0,
        "temporal": 0,
        "loop": 0,
        "cascade": 0,
        "cross": 0,
        "stereo_inliers": 0,
        "stereo_snapped": 0,
        "motion_inliers": 0,
        "pruned_tracks": 0,
        "pnp_localized": 0,
        "pnp_failed": 0,
        "pnp_correspondences": 0,
    }

    kf_paths = [left_paths[i] for i in keyframes] + [
        right_paths[i] for i in keyframes
    ]
    if use_detect_once:
        detect_cache.extract_paths(kf_paths)
        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass

    match_pair_cache_dir = resolve_match_cache_dir(
        options, list(left_paths) + list(right_paths)
    )
    match_pair_cache = None
    if match_pair_cache_dir:
        os.makedirs(match_pair_cache_dir, exist_ok=True)
        match_pair_cache = MatchPairDiskCache(
            match_pair_cache_dir,
            options.matcher,
            options.resize,
            options.max_keypoints,
            options.resize_mode,
        )
        print(f"Match pair cache: {match_pair_cache_dir}")

    def load_pair(path):
        ap = os.path.abspath(path)
        if ap in img_cache:
            return img_cache[ap]
        img = cv2.imread(ap)
        h, w = img.shape[:2]
        resize_arg = matcher_resize_arg(
            options.resize, options.resize_mode, w, h
        )
        tensor = matcher.load_image(ap, resize=resize_arg)
        mh, mw = _tensor_hw(tensor, (h, w))
        out = (tensor, (w, h), (mh, mw))
        img_cache[ap] = out
        return out

    def match_pair(path_a, path_b, min_n):
        if use_detect_once:
            ap_a = os.path.abspath(path_a)
            ap_b = os.path.abspath(path_b)
            a_rec = detect_cache._cache.get(ap_a)
            b_rec = detect_cache._cache.get(ap_b)
            if a_rec is None or b_rec is None:
                raise RuntimeError(
                    f"detect-once cache missing features for {path_a!r} or {path_b!r}"
                )
            full_a, full_b = a_rec["full_wh"], b_rec["full_wh"]
            hw_a, hw_b = a_rec["full_hw"], b_rec["full_hw"]
            cached = None
            if match_pair_cache is not None:
                cached = match_pair_cache.try_load(path_a, path_b)
            if cached is not None:
                mk0, mk1 = cached
                result = {"matched_kpts0": mk0, "matched_kpts1": mk1}
                ok_cors, cors = correspondences_from_result(
                    pt, result, full_a, full_b, hw_a, hw_b, min_n
                )
                return ok_cors, cors
            ok, result, full_a, full_b, hw_a, hw_b = detect_cache.match(path_a, path_b)
            if not ok:
                return False, []
            if match_pair_cache is not None:
                mk0 = result.get("matched_kpts0")
                mk1 = result.get("matched_kpts1")
                if mk0 is not None and mk1 is not None:
                    match_pair_cache.save(path_a, path_b, mk0, mk1)
            return correspondences_from_result(
                pt, result, full_a, full_b, hw_a, hw_b, min_n
            )
        ta, full_a, matched_a = load_pair(path_a)
        tb, full_b, matched_b = load_pair(path_b)
        result = matcher(ta, tb)
        return correspondences_from_result(
            pt, result, full_a, full_b, matched_a, matched_b, min_n
        )

    def log(msg: str) -> None:
        if options.verbose_matches:
            print(msg)

    feature_index = {
        vid: _FeatureIndex(options.stereo_snap_pixels)
        for vid in list(left_views) + list(right_views)
        if vid is not None
    }
    if use_detect_once:
        for i in keyframes:
            for path, vid in (
                (left_paths[i], left_views[i]),
                (right_paths[i], right_views[i]),
            ):
                kpts = detect_cache.kpts_fullres(path)
                if kpts is None:
                    continue
                feature_index[vid].seed_from_kpts(kpts, pt)
        print(
            f"Seeded per-view feature indices from detect-once keypoints "
            f"({len(keyframes)} keyframes)."
        )

    def add_snapped_tracks(view_a, view_b, cors, inlier_idx, index_a=None, index_b=None):
        """Snap matcher kpts onto per-view features so tracks chain."""
        ia = index_a if index_a is not None else feature_index[view_a]
        ib = index_b if index_b is not None else feature_index[view_b]
        used_a: set[int] = set()
        used_b: set[int] = set()
        n_added = 0
        for k in inlier_idx:
            c = cors[k]
            fa = ia.snap_or_add(_feature_xy(c.feature1), pt)
            fb = ib.snap_or_add(_feature_xy(c.feature2), pt)
            ida, idb = id(fa), id(fb)
            if ida in used_a or idb in used_b:
                continue
            used_a.add(ida)
            used_b.add(idb)
            track_builder.AddFeatureCorrespondence(view_a, fa, view_b, fb)
            n_added += 1
        return n_added

    def match_motion_edge(
        path_a,
        path_b,
        view_a,
        view_b,
        label: str,
        *,
        frame_a: int | None = None,
        frame_b: int | None = None,
        dump_left: bool = False,
    ) -> bool:
        min_n = 4 if dump_left else options.min_matches
        ok, cors = match_pair(path_a, path_b, min_n)
        eval_stats = _MotionMatchEval(n_matched=len(cors) if ok else 0)
        inlier_idx: list = []
        if ok:
            opts = pt.sfm.EstimateTwoViewInfoOptions()
            opts.ransac_type = pt.sfm.RansacType(0)
            opts.use_lo = True
            opts.use_mle = True
            opts.max_sampson_error_pixels = float(options.max_sampson_error)
            ok2, twoview_info, inlier_idx = pt.sfm.EstimateTwoViewInfo(
                opts, prior, prior, cors
            )
            eval_stats.n_ransac_inliers = len(inlier_idx)
            eval_stats.ransac_ok = bool(ok2)
            if ok2 and len(inlier_idx) >= options.min_matches:
                n_added = add_snapped_tracks(view_a, view_b, cors, inlier_idx)
                eval_stats.n_snapped = n_added
                twoview_info.num_verified_matches = n_added
                view_graph.AddEdge(view_a, view_b, twoview_info)
                counts["motion_inliers"] += n_added
                eval_stats.edge_added = True
                log(f"  {label}: {len(inlier_idx)} RANSAC inliers, {n_added} snapped")
        if dump_left and options.match_debug_dir and ok:
            dump_left_match_debug(
                options.match_debug_dir,
                path_a,
                path_b,
                int(frame_a if frame_a is not None else -1),
                int(frame_b if frame_b is not None else -1),
                "temporal" if options.left_match_mode == "window" else "motion",
                cors,
                inlier_idx,
                eval_stats,
            )
        return eval_stats.edge_added

    recon_rig = recon.GetCameraRig(rig_id)
    E_lr = pt.sfm.EssentialMatrixFromRigSensors(
        recon_rig.GetSensor(left_id), recon_rig.GetSensor(right_id)
    )

    def match_guided_stereo(i: int, left_idx: _FeatureIndex) -> bool:
        if len(left_idx) == 0:
            return False
        ok, cors = match_pair(
            left_paths[i], right_paths[i], max(8, options.min_matches // 4)
        )
        view_a = left_views[i]
        view_b = right_views[i]
        cam_a = recon.View(view_a).Camera()
        cam_b = recon.View(view_b).Camera()
        inlier_idx = pt.sfm.FilterCorrespondencesWithEssential(
            E_lr, cam_a, cam_b, cors, float(options.max_sampson_error)
        )
        if not inlier_idx:
            return False
        used_a: set[int] = set()
        used_b: set[int] = set()
        n_snap = 0
        right_idx = feature_index[view_b]
        for k in inlier_idx:
            c = cors[k]
            fa = left_idx.snap(_feature_xy(c.feature1))
            if fa is None:
                continue
            fb = right_idx.snap_or_add(_feature_xy(c.feature2), pt)
            ida, idb = id(fa), id(fb)
            if ida in used_a or idb in used_b:
                continue
            used_a.add(ida)
            used_b.add(idb)
            track_builder.AddFeatureCorrespondence(view_a, fa, view_b, fb)
            n_snap += 1
        if n_snap == 0:
            return False
        counts["stereo"] += 1
        counts["stereo_inliers"] += len(inlier_idx)
        counts["stereo_snapped"] += n_snap
        log(f"  stereo t={i}: {len(inlier_idx)} E-inliers, {n_snap} on long tracks")
        return True

    def match_stereo_all(i: int) -> bool:
        ok, cors = match_pair(left_paths[i], right_paths[i], options.min_matches)
        if not ok:
            return False
        view_a = left_views[i]
        view_b = right_views[i]
        inlier_idx = pt.sfm.FilterCorrespondencesWithEssential(
            E_lr,
            recon.View(view_a).Camera(),
            recon.View(view_b).Camera(),
            cors,
            float(options.max_sampson_error),
        )
        if len(inlier_idx) < options.min_matches:
            return False
        n_added = add_snapped_tracks(view_a, view_b, cors, inlier_idx)
        counts["stereo"] += 1
        counts["stereo_inliers"] += n_added
        log(f"  stereo t={i}: {n_added} inliers (unguided)")
        return True

    matched_pairs = set()
    debug_written = 0
    debug_max = int(options.match_debug_max_pairs)
    if options.match_debug_dir:
        os.makedirs(options.match_debug_dir, exist_ok=True)
        jsonl = os.path.join(options.match_debug_dir, "left_matches.jsonl")
        if os.path.isfile(jsonl):
            os.remove(jsonl)
        counts["match_debug_dir"] = os.path.abspath(options.match_debug_dir)

    def maybe_motion(
        path_a,
        path_b,
        view_a,
        view_b,
        label: str,
        kind: str,
        *,
        dump_left: bool = False,
        frame_a: int | None = None,
        frame_b: int | None = None,
    ):
        key = (min(view_a, view_b), max(view_a, view_b))
        if key in matched_pairs:
            return
        if match_motion_edge(
            path_a,
            path_b,
            view_a,
            view_b,
            label,
            dump_left=dump_left,
            frame_a=frame_a,
            frame_b=frame_b,
        ):
            matched_pairs.add(key)
            counts[kind] += 1

    motion_pairs = [(keyframes[a], keyframes[b], kind) for a, b, kind in compact_pairs]
    n_motion = len(motion_pairs)
    gap = int(options.left_match_max_gap)
    exhaustive = options.left_match_mode == "cascade" and (
        gap <= 0 or gap >= len(keyframes) - 1
    )
    n_loop = sum(1 for _, _, kind in compact_pairs if kind == "loop")
    print(
        f"Matching {n_motion} left–left pairs among {len(keyframes)} keyframes "
        f"(mode={options.left_match_mode}, max_gap={gap or 'all'}, "
        f"loops={n_loop} trajectory_has_loops={bool(options.trajectory_has_loops)}, "
        f"right_temporal={bool(options.match_right_temporal)}, "
        f"guided_stereo={bool(options.guided_stereo)}, "
        f"detect_once={use_detect_once}"
        + (f"/{detect_cache.backend}" if use_detect_once else "")
        + ")"
        + (
            "  [O(n^2); set --left_match_max_gap or --recon_stride for long sequences]"
            if exhaustive and len(keyframes) > 40
            else ""
        )
    )

    for p_i, (i, j, kind) in enumerate(motion_pairs, 1):
        dump_left = bool(options.match_debug_dir) and (
            debug_max <= 0 or debug_written < debug_max
        )
        maybe_motion(
            left_paths[i],
            left_paths[j],
            left_views[i],
            left_views[j],
            f"left {i}-{j}",
            kind,
            dump_left=dump_left,
            frame_a=i,
            frame_b=j,
        )
        if dump_left:
            debug_written += 1
        if options.match_right_temporal:
            maybe_motion(
                right_paths[i],
                right_paths[j],
                right_views[i],
                right_views[j],
                f"right {i}-{j}",
                kind,
            )
        if options.cross_sensor_temporal:
            maybe_motion(
                left_paths[i],
                right_paths[j],
                left_views[i],
                right_views[j],
                f"L{i}-R{j}",
                "cross",
            )
            maybe_motion(
                right_paths[i],
                left_paths[j],
                right_views[i],
                left_views[j],
                f"R{i}-L{j}",
                "cross",
            )
        if p_i == 1 or p_i == n_motion or p_i % 25 == 0:
            print(f"  motion matching {p_i}/{n_motion}")

    track_builder.BuildTracksIncremental(recon)
    print(
        f"After left motion: ViewGraph edges={view_graph.NumEdges()}, "
        f"tracks={recon.NumTracks()}  "
        f"pairs cascade={counts['cascade']} temporal={counts['temporal']} "
        f"loop={counts['loop']} cross={counts['cross']}  "
        f"motion_inliers={counts['motion_inliers']}"
    )
    if options.match_debug_dir:
        counts["debug_pairs_written"] = debug_written
        print(
            f"Match debug: wrote {debug_written} left–left panels under "
            f"{options.match_debug_dir}\n"
            f"  {os.path.join(options.match_debug_dir, 'left_matches.jsonl')}"
        )
    if options.match_debug_only:
        release_torch_cuda(matcher, detect_cache, img_cache)
        summary = type(
            "MatchDebugSummary",
            (),
            {
                "success": False,
                "message": "match_debug_only (skipped stereo attach and SfM)",
                "estimated_views": [],
                "estimated_tracks": [],
            },
        )()
        return StereoRigRunResult(
            recon=recon,
            summary=summary,
            view_graph=view_graph,
            left_view_ids=left_views,
            right_view_ids=right_views,
            capture_ids=capture_ids,
            left_sensor_id=left_id,
            right_sensor_id=right_id,
            match_counts=counts,
        )

    min_views = max(int(options.min_track_length), 2)
    min_caps = max(int(options.min_track_captures), 1)
    n_kf = len(keyframes)
    if options.guided_stereo:
        print(f"Guided stereo onto tracks with ≥{min_views} views / ≥{min_caps} captures...")
        for k, i in enumerate(keyframes, 1):
            left_long = long_track_feature_index(
                recon, left_views[i], feature_index[left_views[i]], min_views, min_caps
            )
            match_guided_stereo(i, left_long)
            if k == 1 or k == n_kf or k % 20 == 0:
                print(f"  guided stereo {k}/{n_kf}")
        track_builder.BuildTracksIncremental(recon)
    else:
        print("Unguided stereo (all E-inliers, still snapped to left features)...")
        for i in keyframes:
            match_stereo_all(i)
        track_builder.BuildTracksIncremental(recon)

    counts["pruned_tracks"] = prune_tracks_by_capture_count(recon, min_caps)
    print(
        f"ViewGraph edges={view_graph.NumEdges()}, "
        f"tracks={recon.NumTracks()} (pruned {counts['pruned_tracks']} "
        f"with <{min_caps} captures), views={recon.NumViews()}  "
        f"stereo_pairs={counts['stereo']} snapped={counts['stereo_snapped']} "
        f"E-inliers={counts['stereo_inliers']}"
    )
    if match_pair_cache is not None:
        counts["match_cache_hits"] = match_pair_cache.disk_hits
        counts["match_cache_writes"] = match_pair_cache.disk_misses
        print(
            f"Match cache: {match_pair_cache.disk_hits} loaded, "
            f"{match_pair_cache.disk_misses} written under {match_pair_cache_dir}"
        )

    if options.match_cache_only:
        release_torch_cuda(matcher, detect_cache, img_cache)
        summary = type(
            "MatchCacheSummary",
            (),
            {
                "success": False,
                "message": "match_cache_only (skipped SfM/BA)",
                "estimated_views": [],
                "estimated_tracks": [],
            },
        )()
        return StereoRigRunResult(
            recon=recon,
            summary=summary,
            view_graph=view_graph,
            left_view_ids=left_views,
            right_view_ids=right_views,
            capture_ids=capture_ids,
            left_sensor_id=left_id,
            right_sensor_id=right_id,
            match_counts=counts,
        )

    feature_disk_dir = detect_cache.disk_dir
    print("Releasing CUDA matcher before bundle adjustment...")
    release_torch_cuda(matcher, detect_cache, img_cache)
    matcher = None
    detect_cache = None

    ba_loss_type = ba_loss_function_type(pt, options.ba_loss)
    ba_width = float(options.ba_robust_width)

    if options.method == "global":
        gro = pt.sfm.GlobalRigReconstructorOptions()
        gro.sfm_options.global_rotation_estimator_type = getattr(
            pt.sfm.GlobalRotationEstimatorType, options.rotation_estimator
        )
        gro.sfm_options.global_position_estimator_type = getattr(
            pt.sfm.GlobalPositionEstimatorType, options.position_estimator
        )
        gro.sfm_options.min_triangulation_angle_degrees = (
            options.min_triangulation_angle_degrees
        )
        gro.sfm_options.triangulation_max_reprojection_error_in_pixels = (
            options.max_reprojection_error_in_pixels
        )
        gro.sfm_options.num_retriangulation_iterations = (
            options.retriangulation_iterations
        )
        gro.sfm_options.num_threads = 1
        gro.sfm_options.bundle_adjustment_loss_function_type = ba_loss_type
        gro.sfm_options.bundle_adjustment_robust_loss_width = ba_width
        gro.rescale_positions_to_metric_edges = True
        gro.capture_graph_options.metric_only_for_viewgraph_pairs = True
        gro.capture_graph_options.skip_metric_if_baseline_degenerate = True
        summary = pt.sfm.GlobalRigReconstructor(gro).Estimate(view_graph, recon)
    else:
        iro = pt.sfm.IncrementalRigReconstructorOptions()
        iro.min_triangulation_angle_degrees = options.min_triangulation_angle_degrees
        iro.max_reprojection_error_in_pixels = options.max_reprojection_error_in_pixels
        iro.bundle_adjust_every_n_captures = 5
        iro.partial_bundle_adjustment_num_captures = 12
        iro.ba_options.num_threads = 1
        iro.ba_options.use_inner_iterations = False
        iro.ba_options.use_rig_constraints = True
        iro.ba_options.loss_function_type = ba_loss_type
        iro.ba_options.robust_loss_width = ba_width
        summary = pt.sfm.IncrementalRigReconstructor(iro).Estimate(view_graph, recon)

    print(
        f"success={summary.success} views={len(summary.estimated_views)} "
        f"tracks={len(summary.estimated_tracks)} msg={summary.message}"
    )

    if skipped:
        for i in skipped:
            cap, vl, vr = add_calibrated_stereo_capture(
                pt,
                recon,
                rig_id,
                left_id,
                right_id,
                i,
                left_paths[i],
                right_paths[i],
                prior,
            )
            capture_ids[i] = cap
            left_views[i] = vl
            right_views[i] = vr

    if summary.success and skipped:
        print("Reloading matcher for PnP fill of skipped captures...")
        matcher = get_matcher(
            options.matcher,
            device=options.device,
            max_num_keypoints=options.max_keypoints,
            thresh=options.match_thresh,
        )
        detect_cache = DetectOnceCache(
            matcher,
            options.resize,
            options.device,
            matcher_name=options.matcher,
            max_keypoints=options.max_keypoints,
            disk_dir=feature_disk_dir,
            resize_mode=options.resize_mode,
        )
        if use_detect_once:
            pnp_paths = [left_paths[i] for i in keyframes] + [
                left_paths[i] for i in skipped
            ]
            detect_cache.extract_paths(pnp_paths)
        pnp_stats = localize_skipped_rig_captures(
            pt,
            recon,
            skipped=skipped,
            keyframes=keyframes,
            left_paths=left_paths,
            left_views=left_views,
            capture_ids=capture_ids,
            left_sensor_id=left_id,
            rig_id=rig_id,
            match_pair=match_pair,
            options=options,
        )
        counts.update(pnp_stats)

    release_torch_cuda(matcher, detect_cache, img_cache)
    matcher = None
    detect_cache = None

    print_track_length_stats(recon)
    if summary.success:
        print_reprojection_stats(recon)
        bl = mean_stereo_baseline(recon, left_views, right_views)
        print(
            f"Mean reconstructed stereo baseline: {bl:.4f} m  "
            f"(calib {calib.baseline:.4f} m)"
        )

    return StereoRigRunResult(
        recon=recon,
        summary=summary,
        view_graph=view_graph,
        left_view_ids=left_views,
        right_view_ids=right_views,
        capture_ids=capture_ids,
        left_sensor_id=left_id,
        right_sensor_id=right_id,
        match_counts=counts,
    )
