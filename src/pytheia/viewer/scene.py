"""Convert a :class:`pytheia.sfm.Reconstruction` into viewer-ready numpy arrays.

The scene is a plain ``dict`` of numpy arrays (see :func:`recon_to_scene`).
:func:`scene_to_payload` turns it into a JSON-serialisable dict in which every
array is a base64 encoded little-endian buffer (``float32`` or ``uint8``), which
is what the HTML viewer embeds.

Conventions
-----------
* Coordinates are the reconstruction's world frame, unchanged.
* Camera ``rotation_w2c`` is the world-to-camera rotation returned by
  ``Camera.GetOrientationAsRotationMatrix()`` and ``center`` is
  ``Camera.GetPosition()``. The camera looks along +z of its frame, with x right
  and y down (OpenCV convention), which is the convention used by
  ``Camera.ProjectPoint``.
* Points are the estimated tracks with a finite point. The reprojection error of
  a point is the mean pixel residual over its observations in estimated views.
* Observations are grouped by point (``observations`` sorted by point index);
  ``points.obs_offsets`` holds the start of each point's block (length N+1).
"""

from __future__ import annotations

import base64
import math
import os
from typing import Dict, List, Optional

import numpy as np

import pytheia as pt
from pytheia.viewer.compare import align_scene

SCENE_FORMAT = "pytheia-scene"
SCENE_VERSION = 1

_IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".webp", ".gif", ".bmp")
_MIME = {
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".png": "image/png",
    ".webp": "image/webp",
    ".gif": "image/gif",
    ".bmp": "image/bmp",
}


def _as_float_list(values) -> List[float]:
    return [float(v) for v in values]


def _find_thumbnail(thumbnails_dir: str, view_name: str) -> Optional[str]:
    """Return the image path for ``view_name`` inside ``thumbnails_dir``, or None.

    Tries the exact relative path first, then the basename with the same stem
    (any extension in ``_IMAGE_EXTS``).
    """
    exact = os.path.join(thumbnails_dir, view_name)
    if os.path.isfile(exact):
        return exact
    base = os.path.basename(view_name)
    stem = os.path.splitext(base)[0]
    for ext in _IMAGE_EXTS:
        cand = os.path.join(thumbnails_dir, stem + ext)
        if os.path.isfile(cand):
            return cand
        cand = os.path.join(thumbnails_dir, stem + ext.upper())
        if os.path.isfile(cand):
            return cand
    return None


def _thumbnail_data_uri(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    mime = _MIME.get(ext, "application/octet-stream")
    with open(path, "rb") as fh:
        data = base64.b64encode(fh.read()).decode("ascii")
    return "data:{};base64,{}".format(mime, data)


def _camera_pose_and_intrinsics(cam):
    """Return (center, R_w2c, fx, fy, cx, cy, width, height) for a pytheia Camera."""
    center = np.asarray(cam.GetPosition(), dtype=np.float64).reshape(3)
    rot = np.asarray(cam.GetOrientationAsRotationMatrix(), dtype=np.float64).reshape(3, 3)
    k = np.asarray(cam.GetCalibrationMatrix(), dtype=np.float64).reshape(3, 3)
    width = float(cam.ImageWidth())
    height = float(cam.ImageHeight())
    return center, rot, float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]), width, height


def _frustum_corners(center, rot_w2c, fx, fy, cx, cy, width, height, depth):
    """World-space corners of the image plane at ``depth`` (TL, TR, BR, BL)."""
    if width <= 0 or height <= 0:
        width, height = 2.0 * cx, 2.0 * cy
    pix = np.array(
        [[0.0, 0.0], [width, 0.0], [width, height], [0.0, height]], dtype=np.float64
    )
    cam_pts = np.empty((4, 3), dtype=np.float64)
    cam_pts[:, 0] = (pix[:, 0] - cx) / fx * depth
    cam_pts[:, 1] = (pix[:, 1] - cy) / fy * depth
    cam_pts[:, 2] = depth
    # x_cam = R (X - C)  =>  X = C + R^T x_cam
    return center[None, :] + cam_pts @ rot_w2c


def _homogeneous_to_xyz(p) -> Optional[np.ndarray]:
    p = np.asarray(p, dtype=np.float64).reshape(-1)
    if p.size == 3:
        xyz = p
    elif p.size == 4:
        if abs(p[3]) < 1e-12:
            return None
        xyz = p[:3] / p[3]
    else:
        return None
    if not np.all(np.isfinite(xyz)):
        return None
    return xyz


def recon_to_scene(
    reconstruction,
    max_points=None,
    thumbnails_dir=None,
    include_view_graph=None,
    *,
    frustum_scale=None,
    seed=0,
    compare_with=None,
):
    """Collect the data needed by the viewer from a reconstruction.

    Args:
        reconstruction: A ``pytheia.sfm.Reconstruction``.
        max_points: Keep at most this many estimated points (random subset with
            a fixed ``seed``). ``None`` keeps all of them. Observations and the
            per-view statistics refer to the kept points only.
        thumbnails_dir: Directory with one image per view. Images are matched by
            view name (relative path first, then file stem). They are embedded
            as base64 data URIs, so pre-resize them (e.g. to 256 px) to keep the
            HTML small. ``None`` embeds no thumbnails.
        include_view_graph: If true, add covisibility edges between estimated
            views: ``weight`` is the number of kept points seen by both views.
            ``None`` and ``False`` add no edges. pyTheia does not keep the
            two-view view graph in the reconstruction, so edges come from
            shared tracks.
        frustum_scale: Depth of the drawn image planes in world units. ``None``
            uses 5 % of the bounding-box diagonal of the kept points.
        seed: Seed for the random point subset when ``max_points`` is set.
        compare_with: A second reconstruction. It is aligned onto
            ``reconstruction`` with a Sim(3) fit on the camera centers of the
            views registered in both under the same name (see
            :mod:`pytheia.viewer.compare`). The result is stored under
            ``scene["compare"]``; it is ``None`` when no alignment was done
            (no ``compare_with``, or fewer than 3 shared views, which is
            reported with a warning).

    Returns:
        dict with the keys ``format``, ``version``, ``stats``, ``bounds``,
        ``points`` (dict of arrays), ``observations`` (dict of arrays),
        ``cameras`` (dict with arrays and a ``names`` list), ``edges``
        (dict or None) and ``compare`` (dict or None). See the module docstring
        for conventions.
    """
    if max_points is not None and int(max_points) < 0:
        raise ValueError("max_points must be None or >= 0")

    view_ids = sorted(int(v) for v in reconstruction.ViewIds())
    cam_of_view = {vid: i for i, vid in enumerate(view_ids)}
    views = {vid: reconstruction.View(vid) for vid in view_ids}
    cams = {vid: views[vid].Camera() for vid in view_ids}
    estimated_view = {vid: bool(views[vid].IsEstimated()) for vid in view_ids}

    # ---- cameras -----------------------------------------------------------
    num_views = len(view_ids)
    center = np.zeros((num_views, 3), dtype=np.float32)
    rot_w2c = np.zeros((num_views, 3, 3), dtype=np.float32)
    intrinsics = np.zeros((num_views, 4), dtype=np.float32)
    image_size = np.zeros((num_views, 2), dtype=np.float32)
    estimated = np.zeros((num_views,), dtype=np.uint8)
    names: List[str] = []
    pose_cache = {}
    for i, vid in enumerate(view_ids):
        c, r, fx, fy, cx, cy, w, h = _camera_pose_and_intrinsics(cams[vid])
        pose_cache[vid] = (c, r, fx, fy, cx, cy, w, h)
        center[i] = c
        rot_w2c[i] = r
        intrinsics[i] = (fx, fy, cx, cy)
        image_size[i] = (w, h)
        estimated[i] = 1 if estimated_view[vid] else 0
        names.append(str(views[vid].Name()))

    # ---- points --------------------------------------------------------------
    candidate_tracks = []
    for tid in sorted(int(t) for t in reconstruction.TrackIds()):
        track = reconstruction.Track(tid)
        if not track.IsEstimated():
            continue
        if not any(estimated_view.get(int(v), False) for v in track.ViewIds()):
            continue
        if _homogeneous_to_xyz(track.Point()) is None:
            continue
        candidate_tracks.append(tid)
    num_tracks_total = len(candidate_tracks)

    if max_points is not None and len(candidate_tracks) > int(max_points):
        rng = np.random.default_rng(seed)
        chosen = rng.choice(len(candidate_tracks), size=int(max_points), replace=False)
        candidate_tracks = [candidate_tracks[i] for i in sorted(chosen.tolist())]

    xyz_list = []
    rgb_list = []
    length_list = []
    error_list = []
    track_id_list = []
    obs_offsets = [0]
    obs_view: List[int] = []
    obs_x: List[float] = []
    obs_y: List[float] = []
    obs_res: List[float] = []
    per_view_obs = np.zeros((num_views,), dtype=np.int64)
    per_view_err_sum = np.zeros((num_views,), dtype=np.float64)
    edge_counts: Dict[tuple, int] = {}

    for tid in candidate_tracks:
        track = reconstruction.Track(tid)
        p4 = np.asarray(track.Point(), dtype=np.float64).reshape(-1)
        if p4.size == 3:
            p4 = np.append(p4, 1.0)
        xyz = _homogeneous_to_xyz(p4)
        color = np.asarray(track.Color(), dtype=np.uint8).reshape(-1)
        if color.size != 3:
            color = np.zeros((3,), dtype=np.uint8)

        residuals = []
        track_views = []
        for vid in sorted(int(v) for v in track.ViewIds()):
            if not estimated_view.get(vid, False):
                continue
            try:
                feat = views[vid].GetFeature(tid)
            except Exception:  # view/track bookkeeping mismatch: skip the observation
                continue
            depth, proj = cams[vid].ProjectPoint(p4)
            if not (depth > 0.0) or not np.all(np.isfinite(proj)):
                continue
            u, v_ = float(feat.x()), float(feat.y())
            res = math.hypot(float(proj[0]) - u, float(proj[1]) - v_)
            if not math.isfinite(res):
                continue
            ci = cam_of_view[vid]
            obs_view.append(ci)
            obs_x.append(u)
            obs_y.append(v_)
            obs_res.append(res)
            residuals.append(res)
            track_views.append(ci)
            per_view_obs[ci] += 1
            per_view_err_sum[ci] += res

        xyz_list.append(xyz)
        rgb_list.append(color)
        length_list.append(float(track.NumViews()))
        error_list.append(float(np.mean(residuals)) if residuals else float("nan"))
        track_id_list.append(float(tid))
        obs_offsets.append(len(obs_view))

        if include_view_graph:
            uniq = sorted(set(track_views))
            for a in range(len(uniq)):
                for b in range(a + 1, len(uniq)):
                    key = (uniq[a], uniq[b])
                    edge_counts[key] = edge_counts.get(key, 0) + 1

    num_points = len(xyz_list)
    if num_points:
        points_xyz = np.asarray(xyz_list, dtype=np.float32).reshape(num_points, 3)
        points_rgb = np.asarray(rgb_list, dtype=np.uint8).reshape(num_points, 3)
    else:
        points_xyz = np.zeros((0, 3), dtype=np.float32)
        points_rgb = np.zeros((0, 3), dtype=np.uint8)

    points = {
        "xyz": points_xyz,
        "rgb": points_rgb,
        "track_length": np.asarray(length_list, dtype=np.float32),
        "reprojection_error": np.asarray(error_list, dtype=np.float32),
        "track_id": np.asarray(track_id_list, dtype=np.float32),
        "obs_offsets": np.asarray(obs_offsets, dtype=np.float32),
    }

    observations = {
        "view": np.asarray(obs_view, dtype=np.float32),
        "x": np.asarray(obs_x, dtype=np.float32),
        "y": np.asarray(obs_y, dtype=np.float32),
        "residual": np.asarray(obs_res, dtype=np.float32),
    }

    # ---- per-camera stats (over kept points) --------------------------------
    num_obs = per_view_obs.astype(np.float32)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_err = np.where(
            per_view_obs > 0, per_view_err_sum / np.maximum(per_view_obs, 1), np.nan
        ).astype(np.float32)

    # ---- frustum corners ----------------------------------------------------
    if points_xyz.size:
        lo = points_xyz.min(axis=0)
        hi = points_xyz.max(axis=0)
    elif num_views:
        lo = center.min(axis=0)
        hi = center.max(axis=0)
    else:
        lo = np.zeros(3, dtype=np.float32)
        hi = np.zeros(3, dtype=np.float32)
    diag = float(np.linalg.norm(hi.astype(np.float64) - lo.astype(np.float64)))
    if frustum_scale is None:
        depth = 0.05 * diag if diag > 0 else 1.0
    else:
        depth = float(frustum_scale)
        if not depth > 0:
            raise ValueError("frustum_scale must be positive")
    frustum = np.zeros((num_views, 4, 3), dtype=np.float32)
    for i, vid in enumerate(view_ids):
        c, r, fx, fy, cx, cy, w, h = pose_cache[vid]
        frustum[i] = _frustum_corners(c, r, fx, fy, cx, cy, w, h, depth)

    cameras = {
        "names": names,
        "view_id": np.asarray(view_ids, dtype=np.float32),
        "estimated": estimated,
        "center": center,
        "rotation_w2c": rot_w2c,
        "intrinsics": intrinsics,
        "image_size": image_size,
        "frustum": frustum,
        "num_observations": num_obs,
        "mean_reprojection_error": mean_err,
        "thumbnails": [None] * num_views,
    }

    if thumbnails_dir is not None:
        if not os.path.isdir(thumbnails_dir):
            raise FileNotFoundError("thumbnails_dir does not exist: {}".format(thumbnails_dir))
        for i, name in enumerate(names):
            path = _find_thumbnail(thumbnails_dir, name)
            if path is not None:
                cameras["thumbnails"][i] = _thumbnail_data_uri(path)

    edges = None
    if include_view_graph:
        pairs = sorted(edge_counts.items())
        edges = {
            "pairs": np.asarray([k for k, _ in pairs], dtype=np.float32).reshape(-1, 2),
            "weight": np.asarray([w for _, w in pairs], dtype=np.float32),
        }

    bounds_min = _as_float_list(lo)
    bounds_max = _as_float_list(hi)

    valid_err = points["reprojection_error"][np.isfinite(points["reprojection_error"])]
    stats = {
        "num_views": int(num_views),
        "num_estimated_views": int(estimated.sum()),
        "num_tracks_total": int(num_tracks_total),
        "num_points": int(num_points),
        "num_observations": int(len(obs_view)),
        "mean_reprojection_error": float(valid_err.mean()) if valid_err.size else float("nan"),
        "max_points": None if max_points is None else int(max_points),
    }

    scene = {
        "format": SCENE_FORMAT,
        "version": SCENE_VERSION,
        "stats": stats,
        "bounds": {"min": bounds_min, "max": bounds_max, "frustum_scale": depth},
        "points": points,
        "observations": observations,
        "cameras": cameras,
        "edges": edges,
        "compare": None,
    }
    if compare_with is not None:
        other = recon_to_scene(
            compare_with,
            max_points=max_points,
            thumbnails_dir=thumbnails_dir,
            include_view_graph=include_view_graph,
            frustum_scale=frustum_scale,
            seed=seed,
        )
        scene["compare"] = align_scene(scene, other)
    return scene


def _encode_array(arr: np.ndarray) -> Dict:
    arr = np.asarray(arr)
    if arr.dtype == np.uint8:
        dtype = "uint8"
        raw = np.ascontiguousarray(arr, dtype="u1").tobytes()
    else:
        dtype = "float32"
        raw = np.ascontiguousarray(arr, dtype="<f4").tobytes()
    return {
        "dtype": dtype,
        "shape": [int(s) for s in arr.shape],
        "data": base64.b64encode(raw).decode("ascii"),
    }


def scene_to_payload(scene: Dict) -> Dict:
    """Encode a scene from :func:`recon_to_scene` as JSON-serialisable data.

    Every numeric array becomes ``{"dtype", "shape", "data"}`` where ``data`` is
    the base64 of the little-endian buffer. NaN values are stored as-is in the
    float32 buffers.
    """
    cams = scene["cameras"]
    payload = {
        "format": scene["format"],
        "version": scene["version"],
        "stats": scene["stats"],
        "bounds": scene["bounds"],
        "points": {k: _encode_array(v) for k, v in scene["points"].items()},
        "observations": {k: _encode_array(v) for k, v in scene["observations"].items()},
        "cameras": {
            "names": list(cams["names"]),
            "thumbnails": list(cams["thumbnails"]),
            **{
                k: _encode_array(v)
                for k, v in cams.items()
                if k not in ("names", "thumbnails")
            },
        },
        "edges": None,
    }
    if scene["edges"] is not None:
        payload["edges"] = {k: _encode_array(v) for k, v in scene["edges"].items()}
    payload["compare"] = _encode_compare(scene.get("compare"))
    return payload


def _encode_compare(cmp: Optional[Dict]) -> Optional[Dict]:
    """Payload of the aligned second reconstruction and the per-camera errors."""
    if not cmp:
        return None
    sim3 = cmp["sim3"]
    return {
        "scene": scene_to_payload(cmp["scene"]),
        "sim3": {
            "scale": float(sim3["scale"]),
            "rotation": np.asarray(sim3["rotation"], dtype=np.float64).tolist(),
            "translation": np.asarray(sim3["translation"], dtype=np.float64).tolist(),
        },
        "num_shared": int(cmp["num_shared"]),
        "rms_center_error": float(cmp["rms_center_error"]),
        "cameras": {k: _encode_array(v) for k, v in cmp["cameras"].items()},
    }


def decode_array(desc: Dict) -> np.ndarray:
    """Inverse of the array encoding in :func:`scene_to_payload` (for tests and tools)."""
    raw = base64.b64decode(desc["data"])
    if desc["dtype"] == "uint8":
        arr = np.frombuffer(raw, dtype=np.uint8)
    elif desc["dtype"] == "float32":
        arr = np.frombuffer(raw, dtype="<f4")
    else:
        raise ValueError("unsupported dtype: {}".format(desc["dtype"]))
    return arr.reshape(desc["shape"]).copy()
