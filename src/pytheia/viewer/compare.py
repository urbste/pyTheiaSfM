"""Compare two reconstructions: Sim(3) alignment of the second onto the first.

The alignment is a least-squares similarity transform (Umeyama, 1991) fitted to
the camera centers of the views that are registered in both reconstructions and
have the same view name. ``dst ~= scale * R @ src + translation`` maps the second
reconstruction (``src``) into the frame of the first (``dst``).

Only numpy is needed. The functions here work on the dict returned by
:func:`pytheia.viewer.scene.recon_to_scene`.
"""

from __future__ import annotations

import warnings
from typing import Dict, List, Optional, Tuple

import numpy as np

MIN_SHARED_VIEWS = 3


class DegenerateAlignment(ValueError):
    """The camera centers do not determine a similarity transform."""


def umeyama_sim3(src, dst) -> Tuple[float, np.ndarray, np.ndarray]:
    """Fit ``dst ~= scale * R @ src + t`` in the least-squares sense.

    Args:
        src: (N, 3) points in the source frame, N >= 3.
        dst: (N, 3) corresponding points in the target frame.

    Returns:
        ``(scale, R, t)`` with ``R`` a proper rotation (det +1), ``t`` shape (3,).

    Raises:
        DegenerateAlignment: fewer than 3 pairs, or the source points are
            coincident or collinear (the rotation is then not determined).
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.ndim != 2 or src.shape[1] != 3 or src.shape != dst.shape:
        raise ValueError("src and dst must be (N, 3) arrays with the same shape")
    n = src.shape[0]
    if n < 3:
        raise DegenerateAlignment("at least 3 point pairs are needed, got {}".format(n))
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    xs = src - mu_s
    yd = dst - mu_d
    sv = np.linalg.svd(xs, compute_uv=False)
    if not sv[0] > 1e-12 or sv[1] <= 1e-9 * sv[0]:
        raise DegenerateAlignment("the points are coincident or collinear")
    var_s = float(np.sum(xs * xs) / n)
    cov = yd.T @ xs / n
    u, d, vt = np.linalg.svd(cov)
    sign = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sign[2, 2] = -1.0
    rot = u @ sign @ vt
    scale = float(np.trace(np.diag(d) @ sign) / var_s)
    trans = mu_d - scale * (rot @ mu_s)
    return scale, rot, trans


def transform_points(points, scale, rotation, translation) -> np.ndarray:
    """Apply ``scale * R @ p + t`` to an (N, 3) array (float64 result)."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return scale * (p @ np.asarray(rotation, dtype=np.float64).T) + np.asarray(
        translation, dtype=np.float64
    )


def shared_view_pairs(names_a, est_a, names_b, est_b) -> List[Tuple[int, int]]:
    """Return ``(index_in_a, index_in_b)`` for views registered in both with the same name."""
    index_b: Dict[str, int] = {}
    for j, name in enumerate(names_b):
        if est_b[j] and name not in index_b:
            index_b[name] = j
    pairs = []
    for i, name in enumerate(names_a):
        if est_a[i] and name in index_b:
            pairs.append((i, index_b[name]))
    return pairs


def transform_scene(scene: Dict, scale, rotation, translation) -> Dict:
    """Return a copy of ``scene`` with the similarity transform applied.

    Points, camera centers and frustum corners move with ``scale * R p + t``.
    A camera rotation ``R_w2c`` becomes ``R_w2c @ R^T``, and the frustum depth is
    multiplied by ``scale``. Observations and residuals do not change.
    """
    rot = np.asarray(rotation, dtype=np.float64)
    out = dict(scene)

    points = dict(scene["points"])
    xyz = transform_points(scene["points"]["xyz"], scale, rot, translation).astype(np.float32)
    points["xyz"] = xyz.reshape(-1, 3)

    cams = dict(scene["cameras"])
    cams["center"] = transform_points(scene["cameras"]["center"], scale, rot, translation).astype(
        np.float32
    ).reshape(-1, 3)
    w2c = np.asarray(scene["cameras"]["rotation_w2c"], dtype=np.float64)
    cams["rotation_w2c"] = (w2c @ rot.T).astype(np.float32).reshape(-1, 3, 3)
    frustum = transform_points(scene["cameras"]["frustum"], scale, rot, translation)
    cams["frustum"] = frustum.astype(np.float32).reshape(-1, 4, 3)

    if xyz.shape[0]:
        lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    elif cams["center"].shape[0]:
        lo, hi = cams["center"].min(axis=0), cams["center"].max(axis=0)
    else:
        lo = hi = np.zeros(3, dtype=np.float32)
    bounds = dict(scene["bounds"])
    bounds["min"] = [float(v) for v in lo]
    bounds["max"] = [float(v) for v in hi]
    bounds["frustum_scale"] = float(scene["bounds"]["frustum_scale"]) * float(scale)

    out["points"] = points
    out["cameras"] = cams
    out["bounds"] = bounds
    out["compare"] = None
    return out


def _rotation_angle_deg(rot_a, rot_b) -> np.ndarray:
    """Angle (degrees) of the relative rotation between stacks of (3, 3) matrices.

    Uses atan2 of the sine and cosine parts, which stays accurate for tiny angles
    where ``arccos`` of the trace loses precision.
    """
    rel = rot_a @ np.transpose(rot_b, (0, 2, 1))
    cos = 0.5 * (rel[:, 0, 0] + rel[:, 1, 1] + rel[:, 2, 2] - 1.0)
    sin = 0.5 * np.sqrt(
        (rel[:, 2, 1] - rel[:, 1, 2]) ** 2
        + (rel[:, 0, 2] - rel[:, 2, 0]) ** 2
        + (rel[:, 1, 0] - rel[:, 0, 1]) ** 2
    )
    return np.degrees(np.arctan2(sin, cos))


def align_scene(scene_a: Dict, scene_b: Dict, *, min_shared: int = MIN_SHARED_VIEWS) -> Optional[Dict]:
    """Align ``scene_b`` onto ``scene_a`` with a Sim(3) fit on shared view names.

    The fit uses the camera centers of the views that are registered in both
    scenes under the same name. If fewer than ``min_shared`` such views exist, or
    their centers are degenerate, a ``UserWarning`` is issued and ``None`` is
    returned.

    Returns:
        ``None``, or a dict with ``scene`` (``scene_b`` in the frame of
        ``scene_a``), ``sim3`` (``scale``, ``rotation``, ``translation``),
        ``num_shared``, ``rms_center_error`` (scene units) and ``cameras``: arrays
        over the cameras of ``scene_a`` with ``matched_b`` (index into ``scene_b``
        cameras, -1 if none), ``center_error`` (scene units) and
        ``rotation_error_deg``; NaN where there is no match.
    """
    cam_a = scene_a["cameras"]
    cam_b = scene_b["cameras"]
    pairs = shared_view_pairs(cam_a["names"], cam_a["estimated"], cam_b["names"], cam_b["estimated"])
    if len(pairs) < min_shared:
        warnings.warn(
            "compare_with: {} view(s) are registered in both reconstructions under the same name; "
            "at least {} are needed to align them. The second reconstruction is not shown.".format(
                len(pairs), min_shared
            ),
            UserWarning,
            stacklevel=3,
        )
        return None

    ia = np.array([p[0] for p in pairs], dtype=np.int64)
    jb = np.array([p[1] for p in pairs], dtype=np.int64)
    src = np.asarray(cam_b["center"], dtype=np.float64)[jb]
    dst = np.asarray(cam_a["center"], dtype=np.float64)[ia]
    try:
        scale, rot, trans = umeyama_sim3(src, dst)
    except DegenerateAlignment as exc:
        warnings.warn(
            "compare_with: cannot align the reconstructions ({}). "
            "The second reconstruction is not shown.".format(exc),
            UserWarning,
            stacklevel=3,
        )
        return None

    aligned = transform_scene(scene_b, scale, rot, trans)
    num_views = len(cam_a["names"])
    matched = np.full(num_views, -1.0, dtype=np.float32)
    center_err = np.full(num_views, np.nan, dtype=np.float32)
    rot_err = np.full(num_views, np.nan, dtype=np.float32)

    cen_a = np.asarray(cam_a["center"], dtype=np.float64)[ia]
    cen_b = np.asarray(aligned["cameras"]["center"], dtype=np.float64)[jb]
    rot_a = np.asarray(cam_a["rotation_w2c"], dtype=np.float64)[ia]
    rot_b = np.asarray(aligned["cameras"]["rotation_w2c"], dtype=np.float64)[jb]
    cerr = np.linalg.norm(cen_a - cen_b, axis=1)
    rerr = _rotation_angle_deg(rot_a, rot_b)

    matched[ia] = jb.astype(np.float32)
    center_err[ia] = cerr.astype(np.float32)
    rot_err[ia] = rerr.astype(np.float32)

    return {
        "scene": aligned,
        "sim3": {"scale": float(scale), "rotation": rot.tolist(), "translation": trans.tolist()},
        "num_shared": int(len(pairs)),
        "rms_center_error": float(np.sqrt(np.mean(cerr * cerr))),
        "cameras": {
            "matched_b": matched,
            "center_error": center_err,
            "rotation_error_deg": rot_err,
        },
    }
