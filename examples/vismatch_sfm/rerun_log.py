# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Log a pyTheia Reconstruction to Rerun (https://rerun.io/)."""

from __future__ import annotations

import re
from typing import Any

import numpy as np

# Scale Rerun Pinhole `image_plane_distance` (frustum depth) relative to defaults.
FRUSTUM_DEPTH_SCALE = 5.0


def extract_triangulated_points(rec) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Return (positions N×3, colors N×3 uint8) or (None, None) if no estimated tracks."""
    xs: list[np.ndarray] = []
    cs: list[np.ndarray] = []
    for tid in rec.TrackIds():
        tr = rec.Track(tid)
        if not tr.IsEstimated():
            continue
        p = np.asarray(tr.Point()).reshape(4)
        w = float(p[3]) if abs(p[3]) > 1e-12 else 1.0
        xs.append(p[:3] / w)
        c = np.asarray(tr.Color()).reshape(3)
        cs.append(c.astype(np.uint8))
    if not xs:
        return None, None
    return np.stack(xs, axis=0), np.stack(cs, axis=0)


def extract_camera_centers(rec) -> np.ndarray | None:
    cc: list[np.ndarray] = []
    for vid in rec.ViewIds():
        v = rec.View(vid)
        if not v.IsEstimated():
            continue
        cc.append(np.asarray(v.Camera().GetPosition()).reshape(3))
    if not cc:
        return None
    return np.stack(cc, axis=0)


def snapshot_estimated_cameras(rec) -> list[dict[str, Any]]:
    """
    Copy poses and intrinsics for estimated views (before e.g. Sim3 alignment).

    Each entry: name, R_wc (3×3 world→camera), C (camera center world), K (3×3), width, height.
    """
    out: list[dict[str, Any]] = []
    for vid in sorted(rec.ViewIds()):
        v = rec.View(vid)
        if not v.IsEstimated():
            continue
        cam = v.Camera()
        w, h = int(cam.ImageWidth()), int(cam.ImageHeight())
        if w <= 0 or h <= 0:
            continue
        R_wc = np.asarray(cam.GetOrientationAsRotationMatrix(), dtype=np.float64).reshape(3, 3).copy()
        C = np.asarray(cam.GetPosition(), dtype=np.float64).reshape(3).copy()
        K = np.asarray(cam.GetCalibrationMatrix(), dtype=np.float64).reshape(3, 3).copy()
        out.append(
            {
                "name": v.Name(),
                "R_wc": R_wc,
                "C": C,
                "K": K,
                "width": w,
                "height": h,
            }
        )
    return out


def _sanitize_entity_name(name: str) -> str:
    return re.sub(r"[^a-zA-Z0-9._-]+", "_", name)


def _default_image_plane_distance(K: np.ndarray, width: int, height: int) -> float:
    fx = float(max(K[0, 0], 1e-6))
    return max(0.02, 0.12 * float(max(width, height)) / fx)


def log_pinhole_camera_batch(
    rr_mod,
    cameras: list[dict[str, Any]],
    path_prefix: str,
    *,
    color: tuple[int, int, int],
    line_width: float = 0.004,
    image_plane_distance: float | None = None,
) -> None:
    """
    For each camera: child→parent transform (Theia: Xc = R_wc (Xw - C)) and Pinhole frustum.

    Rerun default transform maps entity frame → parent; we use p_world = R_wcᵀ p_cam + C.
    """
    for snap in cameras:
        name = _sanitize_entity_name(str(snap["name"]))
        ent = f"{path_prefix}/{name}"
        R_wc = np.asarray(snap["R_wc"], dtype=np.float64).reshape(3, 3)
        C = np.asarray(snap["C"], dtype=np.float64).reshape(3)
        K = np.asarray(snap["K"], dtype=np.float64).reshape(3, 3)
        w, h = int(snap["width"]), int(snap["height"])
        rr_mod.log(
            ent,
            rr_mod.Transform3D(translation=C, mat3x3=R_wc.T),
        )
        ipd = image_plane_distance if image_plane_distance is not None else _default_image_plane_distance(K, w, h)
        ipd *= FRUSTUM_DEPTH_SCALE
        rr_mod.log(
            ent,
            rr_mod.Pinhole(
                image_from_camera=K,
                resolution=[w, h],
                camera_xyz=rr_mod.ViewCoordinates.RDF,
                image_plane_distance=ipd,
                color=color,
                line_width=line_width,
            ),
        )


def log_cameras_pinhole_frustums(
    rr_mod,
    rec,
    path_prefix: str,
    *,
    color: tuple[int, int, int],
    line_width: float = 0.004,
    image_plane_distance: float | None = None,
) -> None:
    """Log estimated views as pinhole frustums under path_prefix (one entity per view)."""
    snaps = snapshot_estimated_cameras(rec)
    if not snaps:
        return
    log_pinhole_camera_batch(
        rr_mod,
        snaps,
        path_prefix,
        color=color,
        line_width=line_width,
        image_plane_distance=image_plane_distance,
    )


def log_reconstruction(rec, *, app_id: str = "pytheia_vismatch_sfm") -> None:
    """Spawn a Rerun viewer and log estimated 3D points, camera centers, and frustums."""
    import rerun as rr

    rr.init(app_id, spawn=True)
    positions, colors = extract_triangulated_points(rec)
    if positions is not None and colors is not None:
        rr.log("world/points", rr.Points3D(positions, colors=colors))
    cc = extract_camera_centers(rec)
    if cc is not None:
        rr.log(
            "world/camera_centers",
            rr.Points3D(cc, radii=0.02, colors=[(255, 180, 0)]),
        )
    log_cameras_pinhole_frustums(
        rr,
        rec,
        "world/camera_frustums",
        color=(255, 200, 80),
        line_width=0.004,
    )


def log_eval_comparison(
    gt_rec,
    *,
    aligned_rec,
    app_id: str = "pytheia_vismatch_sfm_eval",
) -> None:
    """Ground truth vs Sim3-aligned estimate in one Rerun session (no pre-alignment layer)."""
    import rerun as rr

    rr.init(app_id, spawn=True)

    gt_cc = extract_camera_centers(gt_rec)
    if gt_cc is not None:
        rr.log(
            "world/gt/camera_centers",
            rr.Points3D(gt_cc, radii=0.025, colors=[(80, 255, 120)]),
        )
    log_cameras_pinhole_frustums(
        rr,
        gt_rec,
        "world/gt/camera_frustums",
        color=(70, 240, 110),
        line_width=0.004,
    )

    ap, ac = extract_triangulated_points(aligned_rec)
    if ap is not None and ac is not None:
        rr.log(
            "world/estimated/aligned/points",
            rr.Points3D(ap, colors=ac),
        )
    acc = extract_camera_centers(aligned_rec)
    if acc is not None:
        rr.log(
            "world/estimated/aligned/camera_centers",
            rr.Points3D(acc, radii=0.02, colors=[(120, 180, 255)]),
        )
    log_cameras_pinhole_frustums(
        rr,
        aligned_rec,
        "world/estimated/aligned/camera_frustums",
        color=(110, 170, 255),
        line_width=0.004,
    )
