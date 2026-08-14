# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""KITTI odometry pose evaluation (MGSfM Table 1 style)."""

from __future__ import annotations

import os
from typing import Any

import numpy as np


def load_kitti_poses(path: str) -> np.ndarray:
    """Load KITTI pose file: N×12 row-major 3×4 cam→world. Returns N×4×4."""
    rows = np.loadtxt(path)
    if rows.ndim == 1:
        rows = rows.reshape(1, -1)
    if rows.shape[1] < 12:
        raise ValueError(f"Expected 12 columns in {path}, got {rows.shape[1]}")
    n = rows.shape[0]
    out = np.repeat(np.eye(4)[None, ...], n, axis=0)
    out[:, :3, :] = rows[:, :12].reshape(-1, 3, 4)
    return out


def kitti_pose_to_matrix(row34: np.ndarray) -> np.ndarray:
    """3×4 cam→world to 4×4 homogeneous."""
    t = np.eye(4, dtype=np.float64)
    t[:3, :] = np.asarray(row34, dtype=np.float64).reshape(3, 4)
    return t


def angle_axis_to_matrix(angle_axis: np.ndarray) -> np.ndarray:
    """World-to-camera rotation from angle-axis (Theia convention)."""
    aa = np.asarray(angle_axis, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(aa))
    if theta < 1e-12:
        return np.eye(3, dtype=np.float64)
    k = aa / theta
    kx, ky, kz = k
    kmat = np.array(
        [[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]], dtype=np.float64
    )
    return np.eye(3) + np.sin(theta) * kmat + (1.0 - np.cos(theta)) * (kmat @ kmat)


def view_cam_to_world_matrix(view) -> np.ndarray | None:
    """4×4 cam→world from an estimated pyTheia view."""
    if view is None or not view.IsEstimated():
        return None
    cam = view.Camera()
    c = np.asarray(cam.GetPosition(), dtype=np.float64).reshape(3)
    r_wc = angle_axis_to_matrix(cam.GetOrientationAsAngleAxis())
    r_cw = r_wc.T
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = r_cw
    t[:3, 3] = c
    return t


def rotation_angle_deg(r_gt: np.ndarray, r_est: np.ndarray) -> float:
    """Geodesic angle between two 3×3 rotation matrices (degrees)."""
    r_err = r_gt.T @ r_est
    trace = float(np.trace(r_err))
    c = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return float(np.degrees(np.arccos(c)))


def apply_sim3_to_pose(t_cw: np.ndarray, r_s: np.ndarray, t_s: np.ndarray, scale: float) -> np.ndarray:
    """Apply Sim(3) alignment (src→dst Umeyama) to a cam→world pose."""
    t = np.asarray(t_cw, dtype=np.float64).reshape(4, 4)
    r_cw = t[:3, :3]
    c = t[:3, 3]
    r_s = np.asarray(r_s, dtype=np.float64).reshape(3, 3)
    t_s = np.asarray(t_s, dtype=np.float64).reshape(3)
    s = float(scale)
    c_al = s * (r_s @ c) + t_s
    r_al = r_cw @ r_s.T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = r_al
    out[:3, 3] = c_al
    return out


def stereo_gt_poses(
    t_gt_left: np.ndarray,
    baseline: float,
) -> tuple[np.ndarray, np.ndarray]:
    """GT cam→world for left (KITTI cam0) and right (baseline along +X in cam frame)."""
    left = np.asarray(t_gt_left, dtype=np.float64).reshape(4, 4)
    right = left.copy()
    offset = left[:3, :3] @ np.array([baseline, 0.0, 0.0], dtype=np.float64)
    right[:3, 3] = left[:3, 3] + offset
    return left, right


def summarize_errors(rot_deg: np.ndarray, pos_m: np.ndarray) -> dict[str, float]:
    rot = np.asarray(rot_deg, dtype=np.float64).reshape(-1)
    pos = np.asarray(pos_m, dtype=np.float64).reshape(-1)
    if rot.size == 0:
        return {
            "er_median": float("nan"),
            "er_mean": float("nan"),
            "et_median": float("nan"),
            "et_mean": float("nan"),
            "n": 0,
        }
    return {
        "er_median": float(np.median(rot)),
        "er_mean": float(np.mean(rot)),
        "et_median": float(np.median(pos)),
        "et_mean": float(np.mean(pos)),
        "n": int(rot.size),
    }


def mgsfm_metrics_from_poses(
    est_poses: list[np.ndarray],
    gt_poses: list[np.ndarray],
) -> dict[str, Any]:
    """
    MGSfM-style metrics after Sim(3) Umeyama alignment on camera centers.

    est_poses / gt_poses: list of 4×4 cam→world matrices, equal length.
    """
    import pytheia as pt

    if len(est_poses) < 3 or len(est_poses) != len(gt_poses):
        return {"error": "need ≥3 matched poses", "n": 0}

    est_c = [np.asarray(p[:3, 3], dtype=np.float64).reshape(3) for p in est_poses]
    gt_c = [np.asarray(p[:3, 3], dtype=np.float64).reshape(3) for p in gt_poses]
    r_s, t_s, scale = pt.sfm.AlignPointCloudsUmeyama(est_c, gt_c)

    rot_err: list[float] = []
    pos_err: list[float] = []
    for est, gt in zip(est_poses, gt_poses):
        est_al = apply_sim3_to_pose(est, r_s, t_s, float(scale))
        r_gt = np.asarray(gt[:3, :3], dtype=np.float64)
        r_est = est_al[:3, :3]
        c_gt = np.asarray(gt[:3, 3], dtype=np.float64)
        c_est = est_al[:3, 3]
        rot_err.append(rotation_angle_deg(r_gt, r_est))
        pos_err.append(float(np.linalg.norm(c_est - c_gt)))

    stats = summarize_errors(np.array(rot_err), np.array(pos_err))
    stats["sim3_scale"] = float(scale)
    return stats


def mgsfm_metrics_for_sequence(
    recon,
    left_view_ids: list,
    right_view_ids: list,
    t_gt_all: np.ndarray,
    baseline: float,
    frame_indices: list[int] | None = None,
) -> dict[str, Any]:
    """
    Evaluate all estimated left+right views against KITTI GT (cam0 + stereo right).

    left_view_ids / right_view_ids are indexed by capture/frame index (may contain None).
    t_gt_all: N×4×4 GT poses for the full sequence (cam0).
    """
    est_poses: list[np.ndarray] = []
    gt_poses: list[np.ndarray] = []

    n = len(left_view_ids)
    for i in range(n):
        if frame_indices is not None:
            fi = frame_indices[i]
        else:
            fi = i
        if fi < 0 or fi >= t_gt_all.shape[0]:
            continue
        t_gt_left = t_gt_all[fi]
        gt_left, gt_right = stereo_gt_poses(t_gt_left, baseline)

        for vid, gt_pose in (
            (left_view_ids[i], gt_left),
            (right_view_ids[i], gt_right),
        ):
            if vid is None:
                continue
            view = recon.View(vid)
            est = view_cam_to_world_matrix(view)
            if est is None:
                continue
            est_poses.append(est)
            gt_poses.append(gt_pose)

    if len(est_poses) < 3:
        return {"error": "fewer than 3 estimated cameras", "n": len(est_poses)}

    stats = mgsfm_metrics_from_poses(est_poses, gt_poses)
    stats["sequence_n"] = len(est_poses)
    return stats


def write_kitti_poses_file(path: str, poses: list[np.ndarray]) -> None:
    """Write list of 4×4 cam→world poses as KITTI 3×4 rows."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for t in poses:
            mat = np.asarray(t, dtype=np.float64).reshape(4, 4)
            row = mat[:3, :].reshape(-1)
            f.write(" ".join(f"{x:.12e}" for x in row) + "\n")


def export_left_camera_poses(
    recon,
    left_view_ids: list,
    frame_indices: list[int] | None = None,
) -> list[np.ndarray]:
    """Export cam→world poses for frames with estimated left views (sorted by frame)."""
    items: list[tuple[int, np.ndarray]] = []
    for i, vid in enumerate(left_view_ids):
        if vid is None:
            continue
        fi = frame_indices[i] if frame_indices is not None else i
        est = view_cam_to_world_matrix(recon.View(vid))
        if est is not None:
            items.append((int(fi), est))
    items.sort(key=lambda x: x[0])
    return [p for _, p in items]
