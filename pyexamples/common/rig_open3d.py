# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Optional Open3D visualization for calibrated-rig reconstructions."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def try_import_open3d():
    try:
        import open3d as o3d

        return o3d
    except ImportError:
        return None


def geometries_from_reconstruction(
    recon,
    *,
    min_track_observations: int = 2,
    gt_xyz: Optional[np.ndarray] = None,
    aligned_est_xyz: Optional[np.ndarray] = None,
):
    """Build Open3D geometries: tracks, capture trajectory, stereo baselines.

    If |gt_xyz| / |aligned_est_xyz| are given (N×3, already in the same frame),
    they are drawn as green / blue polylines.
    """
    o3d = try_import_open3d()
    if o3d is None:
        return None

    geoms = []
    pts = []
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        if track is None or not track.IsEstimated():
            continue
        if track.NumViews() < min_track_observations:
            continue
        p = np.asarray(track.Point(), dtype=np.float64).reshape(-1)
        pts.append(p[:3] / p[3] if p.size == 4 else p[:3])
    if pts:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
        cloud.paint_uniform_color([0.65, 0.65, 0.65])
        geoms.append(cloud)

    capture_ids = list(recon.CaptureIds())
    capture_ids.sort(
        key=lambda cid: recon.GetRigCapture(cid).GetTimestamp()
        if recon.GetRigCapture(cid) is not None
        else 0.0
    )
    body_xyz = []
    baseline_pts = []
    baseline_lines = []
    for cid in capture_ids:
        cap = recon.GetRigCapture(cid)
        if cap is None or not cap.IsEstimated():
            continue
        body = np.asarray(cap.GetPosition(), dtype=np.float64).reshape(3)
        body_xyz.append(body)
        sensor_xyz = []
        for vid in cap.GetViewIds():
            view = recon.View(vid)
            if view is None or not view.IsEstimated():
                continue
            sensor_xyz.append(np.asarray(view.Camera().GetPosition()).reshape(3))
        if len(sensor_xyz) >= 2:
            i0 = len(baseline_pts)
            baseline_pts.extend(sensor_xyz)
            for a in range(len(sensor_xyz) - 1):
                baseline_lines.append([i0 + a, i0 + a + 1])

    if len(body_xyz) >= 2:
        traj = o3d.geometry.LineSet()
        traj.points = o3d.utility.Vector3dVector(np.asarray(body_xyz))
        traj.lines = o3d.utility.Vector2iVector(
            [[i, i + 1] for i in range(len(body_xyz) - 1)]
        )
        traj.paint_uniform_color([0.1, 0.2, 0.9])
        geoms.append(traj)

    if baseline_lines:
        bases = o3d.geometry.LineSet()
        bases.points = o3d.utility.Vector3dVector(np.asarray(baseline_pts))
        bases.lines = o3d.utility.Vector2iVector(baseline_lines)
        bases.paint_uniform_color([0.9, 0.15, 0.1])
        geoms.append(bases)

    def _polyline(xyz: np.ndarray, color: Sequence[float]):
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        if xyz.shape[0] < 2:
            return None
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(xyz)
        ls.lines = o3d.utility.Vector2iVector(
            [[i, i + 1] for i in range(xyz.shape[0] - 1)]
        )
        ls.paint_uniform_color(list(color))
        return ls

    if gt_xyz is not None:
        g = _polyline(gt_xyz, (0.1, 0.75, 0.2))
        if g is not None:
            geoms.append(g)
    if aligned_est_xyz is not None:
        e = _polyline(aligned_est_xyz, (0.95, 0.55, 0.1))
        if e is not None:
            geoms.append(e)

    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))
    return geoms


def visualize_rig_reconstruction(
    recon,
    *,
    window_name: str = "pyTheia rig",
    min_track_observations: int = 2,
    gt_xyz: Optional[np.ndarray] = None,
    aligned_est_xyz: Optional[np.ndarray] = None,
) -> bool:
    """Show the reconstruction in Open3D. Returns False if Open3D is missing."""
    o3d = try_import_open3d()
    if o3d is None:
        print("Open3D not installed; skip visualization (pip install open3d)")
        return False
    geoms = geometries_from_reconstruction(
        recon,
        min_track_observations=min_track_observations,
        gt_xyz=gt_xyz,
        aligned_est_xyz=aligned_est_xyz,
    )
    if not geoms:
        print("Nothing to visualize (no estimated tracks or captures)")
        return False
    o3d.visualization.draw_geometries(geoms, window_name=window_name)
    return True
