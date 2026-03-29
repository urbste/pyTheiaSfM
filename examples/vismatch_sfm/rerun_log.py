# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Log a pyTheia Reconstruction to Rerun (https://rerun.io/)."""

from __future__ import annotations

import numpy as np


def log_reconstruction(rec, *, app_id: str = "pytheia_vismatch_sfm") -> None:
    """Spawn a Rerun viewer and log estimated 3D points and camera centers."""
    import rerun as rr

    rr.init(app_id, spawn=True)

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

    if xs:
        positions = np.stack(xs, axis=0)
        colors = np.stack(cs, axis=0)
        rr.log("world/points", rr.Points3D(positions, colors=colors))

    cam_centers: list[np.ndarray] = []
    for vid in rec.ViewIds():
        v = rec.View(vid)
        if not v.IsEstimated():
            continue
        cam_centers.append(np.asarray(v.Camera().GetPosition()).reshape(3))

    if cam_centers:
        cc = np.stack(cam_centers, axis=0)
        rr.log(
            "world/camera_centers",
            rr.Points3D(cc, radii=0.02, colors=[(255, 180, 0)]),
        )
