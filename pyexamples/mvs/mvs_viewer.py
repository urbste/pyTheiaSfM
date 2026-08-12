#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Minimal Open3D viewer for a PLY point cloud (e.g. pyTheia export)."""

from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description="View a PLY point cloud with Open3D"
    )
    parser.add_argument(
        "--ply",
        type=str,
        required=True,
        help="Path to a .ply file (e.g. from WritePlyFile)",
    )
    parser.add_argument(
        "--point_size",
        type=float,
        default=1.0,
        help="Open3D render point size",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.ply):
        print(f"PLY not found: {args.ply}", file=sys.stderr)
        return 1

    try:
        import open3d as o3d
    except ImportError:
        print("open3d is required: pip install open3d", file=sys.stderr)
        return 1

    cloud = o3d.io.read_point_cloud(args.ply)
    if cloud.is_empty():
        print(f"Empty or unreadable point cloud: {args.ply}", file=sys.stderr)
        return 1

    print(
        f"Loaded {len(cloud.points)} points from {args.ply} "
        f"(has_colors={cloud.has_colors()}, has_normals={cloud.has_normals()})"
    )
    o3d.visualization.draw_geometries(
        [cloud],
        window_name=os.path.basename(args.ply),
        point_show_normal=False,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
