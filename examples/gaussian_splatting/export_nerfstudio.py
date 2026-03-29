#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""CLI wrapper around pytheia.io.WriteNerfStudio for splat / NeRF pipelines."""

from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    p = argparse.ArgumentParser(description="Export pyTheia reconstruction to Nerfstudio transforms.json")
    p.add_argument("--path_to_images", type=str, required=True)
    p.add_argument("--path_to_recon", type=str, required=True)
    p.add_argument("--path_out_json", type=str, required=True)
    p.add_argument("--aabb_scale", type=int, default=16)
    args = p.parse_args()

    try:
        import pytheia as pt
    except ImportError:
        print("pytheia is not installed.", file=sys.stderr)
        return 1

    ok, recon = pt.io.ReadReconstruction(args.path_to_recon)
    if not ok:
        print(f"Failed to read reconstruction: {args.path_to_recon}", file=sys.stderr)
        return 1

    out_json = os.path.abspath(args.path_out_json)
    out_dir = os.path.dirname(out_json)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    ok = pt.io.WriteNerfStudio(
        args.path_to_images, recon, args.aabb_scale, args.path_out_json
    )
    if not ok:
        print("WriteNerfStudio failed.", file=sys.stderr)
        return 1
    print("Wrote", args.path_out_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
