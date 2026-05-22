#!/usr/bin/env python3
"""
Load a pytheia .recon and write a smaller file containing only estimated views and
valid tracks (CreateEstimatedSubreconstruction).

pt.io.WriteReconstruction now applies the same filter by default; this script is a small
CLI that reports counts and writes without double-filtering (subreconstruction +
write_full_reconstruction=True).

IDs are NOT remapped for surviving views/tracks. Removed tracks are absent—see
scripts/recon_utils.py if you need to align external track_id usage.

Usage:
  python scripts/save_estimated_subreconstruction.py input.recon output.recon
"""

from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input_recon", type=str, help="Path to input pytheia .recon")
    p.add_argument("output_recon", type=str, help="Path to write filtered .recon")
    args = p.parse_args()

    path_in = os.path.abspath(args.input_recon)
    path_out = os.path.abspath(args.output_recon)

    if not os.path.isfile(path_in):
        print(f"Input not found: {path_in}", file=sys.stderr)
        return 1

    try:
        import pytheia as pt
    except ImportError:
        print("Import pytheia failed. Activate the env where pytheia is built.", file=sys.stderr)
        return 1

    ok, recon = pt.io.ReadReconstruction(path_in)
    if not ok:
        print("ReadReconstruction failed.", file=sys.stderr)
        return 1

    nv = recon.NumViews()
    nt = recon.NumTracks()
    ev = pt.sfm.NumEstimatedViews(recon)
    et = pt.sfm.NumEstimatedTracks(recon)
    print(f"Loaded: {path_in}")
    print(f"  views {nv} (estimated {ev}), tracks {nt} (estimated {et})")

    sub = pt.sfm.Reconstruction()
    pt.sfm.CreateEstimatedSubreconstruction(recon, sub)

    nv2 = sub.NumViews()
    nt2 = sub.NumTracks()
    ev2 = pt.sfm.NumEstimatedViews(sub)
    et2 = pt.sfm.NumEstimatedTracks(sub)
    print(f"Writing estimated-only model: {path_out}")
    print(f"  views {nv2} (estimated {ev2}), tracks {nt2} (estimated {et2})")

    # Already filtered; save as-is without running CreateEstimatedSubreconstruction again.
    if not pt.io.WriteReconstruction(sub, path_out, write_full_reconstruction=True):
        print("WriteReconstruction failed.", file=sys.stderr)
        return 1

    size = os.path.getsize(path_out)
    print(f"Wrote {size / (1024**3):.3f} GiB ({size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
