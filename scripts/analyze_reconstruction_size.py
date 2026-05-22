#!/usr/bin/env python3
"""
Measure approximate in-memory / on-disk contribution breakdown for a pytheia .recon file.

Usage:
  python scripts/analyze_reconstruction_size.py /path/to/pytheia.recon

Requires: pytheia (same env as your pipeline). Loads the full reconstruction into RAM.
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def _desc_len(ref_desc) -> int:
    if ref_desc is None:
        return 0
    try:
        return int(ref_desc.size)
    except AttributeError:
        return len(ref_desc)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "recon_path",
        type=str,
        help="Path to pytheia binary reconstruction (.recon)",
    )
    args = p.parse_args()

    path = os.path.abspath(args.recon_path)
    if not os.path.isfile(path):
        print(f"Not a file: {path}", file=sys.stderr)
        return 1

    try:
        import pytheia as pt
    except ImportError:
        print("Import pytheia failed. Activate the env where pytheia is built.", file=sys.stderr)
        return 1

    file_size = os.path.getsize(path)
    print(f"File on disk: {path}")
    print(f"  size: {file_size / (1024**3):.3f} GiB ({file_size} bytes)\n")

    t0 = time.perf_counter()
    ok, recon = pt.io.ReadReconstruction(path)
    load_s = time.perf_counter() - t0
    if not ok:
        print("ReadReconstruction returned False", file=sys.stderr)
        return 1

    nv = recon.NumViews()
    nt = recon.NumTracks()
    ev = pt.sfm.NumEstimatedViews(recon)
    et = pt.sfm.NumEstimatedTracks(recon)
    print("Counts")
    print(f"  views:  {nv}  (estimated: {ev})")
    print(f"  tracks: {nt}  (estimated: {et})")
    if nt > et:
        print(
            f"  unestimated tracks still in file: {nt - et}  "
            f"({100.0 * (nt - et) / nt:.1f}% of tracks; each still serializes 3D point, view set, etc.)"
        )

    view_ids = recon.ViewIds()
    total_obs = 0
    for vid in view_ids:
        total_obs += recon.View(vid).NumFeatures()

    print(f"  total 2D observations (sum of View.NumFeatures): {total_obs}")
    if total_obs > 0 and nv > 0:
        print(f"  mean observations / view: {total_obs / nv:.1f}")

    track_ids = recon.TrackIds()
    t0 = time.perf_counter()
    desc_bytes = 0
    nonzero_desc_tracks = 0
    sum_track_views = 0
    max_desc = 0
    hist: dict[int, int] = {}

    for i, tid in enumerate(track_ids):
        tr = recon.Track(tid)
        sum_track_views += tr.NumViews()
        L = _desc_len(tr.ReferenceDescriptor())
        if L > 0:
            nonzero_desc_tracks += 1
            desc_bytes += L * 4
            max_desc = max(max_desc, L)
            hist[L] = hist.get(L, 0) + 1
        if (i + 1) % 500000 == 0:
            print(f"  ... scanned {i + 1} tracks ({time.perf_counter() - t0:.1f}s)", flush=True)

    print(f"\nTrack scan time: {time.perf_counter() - t0:.2f}s")

    print("\nReference descriptors (Track.ReferenceDescriptor, float32)")
    print(f"  tracks with nonzero descriptor length: {nonzero_desc_tracks} / {nt}")
    print(f"  total descriptor payload (approx): {desc_bytes / (1024**2):.2f} MiB")
    print(f"  max descriptor length: {max_desc}")
    if hist:
        for length, count in sorted(hist.items(), key=lambda x: -x[1])[:12]:
            print(f"    dim {length}: {count} tracks")

    if sum_track_views != total_obs:
        print(
            f"\nNote: sum(Track.NumViews)={sum_track_views} vs "
            f"sum(View.NumFeatures)={total_obs} (expect equal for consistent graphs)."
        )

    # Feature.covariance_: Eigen::Matrix2d = 2×2 doubles (see theia/sfm/feature.h)
    cov_matrix_elems = 2 * 2
    cov_bytes_per_matrix = cov_matrix_elems * 8  # double
    cov_total_bytes = total_obs * cov_bytes_per_matrix

    print("\n2D feature covariance (Feature.covariance_, Matrix2d)")
    print(f"  one matrix: {cov_matrix_elems} doubles × 8 B = {cov_bytes_per_matrix} B")
    print(
        f"  all observations: {total_obs} × {cov_bytes_per_matrix} B = "
        f"{cov_total_bytes / (1024**3):.3f} GiB ({cov_total_bytes} bytes)"
    )

    bytes_per_feature_point = 16  # Vector2d
    bytes_per_feature_cov = cov_bytes_per_matrix
    bytes_per_feature_depth = 16  # depth_prior + depth_prior_variance
    bytes_per_feature_payload = bytes_per_feature_point + bytes_per_feature_cov + bytes_per_feature_depth
    feat_payload = total_obs * bytes_per_feature_payload

    print("\nOrder-of-magnitude payload (data fields only, no STL / cereal overhead)")
    print(
        f"  2D feature fields ({bytes_per_feature_point} B point + {bytes_per_feature_cov} B cov + "
        f"{bytes_per_feature_depth} B depth = {bytes_per_feature_payload} B/obs): "
        f"{feat_payload / (1024**3):.3f} GiB  (#obs={total_obs})"
    )
    print(f"  reference descriptors (float32): {desc_bytes / (1024**3):.3f} GiB")
    print(
        f"  track–view membership (very rough, ~16 B/obs): "
        f"{(sum_track_views * 16) / (1024**3):.3f} GiB"
    )

    if nv > 0:
        cov_avg = cov_total_bytes / nv
        feat_avg = feat_payload / nv
        desc_avg = desc_bytes / nv
        tv_avg = (sum_track_views * 16) / nv
        print("\nPer-view averages (spread total payload evenly across views)")
        print(
            f"  if you dropped only covariance:  ~{cov_avg / (1024**2):.2f} MiB/view "
            f"({cov_avg:,.0f} B/view); equals mean_obs × {cov_bytes_per_matrix} B"
        )
        print(
            f"  if you dropped full 2D feature record: ~{feat_avg / (1024**2):.2f} MiB/view "
            f"({feat_avg:,.0f} B/view); equals mean_obs × {bytes_per_feature_payload} B"
        )
        print(
            f"  reference descriptors (global total ÷ views): ~{desc_avg / (1024**2):.2f} MiB/view "
            f"({desc_avg:,.0f} B/view)  [descriptors live on tracks, not views]"
        )
        print(
            f"  track–view links (rough total ÷ views): ~{tv_avg / (1024**2):.2f} MiB/view "
            f"({tv_avg:,.0f} B/view)"
        )

    dense_sum = feat_payload + desc_bytes + (sum_track_views * 16)
    print("\nWhy is the .recon file much larger than the dense payload sums above?")
    print(
        "  - Those lines count only raw floats/doubles (point, cov, depth, descriptors, rough links). "
        "They do NOT include Cereal tags, alignment padding, or hash-table overhead."
    )
    print(
        f"  - Each view stores features in an unordered_map (≈{total_obs} entries total): "
        "each entry pays for buckets/pointers beyond the Feature struct."
    )
    print(
        "  - Every track stores an unordered_set of view ids, 3D point, bearing, inverse depth, flags — "
        "for all tracks, including unestimated (see counts above)."
    )
    print("  - Camera/intrinsics blobs, view name strings, and Reconstruction index maps add more bytes.")
    if file_size > 0:
        print(
            f"  - Ballpark: naive dense total ≈ {dense_sum / (1024**3):.3f} GiB vs file "
            f"{file_size / (1024**3):.3f} GiB (~{100.0 * dense_sum / file_size:.0f}% of file; rest is structure/overhead)."
        )

    if file_size > 0:
        cov_frac = cov_total_bytes / file_size
        print("\nIf you keep 2D points + depth but remove only covariance (Matrix2d per observation):")
        print(
            f"  - Raw covariance payload ≈ {cov_total_bytes / (1024**2):.1f} MiB "
            f"(~{100.0 * cov_frac:.1f}% of current file size if it were stored that tightly; "
            "actual .recon savings depend on layout and may be a bit less)."
        )

    print(
        "\nAlignment often needs: poses + 3D points + 2D observations; dropping covariance only "
        "does not remove observations. Unestimated tracks can be dropped only if your pipeline allows "
        "(e.g. CreateEstimatedSubreconstruction before save)."
    )
    print(f"\nLoad time: {load_s:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
