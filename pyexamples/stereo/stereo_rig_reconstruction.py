#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Stereo / calibrated-rig reconstruction example.

Sets pinhole intrinsics and a stereo baseline (abstract body at identity,
left/right sensors offset along +X), detects keypoints once per image and
matches with LightGlue (default: disk-lightglue), builds tracks (including
same-timestamp stereo via the known essential) and a ViewGraph of
*inter-capture* motion edges, then runs GlobalRigReconstructor or
IncrementalRigReconstructor.

Same-timestamp left↔right matches are used for triangulation/scale only — they
are not ViewGraph motion edges (pure baseline translation is degenerate for
relative pose). By default only **left** images are matched (cascade: frame i
against every later frame) and stereo is attached only to tracks that already
span several rig poses. Pass `--left_match_mode window` / `--match_right_temporal`
for the older dense temporal schedule. Pass `--trajectory_has_loops` on long
trajectories that revisit places: CosPlace (ResNet18/128) is extracted on left
keyframes and `GraphMatch` loop candidates are matched in addition to the
temporal/cascade schedule.

Bundle adjustment uses Huber loss by default (`--ba_loss trivial` for plain L2).
`--recon_stride N` reconstructs every Nth frame (plus first/last) and PnP-
localizes the skipped captures from neighboring keyframes.

From a ZED extract folder (left/, right/, rig_calibration.json):
  python pyexamples/stereo/stereo_rig_reconstruction.py \\
    --frames_dir /home/steffen/Dokumente/ZED/test_frames \\
    --matcher disk-lightglue --method global --visualize

KITTI odometry (ATE / RPE vs ground truth):
  python pyexamples/stereo/kitti_rig_benchmark.py --kitti_root ... --sequences 00
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_EXAMPLES = os.path.dirname(_THIS)
if _EXAMPLES not in sys.path:
    sys.path.insert(0, _EXAMPLES)
if _THIS not in sys.path:
    sys.path.insert(0, _THIS)

from calibrated_stereo_rig import (  # noqa: E402
    BA_LOSS_TYPES,
    LIGHTGLUE_MATCHERS,
    RESIZE_MODES,
    StereoCalib,
    StereoRigRunOptions,
    run_calibrated_stereo_rig,
)
from common.rig_open3d import visualize_rig_reconstruction  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Calibrated stereo rig SfM with vismatch + pyTheia"
    )
    p.add_argument(
        "--frames_dir",
        type=str,
        default="",
        help=(
            "SVO-extract root with left/, right/, and optional "
            "rig_calibration.json (from zed_svo_extract_stereo.py)"
        ),
    )
    p.add_argument("--left_dir", type=str, default="")
    p.add_argument("--right_dir", type=str, default="")
    p.add_argument("--img_ext", type=str, default="")
    p.add_argument(
        "--baseline",
        type=float,
        default=None,
        help="Meters (right − left along +X in rig frame)",
    )
    p.add_argument("--focal", type=float, default=None)
    p.add_argument("--cx", type=float, default=None)
    p.add_argument("--cy", type=float, default=None)
    p.add_argument("--width", type=int, default=0, help="If 0, read from first image")
    p.add_argument("--height", type=int, default=0)
    p.add_argument(
        "--matcher",
        type=str,
        default="disk-lightglue",
        choices=LIGHTGLUE_MATCHERS,
        help="Detect-once + LightGlue matcher (default: disk-lightglue)",
    )
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument(
        "--resize",
        type=int,
        default=960,
        help="Target image width in pixels when --resize_mode=width (default)",
    )
    p.add_argument(
        "--resize_mode",
        choices=RESIZE_MODES,
        default="width",
        help="width: scale to --resize px wide; max: longest-side resize",
    )
    p.add_argument(
        "--match_thresh",
        type=float,
        default=0.5,
        help="Unused for LightGlue matchers (kept for CLI compatibility)",
    )
    p.add_argument(
        "--max_keypoints",
        type=int,
        default=2048,
        help="Max keypoints for SuperPoint / DISK / ALIKED / SIFT extractors",
    )
    p.add_argument("--min_matches", type=int, default=40)
    p.add_argument(
        "--max_sampson_error",
        type=float,
        default=2.0,
        help="Sampson threshold (pixels) for stereo E filtering and two-view RANSAC",
    )
    p.add_argument(
        "--left_match_mode",
        choices=("cascade", "window"),
        default="cascade",
        help=(
            "cascade: left_i vs later left_j (default). "
            "window: left_i vs i+1..i+temporal_window plus loop_stride."
        ),
    )
    p.add_argument(
        "--left_match_max_gap",
        type=int,
        default=0,
        help=(
            "Cascade only: match i to i+1..i+gap. 0 = all later frames. "
            "Cap this on long sequences (KITTI)."
        ),
    )
    p.add_argument(
        "--temporal_window",
        type=int,
        default=5,
        help="Used when --left_match_mode window: match ±N left frames.",
    )
    p.add_argument(
        "--loop_stride",
        type=int,
        default=20,
        help=(
            "Also match left frames i to i+k*stride (0 disables). "
            "With cascade max_gap=0 this is redundant."
        ),
    )
    p.add_argument(
        "--trajectory_has_loops",
        action="store_true",
        help=(
            "Precompute CosPlace (ResNet18/128) on left keyframes and match "
            "GraphMatch loop candidates. For long trajectories that revisit places."
        ),
    )
    p.add_argument(
        "--cosplace_neighbors",
        type=int,
        default=5,
        help="GraphMatch k nearest CosPlace neighbours per keyframe (default 5).",
    )
    p.add_argument(
        "--cosplace_min_frame_gap",
        type=int,
        default=0,
        help=(
            "Minimum keyframe index gap for a CosPlace pair (0 = auto: just "
            "beyond --temporal_window / --left_match_max_gap)."
        ),
    )
    p.add_argument(
        "--match_right_temporal",
        action="store_true",
        help="Also match right–right with the same pair schedule (off by default).",
    )
    p.add_argument(
        "--cross_sensor_temporal",
        action="store_true",
        help=(
            "Also match left_i↔right_j across time (motion edges + tracks). "
            "Same-timestamp left↔right is tracks-only (known essential)."
        ),
    )
    p.add_argument(
        "--min_track_length",
        type=int,
        default=3,
        help="TrackBuilder min length (default 3 drops two-view stereo-only tracks)",
    )
    p.add_argument(
        "--min_track_captures",
        type=int,
        default=3,
        help="Drop tracks observed in fewer than this many rig poses.",
    )
    p.add_argument(
        "--stereo_snap_pixels",
        type=float,
        default=3.0,
        help="Snap per-pair matcher keypoints onto existing features (pixels).",
    )
    p.add_argument(
        "--no_guided_stereo",
        action="store_true",
        help="Attach all stereo E-inliers instead of only long left tracks.",
    )
    p.add_argument(
        "--no_detect_once",
        action="store_true",
        help="Re-detect keypoints on every image pair (default: extract once per frame).",
    )
    p.add_argument(
        "--feature_cache_dir",
        type=str,
        default="",
        help=(
            "Directory for on-disk keypoint/descriptor cache. "
            "Default: <dataset>/.pytheia_features/<matcher>_w<resize>_k<max_keypoints>/"
        ),
    )
    p.add_argument(
        "--no_feature_cache",
        action="store_true",
        help="Do not read or write the on-disk feature cache.",
    )
    p.add_argument("--match_cache_dir", type=str, default="")
    p.add_argument(
        "--no_match_cache",
        action="store_true",
        help="Do not read or write the on-disk LightGlue match cache.",
    )
    p.add_argument(
        "--match_cache_only",
        action="store_true",
        help="Run full matching schedule and skip SfM/BA (populate match cache).",
    )
    p.add_argument(
        "--verbose_matches",
        action="store_true",
        help="Print every image pair (default: progress every 25 pairs).",
    )
    p.add_argument(
        "--retriangulation_iterations",
        type=int,
        default=0,
        help=(
            "Extra triangulate+BA passes after the first "
            "(Theia default is 1; 0 = one triangulate+BA only)"
        ),
    )
    p.add_argument(
        "--ba_loss",
        type=str,
        default="huber",
        choices=BA_LOSS_TYPES,
        help="Bundle adjustment robust kernel (default: huber).",
    )
    p.add_argument(
        "--ba_robust_width",
        type=float,
        default=2.0,
        help="Huber/Cauchy/… width in pixels (default 2).",
    )
    p.add_argument(
        "--recon_stride",
        type=int,
        default=1,
        help=(
            "Reconstruct every Nth stereo pair (first and last always kept), "
            "then PnP-localize the skipped captures. 1 = all frames in SfM."
        ),
    )
    p.add_argument(
        "--pnp_min_inliers",
        type=int,
        default=30,
        help="Minimum PnP inliers to accept a skipped-capture pose.",
    )
    p.add_argument(
        "--method",
        choices=("global", "incremental"),
        default="global",
    )
    p.add_argument(
        "--rotation_estimator",
        type=str,
        default="ROBUST_L1L2",
        choices=("ROBUST_L1L2", "NONLINEAR", "LINEAR", "LAGRANGE_DUAL", "HYBRID"),
    )
    p.add_argument(
        "--position_estimator",
        type=str,
        default="LEAST_UNSQUARED_DEVIATION",
        choices=(
            "LEAST_UNSQUARED_DEVIATION",
            "NONLINEAR",
            "LINEAR_TRIPLET",
            "LIGT",
            "GLOMAP",
        ),
    )
    p.add_argument("--out_reconstruction", type=str, default="")
    p.add_argument(
        "--match_debug_dir",
        type=str,
        default="",
        help="Write side-by-side PNGs + left_matches.jsonl for left–left motion pairs.",
    )
    p.add_argument("--match_debug_max_pairs", type=int, default=0)
    p.add_argument(
        "--match_debug_only",
        action="store_true",
        help="Stop after left–left matching (skip stereo attach + SfM).",
    )
    p.add_argument(
        "--visualize",
        action="store_true",
        help="Open an Open3D window after a successful reconstruction (if installed)",
    )
    return p.parse_args()


def _sorted_images(folder: str, ext: str) -> list[str]:
    paths = sorted(glob.glob(os.path.join(folder, f"*.{ext}")))
    if not paths:
        paths = sorted(glob.glob(os.path.join(folder, f"*.{ext.upper()}")))
    return paths


def _guess_img_ext(left_dir: str) -> str:
    for ext in ("png", "jpg", "jpeg", "PNG", "JPG", "JPEG"):
        if glob.glob(os.path.join(left_dir, f"*.{ext}")):
            return ext.lower()
    return "png"


def _apply_frames_dir(args: argparse.Namespace) -> argparse.Namespace:
    """Fill left/right dirs and calibration from an SVO-extract folder."""
    if not args.frames_dir:
        return args
    root = os.path.abspath(args.frames_dir)
    if not os.path.isdir(root):
        raise FileNotFoundError(f"--frames_dir not found: {root}")
    left = os.path.join(root, "left")
    right = os.path.join(root, "right")
    if not args.left_dir:
        args.left_dir = left
    if not args.right_dir:
        args.right_dir = right
    if not os.path.isdir(args.left_dir) or not os.path.isdir(args.right_dir):
        raise FileNotFoundError(
            f"Expected left/ and right/ under {root} "
            f"(got {args.left_dir!r}, {args.right_dir!r})"
        )

    calib_path = os.path.join(root, "rig_calibration.json")
    if os.path.isfile(calib_path):
        with open(calib_path, encoding="utf-8") as f:
            calib = json.load(f)
        py = calib.get("pytheia", calib)
        if args.baseline is None and py.get("baseline") is not None:
            args.baseline = float(py["baseline"])
        if args.focal is None and py.get("focal") is not None:
            args.focal = float(py["focal"])
        if args.cx is None and py.get("cx") is not None:
            args.cx = float(py["cx"])
        if args.cy is None and py.get("cy") is not None:
            args.cy = float(py["cy"])
        if not args.width and py.get("width"):
            args.width = int(py["width"])
        if not args.height and py.get("height"):
            args.height = int(py["height"])
        print(f"Loaded calibration from {calib_path}")
    return args


def _require_calibration(args: argparse.Namespace) -> None:
    missing = [
        name
        for name, val in (
            ("--baseline", args.baseline),
            ("--focal", args.focal),
            ("--cx", args.cx),
            ("--cy", args.cy),
        )
        if val is None
    ]
    if missing or not args.left_dir or not args.right_dir:
        raise SystemExit(
            "Need image dirs and calibration. Either pass:\n"
            "  --frames_dir <SVO extract with left/, right/, rig_calibration.json>\n"
            "or:\n"
            "  --left_dir … --right_dir … --baseline … --focal … --cx … --cy …\n"
            f"Missing: {', '.join(missing) if missing else 'left/right dirs'}"
        )


def main() -> int:
    args = _parse_args()
    try:
        _apply_frames_dir(args)
        _require_calibration(args)
    except (FileNotFoundError, OSError, ValueError, KeyError, TypeError) as exc:
        print(exc, file=sys.stderr)
        return 1

    if not args.img_ext:
        args.img_ext = _guess_img_ext(args.left_dir)

    try:
        import vismatch  # noqa: F401
    except ImportError:
        print(
            "vismatch is required for this example. Install with:\n"
            '  pip install "pytheia[examples]"   # or: pip install vismatch',
            file=sys.stderr,
        )
        return 1

    import numpy as np
    import pytheia as pt

    left_paths = _sorted_images(args.left_dir, args.img_ext)
    right_paths = _sorted_images(args.right_dir, args.img_ext)
    n = min(len(left_paths), len(right_paths))
    if n < 2:
        print(
            f"Need at least 2 synchronized stereo pairs "
            f"(found left={len(left_paths)} right={len(right_paths)} "
            f"ext=*.{args.img_ext} under {args.left_dir!r}).",
            file=sys.stderr,
        )
        return 1

    calib = StereoCalib(
        focal=float(args.focal),
        cx=float(args.cx),
        cy=float(args.cy),
        baseline=float(args.baseline),
        width=int(args.width),
        height=int(args.height),
    )
    options = StereoRigRunOptions(
        matcher=args.matcher,
        device=args.device,
        resize=args.resize,
        resize_mode=args.resize_mode,
        match_thresh=args.match_thresh,
        max_keypoints=args.max_keypoints,
        min_matches=args.min_matches,
        max_sampson_error=args.max_sampson_error,
        left_match_mode=args.left_match_mode,
        left_match_max_gap=args.left_match_max_gap,
        temporal_window=args.temporal_window,
        loop_stride=args.loop_stride,
        trajectory_has_loops=args.trajectory_has_loops,
        cosplace_neighbors=args.cosplace_neighbors,
        cosplace_min_frame_gap=args.cosplace_min_frame_gap,
        match_right_temporal=args.match_right_temporal,
        cross_sensor_temporal=args.cross_sensor_temporal,
        stereo_snap_pixels=args.stereo_snap_pixels,
        guided_stereo=not args.no_guided_stereo,
        detect_once=not args.no_detect_once,
        feature_cache=not args.no_feature_cache,
        feature_cache_dir=args.feature_cache_dir,
        match_cache=not args.no_match_cache,
        match_cache_dir=args.match_cache_dir,
        match_cache_only=args.match_cache_only,
        min_track_length=args.min_track_length,
        min_track_captures=args.min_track_captures,
        retriangulation_iterations=args.retriangulation_iterations,
        ba_loss=args.ba_loss,
        ba_robust_width=args.ba_robust_width,
        recon_stride=args.recon_stride,
        pnp_min_inliers=args.pnp_min_inliers,
        method=args.method,
        rotation_estimator=args.rotation_estimator,
        position_estimator=args.position_estimator,
        verbose_matches=args.verbose_matches,
        match_debug_dir=args.match_debug_dir,
        match_debug_max_pairs=args.match_debug_max_pairs,
        match_debug_only=args.match_debug_only,
    )
    try:
        result = run_calibrated_stereo_rig(left_paths, right_paths, calib, options)
    except (FileNotFoundError, ValueError, ImportError) as exc:
        print(exc, file=sys.stderr)
        return 1

    if not args.out_reconstruction and args.frames_dir:
        args.out_reconstruction = os.path.join(
            os.path.abspath(args.frames_dir), "stereo_rig.recon"
        )
    if args.out_reconstruction and result.summary.success:
        pt.io.WriteReconstruction(result.recon, args.out_reconstruction)
        print(f"Wrote {args.out_reconstruction}")
        ply_path = os.path.splitext(args.out_reconstruction)[0] + ".ply"
        pt.io.WriteRigPlyFile(
            ply_path,
            result.recon,
            sensor_color=np.array([255, 0, 0], dtype=np.int32),
            min_num_observations_per_point=2,
        )
        print(f"Wrote {ply_path} (tracks + capture trajectory + sensor baselines)")

    if args.visualize and result.summary.success:
        visualize_rig_reconstruction(
            result.recon, window_name="pyTheia stereo rig"
        )
    return 0 if result.summary.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
