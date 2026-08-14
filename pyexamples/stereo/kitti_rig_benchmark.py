#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Benchmark the calibrated stereo-rig pipeline on KITTI Odometry sequences.

Expects the standard odometry layout (gray stereo by default)::

    <kitti_root>/
      sequences/00/image_0/*.png
      sequences/00/image_1/*.png
      sequences/00/calib.txt
      poses/00.txt                 # optional, sequences 00–10

Ground-truth poses (when present) are left-camera 3×4 [R|t] transforming
camera → world. Evaluation reports:

  * Sim(3) ATE after Umeyama (rotation, translation, scale)
  * SE(3) ATE with scale fixed at 1 (stereo baseline should pin metric scale)
  * relative pose error (translation / rotation) at Δ=1 and Δ=10 frames
  * reconstructed vs calibrated stereo baseline

Examples::

  python pyexamples/stereo/kitti_rig_benchmark.py \\
    --kitti_root /data/kitti/odometry --sequences 04 --cameras color

  python pyexamples/stereo/kitti_rig_benchmark.py \\
    --kitti_root /data/kitti/odometry --sequences 00 --max_frames 200

Matching defaults to a short temporal window (forward motion only; no loop
closures). Use ``--left_match_mode cascade`` for exhaustive later-frame pairs.
Pass ``--trajectory_has_loops`` on sequences that revisit places (KITTI 00):
that precomputes CosPlace descriptors and matches GraphMatch loop candidates.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

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
    left_camera_trajectory,
    run_calibrated_stereo_rig,
)
from common.rig_open3d import visualize_rig_reconstruction  # noqa: E402
from kitti_odometry_eval import (  # noqa: E402
    export_left_camera_poses,
    mgsfm_metrics_for_sequence,
    write_kitti_poses_file,
)


def parse_kitti_calib(calib_path: str, cameras: str = "gray") -> StereoCalib:
    """Parse KITTI odometry ``calib.txt`` (P0/P1 gray or P2/P3 color)."""
    keys = ("P0", "P1") if cameras == "gray" else ("P2", "P3")
    mats: dict[str, np.ndarray] = {}
    with open(calib_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            name, rest = line.split(":", 1)
            name = name.strip()
            vals = [float(x) for x in rest.split()]
            if len(vals) >= 12:
                mats[name] = np.array(vals[:12], dtype=np.float64).reshape(3, 4)
    if keys[0] not in mats or keys[1] not in mats:
        raise ValueError(
            f"{calib_path} missing {keys[0]}/{keys[1]} "
            f"(found {sorted(mats.keys())})"
        )
    p_left, p_right = mats[keys[0]], mats[keys[1]]
    fx, fy = float(p_left[0, 0]), float(p_left[1, 1])
    cx, cy = float(p_left[0, 2]), float(p_left[1, 2])
    if abs(fx) < 1e-9:
        raise ValueError(f"Invalid fx in {keys[0]} of {calib_path}")
    # KITTI rectified stereo: P_right[0, 3] = -fx * baseline (left-origin).
    baseline = abs(float(p_right[0, 3]) / fx)
    if baseline < 1e-4:
        raise ValueError(f"Degenerate baseline from {keys[1]} in {calib_path}")
    aspect = fy / fx if abs(fx) > 1e-12 else 1.0
    return StereoCalib(
        focal=fx,
        cx=cx,
        cy=cy,
        baseline=baseline,
        aspect_ratio=aspect,
        left_in_rig=np.array([0.0, 0.0, 0.0]),
        right_in_rig=np.array([baseline, 0.0, 0.0]),
    )


def load_kitti_poses(path: str) -> np.ndarray:
    """Load KITTI pose file: N×12 row-major 3×4, camera→world. Returns N×4×4."""
    rows = np.loadtxt(path)
    if rows.ndim == 1:
        rows = rows.reshape(1, -1)
    if rows.shape[1] < 12:
        raise ValueError(f"Expected 12 columns in {path}, got {rows.shape[1]}")
    Ts = np.repeat(np.eye(4)[None, ...], rows.shape[0], axis=0)
    Ts[:, :3, :] = rows[:, :12].reshape(-1, 3, 4)
    return Ts


def kitti_camera_centers(T_w_cam: np.ndarray) -> np.ndarray:
    """Camera centers from KITTI camera→world poses (translation column)."""
    return np.asarray(T_w_cam[:, :3, 3], dtype=np.float64)


def align_se3(src: np.ndarray, dst: np.ndarray):
    """Rigid alignment dst ≈ R @ src + t (scale = 1)."""
    src = np.asarray(src, dtype=np.float64).reshape(-1, 3)
    dst = np.asarray(dst, dtype=np.float64).reshape(-1, 3)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    u, _, vt = np.linalg.svd(H)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1.0
        r = vt.T @ u.T
    t = mu_d - r @ mu_s
    return r, t


def apply_sim3(xyz: np.ndarray, r, t, scale: float) -> np.ndarray:
    return scale * (np.asarray(xyz) @ np.asarray(r).T) + np.asarray(t)


def apply_se3(xyz: np.ndarray, r, t) -> np.ndarray:
    return np.asarray(xyz) @ np.asarray(r).T + np.asarray(t)


def ate_stats(aligned: np.ndarray, gt: np.ndarray) -> dict:
    err = np.linalg.norm(aligned - gt, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "median": float(np.median(err)),
        "max": float(np.max(err)),
        "n": int(err.size),
    }


def rpe_stats(est: np.ndarray, gt: np.ndarray, delta: int) -> dict:
    """Relative translation error (meters) between frames i and i+delta."""
    if est.shape[0] <= delta:
        return {"rmse": float("nan"), "mean": float("nan"), "n": 0, "delta": delta}
    d_est = np.linalg.norm(est[delta:] - est[:-delta], axis=1)
    d_gt = np.linalg.norm(gt[delta:] - gt[:-delta], axis=1)
    err = np.abs(d_est - d_gt)
    return {
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "n": int(err.size),
        "delta": int(delta),
    }


def discover_sequence(sequence_dir: str, cameras: str) -> tuple[str, str, str]:
    left_name, right_name = (
        ("image_0", "image_1") if cameras == "gray" else ("image_2", "image_3")
    )
    left = os.path.join(sequence_dir, left_name)
    right = os.path.join(sequence_dir, right_name)
    calib = os.path.join(sequence_dir, "calib.txt")
    if not os.path.isdir(left) or not os.path.isdir(right):
        raise FileNotFoundError(
            f"Expected {left_name}/ and {right_name}/ under {sequence_dir}"
        )
    if not os.path.isfile(calib):
        raise FileNotFoundError(f"Missing {calib}")
    return left, right, calib


def find_pose_file(kitti_root: str, sequence_dir: str, seq: str) -> str:
    candidates = [
        os.path.join(kitti_root, "poses", f"{seq}.txt") if kitti_root else "",
        os.path.join(sequence_dir, "poses.txt"),
        os.path.join(sequence_dir, f"{seq}.txt"),
        os.path.join(os.path.dirname(sequence_dir), "..", "poses", f"{seq}.txt"),
    ]
    for path in candidates:
        if path and os.path.isfile(os.path.abspath(path)):
            return os.path.abspath(path)
    return ""


def list_images(folder: str) -> list[str]:
    paths = sorted(glob.glob(os.path.join(folder, "*.png")))
    if not paths:
        paths = sorted(glob.glob(os.path.join(folder, "*.jpg")))
    return paths


def evaluate_against_gt(
    est_xyz: np.ndarray,
    gt_xyz: np.ndarray,
) -> dict:
    import pytheia as pt

    if est_xyz.shape[0] < 3:
        return {"error": "need ≥3 estimated cameras for alignment"}
    left = [est_xyz[i] for i in range(est_xyz.shape[0])]
    right = [gt_xyz[i] for i in range(gt_xyz.shape[0])]
    r_s, t_s, scale = pt.sfm.AlignPointCloudsUmeyama(left, right)
    aligned_sim3 = apply_sim3(est_xyz, r_s, t_s, float(scale))
    r_e, t_e = align_se3(est_xyz, gt_xyz)
    aligned_se3 = apply_se3(est_xyz, r_e, t_e)
    return {
        "sim3_scale": float(scale),
        "sim3_ate": ate_stats(aligned_sim3, gt_xyz),
        "se3_ate": ate_stats(aligned_se3, gt_xyz),
        "rpe_se3_d1": rpe_stats(aligned_se3, gt_xyz, 1),
        "rpe_se3_d10": rpe_stats(aligned_se3, gt_xyz, 10),
        "rpe_sim3_d1": rpe_stats(aligned_sim3, gt_xyz, 1),
        "aligned_sim3": aligned_sim3,
        "aligned_se3": aligned_se3,
        "gt_xyz": gt_xyz,
    }


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="KITTI odometry benchmark for the calibrated stereo-rig pipeline"
    )
    p.add_argument(
        "--kitti_root",
        type=str,
        default="",
        help="Odometry dataset root containing sequences/ and optional poses/",
    )
    p.add_argument(
        "--sequences",
        type=str,
        default="00",
        help="Comma-separated sequence ids (default: 00)",
    )
    p.add_argument(
        "--sequence_dir",
        type=str,
        default="",
        help="Single sequence folder (image_0/, image_1/, calib.txt)",
    )
    p.add_argument(
        "--cameras",
        choices=("gray", "color"),
        default="gray",
        help="gray=image_0/1 + P0/P1, color=image_2/3 + P2/P3",
    )
    p.add_argument("--start_frame", type=int, default=0)
    p.add_argument("--max_frames", type=int, default=0, help="0 = all frames")
    p.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Subsample the sequence (and GT) before anything else: keep every Nth frame.",
    )
    p.add_argument(
        "--recon_stride",
        type=int,
        default=1,
        help=(
            "After loading frames, reconstruct every Nth pair and PnP-localize "
            "the rest. Unlike --stride, skipped frames stay in the trajectory."
        ),
    )
    p.add_argument(
        "--pnp_min_inliers",
        type=int,
        default=30,
        help="Minimum PnP inliers to accept a skipped-capture pose.",
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
        "--matcher",
        type=str,
        default="disk-lightglue",
        choices=LIGHTGLUE_MATCHERS,
    )
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument("--resize", type=int, default=960)
    p.add_argument(
        "--resize_mode",
        choices=RESIZE_MODES,
        default="width",
        help="width: scale to --resize px wide; max: longest-side resize",
    )
    p.add_argument("--match_thresh", type=float, default=0.5)
    p.add_argument("--max_keypoints", type=int, default=2048)
    p.add_argument("--min_matches", type=int, default=40)
    p.add_argument("--max_sampson_error", type=float, default=2.0)
    p.add_argument(
        "--temporal_window",
        type=int,
        default=10,
        help=(
            "Forward-motion window: match left_i to i+1..i+N (default 10). "
            "Used when --left_match_mode=window."
        ),
    )
    p.add_argument(
        "--loop_stride",
        type=int,
        default=0,
        help=(
            "Also match left_i to i+k*stride for loop closure (0 = off; "
            "KITTI odometry is forward-only)."
        ),
    )
    p.add_argument(
        "--trajectory_has_loops",
        action="store_true",
        help=(
            "Precompute CosPlace (ResNet18/128) on left keyframes and match "
            "GraphMatch loop candidates. Use on long trajectories that revisit "
            "places (e.g. KITTI 00). Off by default."
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
        "--left_match_mode",
        choices=("cascade", "window"),
        default="window",
        help=(
            "window: nearby frames only (default for forward odometry). "
            "cascade: left_i vs later left_j (use --left_match_max_gap on long seqs)."
        ),
    )
    p.add_argument(
        "--left_match_max_gap",
        type=int,
        default=0,
        help=(
            "Cascade only: match i to i+1..i+gap (0 = all later frames). "
            "Ignored in window mode."
        ),
    )
    p.add_argument(
        "--match_right_temporal",
        action="store_true",
        help="Also match right–right with the same pair schedule.",
    )
    p.add_argument("--cross_sensor_temporal", action="store_true")
    p.add_argument("--min_track_length", type=int, default=2)
    p.add_argument("--min_track_captures", type=int, default=3)
    p.add_argument("--stereo_snap_pixels", type=float, default=3.0)
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
    p.add_argument("--feature_cache_dir", type=str, default="")
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
        "--write_kitti_poses",
        action="store_true",
        help="Write estimated poses in KITTI 3x4 format to out_dir.",
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
    p.add_argument("--method", choices=("global", "incremental"), default="global")
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
    p.add_argument("--verbose_matches", action="store_true")
    p.add_argument(
        "--match_debug_dir",
        type=str,
        default="",
        help=(
            "Write side-by-side PNGs + left_matches.jsonl for each left–left "
            "temporal pair (green=RANSAC inliers, gray=raw matches)."
        ),
    )
    p.add_argument(
        "--match_debug_max_pairs",
        type=int,
        default=0,
        help="Cap debug PNG count (0 = all scheduled left–left pairs).",
    )
    p.add_argument(
        "--match_debug_only",
        action="store_true",
        help="Run left–left matching (+ optional debug dump) and skip SfM/BA.",
    )
    p.add_argument("--visualize", action="store_true", help="Open3D window if installed")
    p.add_argument("--out_dir", type=str, default="")
    p.add_argument("--out_json", type=str, default="")
    return p.parse_args()


def _run_options(args: argparse.Namespace) -> StereoRigRunOptions:
    return StereoRigRunOptions(
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


def run_sequence(
    seq: str,
    sequence_dir: str,
    kitti_root: str,
    args: argparse.Namespace,
) -> dict:
    import pytheia as pt

    from calibrated_stereo_rig import mean_stereo_baseline

    left_dir, right_dir, calib_path = discover_sequence(sequence_dir, args.cameras)
    calib = parse_kitti_calib(calib_path, cameras=args.cameras)
    left_paths = list_images(left_dir)
    right_paths = list_images(right_dir)
    n = min(len(left_paths), len(right_paths))
    if n < 2:
        raise FileNotFoundError(
            f"Sequence {seq}: need ≥2 images (left={len(left_paths)} right={len(right_paths)})"
        )

    start = max(0, args.start_frame)
    stride = max(1, args.stride)
    indices = list(range(start, n, stride))
    if args.max_frames > 0:
        indices = indices[: args.max_frames]
    left_paths = [left_paths[i] for i in indices]
    right_paths = [right_paths[i] for i in indices]
    print(
        f"\n=== KITTI {seq}  cameras={args.cameras}  "
        f"frames={len(indices)} (start={start} stride={stride})  "
        f"baseline={calib.baseline:.4f} m ==="
    )

    result = run_calibrated_stereo_rig(
        left_paths, right_paths, calib, _run_options(args)
    )
    metrics: dict = {
        "sequence": seq,
        "n_frames": len(indices),
        "frame_indices": indices,
        "calib_baseline_m": calib.baseline,
        "success": bool(result.summary.success),
        "n_estimated_views": len(result.summary.estimated_views),
        "n_estimated_tracks": len(result.summary.estimated_tracks),
        "message": result.summary.message,
        "match_counts": result.match_counts,
    }
    if result.match_counts.get("match_debug_dir"):
        metrics["match_debug_dir"] = result.match_counts["match_debug_dir"]
        metrics["debug_pairs_written"] = result.match_counts.get(
            "debug_pairs_written", 0
        )
    if result.summary.success:
        metrics["mean_baseline_m"] = mean_stereo_baseline(
            result.recon, result.left_view_ids, result.right_view_ids
        )

    pose_file = find_pose_file(kitti_root, sequence_dir, seq)
    aligned_se3 = None
    gt_used = None
    if pose_file and result.summary.success:
        T_gt = load_kitti_poses(pose_file)
        est_xyz, est_idx = left_camera_trajectory(
            result.recon, result.left_view_ids
        )
        # est_idx is index into the *subsampled* list; map to original frame ids.
        orig_idx = [indices[i] for i in est_idx]
        keep = [j for j, fi in enumerate(orig_idx) if fi < T_gt.shape[0]]
        if len(keep) >= 3:
            est_xyz = est_xyz[keep]
            orig_idx = [orig_idx[j] for j in keep]
            gt_xyz = kitti_camera_centers(T_gt)[orig_idx]
            ev = evaluate_against_gt(est_xyz, gt_xyz)
            aligned_se3 = ev.pop("aligned_se3", None)
            ev.pop("aligned_sim3", None)
            gt_used = ev.pop("gt_xyz", None)
            metrics["gt_poses"] = pose_file
            metrics["n_eval_poses"] = len(orig_idx)
            metrics["sim3_scale"] = ev["sim3_scale"]
            metrics["sim3_ate"] = ev["sim3_ate"]
            metrics["se3_ate"] = ev["se3_ate"]
            metrics["rpe_se3_d1"] = ev["rpe_se3_d1"]
            metrics["rpe_se3_d10"] = ev["rpe_se3_d10"]
            print(
                f"GT poses: {pose_file}  ({len(orig_idx)} cameras)\n"
                f"  Sim3 scale={ev['sim3_scale']:.4f}  "
                f"ATE RMSE={ev['sim3_ate']['rmse']:.4f} m\n"
                f"  SE3  ATE RMSE={ev['se3_ate']['rmse']:.4f} m  "
                f"(scale fixed; stereo should be ~1×)\n"
                f"  RPE Δ=1  RMSE={ev['rpe_se3_d1']['rmse']:.4f} m  "
                f"Δ=10 RMSE={ev['rpe_se3_d10']['rmse']:.4f} m"
            )
            mgsfm = mgsfm_metrics_for_sequence(
                result.recon,
                result.left_view_ids,
                result.right_view_ids,
                T_gt,
                calib.baseline,
                frame_indices=indices,
            )
            if mgsfm.get("n", 0) > 0 or mgsfm.get("sequence_n", 0) > 0:
                metrics["mgsfm_n"] = int(
                    mgsfm.get("sequence_n", mgsfm.get("n", 0))
                )
                metrics["mgsfm_er_median"] = mgsfm.get("er_median")
                metrics["mgsfm_er_mean"] = mgsfm.get("er_mean")
                metrics["mgsfm_et_median"] = mgsfm.get("et_median")
                metrics["mgsfm_et_mean"] = mgsfm.get("et_mean")
                if "sim3_scale" in mgsfm:
                    metrics["mgsfm_sim3_scale"] = mgsfm["sim3_scale"]
                print(
                    f"  MGSfM metrics: N={metrics['mgsfm_n']}  "
                    f"e~r={metrics['mgsfm_er_median']:.2f}/{metrics['mgsfm_er_mean']:.2f} deg  "
                    f"e~t={metrics['mgsfm_et_median']:.2f}/{metrics['mgsfm_et_mean']:.2f} m"
                )
            elif "error" in mgsfm:
                print(f"  MGSfM metrics skipped: {mgsfm['error']}")
        else:
            print(f"Not enough overlapping GT poses in {pose_file}")
    elif not pose_file:
        print("No GT pose file found (reconstruction-only)")

    out_dir = args.out_dir
    if not out_dir and kitti_root:
        out_dir = os.path.join(os.path.abspath(kitti_root), "pytheia_rig", seq)
    elif not out_dir:
        out_dir = os.path.join(os.path.abspath(sequence_dir), "pytheia_rig")
    if result.summary.success and out_dir:
        os.makedirs(out_dir, exist_ok=True)
        recon_path = os.path.join(out_dir, f"{seq}_stereo_rig.recon")
        ply_path = os.path.join(out_dir, f"{seq}_stereo_rig.ply")
        pt.io.WriteReconstruction(result.recon, recon_path)
        pt.io.WriteRigPlyFile(
            ply_path,
            result.recon,
            sensor_color=np.array([255, 0, 0], dtype=np.int32),
            min_num_observations_per_point=2,
        )
        print(f"Wrote {recon_path}\nWrote {ply_path}")
        metrics["recon"] = recon_path
        metrics["ply"] = ply_path
        if args.write_kitti_poses:
            pose_out = os.path.join(out_dir, f"{seq}.txt")
            poses = export_left_camera_poses(
                result.recon, result.left_view_ids, indices
            )
            write_kitti_poses_file(pose_out, poses)
            metrics["kitti_poses"] = pose_out
            print(f"Wrote {pose_out} ({len(poses)} left-camera poses)")

    if args.visualize and result.summary.success:
        visualize_rig_reconstruction(
            result.recon,
            window_name=f"KITTI {seq} rig",
            gt_xyz=gt_used,
            aligned_est_xyz=aligned_se3,
        )
    return metrics


def main() -> int:
    args = _parse_args()
    jobs: list[tuple[str, str]] = []
    if args.sequence_dir:
        seq_dir = os.path.abspath(args.sequence_dir)
        seq = os.path.basename(seq_dir.rstrip(os.sep))
        root = os.path.abspath(args.kitti_root) if args.kitti_root else os.path.dirname(
            os.path.dirname(seq_dir)
        )
        jobs.append((seq, seq_dir))
        kitti_root = root
    else:
        if not args.kitti_root:
            print("Pass --kitti_root or --sequence_dir", file=sys.stderr)
            return 1
        kitti_root = os.path.abspath(args.kitti_root)
        seqs_root = os.path.join(kitti_root, "sequences")
        for seq in [s.strip() for s in args.sequences.split(",") if s.strip()]:
            jobs.append((seq, os.path.join(seqs_root, seq)))

    all_metrics = []
    for seq, seq_dir in jobs:
        if not os.path.isdir(seq_dir):
            print(f"Skip {seq}: missing {seq_dir}", file=sys.stderr)
            continue
        try:
            metrics = run_sequence(seq, seq_dir, kitti_root, args)
        except (FileNotFoundError, ValueError, OSError) as exc:
            print(f"Sequence {seq} failed: {exc}", file=sys.stderr)
            all_metrics.append({"sequence": seq, "success": False, "error": str(exc)})
            continue
        all_metrics.append(
            {k: v for k, v in metrics.items() if k not in ("frame_indices",)}
        )

    if args.out_json and all_metrics:
        out_json = os.path.abspath(args.out_json)
        os.makedirs(os.path.dirname(out_json) or ".", exist_ok=True)
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(all_metrics, f, indent=2)
        print(f"Wrote {out_json}")

    if args.match_debug_only:
        n_debug = sum(int(m.get("debug_pairs_written", 0)) for m in all_metrics)
        return 0 if n_debug else 2

    if args.match_cache_only:
        n_cached = sum(
            int(m.get("match_counts", {}).get("match_cache_writes", 0))
            for m in all_metrics
        )
        return 0 if n_cached or all_metrics else 2

    n_ok = sum(1 for m in all_metrics if m.get("success"))
    return 0 if n_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
