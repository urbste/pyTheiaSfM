#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Stereo / calibrated-rig reconstruction example.

Sets pinhole intrinsics and a stereo baseline (abstract body at identity,
left/right sensors offset along +X), matches with vismatch (default: xfeat),
builds ViewGraph + tracks, then runs GlobalRigReconstructor or
IncrementalRigReconstructor.

From a ZED extract folder (left/, right/, rig_calibration.json):
  python pyexamples/stereo/stereo_rig_reconstruction.py \\
    --frames_dir /home/steffen/Dokumente/ZED/test_frames \\
    --matcher xfeat --method global

Or pass dirs + calibration explicitly:
  python pyexamples/stereo/stereo_rig_reconstruction.py \\
    --left_dir /data/left --right_dir /data/right \\
    --baseline 0.12 --focal 700 --cx 640 --cy 360 \\
    --matcher xfeat --method global
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys


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
        default="xfeat",
        help="vismatch get_matcher name (default: xfeat sparse detector+descriptor)",
    )
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument(
        "--resize",
        type=int,
        default=1024,
        help="vismatch load_image longest-side resize",
    )
    p.add_argument(
        "--match_thresh",
        type=float,
        default=0.5,
        help="Dense-matcher confidence threshold (EDM MCONF_THR; ignored by xfeat)",
    )
    p.add_argument(
        "--max_keypoints",
        type=int,
        default=2048,
        help="Max keypoints for detector/descriptor matchers (xfeat)",
    )
    p.add_argument("--min_matches", type=int, default=40)
    p.add_argument(
        "--temporal_window",
        type=int,
        default=5,
        help="Match ±N frames same camera (larger → longer tracks with xfeat)",
    )
    p.add_argument(
        "--min_track_length",
        type=int,
        default=2,
        help="TrackBuilder min length (2 keeps pure stereo tracks)",
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


def _make_prior(pt, focal: float, cx: float, cy: float, width: int, height: int):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [float(focal)]
    prior.focal_length.is_set = True
    prior.principal_point.value = [float(cx), float(cy)]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [1.0]
    prior.aspect_ratio.is_set = True
    prior.image_width = int(width)
    prior.image_height = int(height)
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def _scale_kpts(kpts, full_wh, matched_hw):
    import numpy as np

    k = np.asarray(kpts, dtype=np.float64).reshape(-1, 2)
    fw, fh = full_wh
    mh, mw = matched_hw
    if mw <= 0 or mh <= 0:
        return k
    sx = fw / float(mw)
    sy = fh / float(mh)
    out = k.copy()
    out[:, 0] *= sx
    out[:, 1] *= sy
    return out


def _print_reprojection_stats(recon) -> None:
    """Mean L2 reprojection error: overall and per rig camera (sensor)."""
    import collections

    import numpy as np

    per_cam_sum: dict[int, float] = collections.defaultdict(float)
    per_cam_n: dict[int, int] = collections.defaultdict(int)
    cam_names: dict[int, str] = {}
    total_sum = 0.0
    total_n = 0

    for vid in recon.ViewIds():
        view = recon.View(vid)
        if view is None or not view.IsEstimated():
            continue
        cam_id = None
        if recon.ViewHasRigMembership(vid):
            memb = recon.GetViewRigMembership(vid)
            cam_id = int(memb.rig_camera_id)
            if cam_id not in cam_names:
                rig = recon.GetCameraRig(memb.rig_id)
                sensor = rig.GetSensor(memb.rig_camera_id) if rig is not None else None
                cam_names[cam_id] = (
                    sensor.name if sensor is not None else f"camera_{cam_id}"
                )
        for tid in view.TrackIds():
            track = recon.Track(tid)
            if track is None or not track.IsEstimated():
                continue
            feat = view.GetFeature(tid)
            if feat is None:
                continue
            # ProjectPoint → (depth, xy); match pyexamples/common/utils.py
            proj = view.Camera().ProjectPoint(track.Point())[1]
            err = float(np.linalg.norm(np.asarray(proj) - np.asarray(feat.point)))
            total_sum += err
            total_n += 1
            if cam_id is not None:
                per_cam_sum[cam_id] += err
                per_cam_n[cam_id] += 1

    if total_n == 0:
        print("Reprojection error: no estimated observations")
        return

    mean_total = total_sum / total_n
    print(f"Mean reprojection error (all): {mean_total:.4f} px  ({total_n} observations)")
    for cam_id in sorted(per_cam_n.keys()):
        n = per_cam_n[cam_id]
        mean = per_cam_sum[cam_id] / n
        name = cam_names.get(cam_id, f"camera_{cam_id}")
        print(f"  {name}: {mean:.4f} px  ({n} observations)")


def _correspondences_from_result(
    pt, result, full_wh_a, full_wh_b, matched_hw_a, matched_hw_b, min_n
):
    import numpy as np

    # Prefer pre-RANSAC matches: vismatch's inlier_* are from a homography model,
    # which is wrong for general stereo/3D scenes. EstimateTwoViewInfo does E/F RANSAC.
    k0 = result.get("matched_kpts0")
    k1 = result.get("matched_kpts1")
    if k0 is None or k1 is None or np.asarray(k0).size == 0:
        k0 = result.get("inlier_kpts0")
        k1 = result.get("inlier_kpts1")
    if k0 is None or k1 is None:
        return False, []
    k0 = _scale_kpts(k0, full_wh_a, matched_hw_a)
    k1 = _scale_kpts(k1, full_wh_b, matched_hw_b)
    n = min(len(k0), len(k1))
    if n < min_n:
        return False, []
    cors = []
    for i in range(n):
        cors.append(
            pt.matching.FeatureCorrespondence(
                pt.sfm.Feature(k0[i]), pt.sfm.Feature(k1[i])
            )
        )
    return True, cors


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
        from vismatch import get_matcher
    except ImportError:
        print(
            "vismatch is required for this example. Install with:\n"
            '  pip install "pytheia[examples]"   # or: pip install vismatch',
            file=sys.stderr,
        )
        return 1

    import cv2
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
    left_paths, right_paths = left_paths[:n], right_paths[:n]
    print(
        f"Using {n} pairs from\n  left:  {args.left_dir}\n  right: {args.right_dir}\n"
        f"  baseline={args.baseline:.6f} m  focal={args.focal:.3f}  "
        f"cx={args.cx:.3f} cy={args.cy:.3f}"
    )

    im0 = cv2.imread(left_paths[0])
    if im0 is None:
        print(f"Failed to read {left_paths[0]}", file=sys.stderr)
        return 1
    h0, w0 = im0.shape[:2]
    width = args.width or w0
    height = args.height or h0
    prior = _make_prior(pt, args.focal, args.cx, args.cy, width, height)

    # Abstract body at identity; left/right offset symmetrically along +X.
    half_b = 0.5 * float(args.baseline)
    rig = pt.sfm.CameraRig("stereo")
    left_id = rig.AddSensor("left", np.array([-half_b, 0.0, 0.0]), np.zeros(3))
    right_id = rig.AddSensor("right", np.array([half_b, 0.0, 0.0]), np.zeros(3))

    recon = pt.sfm.Reconstruction()
    rig_id = recon.AddCameraRig(rig)
    view_graph = pt.sfm.ViewGraph()
    # min_track_length=2 keeps pure left↔right stereo tracks (needed for metric scale).
    track_builder = pt.sfm.TrackBuilder(int(args.min_track_length), 30)

    captures = []
    left_views = []
    right_views = []
    for i in range(n):
        t = float(i)
        cap = recon.AddRigCapture(
            rig_id,
            t,
            {
                left_id: f"L_{i:05d}_{os.path.basename(left_paths[i])}",
                right_id: f"R_{i:05d}_{os.path.basename(right_paths[i])}",
            },
        )
        captures.append(cap)
        capture = recon.GetRigCapture(cap)
        vl = capture.ViewIdForCamera(left_id)
        vr = capture.ViewIdForCamera(right_id)
        left_views.append(vl)
        right_views.append(vr)
        for vid in (vl, vr):
            view = recon.MutableView(vid)
            view.SetCameraIntrinsicsPrior(prior)
            cam = view.MutableCamera()
            cam.SetFromCameraIntrinsicsPriors(prior)

    print(
        f"Added {n} stereo captures ({2 * n} views). "
        f"Loading matcher={args.matcher} resize={args.resize} "
        f"thresh={args.match_thresh} max_keypoints={args.max_keypoints}..."
    )
    matcher = get_matcher(
        args.matcher,
        device=args.device,
        max_num_keypoints=args.max_keypoints,
        thresh=args.match_thresh,
    )
    _img_cache: dict = {}

    def load_pair(path):
        ap = os.path.abspath(path)
        if ap in _img_cache:
            return _img_cache[ap]
        # load_image is a BaseMatcher staticmethod, not a top-level vismatch export
        tensor = matcher.load_image(ap, resize=args.resize)
        img = cv2.imread(ap)
        h, w = img.shape[:2]
        if hasattr(tensor, "shape") and len(tensor.shape) >= 2:
            sh = tuple(int(x) for x in tensor.shape)
            if sh[0] in (1, 3) and len(sh) == 3:
                mh, mw = sh[1], sh[2]
            else:
                mh, mw = sh[0], sh[1]
        else:
            mh, mw = h, w
        out = (tensor, (w, h), (mh, mw))
        _img_cache[ap] = out
        return out

    def match_and_add(path_a, path_b, view_a, view_b, label: str) -> bool:
        ta, full_a, matched_a = load_pair(path_a)
        tb, full_b, matched_b = load_pair(path_b)
        result = matcher(ta, tb)
        ok, cors = _correspondences_from_result(
            pt, result, full_a, full_b, matched_a, matched_b, args.min_matches
        )
        if not ok:
            return False
        opts = pt.sfm.EstimateTwoViewInfoOptions()
        opts.ransac_type = pt.sfm.RansacType(0)
        opts.use_lo = True
        opts.use_mle = True
        opts.max_sampson_error_pixels = 2.0
        ok2, twoview_info, inlier_idx = pt.sfm.EstimateTwoViewInfo(
            opts, prior, prior, cors
        )
        if not ok2 or len(inlier_idx) < args.min_matches:
            return False
        twoview_info.num_verified_matches = len(inlier_idx)
        view_graph.AddEdge(view_a, view_b, twoview_info)
        for i in inlier_idx:
            c = cors[i]
            track_builder.AddFeatureCorrespondence(
                view_a, c.feature1, view_b, c.feature2
            )
        print(f"  {label}: {len(inlier_idx)} inliers")
        return True

    # Stereo pairs at each timestamp + temporal same-camera links.
    for i in range(n):
        match_and_add(
            left_paths[i],
            right_paths[i],
            left_views[i],
            right_views[i],
            f"stereo t={i}",
        )
        for dt in range(1, args.temporal_window + 1):
            j = i + dt
            if j >= n:
                break
            match_and_add(
                left_paths[i],
                left_paths[j],
                left_views[i],
                left_views[j],
                f"left {i}-{j}",
            )
            match_and_add(
                right_paths[i],
                right_paths[j],
                right_views[i],
                right_views[j],
                f"right {i}-{j}",
            )

    track_builder.BuildTracks(recon)
    print(
        f"ViewGraph edges={view_graph.NumEdges()}, "
        f"tracks={recon.NumTracks()}, views={recon.NumViews()}"
    )

    # Release GPU matcher before Ceres BA: Torch CUDA + multi-threaded Ceres
    # has been observed to SIGSEGV on this machine.
    del matcher
    _img_cache.clear()
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
    except Exception:
        pass

    if not args.out_reconstruction and args.frames_dir:
        args.out_reconstruction = os.path.join(
            os.path.abspath(args.frames_dir), "stereo_rig.recon"
        )

    if args.method == "global":
        gro = pt.sfm.GlobalRigReconstructorOptions()
        gro.sfm_options.global_rotation_estimator_type = getattr(
            pt.sfm.GlobalRotationEstimatorType, args.rotation_estimator
        )
        gro.sfm_options.global_position_estimator_type = getattr(
            pt.sfm.GlobalPositionEstimatorType, args.position_estimator
        )
        # Indoor / short-baseline stereo: allow smaller triangulation angles.
        gro.sfm_options.min_triangulation_angle_degrees = 0.5
        gro.sfm_options.triangulation_max_reprojection_error_in_pixels = 6.0
        gro.sfm_options.num_retriangulation_iterations = args.retriangulation_iterations
        gro.sfm_options.num_threads = 1
        if hasattr(gro, "rescale_positions_to_metric_edges"):
            gro.rescale_positions_to_metric_edges = True
        summary = pt.sfm.GlobalRigReconstructor(gro).Estimate(view_graph, recon)
    else:
        iro = pt.sfm.IncrementalRigReconstructorOptions()
        iro.min_triangulation_angle_degrees = 0.5
        iro.max_reprojection_error_in_pixels = 6.0
        # Avoid OpenMP/Ceres crash after CUDA matching (see above).
        iro.ba_options.num_threads = 1
        iro.ba_options.use_inner_iterations = False
        summary = pt.sfm.IncrementalRigReconstructor(iro).Estimate(view_graph, recon)

    print(
        f"success={summary.success} views={len(summary.estimated_views)} "
        f"tracks={len(summary.estimated_tracks)} msg={summary.message}"
    )
    if summary.success:
        _print_reprojection_stats(recon)
    if args.out_reconstruction and summary.success:
        pt.io.WriteReconstruction(recon, args.out_reconstruction)
        print(f"Wrote {args.out_reconstruction}")
        ply_path = os.path.splitext(args.out_reconstruction)[0] + ".ply"
        pt.io.WriteRigPlyFile(
            ply_path,
            recon,
            sensor_color=np.array([255, 0, 0], dtype=np.int32),
            min_num_observations_per_point=2,
        )
        print(f"Wrote {ply_path} (tracks + capture trajectory + sensor baselines)")
    return 0 if summary.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
