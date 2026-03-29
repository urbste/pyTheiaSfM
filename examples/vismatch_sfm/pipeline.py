#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Match images with vismatch, build a view graph with pyTheia, run SfM.

Heavy imports (vismatch, torch) load only when running the pipeline, not for --help.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="vismatch correspondences + pyTheia global/incremental/hybrid SfM"
    )
    p.add_argument("--image_dir", type=str, required=True)
    p.add_argument("--img_ext", type=str, default="jpg")
    p.add_argument("--matcher", type=str, default="superpoint-lightglue")
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument("--resize", type=int, default=512, help="vismatch load_image resize (pixels)")
    p.add_argument(
        "--reconstruction",
        type=str,
        default="incremental",
        choices=("global", "incremental", "hybrid"),
    )
    p.add_argument("--min_inliers_twoview", type=int, default=30)
    p.add_argument("--num_threads", type=int, default=4)
    p.add_argument(
        "--focal_length",
        type=float,
        default=None,
        help="Pinhole focal length in pixels; default 1.2 * max(w,h)",
    )
    p.add_argument("--output_dir", type=str, default=None)
    p.add_argument("--rerun", action="store_true", help="Open Rerun after reconstruction")
    return p.parse_args()


def _default_prior(w: int, h: int, focal: float) -> "object":
    import pytheia as pt

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [float(focal)]
    prior.aspect_ratio.value = [1.0]
    prior.principal_point.value = [w / 2.0, h / 2.0]
    prior.image_width = w
    prior.image_height = h
    prior.skew.value = [0.0]
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def _match_pair_vismatch(
    matcher,
    path_a: str,
    path_b: str,
    resize: int,
    min_putative: int,
):
    """Returns (success, list of FeatureCorrespondence) or (False, None)."""
    import numpy as np
    import pytheia as pt

    im_a = matcher.load_image(path_a, resize=resize)
    im_b = matcher.load_image(path_b, resize=resize)
    result = matcher(im_a, im_b)

    k0 = result.get("inlier_kpts0")
    k1 = result.get("inlier_kpts1")
    if k0 is None or k1 is None:
        k0 = result.get("matched_kpts0")
        k1 = result.get("matched_kpts1")
    if k0 is None or k1 is None:
        return False, None

    k0 = np.asarray(k0)
    k1 = np.asarray(k1)
    n = min(len(k0), len(k1))
    if n < min_putative:
        return False, None

    correspondences = []
    for i in range(n):
        p0 = np.asarray(k0[i]).reshape(2).astype(float)
        p1 = np.asarray(k1[i]).reshape(2).astype(float)
        correspondences.append(
            pt.matching.FeatureCorrespondence(
                pt.sfm.Feature(p0), pt.sfm.Feature(p1)
            )
        )
    return True, correspondences


def _estimate_twoview_and_tracks(
    prior,
    correspondences,
    track_builder,
    view_id_a,
    view_id_b,
    min_inliers: int,
):
    import pytheia as pt

    opts = pt.sfm.EstimateTwoViewInfoOptions()
    opts.ransac_type = pt.sfm.RansacType(0)
    opts.use_lo = True
    opts.use_mle = True
    opts.max_sampson_error_pixels = 2.0

    ok, twoview_info, inlier_idx = pt.sfm.EstimateTwoViewInfo(
        opts, prior, prior, correspondences
    )
    if not ok or len(inlier_idx) < min_inliers:
        return False, None

    twoview_info.num_verified_matches = len(inlier_idx)
    for i in inlier_idx:
        c = correspondences[i]
        track_builder.AddFeatureCorrespondence(
            view_id_a, c.feature1, view_id_b, c.feature2
        )
    return True, twoview_info


def main() -> int:
    args = _parse_args()

    try:
        from vismatch import get_matcher
    except ImportError:
        print("Install vismatch: pip install vismatch  (or pip install '.[examples]')", file=sys.stderr)
        return 1

    import cv2
    import numpy as np
    import pytheia as pt

    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    import rerun_log as _rerun_log

    img_dir = os.path.abspath(args.image_dir)
    pattern = os.path.join(img_dir, f"*.{args.img_ext}")
    image_paths = sorted(glob.glob(pattern))
    if len(image_paths) < 2:
        print(f"No images matching {pattern}", file=sys.stderr)
        return 1

    out_dir = args.output_dir or img_dir
    os.makedirs(out_dir, exist_ok=True)

    # Image size & intrinsics from first image
    sample = cv2.imread(image_paths[0])
    if sample is None:
        print(f"Could not read {image_paths[0]}", file=sys.stderr)
        return 1
    h, w = sample.shape[:2]
    focal = args.focal_length if args.focal_length else 1.2 * float(max(w, h))
    prior = _default_prior(w, h, focal)

    matcher = get_matcher(args.matcher, device=args.device)

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(4, 30)

    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(prior)

    # Register views (names = basenames with extension)
    for idx, pth in enumerate(image_paths):
        name = os.path.basename(pth)
        vid = recon.AddView(name, 0, idx)
        recon.MutableView(vid).SetCameraIntrinsicsPrior(prior)

    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

    view_ids = sorted(recon.ViewIds())
    n = len(view_ids)
    print(f"Matching {n} views ({n * (n - 1) // 2} pairs) with {args.matcher} …")

    for i in range(n):
        vi = view_ids[i]
        path_i = os.path.join(img_dir, recon.View(vi).Name())
        for j in range(i + 1, n):
            vj = view_ids[j]
            path_j = os.path.join(img_dir, recon.View(vj).Name())
            ok_c, cors = _match_pair_vismatch(
                matcher,
                path_i,
                path_j,
                args.resize,
                min_putative=args.min_inliers_twoview,
            )
            if not ok_c:
                print(f"  skip {recon.View(vi).Name()} — {recon.View(vj).Name()} (few matches)")
                continue
            ok_t, tv = _estimate_twoview_and_tracks(
                prior,
                cors,
                track_builder,
                vi,
                vj,
                args.min_inliers_twoview,
            )
            if not ok_t:
                print(f"  reject {recon.View(vi).Name()} — {recon.View(vj).Name()} (geom verify)")
                continue
            view_graph.AddEdge(vi, vj, tv)
            print(f"  edge {recon.View(vi).Name()} — {recon.View(vj).Name()}")

    print(f"View graph edges: {view_graph.NumEdges()}")
    if view_graph.NumEdges() < 1:
        print("Not enough edges for reconstruction.", file=sys.stderr)
        return 1

    track_builder.BuildTracks(recon)

    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = args.num_threads
    options.rotation_filtering_max_difference_degrees = 20.0
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType.HUBER
    options.subsample_tracks_for_bundle_adjustment = False

    if args.reconstruction == "global":
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
        est = pt.sfm.GlobalReconstructionEstimator(options)
    elif args.reconstruction == "hybrid":
        est = pt.sfm.HybridReconstructionEstimator(options)
    else:
        est = pt.sfm.IncrementalReconstructionEstimator(options)

    summary = est.Estimate(view_graph, recon)
    print("Reconstruction:", summary.message)

    ply_path = os.path.join(out_dir, "reconstruction.ply")
    recon_path = os.path.join(out_dir, "reconstruction.recon")
    pt.io.WritePlyFile(ply_path, recon, np.array([255, 0, 0], dtype=np.int32), 2)
    pt.io.WriteReconstruction(recon, recon_path)
    print("Wrote", ply_path, recon_path)

    if args.rerun:
        try:
            _rerun_log.log_reconstruction(recon)
        except ImportError:
            print("Rerun not installed; skip --rerun", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
