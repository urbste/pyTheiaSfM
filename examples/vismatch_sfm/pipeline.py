#!/usr/bin/env python3
# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Match images with vismatch, build a view graph with pyTheia, run SfM.

Heavy imports (vismatch, torch) load only when running the pipeline, not for --help.
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def _progress_step(min_step: int, n: int) -> int:
    """~20 progress ticks over n items, at least min_step."""
    if n <= 0:
        return 1
    return max(min_step, (n + 19) // 20)

# vismatch get_matcher names, grouped by correspondence density (see vismatch docs).
MATCHER_FEATURE_OPTIONS_EPILOG = """
Matcher names (--matcher) supported by vismatch:

  Dense:
    roma, tiny-roma, duster, master, minima-roma, ufm

  Semi-dense:
    loftr, eloftr, se2loftr, xoftr, minima-loftr, aspanformer, matchformer,
    xfeat-star, xfeat-star-steerers[-perm/-learned], edm, rdd-star,
    topicfm[-plus]

  Sparse:
    [sift, superpoint, disk, aliked, dedode, doghardnet, gim, xfeat]-lightglue,
    dedode, steerers, affine-steerers, xfeat-steerers[-perm/learned],
    dedode-kornia, [sift, orb, doghardnet]-nn, patch2pix, superglue, r2d2, d2net,
    gim-dkm, xfeat, omniglue, [dedode, xfeat, aliked]-subpx,
    [sift, superpoint]-sphereglue, minima-superpoint-lightglue, liftfeat,
    rdd-[sparse,lightglue,aliked], ripe, lisrd, zippypoint
"""


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="vismatch correspondences + pyTheia global/incremental/hybrid SfM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=MATCHER_FEATURE_OPTIONS_EPILOG,
    )
    p.add_argument(
        "--strecha_dir",
        type=str,
        required=True,
        help="Scene root with images/ (jpg + K.txt) and gt_dense_cameras/ (*.camera)",
    )
    p.add_argument(
        "--align_robust",
        type=float,
        default=None,
        metavar="METERS",
        help="If set, align with AlignReconstructionsRobust using this camera-center threshold (m)",
    )
    p.add_argument(
        "--img_ext",
        type=str,
        default="jpg",
        metavar="EXT",
        help="Extension under images/ for each view stem (default %(default)s)",
    )
    p.add_argument(
        "--matcher",
        type=str,
        default="superpoint-lightglue",
        metavar="NAME",
        help="Pass-through to vismatch get_matcher(NAME); default %(default)s. "
        "Feature-type groups are listed after the options below.",
    )
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument("--resize", type=int, default=784, help="vismatch load_image resize (pixels)")
    p.add_argument(
        "--reconstruction",
        type=str,
        default="incremental",
        choices=("global", "incremental", "hybrid"),
    )
    p.add_argument("--min_inliers_twoview", type=int, default=50)
    p.add_argument("--num_threads", type=int, default=4)
    p.add_argument("--output_dir", type=str, default=None)
    p.add_argument(
        "--plot_matches",
        action="store_true",
        help="Write match_plots/<a>__<b>.png (full-res images + lines) under the output folder for every image pair",
    )
    p.add_argument(
        "--plot_matches_max_lines",
        type=int,
        default=400,
        metavar="N",
        help="Max match polylines drawn per pair when using --plot_matches (subsampling if larger)",
    )
    p.add_argument("--rerun", action="store_true", help="Open Rerun after reconstruction")
    args = p.parse_args()
    return args


def _read_image_wh(path: str) -> tuple[int, int]:
    """Return (width, height) of image on disk."""
    import cv2

    im = cv2.imread(path)
    if im is None:
        raise ValueError(f"Could not read image: {path}")
    h, w = im.shape[:2]
    return w, h


def _matcher_image_hw(matcher_image) -> tuple[int, int]:
    """
    Infer (height, width) of `matcher.load_image` output (numpy or torch, HWC/CHW).
    """
    import numpy as np

    try:
        import torch

        if isinstance(matcher_image, torch.Tensor):
            t = matcher_image
            if t.dim() == 3:
                c, h, w = int(t.shape[0]), int(t.shape[1]), int(t.shape[2])
                if c in (1, 3, 4) and h > 1 and w > 1:
                    return h, w
                h, w, c = int(t.shape[0]), int(t.shape[1]), int(t.shape[2])
                if c in (1, 3, 4):
                    return h, w
            raise ValueError(f"Unexpected torch tensor shape: {tuple(t.shape)}")
    except ImportError:
        pass

    a = np.asarray(matcher_image)
    if a.ndim == 3:
        d0, d1, d2 = int(a.shape[0]), int(a.shape[1]), int(a.shape[2])
        if d0 in (1, 3, 4) and d1 > 1 and d2 > 1:
            return d1, d2
        if d2 in (1, 3, 4) and d0 > 1 and d1 > 1:
            return d0, d1
    if a.ndim == 2:
        return int(a.shape[0]), int(a.shape[1])
    raise ValueError(f"Cannot infer H,W from matcher image shape {a.shape}")


def _scale_kpts_to_full_image(
    kpts,
    full_wh: tuple[int, int],
    matched_hw: tuple[int, int],
):
    """Map keypoints from matcher (matched_h, matched_w) to full-res (full_w, full_h)."""
    import numpy as np

    full_w, full_h = full_wh
    mh, mw = matched_hw
    if mw <= 0 or mh <= 0:
        return kpts
    sx = float(full_w) / float(mw)
    sy = float(full_h) / float(mh)
    out = np.asarray(kpts, dtype=np.float64).reshape(-1, 2).copy()
    out[:, 0] *= sx
    out[:, 1] *= sy
    return out


def _correspondences_from_matcher_result(
    result: dict,
    full_wh_a: tuple[int, int],
    full_wh_b: tuple[int, int],
    matched_hw_a: tuple[int, int],
    matched_hw_b: tuple[int, int],
    min_putative: int,
):
    """Build FeatureCorrespondence list from vismatch forward dict; same coordinate scaling as always."""
    import numpy as np
    import pytheia as pt

    k0 = result.get("matched_kpts0")
    k1 = result.get("matched_kpts1")
    if k0 is None or k1 is None:
        return False, None

    k0 = _scale_kpts_to_full_image(k0, full_wh_a, matched_hw_a)
    k1 = _scale_kpts_to_full_image(k1, full_wh_b, matched_hw_b)
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


def _scaled_matched_keypoints_for_plot(
    result: dict,
    full_wh_a: tuple[int, int],
    full_wh_b: tuple[int, int],
    matched_hw_a: tuple[int, int],
    matched_hw_b: tuple[int, int],
):
    """Inlier keypoints if present, else raw matches; scaled to full-res (x,y) like Theia features."""
    import numpy as np

    k0 = result.get("inlier_kpts0")
    k1 = result.get("inlier_kpts1")
    if k0 is None or k1 is None or (np.asarray(k0).size == 0):
        k0 = result.get("matched_kpts0")
        k1 = result.get("matched_kpts1")
    if k0 is None or k1 is None:
        return None, None
    k0 = _scale_kpts_to_full_image(k0, full_wh_a, matched_hw_a)
    k1 = _scale_kpts_to_full_image(k1, full_wh_b, matched_hw_b)
    n = min(len(k0), len(k1))
    if n <= 0:
        return None, None
    return k0[:n], k1[:n]


def _save_match_pair_figure(
    path_a: str,
    path_b: str,
    k0,
    k1,
    out_path: str,
    *,
    caption: str,
    max_lines: int,
) -> None:
    """Side-by-side BGR images with Epipolar-style match lines; keypoints in pixel (x,y)."""
    import numpy as np
    import cv2

    im0 = cv2.imread(path_a)
    im1 = cv2.imread(path_b)
    if im0 is None or im1 is None:
        return
    ha, wa = im0.shape[:2]
    hb, wb = im1.shape[:2]
    H = max(ha, hb)
    left = np.zeros((H, wa, 3), dtype=np.uint8)
    right = np.zeros((H, wb, 3), dtype=np.uint8)
    left[:ha, :wa] = im0
    right[:hb, :wb] = im1
    canvas = np.hstack([left, right])
    xoff = wa
    p0 = np.asarray(k0, dtype=np.float64).reshape(-1, 2)
    p1 = np.asarray(k1, dtype=np.float64).reshape(-1, 2)
    n = min(len(p0), len(p1))
    if n <= 0:
        cv2.putText(
            canvas,
            caption,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(out_path, canvas)
        return
    idx = np.arange(n)
    if n > max_lines:
        idx = np.linspace(0, n - 1, num=max_lines, dtype=int)
    rng = np.random.default_rng(0)
    for ii in idx:
        x0, y0 = float(p0[ii, 0]), float(p0[ii, 1])
        x1, y1 = float(p1[ii, 0]), float(p1[ii, 1])
        c = tuple(int(x) for x in rng.integers(50, 255, size=3))
        q0 = (int(round(x0)), int(round(y0)))
        q1 = (int(round(x1)) + xoff, int(round(y1)))
        if not (0 <= q0[0] < wa and 0 <= q0[1] < ha and 0 <= q1[0] - xoff < wb and 0 <= q1[1] < hb):
            continue
        cv2.line(canvas, q0, q1, c, 1, cv2.LINE_AA)
        cv2.circle(canvas, q0, 2, c, -1, cv2.LINE_AA)
        cv2.circle(canvas, q1, 2, c, -1, cv2.LINE_AA)
    ytxt = min(H - 10, 28)
    cv2.putText(
        canvas,
        caption[:180],
        (10, ytxt),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.imwrite(out_path, canvas)


def _match_pair_preloaded(
    matcher,
    im_a,
    im_b,
    min_putative: int,
    full_wh_a: tuple[int, int],
    full_wh_b: tuple[int, int],
    matched_hw_a: tuple[int, int],
    matched_hw_b: tuple[int, int],
    *,
    return_matcher_result: bool = False,
):
    """Match two already-loaded matcher tensors. Returns (ok, correspondences|None, raw_result|None)."""
    result = matcher(im_a, im_b)
    ok, cors = _correspondences_from_matcher_result(
        result,
        full_wh_a,
        full_wh_b,
        matched_hw_a,
        matched_hw_b,
        min_putative,
    )
    return ok, cors, (result if return_matcher_result else None)


def _estimate_twoview_geometry(
    prior_a,
    prior_b,
    correspondences,
    min_inliers: int,
):
    """Two-view Theia RANSAC only; returns (ok, twoview_info, inlier_indices)."""
    import pytheia as pt

    opts = pt.sfm.EstimateTwoViewInfoOptions()
    opts.ransac_type = pt.sfm.RansacType(0)
    opts.use_lo = True
    opts.use_mle = True
    opts.max_sampson_error_pixels = 2.0

    ok, twoview_info, inlier_idx = pt.sfm.EstimateTwoViewInfo(
        opts, prior_a, prior_b, correspondences
    )
    if not ok or len(inlier_idx) < min_inliers:
        return False, None, ()
    twoview_info.num_verified_matches = len(inlier_idx)
    return True, twoview_info, inlier_idx


def _add_twoview_inliers_to_tracks(
    track_builder,
    view_id_a,
    view_id_b,
    correspondences,
    inlier_idx,
) -> None:
    for i in inlier_idx:
        c = correspondences[i]
        track_builder.AddFeatureCorrespondence(
            view_id_a, c.feature1, view_id_b, c.feature2
        )


def _mean_camera_position_error_vs_gt(
    gt_rec,
    aligned_rec,
    *,
    invalid_view_id: int,
) -> tuple[float, int]:
    """
    Mean Euclidean distance between GT and aligned-estimated camera centers for
    views matched by name (both estimated). Same units as the scene (metric
    Strecha data → meters).
    """
    import numpy as np

    dists: list[float] = []
    for vid in aligned_rec.ViewIds():
        v = aligned_rec.View(vid)
        if not v.IsEstimated():
            continue
        gt_vid = gt_rec.ViewIdFromName(v.Name())
        if gt_vid == invalid_view_id:
            continue
        gtv = gt_rec.View(gt_vid)
        if not gtv.IsEstimated():
            continue
        p_est = np.asarray(v.Camera().GetPosition(), dtype=np.float64).reshape(3)
        p_gt = np.asarray(gtv.Camera().GetPosition(), dtype=np.float64).reshape(3)
        dists.append(float(np.linalg.norm(p_est - p_gt)))
    if not dists:
        return float("nan"), 0
    return float(np.mean(dists)), len(dists)


def main() -> int:
    args = _parse_args()

    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    import rerun_log as _rerun_log
    import strecha_dataset as _strecha

    try:
        from vismatch import get_matcher
    except ImportError:
        print("Install vismatch: pip install vismatch  (or pip install '.[examples]')", file=sys.stderr)
        return 1

    import cv2
    import numpy as np
    import pytheia as pt

    strecha_dir = os.path.abspath(args.strecha_dir)
    if not os.path.isdir(strecha_dir):
        print(f"Not a directory: {strecha_dir}", file=sys.stderr)
        return 1

    try:
        img_dir, gt_dir, _k_txt = _strecha.validate_layout(strecha_dir)
    except FileNotFoundError as e:
        print(str(e), file=sys.stderr)
        return 1

    ok_gt, gt_recon = pt.io.ReadStrechaDataset(gt_dir)
    if not ok_gt:
        print(f"ReadStrechaDataset failed for {gt_dir}", file=sys.stderr)
        return 1

    gt_vids = sorted(gt_recon.ViewIds(), key=lambda vid: gt_recon.View(vid).Name())
    stems = [gt_recon.View(vid).Name() for vid in gt_vids]
    if len(stems) < 2:
        print("Strecha scene must contain at least two views.", file=sys.stderr)
        return 1
    try:
        stem_to_path = _strecha.resolve_images_for_stems(img_dir, stems, args.img_ext)
    except FileNotFoundError as e:
        print(str(e), file=sys.stderr)
        return 1

    image_paths = [stem_to_path[s] for s in stems]
    n_views = len(stems)
    n_pairs_total = n_views * (n_views - 1) // 2
    print(f"Scene root: {strecha_dir}")
    print(f"GT + images: {n_views} views, {n_pairs_total} unordered pairs to match.")
    out_dir = args.output_dir or strecha_dir
    os.makedirs(out_dir, exist_ok=True)
    plot_dir: str | None = None
    if args.plot_matches:
        plot_dir = os.path.join(out_dir, "match_plots")
        os.makedirs(plot_dir, exist_ok=True)

    matcher = get_matcher(args.matcher, device=args.device)

    # Full-res (W,H) per image for scaling matcher keypoints back to prior coordinates.
    wh_cache = {os.path.abspath(p): _read_image_wh(p) for p in image_paths}

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(3, 30)

    view_image_abspath: dict[int, str] = {}
    for idx, stem in enumerate(stems):
        pth = os.path.abspath(stem_to_path[stem])
        vid = recon.AddView(stem, 0, float(idx))
        view_image_abspath[vid] = pth
        gt_vid = gt_recon.ViewIdFromName(stem)
        if gt_vid == pt.sfm.kInvalidViewId:
            print(f"GT reconstruction has no view named {stem!r}", file=sys.stderr)
            return 1
        prior_v = gt_recon.View(gt_vid).Camera().CameraIntrinsicsPriorFromIntrinsics()
        recon.MutableView(vid).SetCameraIntrinsicsPrior(prior_v)

    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

    view_ids = sorted(recon.ViewIds())
    n = len(view_ids)
    n_pairs = n * (n - 1) // 2
    print(
        f"Matcher: {args.matcher} @ resize={args.resize}, device={args.device!r}; "
        f"two-view inliers ≥ {args.min_inliers_twoview}"
    )
    t_wall = time.perf_counter()

    preload_paths = [os.path.abspath(p) for p in image_paths]
    n_pre = len(preload_paths)
    pre_step = _progress_step(1, n_pre)
    print(f"Preloading {n_pre} images for vismatch (resize={args.resize}) …", flush=True)
    img_tensors: dict[str, object] = {}
    matched_hw_matcher: dict[str, tuple[int, int]] = {}
    for k, p in enumerate(preload_paths, start=1):
        ap = os.path.abspath(p)
        t = matcher.load_image(ap, resize=args.resize)
        img_tensors[ap] = t
        matched_hw_matcher[ap] = _matcher_image_hw(t)
        if k == 1 or k == n_pre or k % pre_step == 0:
            print(f"  preload {k}/{n_pre}", flush=True)
    print(f"  preloaded {n_pre} images in {time.perf_counter() - t_wall:.1f}s", flush=True)

    match_results: list[tuple[int, int, bool, object]] = []
    pair_step = _progress_step(1, n_pairs)
    t_match = time.perf_counter()
    pair_idx = 0
    for i in range(n):
        vi = view_ids[i]
        path_i = view_image_abspath[vi]
        name_i = recon.View(vi).Name()
        for j in range(i + 1, n):
            vj = view_ids[j]
            path_j = view_image_abspath[vj]
            name_j = recon.View(vj).Name()
            ok_c, cors, raw_res = _match_pair_preloaded(
                matcher,
                img_tensors[path_i],
                img_tensors[path_j],
                args.min_inliers_twoview,
                wh_cache[path_i],
                wh_cache[path_j],
                matched_hw_matcher[path_i],
                matched_hw_matcher[path_j],
                return_matcher_result=plot_dir is not None,
            )
            pair_idx += 1
            if pair_idx == 1 or pair_idx == n_pairs or pair_idx % pair_step == 0:
                print(
                    f"  match pairs {pair_idx}/{n_pairs} ({name_i} vs {name_j})",
                    flush=True,
                )
            match_results.append((vi, vj, ok_c, cors))
            if plot_dir is not None and raw_res is not None:
                k0p, k1p = _scaled_matched_keypoints_for_plot(
                    raw_res,
                    wh_cache[path_i],
                    wh_cache[path_j],
                    matched_hw_matcher[path_i],
                    matched_hw_matcher[path_j],
                )
                if k0p is None or k1p is None:
                    k0p = np.zeros((0, 2), dtype=np.float64)
                    k1p = np.zeros((0, 2), dtype=np.float64)
                n_m = int(min(len(k0p), len(k1p)))
                mha, mwa = matched_hw_matcher[path_i]
                mhb, mwb = matched_hw_matcher[path_j]
                wi, hi = wh_cache[path_i]
                wj, hj = wh_cache[path_j]
                cap = (
                    f"putative_ok={ok_c} n={n_m} | "
                    f"full {wi}x{hi} {wj}x{hj} | "
                    f"match_tensor {mwa}x{mha} {mwb}x{mhb}"
                )
                safe_i = name_i.replace("/", "_").replace("\\", "_")
                safe_j = name_j.replace("/", "_").replace("\\", "_")
                fig_path = os.path.join(plot_dir, f"{safe_i}__{safe_j}.png")
                _save_match_pair_figure(
                    path_i,
                    path_j,
                    k0p,
                    k1p,
                    fig_path,
                    caption=cap,
                    max_lines=max(50, args.plot_matches_max_lines),
                )

    if plot_dir is not None:
        print(f"  wrote pair match figures under {plot_dir}")

    n_putative = sum(1 for _vi, _vj, ok_c, _cors in match_results if ok_c)
    print(
        f"Matching done in {time.perf_counter() - t_match:.1f}s "
        f"({n_putative}/{n_pairs} pairs with enough putatives).",
        flush=True,
    )

    print(f"Two-view geometry ({n_pairs} pairs, RANSAC) …", flush=True)
    geom_step = _progress_step(1, n_pairs)
    n_skip = n_reject = n_edge = 0
    for g_idx, (vi, vj, ok_c, cors) in enumerate(match_results, start=1):
        name_i = recon.View(vi).Name()
        name_j = recon.View(vj).Name()
        if g_idx == 1 or g_idx == n_pairs or g_idx % geom_step == 0:
            print(f"  geometry {g_idx}/{n_pairs}", flush=True)
        if not ok_c:
            n_skip += 1
            continue
        pa = recon.View(vi).CameraIntrinsicsPrior()
        pb = recon.View(vj).CameraIntrinsicsPrior()
        ok_t, tv, inlier_idx = _estimate_twoview_geometry(
            pa, pb, cors, args.min_inliers_twoview
        )
        if not ok_t:
            n_reject += 1
            continue
        _add_twoview_inliers_to_tracks(track_builder, vi, vj, cors, inlier_idx)
        view_graph.AddEdge(vi, vj, tv)
        n_edge += 1

    print(
        f"View graph: {view_graph.NumEdges()} edges "
        f"({n_edge} accepted; {n_skip} low putatives, {n_reject} failed geom).",
        flush=True,
    )
    if view_graph.NumEdges() < 1:
        print("Not enough edges for reconstruction.", file=sys.stderr)
        return 1

    print("Building tracks from verified correspondences …", flush=True)
    t_tracks = time.perf_counter()
    track_builder.BuildTracks(recon)
    n_tracks = int(recon.NumTracks())
    print(
        f"  tracks: {n_tracks} (built in {time.perf_counter() - t_tracks:.1f}s)",
        flush=True,
    )

    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = args.num_threads
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType.HUBER
    options.subsample_tracks_for_bundle_adjustment = True

    if args.reconstruction == "global":
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
        est = pt.sfm.GlobalReconstructionEstimator(options)
    elif args.reconstruction == "hybrid":
        est = pt.sfm.HybridReconstructionEstimator(options)
    else:
        est = pt.sfm.IncrementalReconstructionEstimator(options)

    print(
        f"Running {args.reconstruction} reconstruction estimator "
        f"({args.num_threads} BA threads) …",
        flush=True,
    )
    t_est = time.perf_counter()
    summary = est.Estimate(view_graph, recon)
    print(
        f"Reconstruction ({time.perf_counter() - t_est:.1f}s): {summary.message}",
        flush=True,
    )

    try:
        print("Sim3 alignment (estimated → GT) …", flush=True)
        t_al = time.perf_counter()
        if args.align_robust is not None:
            R_a, t_a, sc = pt.sfm.AlignReconstructionsRobust(
                float(args.align_robust), gt_recon, recon
            )
        else:
            R_a, t_a, sc = pt.sfm.AlignReconstructions(gt_recon, recon)
        t_r = np.asarray(t_a).reshape(-1)
        print(
            "Sim3 alignment (%.2fs): scale=%.6f, t=[%.4f, %.4f, %.4f]"
            % (
                time.perf_counter() - t_al,
                float(sc),
                float(t_r[0]),
                float(t_r[1]),
                float(t_r[2]),
            ),
            flush=True,
        )
        mean_pos_err, n_cam_eval = _mean_camera_position_error_vs_gt(
            gt_recon, recon, invalid_view_id=pt.sfm.kInvalidViewId
        )
        if n_cam_eval > 0 and not np.isnan(mean_pos_err):
            print(
                "Average camera center error (vs GT, after Sim3): %.6f over %d views "
                "(Euclidean distance, scene units)"
                % (mean_pos_err, n_cam_eval),
                flush=True,
            )
        else:
            print(
                "Average camera center error: could not compute (no overlapping estimated views).",
                flush=True,
            )
    except Exception as ex:  # noqa: BLE001 — surface binding/C++ failures clearly
        print("AlignReconstructions failed:", ex, file=sys.stderr)
        return 1

    ply_path = os.path.join(out_dir, "reconstruction.ply")
    recon_path = os.path.join(out_dir, "reconstruction.recon")
    pt.io.WritePlyFile(ply_path, recon, np.array([255, 0, 0], dtype=np.int32), 2)
    pt.io.WriteReconstruction(recon, recon_path)
    gt_ply_path = os.path.join(out_dir, "gt_reconstruction.ply")
    pt.io.WritePlyFile(
        gt_ply_path,
        gt_recon,
        np.array([80, 255, 120], dtype=np.int32),
        2,
    )
    print(f"Wrote PLY + recon: {ply_path} | {recon_path} | {gt_ply_path}", flush=True)
    print(f"Total wall time: {time.perf_counter() - t_wall:.1f}s", flush=True)
    if args.rerun:
        try:
            _rerun_log.log_eval_comparison(gt_recon, aligned_rec=recon)
        except ImportError:
            print("Rerun not installed; skip --rerun", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
