"""Depth-augmented OSE (DOSE, arXiv:2611.31516) on an ETH3D subset.

Same GT-derived input as init_free_sfm.py: GT 2D observations shared by two views,
pixel noise and outliers, verified pairwise with EstimateTwoViewInfo into a ViewGraph
and chained by TrackBuilder. With the same arguments the matches are identical to
init_free_sfm.py (same RNG sequence); the only addition is a depth prior per observation.

DOSE needs a depth map per image. Maps are read from --depth_dir as
<image name>.npy (float, metres, 0 = invalid). A map whose shape differs from the
ETH3D DSLR image size is resized (nearest neighbour) to that size. Depths are sampled
bilinearly at the observed pixels and attached as Feature(pixel, depth); a sample with
an invalid neighbour gets depth 0 (no prior). With --predict_depth, missing maps are
predicted with MoGe-2 (package `moge`, horizontal fov from the GT focal length) and
cached in --depth_dir when that is given.

DOSE produces Euclidean cameras directly, so no projective refinement and no metric
upgrade run; the metric bundle adjustment does. Both the estimator and the GT-based
evaluation follow init_free_sfm.py: the camera centres and rotations are compared
with GT after a Umeyama Sim(3) alignment (the output frame is arbitrary).

Usage (from this directory, or give the full path):
  python init_free_dose_sfm.py --depth_dir DIR                  # maps DIR/<image>.npy
  python init_free_dose_sfm.py --depth_dir DIR --predict_depth  # predict missing maps with MoGe
  python init_free_dose_sfm.py --scene relief --views $(seq 0 30) --depth_dir DIR --json out.json

Set PYTHONPATH to a pyTheia build that contains DOSE.
"""
import argparse
import dataclasses
import importlib
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import init_free_sfm as IF  # noqa: E402  (Input, evaluate, init_free_diagnostics)
import pytheia as pt  # noqa: E402

# Above this many views the reduced camera system is solved as a sparse matrix.
SPARSE_SOLVER_MIN_VIEWS = 60
# MoGe models tried in order (first importable package wins).
MOGE_MODELS = [("moge.model.v2", "Ruicheng/moge-2-vitl-normal"),
               ("moge.model.v3", "Ruicheng/moge-3-vitl")]


def sample_depth(depth, pix):
    """Bilinear depth at pixels (N,2), COLMAP convention (pixel centres at +0.5).

    A sample is 0 (no prior) if it lies outside the map or if any of its four
    neighbours is 0 or not finite.
    """
    depth = np.where(np.isfinite(depth), depth, 0.0).astype(np.float64)
    H, W = depth.shape
    x, y = pix[:, 0] - 0.5, pix[:, 1] - 0.5
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    inside = (x0 >= 0) & (y0 >= 0) & (x0 + 1 < W) & (y0 + 1 < H)
    x0c, y0c = np.clip(x0, 0, W - 2), np.clip(y0, 0, H - 2)
    d = np.stack([depth[y0c, x0c], depth[y0c, x0c + 1],
                  depth[y0c + 1, x0c], depth[y0c + 1, x0c + 1]])
    valid = inside & np.all(d > 0, axis=0)
    val = ((1 - fx) * (1 - fy) * d[0] + fx * (1 - fy) * d[1]
           + (1 - fx) * fy * d[2] + fx * fy * d[3])
    return np.where(valid, val, 0.0)


def resize_depth(depth, width, height):
    """Nearest-neighbour resize to (height, width). Nearest keeps invalid (0) pixels invalid."""
    if depth.shape == (height, width):
        return depth
    rows = np.minimum((np.arange(height) + 0.5) * depth.shape[0] / height, depth.shape[0] - 1).astype(int)
    cols = np.minimum((np.arange(width) + 0.5) * depth.shape[1] / width, depth.shape[1] - 1).astype(int)
    return depth[np.ix_(rows, cols)]


def load_moge(device):
    """The first importable MoGe model (v2, then v3), or None if no package is importable."""
    for module, repo in MOGE_MODELS:
        try:
            cls = importlib.import_module(module).MoGeModel
        except ImportError:
            continue
        print(f"loading MoGe model {repo} on {device}", flush=True)
        return cls.from_pretrained(repo).to(device).eval()
    return None


def predict_depth(model, m, im, device):
    """MoGe depth (H, W) in metres at the full DSLR resolution, 0 where invalid."""
    import cv2
    import torch

    cam = m.camera(im)
    fov_x = math.degrees(2 * math.atan(cam.width / (2 * float(cam.params[0]))))
    bgr = cv2.imread(m.image_path(im))
    if bgr is None:
        raise FileNotFoundError(f"cannot read image {m.image_path(im)}")
    img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    x = torch.tensor(img / 255.0, dtype=torch.float32, device=device).permute(2, 0, 1)
    with torch.no_grad():
        out = model.infer(x, fov_x=fov_x)
    d = out["depth"].float().cpu().numpy()
    return np.where(np.isfinite(d) & (d > 0), d, 0.0)


class DepthSource:
    """Depth maps per image at the DSLR image size, from --depth_dir and/or MoGe."""

    def __init__(self, depth_dir, predict):
        self.depth_dir, self.predict = depth_dir, predict
        self.model, self.device, self.cache = None, None, {}

    def get(self, m, im):
        if im.name in self.cache:
            return self.cache[im.name]
        cam = m.camera(im)
        path = os.path.join(self.depth_dir, os.path.basename(im.name) + ".npy") if self.depth_dir else None
        if path and os.path.exists(path):
            d = np.load(path).astype(np.float64)
        elif self.predict:
            if self.model is None:
                import torch
                self.device = "cuda" if torch.cuda.is_available() else "cpu"
                self.model = load_moge(self.device)
                if self.model is None:
                    raise SystemExit("depth maps are required: no `moge` package is importable, so "
                                     "--predict_depth cannot run. Provide <image>.npy maps with --depth_dir.")
            d = predict_depth(self.model, m, im, self.device)
            if path:
                os.makedirs(self.depth_dir, exist_ok=True)
                np.save(path, d.astype(np.float16))
        else:
            raise SystemExit(f"depth maps are required: no map for {im.name} at {path}. "
                             "Provide them with --depth_dir or use --predict_depth.")
        d = resize_depth(d, cam.width, cam.height)
        self.cache[im.name] = d
        return d


def build_input(m, a, depths):
    """Same construction as init_free_sfm.build_input, with depth priors on the features."""
    ims = [m.image_list()[i] for i in a.views]
    rng = np.random.default_rng(0)

    # Noise once per observation: identical keypoints in every pair, so the
    # TrackBuilder can chain pairwise matches into multi-view tracks.
    noisy = [dataclasses.replace(im, xys=E.add_pixel_noise(
        im.xys, a.sigma, a.outlier_ratio, (m.camera(im).width, m.camera(im).height),
        rng)[0]) for im in ims]

    recon, vg, tb = pt.sfm.Reconstruction(), pt.sfm.ViewGraph(), pt.sfm.TrackBuilder(2, 50)
    priors = [E.intrinsics_prior(m, im) for im in ims]
    vids = [recon.AddView(im.name, 0, k) for k, im in enumerate(ims)]
    for v, pr in zip(vids, priors):
        recon.MutableView(v).SetCameraIntrinsicsPrior(pr)
    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

    # Two-view RANSAC seeds its generators from the clock unless a default seed
    # is set; without this the verified inliers (and tracks) differ per build.
    pt.solvers.SetDefaultRandomSeed(0)
    opts = pt.sfm.EstimateTwoViewInfoOptions()
    opts.max_sampson_error_pixels, opts.use_lo, opts.use_mle = 1.0, True, True
    n_pairs, n_obs, n_depth = 0, 0, 0
    for i in range(len(ims)):
        for j in range(i + 1, len(ims)):
            pids, pa, pb = E.shared_observations(m, noisy[i], noisy[j])
            if len(pids) < 30:
                continue
            n_pairs += 1
            # Depth at the (noisy) observed pixels of each view.
            dep_a = sample_depth(depths.get(m, ims[i]), pa)
            dep_b = sample_depth(depths.get(m, ims[j]), pb)
            idx = rng.choice(len(pids), min(a.n_matches, len(pids)), replace=False)
            corrs = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(pa[k], float(dep_a[k])),
                                                       pt.sfm.Feature(pb[k], float(dep_b[k])))
                     for k in idx]
            ok, info, inl = pt.sfm.EstimateTwoViewInfo(opts, priors[i], priors[j], corrs)
            if not ok or len(inl) < 30:
                continue
            vg.AddEdge(vids[i], vids[j], info)
            for k in inl:
                n_obs += 2
                n_depth += int(corrs[k].feature1.depth_prior > 0) + int(corrs[k].feature2.depth_prior > 0)
                tb.AddFeatureCorrespondence(vids[i], corrs[k].feature1,
                                            vids[j], corrs[k].feature2)
    tb.BuildTracks(recon)
    focal_px = float(np.mean([m.K(im)[0, 0] for im in ims]))
    print(f"depth priors on {n_depth}/{n_obs} inlier observations "
          f"({100 * n_depth / max(n_obs, 1):.1f} %)", flush=True)
    return IF.Input(ims, recon, vg, vids, focal_px, n_pairs, recon.NumTracks())


def dose_options(a, num_views):
    """INIT_FREE with DOSE on calibrated rays (see docs/content/init_free.md, *Depth-augmented OSE*).

    DOSE returns Euclidean cameras, so projective refinement is off and the upgrade setting is
    unused. The DOSE defaults (depth weight 30, Huber 0.1, three-stage schedule) are kept.
    """
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.num_threads = 1
    ifo = opts.init_free_options
    ifo.formulation = pt.sfm.OseFormulationType.DOSE
    ifo.observation_normalization = pt.sfm.OseObservationNormalization.CALIBRATED_RAYS
    ifo.camera_init = pt.sfm.InitFreeCameraInitType.EUCLIDEAN_RANDOM
    ifo.num_random_restarts = a.restarts
    ifo.seed = a.seed
    ifo.projective_refinement.max_num_iterations = 0
    ifo.varpro.linear_solver_type = (
        pt.sfm.ReducedCameraSolverType.SPARSE_CHOLESKY
        if num_views > SPARSE_SOLVER_MIN_VIEWS
        else pt.sfm.ReducedCameraSolverType.DENSE_CHOLESKY)
    ifo.metric_upgrade = pt.sfm.MetricUpgradeType.LINEAR  # not used by DOSE
    ifo.run_metric_bundle_adjustment = True
    ifo.num_threads = 1
    return opts


def dose_extra(diag):
    """DOSE refinement fields of the InitFree diagnostics object.

    A key is left out if the attribute is missing (older builds) or empty.
    """
    out = {}
    for key, attr in (("rms_px", "dose_rms_reprojection_error_px"),
                      ("median_rel_depth_err", "dose_median_relative_depth_error")):
        v = getattr(diag, attr, None)
        if v is not None:
            out[key] = float(v)
    scales = getattr(diag, "dose_depth_scales", None)
    if scales is not None and len(scales) > 0:
        out["scale_min"], out["scale_max"] = float(min(scales)), float(max(scales))
    msgs = [getattr(r, "pre_refinement_check_message", None) for r in getattr(diag, "restarts", [])]
    if any(m is not None for m in msgs):
        out["pre_refinement"] = msgs
    return out


def run_dose(inp, a):
    estimator = pt.sfm.InitFreeReconstructionEstimator(dose_options(a, len(inp.vids)))
    t0 = time.perf_counter()
    summ = estimator.Estimate(inp.vg, inp.recon)
    dt = time.perf_counter() - t0
    try:
        diag = IF.init_free_diagnostics(estimator.Diagnostics(), inp.focal_px)
    except RuntimeError as exc:  # no diagnostics if the run stopped early
        diag = {"error": str(exc)}
    try:
        extra = dose_extra(estimator.Diagnostics())
    except RuntimeError:
        extra = {}
    return {"success": bool(summ.success), "message": str(summ.message),
            "time_s": dt, "diagnostics": diag, "dose": extra, **IF.evaluate(inp)}


def print_dose_result(name, res):
    def fmt(v, spec):
        return "  n/a" if v is None else format(v, spec)
    print(f"[{name}] success={res['success']} views {res['estimated_views']}/{res['n_views']} "
          f"centre err {fmt(res['centre_err_m'], '.4f')} m "
          f"({fmt(res['centre_err_pct_extent'], '.2f')}% of extent) "
          f"rot err {fmt(res['rotation_err_deg'], '.3f')} deg time {res['time_s']:.2f} s")
    print(f"    message: {res['message'].splitlines()[0] if res['message'] else ''}")
    d = res.get("diagnostics")
    if d and "best" in d:
        b = d["best"]
        # The DOSE residuals scale with the metric depth, so no pixel RMS is printed
        # (the JSON still holds ose_rms_px = raw RMS * focal, which is not a pixel error).
        print(f"    best restart {d['best_restart']} (seed {b['seed']}): OSE cost "
              f"{b['final_cost']:.4g}, fraction_in_front {d['fraction_in_front']:.3f}")
        for k, r in enumerate(d["restarts"]):
            print(f"      restart {k}: cost {r['final_cost']:.4g} iterations {r['iterations']} "
                  f"converged {r['converged']}: {r['message']}")
    elif d and "error" in d:
        print(f"    diagnostics unavailable: {d['error']}")
    x = res.get("dose", {})
    parts = []
    if "rms_px" in x:
        parts.append(f"refined RMS reprojection {x['rms_px']:.3f} px")
    if "median_rel_depth_err" in x:
        parts.append(f"median relative depth error {x['median_rel_depth_err']:.4f}")
    if "scale_min" in x:
        parts.append(f"depth scales min {x['scale_min']:.4f} max {x['scale_max']:.4f}")
    if parts:
        print("    DOSE refinement: " + ", ".join(parts))
    for k, msg in enumerate(x.get("pre_refinement", [])):
        print(f"      restart {k} pre-refinement check: {msg if msg else '(not run)'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--views", type=int, nargs="+", default=list(range(30, 38)))
    ap.add_argument("--n_matches", type=int, default=300)
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--restarts", type=int, default=3, help="random restarts")
    ap.add_argument("--seed", type=int, default=42,
                    help="first restart seed; restart k uses seed + k")
    ap.add_argument("--depth_dir",
                    help="directory with <image name>.npy depth maps (metres, float, 0 = invalid)")
    ap.add_argument("--predict_depth", action="store_true",
                    help="predict maps missing from --depth_dir with MoGe (needs package moge)")
    ap.add_argument("--json", help="write the results to this JSON file")
    a = ap.parse_args()
    if a.depth_dir is None and not a.predict_depth:
        ap.error("DOSE needs depth maps: give --depth_dir DIR and/or --predict_depth")

    m = E.load_model(a.scene)
    inp = build_input(m, a, DepthSource(a.depth_dir, a.predict_depth))
    print(f"{a.scene}: {len(inp.ims)} views, {inp.n_pairs} pairs with >=30 GT matches, "
          f"{inp.vg.NumEdges()} verified edges, {inp.n_tracks} tracks, "
          f"outlier_ratio {a.outlier_ratio}, sigma {a.sigma} px")
    res = run_dose(inp, a)
    res.update(n_views=len(inp.ims), n_edges=inp.vg.NumEdges(),
               n_tracks=inp.n_tracks, focal_px=inp.focal_px)
    print_dose_result("init_free_dose", res)
    results = {"init_free_dose": res}

    if a.json:
        config = {"scene": a.scene, "views": a.views, "n_matches": a.n_matches,
                  "sigma_px": a.sigma, "outlier_ratio": a.outlier_ratio,
                  "restarts": a.restarts, "seed": a.seed,
                  "depth_dir": a.depth_dir, "predict_depth": a.predict_depth}
        with open(a.json, "w") as f:
            json.dump({"config": config, "results": results}, f, indent=2)


if __name__ == "__main__":
    main()
