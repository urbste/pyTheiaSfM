"""Initialization-free SfM (INIT_FREE) versus global SfM on an ETH3D subset.

Both estimators get the same GT-derived input as global_sfm.py: GT 2D
observations shared by two views, pixel noise (once per observation) and
outliers, verified pairwise with EstimateTwoViewInfo into a ViewGraph and
chained by TrackBuilder. The input is rebuilt from the same RNG seed for each
method, so every method sees identical matches.

INIT_FREE runs the experimental initialization-free pipeline (random-start OSE
with VarPro, projective refinement, linear + nonlinear metric upgrade, metric
BA). It is configured for calibrated rays: intrinsics come from the ETH3D
priors. Both methods are evaluated against GT after a Umeyama Sim(3) alignment
of the camera centres (the output frame is arbitrary).

Usage (from this directory, or give the full path):
  python init_free_sfm.py                                  # both methods, defaults
  python init_free_sfm.py --scene courtyard --views $(seq 10 29) --outlier_ratio 0.2
  python init_free_sfm.py --methods init_free --formulation POSE --restarts 5
  python init_free_sfm.py --methods init_free --no_cauchy  # ablation: no robust loss
  python init_free_sfm.py --methods init_free --refine_huber_px 0  # plain LS refinement
  python init_free_sfm.py --methods init_free --rotation_weighting scalar
  python init_free_sfm.py --json out.json                  # machine-readable results

Set PYTHONPATH to a pyTheia build that contains INIT_FREE.
"""
import argparse
import dataclasses
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402

FORMULATIONS = {
    "POSE": pt.sfm.OseFormulationType.POSE,
    "POSE_ROT": pt.sfm.OseFormulationType.POSE_ROT,
    "EXPOSE": pt.sfm.OseFormulationType.EXPOSE,
}
# Robust loss scale in pixels, converted to normalized coordinates below.
ROBUST_LOSS_SCALE_PX = 3.0
# Above this many views the reduced camera system is solved as a sparse matrix.
SPARSE_SOLVER_MIN_VIEWS = 60


@dataclasses.dataclass
class Input:
    ims: list
    recon: pt.sfm.Reconstruction
    vg: pt.sfm.ViewGraph
    vids: list
    focal_px: float      # mean focal length of the views (pixels)
    n_pairs: int         # pairs with >= 30 GT matches
    n_tracks: int


def build_input(m, a):
    """Same construction as global_sfm.py, with the RNG seeded per call."""
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
    n_pairs = 0
    for i in range(len(ims)):
        for j in range(i + 1, len(ims)):
            pids, pa, pb = E.shared_observations(m, noisy[i], noisy[j])
            if len(pids) < 30:
                continue
            n_pairs += 1
            idx = rng.choice(len(pids), min(a.n_matches, len(pids)), replace=False)
            corrs = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(pa[k]),
                                                       pt.sfm.Feature(pb[k])) for k in idx]
            ok, info, inl = pt.sfm.EstimateTwoViewInfo(opts, priors[i], priors[j], corrs)
            if not ok or len(inl) < 30:
                continue
            vg.AddEdge(vids[i], vids[j], info)
            for k in inl:
                tb.AddFeatureCorrespondence(vids[i], corrs[k].feature1,
                                            vids[j], corrs[k].feature2)
    tb.BuildTracks(recon)
    focal_px = float(np.mean([m.K(im)[0, 0] for im in ims]))
    return Input(ims, recon, vg, vids, focal_px, n_pairs, recon.NumTracks())


def evaluate(inp):
    """Centre and rotation errors of the estimated views after Sim(3) alignment."""
    recon, ims, vids = inp.recon, inp.ims, inp.vids
    est = [k for k, v in enumerate(vids) if recon.View(v).IsEstimated()]
    res = {"estimated_views": len(est), "centre_err_m": None,
           "centre_err_pct_extent": None, "rotation_err_deg": None}
    if len(est) < 3:
        return res
    C_est = np.array([recon.View(vids[k]).Camera().GetPosition() for k in est])
    C_gt = np.array([ims[k].center for k in est])
    s, R_al, t_al = E.umeyama(C_est, C_gt)
    c_err = np.linalg.norm(s * C_est @ R_al.T + t_al - C_gt, axis=1)
    r_err = [E.rotation_error_deg(recon.View(vids[k]).Camera().GetOrientationAsRotationMatrix()
                                  @ R_al.T, ims[k].R) for k in est]
    extent = np.linalg.norm(C_gt.max(0) - C_gt.min(0))
    res.update(centre_err_m=float(np.median(c_err)),
               centre_err_pct_extent=float(100 * np.median(c_err) / extent),
               rotation_err_deg=float(np.median(r_err)), scale=float(s),
               extent_m=float(extent))
    return res


def run_global(inp):
    ro = pt.sfm.ReconstructionEstimatorOptions()
    ro.num_threads = 1
    ro.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
    ro.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
    t0 = time.perf_counter()
    summ = pt.sfm.GlobalReconstructionEstimator(ro).Estimate(inp.vg, inp.recon)
    dt = time.perf_counter() - t0
    return {"success": bool(summ.success), "message": str(getattr(summ, "message", "")),
            "time_s": dt, "diagnostics": None, **evaluate(inp)}


def init_free_options(a, inp):
    """INIT_FREE options for calibrated rays (see docs/content/init_free.md)."""
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.num_threads = 1
    ifo = opts.init_free_options
    ifo.formulation = FORMULATIONS[a.formulation]
    ifo.alpha = 0.05
    ifo.observation_normalization = pt.sfm.OseObservationNormalization.CALIBRATED_RAYS
    ifo.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
    ifo.num_random_restarts = a.restarts
    ifo.seed = a.seed
    ifo.projective_refinement.max_num_iterations = 50
    if a.refine_huber_px > 0:
        # Normalized observations m = K^-1 p, so a pixel width becomes px / f.
        ifo.projective_refinement.robust_loss_width = a.refine_huber_px / inp.focal_px
    if a.min_fraction_in_front is not None:
        ifo.min_fraction_in_front = a.min_fraction_in_front
    if a.max_mean_orthogonality_error is not None:
        ifo.max_mean_orthogonality_error = a.max_mean_orthogonality_error
    if a.rotation_weighting == "two_view":
        ifo.rotation_weighting = pt.sfm.RotationWeightingType.TWO_VIEW_INFORMATION
    else:
        ifo.rotation_weighting = pt.sfm.RotationWeightingType.SCALAR
    ifo.rotation_weight = a.rotation_weight
    ifo.varpro.robust_loss_type = (pt.sfm.VarProRobustLossType.NONE if a.no_cauchy
                                   else pt.sfm.VarProRobustLossType.CAUCHY)
    # Normalized observations m = K^-1 p, so a pixel scale becomes px / f.
    ifo.varpro.robust_loss_scale = ROBUST_LOSS_SCALE_PX / inp.focal_px
    ifo.varpro.linear_solver_type = (
        pt.sfm.ReducedCameraSolverType.SPARSE_CHOLESKY
        if len(inp.vids) > SPARSE_SOLVER_MIN_VIEWS
        else pt.sfm.ReducedCameraSolverType.DENSE_CHOLESKY)
    ifo.metric_upgrade = pt.sfm.MetricUpgradeType.LINEAR_AND_NONLINEAR
    ifo.run_metric_bundle_adjustment = True
    ifo.num_threads = 1
    return opts


def init_free_diagnostics(diag, focal_px):
    """Restart costs in pixels (normalized RMS times the mean focal length)."""
    out = {"best_restart": int(diag.best_restart),
           "fraction_in_front": float(diag.fraction_in_front),
           "mean_orthogonality_error": float(diag.mean_orthogonality_error),
           "metric_upgrade_success": bool(diag.metric_upgrade_success),
           "restarts": []}
    for r in diag.restarts:
        out["restarts"].append({
            "seed": int(r.seed), "final_cost": float(r.final_cost),
            "converged": bool(r.converged), "iterations": int(r.num_iterations),
            "ose_rms_px": float(r.ose_rms_error * focal_px),
            "refined_rms_px": float(r.refined_rms_error * focal_px),
            "upgrade_attempted": bool(r.upgrade_attempted),
            "upgrade_success": bool(r.upgrade_success),
            "upgrade_fraction_in_front": float(r.upgrade_fraction_in_front),
            "upgrade_mean_orthogonality_error": float(r.upgrade_mean_orthogonality_error),
            "upgrade_message": str(r.upgrade_message),
            "message": str(r.message)})
    if 0 <= diag.best_restart < len(diag.restarts):
        best = diag.restarts[diag.best_restart]
        out["best"] = out["restarts"][diag.best_restart]
        out["best_ose_cost"] = float(best.final_cost)
    return out


def run_init_free(inp, a):
    opts = init_free_options(a, inp)
    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    t0 = time.perf_counter()
    summ = estimator.Estimate(inp.vg, inp.recon)
    dt = time.perf_counter() - t0
    diag = None
    try:
        diag = init_free_diagnostics(estimator.Diagnostics(), inp.focal_px)
    except RuntimeError as exc:  # no diagnostics if the run stopped early
        diag = {"error": str(exc)}
    return {"success": bool(summ.success), "message": str(summ.message),
            "time_s": dt, "diagnostics": diag, **evaluate(inp)}


def print_result(name, res):
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
        print(f"    best restart {d['best_restart']} (seed {b['seed']}): OSE cost "
              f"{b['final_cost']:.4g}, projective RMS {b['ose_rms_px']:.3f} px -> "
              f"{b['refined_rms_px']:.3f} px after refinement, fraction_in_front "
              f"{d['fraction_in_front']:.3f}, mean orthogonality error "
              f"{d['mean_orthogonality_error']:.4g}")
        for k, r in enumerate(d["restarts"]):
            print(f"      restart {k}: cost {r['final_cost']:.4g} ose {r['ose_rms_px']:.3f} px "
                  f"refined {r['refined_rms_px']:.3f} px converged {r['converged']}; "
                  f"upgrade {'ok' if r['upgrade_success'] else 'no'}"
                  f" (frac in front {r['upgrade_fraction_in_front']:.3f}, ortho "
                  f"{r['upgrade_mean_orthogonality_error']:.4g}): {r['upgrade_message']}")
    elif d and "error" in d:
        print(f"    diagnostics unavailable: {d['error']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--views", type=int, nargs="+", default=list(range(30, 38)))
    ap.add_argument("--n_matches", type=int, default=300)
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--methods", nargs="+", choices=["global", "init_free"],
                    default=["global", "init_free"])
    ap.add_argument("--formulation", choices=list(FORMULATIONS), default="POSE_ROT")
    ap.add_argument("--restarts", type=int, default=3, help="random restarts")
    ap.add_argument("--seed", type=int, default=42,
                    help="first restart seed; restart k uses seed + k")
    ap.add_argument("--no_cauchy", action="store_true",
                    help="disable the Cauchy robust loss in VarPro")
    ap.add_argument("--rotation_weighting", choices=["two_view", "scalar"],
                    default="two_view", help="POSE_ROT relative rotation weights")
    ap.add_argument("--rotation_weight", type=float, default=3.0)
    ap.add_argument("--refine_huber_px", type=float, default=3.0,
                    help="Huber width (px) of the projective refinement; 0 = plain LS "
                         "(the library default). Helps on facade 20-31 with 20%% outliers, "
                         "see docs/content/init_free.md")
    ap.add_argument("--min_fraction_in_front", type=float, default=None,
                    help="validity check of the metric upgrade (library default if unset)")
    ap.add_argument("--max_mean_orthogonality_error", type=float, default=None,
                    help="validity check of the metric upgrade (library default if unset)")
    ap.add_argument("--json", help="write the results to this JSON file")
    a = ap.parse_args()

    m = E.load_model(a.scene)
    results = {}
    for method in a.methods:
        inp = build_input(m, a)  # same seed, so identical input per method
        print(f"{a.scene}: {len(inp.ims)} views, {inp.n_pairs} pairs with >=30 GT matches, "
              f"{inp.vg.NumEdges()} verified edges, {inp.n_tracks} tracks, "
              f"outlier_ratio {a.outlier_ratio}, sigma {a.sigma} px")
        if method == "global":
            res = run_global(inp)
        else:
            res = run_init_free(inp, a)
        res.update(n_views=len(inp.ims), n_edges=inp.vg.NumEdges(),
                   n_tracks=inp.n_tracks, focal_px=inp.focal_px)
        print_result(method, res)
        results[method] = res

    if a.json:
        config = {"scene": a.scene, "views": a.views, "n_matches": a.n_matches,
                  "sigma_px": a.sigma, "outlier_ratio": a.outlier_ratio,
                  "formulation": a.formulation, "restarts": a.restarts,
                  "seed": a.seed,
                  "robust_loss": "NONE" if a.no_cauchy else "CAUCHY",
                  "rotation_weighting": a.rotation_weighting,
                  "rotation_weight": a.rotation_weight,
                  "refine_huber_px": a.refine_huber_px,
                  "min_fraction_in_front": a.min_fraction_in_front,
                  "max_mean_orthogonality_error": a.max_mean_orthogonality_error}
        with open(a.json, "w") as f:
            json.dump({"config": config, "results": results}, f, indent=2)


if __name__ == "__main__":
    main()
