"""Global SfM on an ETH3D subset from GT-derived matches with noise and outliers.

Matches are GT 2D observations shared by two views (point3D ids in images.txt),
with pixel noise (once per observation, like a detector) and outliers added.
EstimateTwoViewInfo verifies each pair; ViewGraph + TrackBuilder feed
GlobalReconstructionEstimator. The output frame is arbitrary, so camera centres
and rotations are compared to GT after a Umeyama similarity alignment.
Usage: python global_sfm.py [--views 30 31 32 33 34 35 36 37] [--outlier_ratio 0.2]
"""
import argparse
import dataclasses
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--views", type=int, nargs="+", default=list(range(30, 38)))
    ap.add_argument("--n_matches", type=int, default=300)
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    a = ap.parse_args()
    m = E.load_model(a.scene)
    ims = [m.image_list()[i] for i in a.views]
    rng = np.random.default_rng(0)

    # Noise once per observation: the same keypoint is identical in every pair,
    # so TrackBuilder can chain pairwise matches into multi-view tracks.
    noisy = [dataclasses.replace(im, xys=E.add_pixel_noise(
        im.xys, a.sigma, a.outlier_ratio, (m.camera(im).width, m.camera(im).height),
        rng)[0]) for im in ims]

    recon, vg, tb = pt.sfm.Reconstruction(), pt.sfm.ViewGraph(), pt.sfm.TrackBuilder(2, 50)
    priors = [E.intrinsics_prior(m, im) for im in ims]
    vids = [recon.AddView(im.name, 0, k) for k, im in enumerate(ims)]
    for v, pr in zip(vids, priors):
        recon.MutableView(v).SetCameraIntrinsicsPrior(pr)
    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

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
    print(f"{len(ims)} views, {n_pairs} pairs with >=30 GT matches, "
          f"{vg.NumEdges()} verified edges, {recon.NumTracks()} tracks")

    ro = pt.sfm.ReconstructionEstimatorOptions()
    ro.num_threads = 1
    ro.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
    ro.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
    summ = pt.sfm.GlobalReconstructionEstimator(ro).Estimate(vg, recon)
    est = [k for k, v in enumerate(vids) if recon.View(v).IsEstimated()]
    print(f"estimator success={summ.success}, estimated views {len(est)}/{len(ims)}")
    if len(est) < 3:
        return

    C_est = np.array([recon.View(vids[k]).Camera().GetPosition() for k in est])
    C_gt = np.array([ims[k].center for k in est])
    s, R_al, t_al = E.umeyama(C_est, C_gt)
    c_err = np.linalg.norm(s * C_est @ R_al.T + t_al - C_gt, axis=1)
    r_err = [E.rotation_error_deg(recon.View(vids[k]).Camera().GetOrientationAsRotationMatrix()
                                  @ R_al.T, ims[k].R) for k in est]
    extent = np.linalg.norm(C_gt.max(0) - C_gt.min(0))
    print(f"Sim(3) aligned (scale {s:.3f}): centre error median {np.median(c_err):.4f} m "
          f"({np.median(c_err) / extent:.2%} of extent {extent:.2f} m), "
          f"rotation error median {np.median(r_err):.3f} deg")


if __name__ == "__main__":
    main()
