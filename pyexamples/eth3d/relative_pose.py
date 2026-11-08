"""Two-view relative pose on an ETH3D GT image pair.

(1) Calibrated 5-point RANSAC (EstimateRelativePose) on GT 2D matches.
(2) 3-point monocular-depth-assisted RANSAC (EstimateMonoDepthRelativePose)
    where each match carries a depth prior. There are no ETH3D depth maps: the
    prior is the camera-frame depth of the GT sparse point projected into each
    view (optionally perturbed with --depth_noise).

Pixel noise and outliers are synthetic. Errors are vs. the GT relative pose.
The translation is only known up to scale, so the direction error is reported.

Usage: python relative_pose.py [--scene courtyard] [--pair 35 36]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--pair", type=int, nargs=2, default=[35, 36])
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--depth_noise", type=float, default=0.0,
                    help="relative (multiplicative) noise on the depth prior")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    m = E.load_model(a.scene)
    ia, ib = [m.image_list()[i] for i in a.pair]
    Ka, Kb = m.K(ia), m.K(ib)
    pids, pa, pb = E.shared_observations(m, ia, ib)
    rng = np.random.default_rng(a.seed)
    pa, out_a = E.add_pixel_noise(pa, a.sigma, a.outlier_ratio,
                                  (m.camera(ia).width, m.camera(ia).height), rng)
    pb, _ = E.add_pixel_noise(pb, a.sigma, 0.0, rng=rng)
    pb[out_a] = E.add_pixel_noise(pb[out_a], 0.0, 1.0,
                                  (m.camera(ib).width, m.camera(ib).height), rng)[0]
    na, nb = E.normalize(Ka, pa), E.normalize(Kb, pb)
    R_gt, t_gt = E.relative_gt(ia, ib)
    C_gt = -R_gt.T @ t_gt  # second camera centre in the first camera frame
    print(f"{ia.name} -> {ib.name}: {len(pids)} GT matches, sigma={a.sigma}px, "
          f"outliers={a.outlier_ratio:.0%}")

    # (1) calibrated 5-point RANSAC on normalized coordinates (thresh in px/f).
    corrs = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(na[i]),
                                               pt.sfm.Feature(nb[i]))
             for i in range(len(pids))]
    p = pt.solvers.RansacParameters()
    # error_thresh is the SQUARED Sampson error in normalized (inv(K)) coords:
    # a 2 px threshold is (2 / f) ** 2.
    p.error_thresh = (2.0 / Ka[0, 0]) ** 2
    p.max_iterations = 2000
    p.use_lo = True
    p.rng = pt.solvers.RandomNumberGenerator(a.seed)
    ok, rel, summ = pt.sfm.EstimateRelativePose(p, pt.sfm.RansacType.RANSAC, corrs)
    print(f"5-point RANSAC: R err {E.rotation_error_deg(rel.rotation, R_gt):.3f} deg, "
          f"direction err {E.direction_error_deg(rel.position, C_gt):.3f} deg, "
          f"inliers {len(summ.inliers)}")

    # (2) 3-point monodepth RANSAC; depth priors = GT camera-frame z of each point.
    Za = E.camera_points(ia.R, ia.t, np.array([m.points[p] for p in pids]))[:, 2]
    Zb = E.camera_points(ib.R, ib.t, np.array([m.points[p] for p in pids]))[:, 2]
    if a.depth_noise > 0:
        Za = Za * (1 + rng.normal(scale=a.depth_noise, size=len(Za)))
        Zb = Zb * (1 + rng.normal(scale=a.depth_noise, size=len(Zb)))
    dcorrs = [pt.matching.FeatureCorrespondence(
        pt.sfm.Feature(na[i], Za[i]), pt.sfm.Feature(nb[i], Zb[i]))
        for i in range(len(pids))]
    ok, res, summ = pt.sfm.EstimateMonoDepthRelativePose(
        p, pt.sfm.RansacType.RANSAC, dcorrs)
    print(f"3-point depth-prior RANSAC (depth_noise={a.depth_noise}): "
          f"R err {E.rotation_error_deg(res.rotation, R_gt):.3f} deg, "
          f"direction err {E.direction_error_deg(res.position, C_gt):.3f} deg, "
          f"inliers {len(summ.inliers)}, depth scale {res.scale:.4f}")


if __name__ == "__main__":
    main()
