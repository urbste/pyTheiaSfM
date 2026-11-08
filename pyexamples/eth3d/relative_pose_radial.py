"""Two-view relative pose with radial (division-model) distortion.

ETH3D images are already undistorted, so distortion is synthesised: GT pixels
(centred on the principal point) are distorted with k_true, then noise and
outliers are added. pyTheia has no radial-aware calibrated two-view estimator
(EstimateRadialHomographyMatrix is planar), so this example undistorts the
observations with the KNOWN k (user-side step, d / (1 + k |d|^2)) and then calls
EstimateRelativePose. The effect of ignoring k is shown for comparison.

Usage: python relative_pose_radial.py [--k_true -2e-8]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402


def run_relative(ka, kb, pa, pb, R_gt, C_gt, seed):
    na, nb = E.normalize(ka, pa), E.normalize(kb, pb)
    corrs = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(na[i]),
                                               pt.sfm.Feature(nb[i]))
             for i in range(len(na))]
    p = pt.solvers.RansacParameters()
    p.error_thresh = (2.0 / ka[0, 0]) ** 2  # squared, normalized coords
    p.max_iterations = 2000
    p.use_lo = True
    p.rng = pt.solvers.RandomNumberGenerator(seed)
    ok, rel, summ = pt.sfm.EstimateRelativePose(p, pt.sfm.RansacType.RANSAC, corrs)
    return (E.rotation_error_deg(rel.rotation, R_gt),
            E.direction_error_deg(rel.position, C_gt), len(summ.inliers))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--pair", type=int, nargs=2, default=[35, 36])
    ap.add_argument("--k_true", type=float, default=-2e-8)
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    m = E.load_model(a.scene)
    ia, ib = [m.image_list()[i] for i in a.pair]
    Ka, Kb = m.K(ia), m.K(ib)
    pids, pa, pb = E.shared_observations(m, ia, ib)
    rng = np.random.default_rng(a.seed)
    size_a = (m.camera(ia).width, m.camera(ia).height)
    size_b = (m.camera(ib).width, m.camera(ib).height)

    # GT -> centred pixels -> division distortion -> noise + outliers.
    da = np.array([E.division_distort(x - Ka[:2, 2], a.k_true) for x in pa])
    db = np.array([E.division_distort(x - Kb[:2, 2], a.k_true) for x in pb])
    da, out = E.add_pixel_noise(da, a.sigma, a.outlier_ratio, size_a, rng)
    db, _ = E.add_pixel_noise(db, a.sigma, 0.0, rng=rng)
    da[out] -= Ka[:2, 2]  # outliers were drawn in image coordinates
    db[out] = rng.uniform(0, size_b[0], size=(out.sum(), 2)) - Kb[:2, 2]
    # Principal-point-centred distorted observations (pixels).
    R_gt, t_gt = E.relative_gt(ia, ib)
    C_gt = -R_gt.T @ t_gt

    def to_pix(c, K):
        return c + K[:2, 2]

    # A: ignore distortion (treat distorted pixels as undistorted).
    r_a, c_a, n_a = run_relative(Ka, Kb, to_pix(da, Ka), to_pix(db, Kb),
                                 R_gt, C_gt, a.seed)
    # B: undistort with known k, then the calibrated estimator.
    ua = np.array([d / (1.0 + a.k_true * (d @ d)) for d in da])
    ub = np.array([d / (1.0 + a.k_true * (d @ d)) for d in db])
    r_b, c_b, n_b = run_relative(Ka, Kb, to_pix(ua, Ka), to_pix(ub, Kb),
                                 R_gt, C_gt, a.seed)
    print(f"{ia.name} -> {ib.name}: {len(pids)} matches, k_true={a.k_true:.1e}, "
          f"sigma={a.sigma}px, outliers={a.outlier_ratio:.0%}")
    print(f"ignore distortion : R err {r_a:.3f} deg, direction err {c_a:.3f} deg, "
          f"inliers {n_a}")
    print(f"undistort (known k): R err {r_b:.3f} deg, direction err {c_b:.3f} deg, "
          f"inliers {n_b}")


if __name__ == "__main__":
    main()
