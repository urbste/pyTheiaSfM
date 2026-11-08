"""Absolute pose (PnP) on ETH3D GT points with pixel noise and outliers.

Noise and outliers are synthetic; points and 2D observations are GT. Errors are
vs the ETH3D GT pose: rotation (deg) and camera centre (scene units).
Usage: python pnp_solvers.py [--scene courtyard] [--view 35] [--sigma 1.0]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402


def p3p_best(xn, X):
    """P3P on 3 points; pick the candidate whose reprojection of the rest is best."""
    _, Rs, ts = pt.sfm.PoseFromThreePoints(list(xn[:3]), list(X[:3]))
    best, best_err = None, np.inf
    for R, t in zip(Rs, ts):
        Xc = X[3:] @ R.T + t
        if np.any(Xc[:, 2] <= 0):
            continue
        err = np.median(np.linalg.norm(Xc[:, :2] / Xc[:, 2:3] - xn[3:], axis=1))
        if err < best_err:
            best, best_err = (R, t), err
    return len(Rs), best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--view", type=int, default=35)
    ap.add_argument("--sigma", type=float, default=1.0, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.3)
    a = ap.parse_args()
    m = E.load_model(a.scene)
    im = m.image_list()[a.view]
    K = m.K(im)
    _, X, px = E.gt_points_in_image(m, im)
    rng = np.random.default_rng(0)
    print(f"{im.name} GT points={len(X)} sigma={a.sigma}px")

    # 1) Noise only, 100 points: P3P, DLS, MLPnP, UPMLPnP (gravity from GT rotation).
    sel = rng.choice(len(X), 100, replace=False)
    pn, _ = E.add_pixel_noise(px[sel], a.sigma, rng=rng)
    xn, Xs = E.normalize(K, pn), X[sel]
    n_sol, best = p3p_best(xn, Xs)
    if best is not None:
        print("P3P (3 pts, %d candidates): rot %.3f deg, C %.4f m"
              % ((n_sol,) + E.pose_errors(best[0], -best[0].T @ best[1], im)))
    q, ts = pt.sfm.DlsPnp(list(xn), list(Xs))
    R = E.quat_wxyz_to_matrix(q[0])
    print("DLS (100 pts): rot %.3f deg, C %.4f m" % E.pose_errors(R, -R.T @ ts[0], im))
    covs = [np.eye(3) * (a.sigma / K[0, 0]) ** 2] * len(sel)
    _, R, t = pt.sfm.MLPnP(list(xn), covs, list(Xs))
    print("MLPnP (100 pts): rot %.3f deg, C %.4f m" % E.pose_errors(R, -R.T @ t, im))
    g_w = np.array([0.0, 0.0, -1.0])  # assumed gravity; GT rotation gives g_cam
    _, R, t = pt.sfm.UPMLPnP(list(xn), covs, list(Xs), im.R @ g_w, g_w,
                             np.eye(2) * 1e-4, True)
    print("UPMLPnP (gravity): rot %.3f deg, C %.4f m" % E.pose_errors(R, -R.T @ t, im))

    # 2) Outliers: RANSAC (+LO) over 200 points, one run per PnPType.
    sel = rng.choice(len(X), 200, replace=False)
    pn, is_out = E.add_pixel_noise(px[sel], a.sigma, a.outlier_ratio,
                                   (m.camera(im).width, m.camera(im).height), rng)
    xn = E.normalize(K, pn)
    corrs = [pt.sfm.FeatureCorrespondence2D3D(xn[i], X[sel[i]]) for i in range(200)]
    print(f"RANSAC PnP, 200 pts, {a.outlier_ratio:.0%} outliers "
          f"(true inliers {200 - is_out.sum()})")
    for name in ["KNEIP", "DLS", "SQPnP", "MLPnP"]:
        p = pt.solvers.RansacParameters()
        p.error_thresh = (2.0 / K[0, 0]) ** 2  # squared, normalized coords
        p.max_iterations = 500
        p.use_lo = True
        p.rng = pt.solvers.RandomNumberGenerator(0)
        ok, pose, summ = pt.sfm.EstimateCalibratedAbsolutePose(
            p, pt.sfm.RansacType.RANSAC, getattr(pt.sfm.PnPType, name), corrs)
        print("  %-6s rot %.3f deg, C %.4f m, inliers %d" %
              ((name,) + E.pose_errors(pose.rotation, pose.position, im) + (len(summ.inliers),)))

    # Direct (non-RANSAC) call; returns candidate quaternions and translations.
    quats, ts = pt.sfm.SQPnP(list(xn[:10]), list(X[sel[:10]]))
    print(f"direct pt.sfm.SQPnP: {len(quats)} candidate pose(s)")
    print("UPnP: not exposed in pytheia")


if __name__ == "__main__":
    main()
