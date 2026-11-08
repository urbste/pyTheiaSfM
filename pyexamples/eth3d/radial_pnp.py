"""Uncalibrated PnP with one radial (division-model) distortion parameter.

ETH3D images are undistorted PINHOLE, so distortion is synthesised here: GT
pixels (centred on the principal point) are passed through the division model
with k_true, then noise and outliers are added. pyTheia then estimates focal
length f, radial k and pose. Errors are reported against the GT f, k and pose.

Usage: python radial_pnp.py [--scene courtyard] [--view 35] [--k_true -2e-8]
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
    ap.add_argument("--view", type=int, default=35)
    ap.add_argument("--k_true", type=float, default=-2e-8)
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    m = E.load_model(a.scene)
    im = m.image_list()[a.view]
    K = m.K(im)
    cm = m.camera(im)
    _, X, px = E.gt_points_in_image(m, im)
    rng = np.random.default_rng(a.seed)
    sel = rng.choice(len(X), min(a.n, len(X)), replace=False)

    # Centred GT pixels -> distorted observations -> noise + outliers.
    u = px[sel] - K[:2, 2]
    d = np.array([E.division_distort(x, a.k_true) for x in u])
    d, is_out = E.add_pixel_noise(d, a.sigma, a.outlier_ratio,
                                  (cm.width, cm.height), rng)
    # outliers were drawn in image coords; move them to centred coords
    d[is_out] -= K[:2, 2]
    corrs = [pt.sfm.FeatureCorrespondence2D3D(d[i], X[sel[i]]) for i in range(len(sel))]

    p = pt.solvers.RansacParameters()
    p.error_thresh = 5.0  # squared pixel error (features are centered pixels, ~2.2 px)
    p.max_iterations = 2000
    p.rng = pt.solvers.RandomNumberGenerator(a.seed)
    meta = pt.sfm.RadialDistUncalibratedAbsolutePoseMetaData()  # default k bounds
    ok, pose, summ = pt.sfm.EstimateRadialDistUncalibratedAbsolutePose(
        p, pt.sfm.RansacType.RANSAC, corrs, meta)
    print(f"scene={a.scene} view={im.name} n={len(sel)} sigma={a.sigma}px "
          f"outliers={a.outlier_ratio:.0%} k_true={a.k_true:.1e}")
    if not ok:
        print("estimation failed")
        return
    f_gt = K[0, 0]
    C = -pose.rotation.T @ pose.translation
    rot_err, c_err = E.pose_errors(pose.rotation, C, im)
    print(f"focal: est {pose.focal_length:.2f} GT {f_gt:.2f} "
          f"(rel err {abs(pose.focal_length - f_gt) / f_gt:.2%})")
    print(f"radial k: est {pose.radial_distortion:.4e} GT {a.k_true:.4e} "
          f"(rel err {abs(pose.radial_distortion - a.k_true) / abs(a.k_true):.2%})")
    print(f"pose: rot {rot_err:.3f} deg, C {c_err:.4f} m, "
          f"inliers {len(summ.inliers)}/{len(sel)} (true inliers {len(sel) - is_out.sum()})")


if __name__ == "__main__":
    main()
