"""Relative pose with a known rotation axis (gravity / partial-rotation prior).

ETH3D has no IMU or gravity, and real pairs do not rotate about a common axis.
So the motion is synthetic: the GT 3D points of an ETH3D view are observed from
a second camera rotated by --angle about a known axis (playing the role of the
gravity direction) with a random translation. Pixel noise and outliers are
added. The minimal solver ThreePointRelativePosePartialRotation returns 8
candidates per 3-point sample; cheirality and epipolar consistency select the
hypothesis inside a small RANSAC loop written here (no pytheia RANSAC wrapper).

Usage: python relative_pose_partial_rotation.py [--trials 300]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--view", type=int, default=35)
    ap.add_argument("--angle", type=float, default=10.0, help="rotation (deg)")
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    ap.add_argument("--outlier_ratio", type=float, default=0.2)
    ap.add_argument("--trials", type=int, default=300)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    m = E.load_model(a.scene)
    ia = m.image_list()[a.view]
    K = m.K(ia)
    _, X, _ = E.gt_points_in_image(m, ia)
    rng = np.random.default_rng(a.seed)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)                    # known rotation axis (cam a)
    R_gt = Rot.from_rotvec(np.radians(a.angle) * axis).as_matrix()
    t_gt = rng.normal(size=3)
    t_gt /= np.linalg.norm(t_gt)

    Xa = E.camera_points(ia.R, ia.t, X)
    Xb = Xa @ R_gt.T + t_gt
    ok = (Xa[:, 2] > 0) & (Xb[:, 2] > 0)
    Xa, Xb = Xa[ok], Xb[ok]
    # Pixels of the synthetic second camera (same intrinsics, no distortion).
    pa = Xa[:, :2] / Xa[:, 2:3] * K[0, 0] + K[:2, 2]
    pb = Xb[:, :2] / Xb[:, 2:3] * K[0, 0] + K[:2, 2]
    size = (m.camera(ia).width, m.camera(ia).height)
    pa, out = E.add_pixel_noise(pa, a.sigma, a.outlier_ratio, size, rng)
    pb, _ = E.add_pixel_noise(pb, a.sigma, 0.0, rng=rng)
    pb[out] = rng.uniform(0, m.camera(ia).width, size=(out.sum(), 2))
    ra = np.hstack([E.normalize(K, pa), np.ones((len(pa), 1))])
    rb = np.hstack([E.normalize(K, pb), np.ones((len(pb), 1))])
    ra /= np.linalg.norm(ra, axis=1, keepdims=True)
    rb /= np.linalg.norm(rb, axis=1, keepdims=True)
    thr = 2.0 / K[0, 0]                              # ~2 px epipolar residual

    best_inl, best = -1, None
    for _ in range(a.trials):
        idx = rng.choice(len(ra), 3, replace=False)
        qs, ts = pt.sfm.ThreePointRelativePosePartialRotation(
            axis, list(ra[idx]), list(rb[idx]))
        for q, t in zip(qs, ts):
            R = E.quat_wxyz_to_matrix(q)
            inl = E.partial_rotation_inliers(R, t, ra, rb, thr)
            if inl.sum() > best_inl:
                best_inl, best = inl.sum(), (R, t)
    R_est, t_est = best
    print(f"{ia.name}: synthetic {a.angle} deg about known axis, "
          f"{len(ra)} rays, sigma={a.sigma}px, outliers={a.outlier_ratio:.0%}")
    print(f"partial-rotation RANSAC ({a.trials} 3-pt samples): "
          f"R err {E.rotation_error_deg(R_est, R_gt):.3f} deg, "
          f"translation direction err {E.direction_error_deg(t_est, t_gt):.3f} deg, "
          f"inliers {best_inl} (true {len(ra) - out.sum()}), no refinement")


if __name__ == "__main__":
    main()
