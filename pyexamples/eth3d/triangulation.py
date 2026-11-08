"""Triangulation of ETH3D GT points from GT camera poses with pixel noise.

GT points seen in >= 4 of the chosen views get Gaussian pixel noise; every
function triangulates the same rays. Errors are vs the GT 3D point (metres).
Usage: python triangulation.py [--views 30 31 32 33] [--sigma 0.5]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402


def hnorm(v):
    v = np.asarray(v, dtype=float).ravel()
    return v[:3] / v[3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--views", type=int, nargs="+", default=[30, 31, 32, 33])
    ap.add_argument("--sigma", type=float, default=0.5, help="pixel noise (px)")
    a = ap.parse_args()
    m = E.load_model(a.scene)
    rng = np.random.default_rng(0)
    tracks = [(p, o) for p, o in E.gt_tracks(m, [m.image_list()[i] for i in a.views]).items()
              if len(o) >= 4]
    print(f"{len(tracks)} GT points seen in >=4 of views {a.views}")

    errs = {}
    for pid, obs in tracks[:200]:
        X_gt = m.points[pid]
        ims = [im for im, _ in obs]
        pix, _ = E.add_pixel_noise(np.array([x for _, x in obs]), a.sigma, rng=rng)
        P = [np.hstack([im.R, im.t[:, None]]) for im in ims]
        nrm = [E.normalize(m.K(im), pix[j:j + 1])[0] for j, im in enumerate(ims)]
        dirs = [im.R.T @ np.append(n, 1.0) for im, n in zip(ims, nrm)]
        dirs = [d / np.linalg.norm(d) for d in dirs]
        out = {
            "Triangulate (first 2 obs)": pt.sfm.Triangulate(P[0], P[1], nrm[0], nrm[1]),
            "TriangulateDLT (first 2 obs)": pt.sfm.TriangulateDLT(P[0], P[1], nrm[0], nrm[1]),
            "TriangulateMidpoint": pt.sfm.TriangulateMidpoint([im.center for im in ims], dirs),
            "TriangulateNView": pt.sfm.TriangulateNView(P, nrm),
            "TriangulateNViewSVD": pt.sfm.TriangulateNViewSVD(P, nrm),
        }
        for k, (_, Xh) in out.items():
            errs.setdefault(k, []).append(np.linalg.norm(hnorm(Xh) - X_gt))
    print(f"median (mean) 3D error over {len(tracks[:200])} points, {a.sigma} px noise:")
    for k, e in errs.items():
        print(f"  {k:30s} {np.median(e):.5f} m ({np.mean(e):.5f})")

    # Known issue: RANSAC triangulation (see report). Count outcomes on 20 points.
    ok_err, n_bad = [], 0
    for pid, obs in tracks[:20]:
        p = pt.solvers.RansacParameters()
        p.error_thresh = 4.0
        p.max_iterations = 200
        p.min_iterations = 1
        p.rng = pt.solvers.RandomNumberGenerator(0)
        ok, Xh, summ = pt.sfm.EstimateTriangulation(
            p, [E.pt_camera(m, im) for im, _ in obs], [x for _, x in obs])
        if ok and len(summ.inliers) > 0:
            ok_err.append(np.linalg.norm(hnorm(Xh) - m.points[pid]))
        else:
            n_bad += 1
    med = np.median(ok_err) if ok_err else float("nan")
    print(f"KNOWN ISSUE EstimateTriangulation (20 pts): {len(ok_err)} with inliers "
          f"(median err {med:.4f} m), {n_bad} success=True with 0 inliers")


if __name__ == "__main__":
    main()
