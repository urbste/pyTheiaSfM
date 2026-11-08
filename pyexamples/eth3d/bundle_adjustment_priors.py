"""Bundle adjustment with position, orientation, gravity, depth and point priors.

Six ETH3D courtyard views and their GT tracks. Initial poses and points are
perturbed (0.10 m / 0.5 deg / 0.05 m). Each condition enables one prior type (or
all), built from GT plus noise: position 2 cm, orientation 0.2 deg, gravity 0.01
(world -z assumed as gravity; ETH3D has no IMU), depth 2 % of the GT camera-frame
depth of the GT point (no ETH3D depth maps), 3D point 2 cm. Intrinsics are fixed.
Errors use a Sim(3) alignment of camera centres (same frame for all conditions).
Usage: python bundle_adjustment_priors.py [--views 30 31 32 33 34 35]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth3d_utils as E  # noqa: E402
import pytheia as pt  # noqa: E402

CONDITIONS = {
    "none": [], "position": ["pos"], "orientation": ["ori"],
    "gravity": ["grav"], "depth": ["depth"], "point": ["point"],
    "all": ["pos", "ori", "grav", "depth", "point"],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="courtyard")
    ap.add_argument("--views", type=int, nargs="+", default=list(range(30, 36)))
    a = ap.parse_args()
    m = E.load_model(a.scene)
    ims = [m.image_list()[i] for i in a.views]
    print(f"{len(ims)} views, {len(E.gt_tracks(m, ims))} GT tracks, perturbed init")
    print("%-12s %-5s %14s %14s %14s %11s" % ("condition", "ok", "cam C err (m)",
                                             "rot err (deg)", "point err (m)", "BA time (s)"))
    recon, vids, pairs = E.build_ba_problem(m, ims, [], 0)
    print("%-12s %-5s %14.4f %14.3f %14.4f" %
          (("initial", "-") + E.ba_alignment_errors(m, ims, recon, vids, pairs)))
    for name, flags in CONDITIONS.items():
        recon, vids, pairs = E.build_ba_problem(m, ims, flags, 0)
        o = pt.sfm.BundleAdjustmentOptions()
        o.num_threads, o.verbose, o.max_num_iterations = 1, False, 50
        o.loss_function_type = pt.sfm.LossFunctionType.TRIVIAL
        o.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
        o.use_position_priors = "pos" in flags
        o.use_orientation_priors = "ori" in flags
        o.use_gravity_priors = "grav" in flags
        o.use_depth_priors = "depth" in flags
        o.use_point_priors = "point" in flags
        s = pt.sfm.BundleAdjustReconstruction(o, recon)
        t = s.solve_time_in_seconds + s.setup_time_in_seconds
        print("%-12s %-5s %14.4f %14.3f %14.4f %11.2f"
              % ((name, str(s.success)) + E.ba_alignment_errors(m, ims, recon, vids, pairs) + (t,)))


if __name__ == "__main__":
    main()
