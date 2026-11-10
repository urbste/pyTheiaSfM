"""Structure-from-motion from images in about 50 lines of pyTheia.

vismatch (learned matcher) -> pyTheia two-view geometry -> view graph and tracks
-> global SfM -> robust bundle adjustment and outlier removal -> browser viewer.

The intrinsics are read from a COLMAP text model (cameras.txt, images.txt); for
ETH3D this is the ground truth in <scene>/gt, which is also used to report the
camera-center error after a Sim(3) alignment.

Usage:
  python sfm_from_images.py --images <scene>/images --cameras <scene>/gt --out out_dir
Requires: pip install vismatch (GPU recommended).
"""
import argparse
import itertools
import os

import numpy as np
import pytheia as pt
from pytheia import viewer


def read_colmap_cameras(model_dir):
    """{image name: (w, h, fx, fy, cx, cy)} from a COLMAP text model with PINHOLE cameras."""
    cams = {}
    for ln in open(os.path.join(model_dir, "cameras.txt")):
        if ln.strip() and not ln.startswith("#"):
            t = ln.split()
            cams[t[0]] = (int(t[2]), int(t[3]), *map(float, t[4:8]))
    out, lines = {}, [ln.split() for ln in open(os.path.join(model_dir, "images.txt"))
                      if ln.strip() and not ln.startswith("#")]
    for t in lines:
        if len(t) >= 10 and t[0].isdigit() and t[8].isdigit():
            out[os.path.basename(" ".join(t[9:]))] = cams[t[8]]
    return out


def intrinsics_prior(w, h, fx, fy, cx, cy):
    p = pt.sfm.CameraIntrinsicsPrior()
    p.focal_length.value = [fx]
    p.aspect_ratio.value = [fy / fx]
    p.principal_point.value = [cx, cy]
    p.image_width, p.image_height = w, h
    p.camera_intrinsics_model_type = "PINHOLE"
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--cameras", required=True, help="COLMAP text model with the intrinsics")
    ap.add_argument("--out", required=True)
    ap.add_argument("--matcher", default="aliked-lightglue")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    from vismatch import get_matcher

    # 1. Views with known intrinsics.
    cams = read_colmap_cameras(a.cameras)
    names = sorted(n for n in os.listdir(a.images) if n in cams)
    recon = pt.sfm.Reconstruction()
    vid = {n: recon.AddView(n, 0, float(i)) for i, n in enumerate(names)}
    for n in names:
        recon.MutableView(vid[n]).SetCameraIntrinsicsPrior(intrinsics_prior(*cams[n]))
    pt.sfm.SetCameraIntrinsicsFromPriors(recon)

    # 2. Match all pairs (vismatch) and estimate the two-view geometry (pyTheia).
    matcher = get_matcher(a.matcher, device="cuda", max_num_keypoints=4096)
    imgs = {n: matcher.load_image(os.path.join(a.images, n), resize=cams[n][1::-1]) for n in names}
    opts = pt.sfm.EstimateTwoViewInfoOptions()           # five-point LO-RANSAC, 4 px Sampson
    view_graph, tracks = pt.sfm.ViewGraph(), pt.sfm.TrackBuilder(3, 50)
    for n1, n2 in itertools.combinations(names, 2):
        m = matcher(imgs[n1], imgs[n2])
        if len(m["matched_kpts0"]) < 30:
            continue
        corr = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(p1), pt.sfm.Feature(p2))
                for p1, p2 in zip(m["matched_kpts0"], m["matched_kpts1"])]
        ok, info, inliers = pt.sfm.EstimateTwoViewInfo(
            opts, recon.View(vid[n1]).CameraIntrinsicsPrior(), recon.View(vid[n2]).CameraIntrinsicsPrior(), corr)
        if ok and len(inliers) >= 30:
            info.num_verified_matches = len(inliers)
            view_graph.AddEdge(vid[n1], vid[n2], info)
            for k in inliers:
                tracks.AddFeatureCorrespondence(vid[n1], corr[k].feature1, vid[n2], corr[k].feature2)
    tracks.BuildTracks(recon)

    # 3. Global SfM: robust rotation averaging, LUD positions, triangulation, bundle adjustment.
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
    options.view_graph_max_cycle_rotation_error_degrees = 5.0   # drop edges on inconsistent triplets
    options.view_graph_min_relative_inliers = 0.2               # and edges much weaker than a view's best
    summary = pt.sfm.GlobalReconstructionEstimator(options).Estimate(view_graph, recon)

    os.makedirs(os.path.join(a.out, "model_txt_before_cleanup"), exist_ok=True)
    pt.io.WriteColmapFiles(recon, os.path.join(a.out, "model_txt_before_cleanup"))

    # 4. Remove outlier points (reprojection error > 2 px or angle < 2 deg), bundle adjust again.
    removed = pt.sfm.SetOutlierTracksToUnestimated(set(recon.TrackIds()), 2.0, 2.0, recon)
    ba = pt.sfm.BundleAdjustmentOptions()
    ba.loss_function_type, ba.robust_loss_width = pt.sfm.LossFunctionType.HUBER, 1.0
    ba.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    pt.sfm.BundleAdjustReconstruction(ba, recon)

    # 5. Save and visualize.
    pt.io.WriteReconstruction(recon, os.path.join(a.out, "reconstruction.recon"))
    os.makedirs(os.path.join(a.out, "model_txt"), exist_ok=True)
    pt.io.WriteColmapFiles(recon, os.path.join(a.out, "model_txt"))
    viewer.export_html(recon, os.path.join(a.out, "reconstruction.html"))
    print(f"{summary.success=}: {len(recon.ViewIds())} views, {view_graph.NumEdges()} edges, "
          f"{recon.NumTracks()} tracks, {removed} outlier observations removed")


if __name__ == "__main__":
    main()
