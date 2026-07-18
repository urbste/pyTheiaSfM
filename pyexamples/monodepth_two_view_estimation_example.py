#!/usr/bin/env python3
"""
Example demonstrating monocular-depth-assisted two-view relative pose
estimation in pytheia.

pytheia.sfm.EstimateTwoViewInfo(..., options.use_monodepth=True) uses a
3-point minimal solver (ported from PoseLib's RePoseD, see
dev/TWO_VIEW_SPEEDUP_PLAN.md and docs/content/license.md) instead of the
standard 5-/8-point solvers whenever each pytheia.sfm.Feature carries a
monocular depth estimate (Feature.depth_prior). Because the minimal sample
drops from 5 (or 8, uncalibrated) points to 3, RANSAC needs far fewer
iterations to reach the same confidence.

This example builds a synthetic two-view scene (no images/monodepth network
required) to show the API end-to-end:
  1. Generate 3D points and project them into two calibrated cameras.
  2. Attach a per-feature monocular depth estimate (here, the ground-truth
     camera-frame z-depth, with camera 2's depth map scaled by an unknown
     factor -- exactly the ambiguity real monodepth networks have).
  3. Estimate the two-view relative pose with use_monodepth=True and inspect
     the recovered relative depth-map scale (TwoViewInfo.scale_estimate).
"""

import numpy as np
import pytheia as pt


def project(points_cam):
    """Perspective-projects Nx3 camera-frame points to Nx2 image coordinates."""
    return points_cam[:, :2] / points_cam[:, 2:3]


def main():
    rng = np.random.default_rng(0)

    # Ground-truth relative pose (camera 1 -> camera 2: X2 = R @ X1 + t).
    angle = np.deg2rad(10.0)
    axis = np.array([0.1, 1.0, -0.2])
    axis /= np.linalg.norm(axis)
    # Rodrigues' formula.
    K = np.array([[0, -axis[2], axis[1]],
                 [axis[2], 0, -axis[0]],
                 [-axis[1], axis[0], 0]])
    rotation = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    translation = np.array([0.3, -0.1, 0.2])

    # Camera 2's monocular depth map is only consistent with camera 1's up to
    # an unknown global scale -- this is exactly the ambiguity the solver
    # recovers via TwoViewInfo.scale_estimate.
    true_relative_scale = 1.4

    num_points = 60
    points_cam1 = np.column_stack([
        rng.uniform(-1.0, 1.0, num_points),
        rng.uniform(-1.0, 1.0, num_points),
        rng.uniform(4.0, 6.0, num_points),
    ])
    points_cam2 = points_cam1 @ rotation.T + translation

    focal_length = 1000.0
    principal_point = np.array([512.0, 384.0])
    pixels1 = focal_length * project(points_cam1) + principal_point
    pixels2 = focal_length * project(points_cam2) + principal_point

    depth1 = points_cam1[:, 2]
    depth2_raw = points_cam2[:, 2] / true_relative_scale

    correspondences = []
    for i in range(num_points):
        corr = pt.matching.FeatureCorrespondence()
        corr.feature1 = pt.sfm.Feature(pixels1[i], float(depth1[i]))
        corr.feature2 = pt.sfm.Feature(pixels2[i], float(depth2_raw[i]))
        correspondences.append(corr)

    def make_intrinsics_prior():
        prior = pt.sfm.CameraIntrinsicsPrior()
        prior.image_width = 1024
        prior.image_height = 768
        prior.focal_length.value = [focal_length]
        prior.principal_point.value = list(principal_point)
        prior.aspect_ratio.value = [1.0]
        prior.camera_intrinsics_model_type = "PINHOLE"
        return prior

    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.use_mle = True
    options.use_monodepth = True
    options.max_sampson_error_pixels = 4.0

    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
        options,
        make_intrinsics_prior(),
        make_intrinsics_prior(),
        correspondences,
    )

    print(f"Success: {success}")
    print(f"Num inliers: {len(inlier_indices)} / {num_points}")
    print(f"Recovered relative rotation (angle-axis): {twoview_info.rotation_2}")
    print(f"Recovered relative depth-map scale: {twoview_info.scale_estimate:.4f} "
          f"(ground truth: {true_relative_scale})")


if __name__ == "__main__":
    main()
