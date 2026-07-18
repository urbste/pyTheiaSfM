"""Smoke test: BulkEstimateTwoViewInfo accepts depth priors and returns scales."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def _project(points_cam):
    return points_cam[:, :2] / points_cam[:, 2:3]


def _make_prior(focal_length=1000.0, principal_point=(512.0, 384.0)):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.image_width = 1024
    prior.image_height = 768
    prior.focal_length.value = [focal_length]
    prior.principal_point.value = list(principal_point)
    prior.aspect_ratio.value = [1.0]
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def test_bulk_estimate_two_view_info_monodepth_depth_and_scales():
    assert hasattr(pt.sfm, "BulkEstimateTwoViewInfo")

    rng = np.random.default_rng(0)
    angle = np.deg2rad(10.0)
    axis = np.array([0.1, 1.0, -0.2])
    axis /= np.linalg.norm(axis)
    K = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )
    rotation = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    translation = np.array([0.3, -0.1, 0.2])
    true_relative_scale = 1.4

    num_points = 60
    points_cam1 = np.column_stack(
        [
            rng.uniform(-1.0, 1.0, num_points),
            rng.uniform(-1.0, 1.0, num_points),
            rng.uniform(4.0, 6.0, num_points),
        ]
    )
    points_cam2 = points_cam1 @ rotation.T + translation

    focal_length = 1000.0
    principal_point = np.array([512.0, 384.0])
    pixels1 = (focal_length * _project(points_cam1) + principal_point).astype(
        np.float32
    )
    pixels2 = (focal_length * _project(points_cam2) + principal_point).astype(
        np.float32
    )
    depth1 = points_cam1[:, 2].astype(np.float32)
    depth2 = (points_cam2[:, 2] / true_relative_scale).astype(np.float32)

    pair_offsets = np.asarray([0, num_points], dtype=np.uint64)
    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.use_mle = True
    options.use_monodepth = True
    options.max_sampson_error_pixels = 4.0
    options.min_ransac_iterations = 20
    options.max_ransac_iterations = 500

    prior = _make_prior(focal_length, principal_point)
    out = pt.sfm.BulkEstimateTwoViewInfo(
        pair_offsets,
        pixels1,
        pixels2,
        prior,
        prior,
        options,
        num_threads=1,
        depth_i=depth1,
        depth_j=depth2,
    )

    assert "scales" in out
    assert bool(np.asarray(out["success"])[0])
    scale = float(np.asarray(out["scales"])[0])
    assert scale > 0.0
    assert abs(scale - true_relative_scale) < 0.25
