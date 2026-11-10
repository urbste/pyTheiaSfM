"""LO-RANSAC: refined models are rescored, and the returned inliers belong to the
returned model. Monocular-depth relative pose with and without the depth term
in the local optimization."""
import numpy as np
import pytest

import pytheia as pt

FOCAL = 500.0
THRESH_PX = 2.0


def rotation(axis, angle_deg):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    a = np.radians(angle_deg)
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k


def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def scene(seed, n=300, outlier_ratio=0.3, pixel_noise=0.5, depth_noise=0.02):
    rng = np.random.default_rng(seed)
    R = rotation(rng.normal(size=3), 10.0)
    c = np.array([1.0, 0.2, 0.1])                      # camera 2 center in camera 1
    X1 = np.c_[rng.uniform(-3, 3, (n, 2)), rng.uniform(4, 8, n)]
    X2 = (X1 - c) @ R.T
    x1 = X1[:, :2] / X1[:, 2:] + rng.normal(0, pixel_noise / FOCAL, (n, 2))
    x2 = X2[:, :2] / X2[:, 2:] + rng.normal(0, pixel_noise / FOCAL, (n, 2))
    scale_gt = 2.0                                     # metric depth2 = scale * raw depth2
    d1 = X1[:, 2] * (1 + rng.normal(0, depth_noise, n))
    d2 = X2[:, 2] / scale_gt * (1 + rng.normal(0, depth_noise, n))
    out = rng.random(n) < outlier_ratio
    x2[out] = rng.uniform(-0.6, 0.6, (out.sum(), 2))
    return R, c, scale_gt, x1, x2, d1, d2


def params(seed=0):
    p = pt.solvers.RansacParameters()
    p.error_thresh = (THRESH_PX / FOCAL) ** 2
    p.failure_probability = 1e-4
    p.min_iterations = 10
    p.max_iterations = 5000
    p.use_lo = True
    p.rng = pt.solvers.RandomNumberGenerator(seed)
    return p


def sampson_sq(R, position, x1, x2):
    t = -R @ position
    E = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]]) @ R
    h1, h2 = np.c_[x1, np.ones(len(x1))], np.c_[x2, np.ones(len(x2))]
    Ex1, Etx2 = h1 @ E.T, h2 @ E
    num = np.sum(h2 * Ex1, axis=1) ** 2
    den = Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2
    return num / den


def test_lo_parameters_exposed():
    p = pt.solvers.RansacParameters()
    assert p.lo_start_iterations == 10
    assert p.lo_max_refinements == 5
    assert pt.sfm.EstimateTwoViewInfoOptions().monodepth_reprojection_weight == 0.0


@pytest.mark.parametrize("seed", range(5))
def test_relative_pose_lo_inliers_match_returned_model(seed):
    R, c, _, x1, x2, _, _ = scene(seed)
    corr = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(a), pt.sfm.Feature(b)) for a, b in zip(x1, x2)]
    for start in (0, 5, 50):
        p = params(seed)
        p.lo_start_iterations = start
        ok, rel, summ = pt.sfm.EstimateRelativePose(p, pt.sfm.RansacType.RANSAC, corr)
        assert ok
        assert summ.num_lo_iterations > 0
        err = sampson_sq(np.asarray(rel.rotation), np.asarray(rel.position), x1, x2)
        expected = set(np.flatnonzero(err < p.error_thresh).tolist())
        # The returned inliers are those of the returned (refined) model.
        # EstimateRelativePose additionally removes inliers that triangulate
        # behind a camera.
        assert set(summ.inliers) <= expected
        assert len(expected) - len(summ.inliers) <= 3
        assert angle_deg(np.asarray(rel.rotation) @ R.T) < 0.5


@pytest.mark.parametrize("seed", range(5))
def test_monodepth_relative_pose_depth_weight(seed):
    R, c, scale_gt, x1, x2, d1, d2 = scene(seed)
    corr = [pt.matching.FeatureCorrespondence(pt.sfm.Feature(a, da), pt.sfm.Feature(b, db))
            for a, b, da, db in zip(x1, x2, d1, d2)]
    direction = c / np.linalg.norm(c)
    for weight in (0.0, 1.0):
        ok, res, summ = pt.sfm.EstimateMonoDepthRelativePose(
            params(seed), pt.sfm.RansacType.RANSAC, corr, weight)
        assert ok
        assert angle_deg(np.asarray(res.rotation) @ R.T) < 0.5
        assert np.degrees(np.arccos(np.clip(np.asarray(res.position) @ direction, -1, 1))) < 2.0
        assert abs(res.scale / scale_gt - 1) < 0.05
    # The default is the Sampson-only refinement.
    ok_default, res_default, _ = pt.sfm.EstimateMonoDepthRelativePose(
        params(seed), pt.sfm.RansacType.RANSAC, corr)
    ok_zero, res_zero, _ = pt.sfm.EstimateMonoDepthRelativePose(
        params(seed), pt.sfm.RansacType.RANSAC, corr, 0.0)
    assert ok_default and ok_zero
    np.testing.assert_allclose(np.asarray(res_default.rotation), np.asarray(res_zero.rotation))
