import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
import pytheia as pt


def generate_synthetic_scene(
    num_points=20,
    rot_deg=(2.0, -1.0, 3.0),
    translation=np.array([0.05, -0.02, -1.0]),
    noise_std=0.0,
    seed=42,
):
    np.random.seed(seed)
    # Generate 3D points in front of both cameras (Z in [3.0, 10.0])
    pts_x = np.random.uniform(-1.5, 1.5, size=num_points)
    pts_y = np.random.uniform(-1.5, 1.5, size=num_points)
    pts_z = np.random.uniform(3.0, 10.0, size=num_points)
    pts3d = np.column_stack([pts_x, pts_y, pts_z])

    # Rotation and translation
    rot = R.from_euler("xyz", rot_deg, degrees=True).as_matrix()
    trans = np.array(translation, dtype=float)
    trans = trans / np.linalg.norm(trans)

    # Project to camera 1: x1 = X / Z
    x1 = pts3d[:, :2] / pts3d[:, 2:3]

    # Transform to camera 2: X2 = R * X1 + t
    pts3d_cam2 = (rot @ pts3d.T).T + trans
    x2 = pts3d_cam2[:, :2] / pts3d_cam2[:, 2:3]

    if noise_std > 0:
        x1 += np.random.normal(0, noise_std, size=x1.shape)
        x2 += np.random.normal(0, noise_std, size=x2.shape)

    return pts3d, x1, x2, rot, trans


def test_fast_iterative_five_point_minimal_exact():
    _, x1, x2, rot_gt, trans_gt = generate_synthetic_scene(
        num_points=5, rot_deg=(1.0, 0.5, -0.5), translation=np.array([0.02, 0.01, -1.0]), noise_std=0.0
    )

    opts = pt.sfm.FastIterativeFivePointOptions()
    opts.max_iterations = 15

    success, Es, relative_poses = pt.sfm.FastIterativeFivePointRelativePose(
        x1[:5], x2[:5], opts
    )
    assert success
    assert len(Es) == 1
    assert len(relative_poses) == 1

    pose = relative_poses[0]
    # Check angular difference
    rot_diff = R.from_matrix(pose.rotation @ rot_gt.T).magnitude() * 180.0 / np.pi
    assert rot_diff < 0.1

    t_dir_est = pose.position / np.linalg.norm(pose.position)
    t_dot = np.abs(np.dot(t_dir_est, trans_gt))
    assert t_dot > 0.999


def test_fast_iterative_five_point_overdetermined():
    _, x1, x2, rot_gt, trans_gt = generate_synthetic_scene(
        num_points=20, rot_deg=(2.0, -1.5, 1.0), translation=np.array([-0.05, 0.02, -1.0]), noise_std=1e-5
    )

    opts = pt.sfm.FastIterativeFivePointOptions()
    opts.max_iterations = 15

    success, Es, relative_poses = pt.sfm.FastIterativeFivePointRelativePose(
        x1, x2, opts
    )
    assert success
    assert len(relative_poses) == 1

    pose = relative_poses[0]
    rot_diff = R.from_matrix(pose.rotation @ rot_gt.T).magnitude() * 180.0 / np.pi
    assert rot_diff < 0.5

    t_dir_est = pose.position / np.linalg.norm(pose.position)
    t_dot = np.abs(np.dot(t_dir_est, trans_gt))
    assert t_dot > 0.99


def test_sturm_five_point_standalone():
    _, x1, x2, rot_gt, trans_gt = generate_synthetic_scene(
        num_points=5, rot_deg=(3.0, 2.0, -1.0), translation=np.array([0.1, -0.05, -1.0]), noise_std=0.0
    )

    num_sol, Es = pt.sfm.FivePointRelativePoseSturm(x1[:5], x2[:5])
    assert num_sol > 0
    assert len(Es) == num_sol


def test_twoview_estimation_method_enum():
    assert hasattr(pt.sfm, "TwoViewEstimationMethod")
    assert pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STEWENIUS is not None
    assert pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STURM is not None
    assert pt.sfm.TwoViewEstimationMethod.FAST_ITERATIVE_FIVE_POINT is not None
    assert pt.sfm.TwoViewEstimationMethod.MONODEPTH_THREE_POINT is not None


def test_estimate_relative_pose_with_fast_iterative():
    _, x1, x2, rot_gt, trans_gt = generate_synthetic_scene(
        num_points=50, rot_deg=(2.0, -1.0, 0.5), translation=np.array([0.02, 0.01, -1.0]), noise_std=1e-4
    )

    corrs = [
        pt.matching.FeatureCorrespondence(pt.sfm.Feature(x1[i]), pt.sfm.Feature(x2[i]))
        for i in range(len(x1))
    ]

    ransac_params = pt.solvers.RansacParameters()
    ransac_params.error_thresh = 0.005
    ransac_params.min_iterations = 10
    ransac_params.max_iterations = 100
    ransac_params.essential_solver_type = pt.sfm.TwoViewEstimationMethod.FAST_ITERATIVE_FIVE_POINT

    success, pose, summary = pt.sfm.EstimateRelativePose(
        ransac_params, pt.sfm.RansacType.RANSAC, corrs
    )
    assert success
    assert len(summary.inliers) > 20

    rot_diff = R.from_matrix(pose.rotation @ rot_gt.T).magnitude() * 180.0 / np.pi
    assert rot_diff < 2.0

    t_dir_est = pose.position / np.linalg.norm(pose.position)
    assert np.abs(np.dot(t_dir_est, trans_gt)) > 0.98


def test_estimate_twoview_info_with_all_methods():
    _, x1, x2, rot_gt, trans_gt = generate_synthetic_scene(
        num_points=60, rot_deg=(1.5, -0.8, 0.2), translation=np.array([0.01, 0.02, -1.0]), noise_std=1e-4
    )

    corrs = [
        pt.matching.FeatureCorrespondence(pt.sfm.Feature(x1[i] * 1000.0 + 500.0), pt.sfm.Feature(x2[i] * 1000.0 + 500.0))
        for i in range(len(x1))
    ]

    cam1 = pt.sfm.Camera()
    cam1.SetFocalLength(1000.0)
    cam1.SetPrincipalPoint(500.0, 500.0)
    cam1.SetImageSize(1000, 1000)

    cam2 = pt.sfm.Camera()
    cam2.SetFocalLength(1000.0)
    cam2.SetPrincipalPoint(500.0, 500.0)
    cam2.SetImageSize(1000, 1000)

    methods = [
        pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STURM,
        pt.sfm.TwoViewEstimationMethod.FIVE_POINT_STEWENIUS,
        pt.sfm.TwoViewEstimationMethod.FAST_ITERATIVE_FIVE_POINT,
    ]

    for method in methods:
        opts = pt.sfm.EstimateTwoViewInfoOptions()
        opts.max_sampson_error_pixels = 4.0
        opts.estimation_method = method
        opts.min_ransac_iterations = 10
        opts.max_ransac_iterations = 200

        success, info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
            opts, cam1.CameraIntrinsicsPriorFromIntrinsics(), cam2.CameraIntrinsicsPriorFromIntrinsics(), corrs
        )
        assert success, f"Failed for method {method}"
        assert len(inlier_indices) > 30

        rot_mat = R.from_rotvec(np.array(info.rotation_2, copy=True)).as_matrix()
        rot_diff = R.from_matrix(rot_mat @ rot_gt.T).magnitude() * 180.0 / np.pi
        assert rot_diff < 2.0, f"Rotation error too high ({rot_diff} deg) for {method}"

        t_dir = info.position_2 / np.linalg.norm(info.position_2)
        assert np.abs(np.dot(t_dir, trans_gt)) > 0.95, f"Translation direction error for {method}"
