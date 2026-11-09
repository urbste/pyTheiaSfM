"""Tests for the experimental initialization-free (OSE / VarPro) module."""

import numpy as np
import pytest

import pytheia as pt

WIDTH, HEIGHT = 1200, 900
FOCAL = 800.0


def make_camera():
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    cam.SetImageSize(WIDTH, HEIGHT)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(FOCAL)
    intr.SetPrincipalPoint(WIDTH / 2.0, HEIGHT / 2.0)
    return cam


def rotmat_to_aa(R):
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def rotation_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def make_scene(n_views=10, n_points=150, keep_prob=0.8, pixel_noise=0.0,
               seed=0):
    """Cameras on a ring looking at points around the origin.

    Returns the reconstruction (poses set to ground truth), a view graph with
    ground-truth relative rotations, and a dict of ground-truth poses.
    """
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    base = make_camera()
    gt = {}
    for i in range(n_views):
        vid = recon.AddView(str(i), i, float(i))
        theta = 1.2 * np.pi * i / n_views  # partial ring
        radius = 6.0 + rng.uniform(-0.5, 0.5)
        center = np.array([radius * np.sin(theta), rng.uniform(-0.5, 0.5),
                           -radius * np.cos(theta)])
        cam = recon.MutableView(vid).MutableCamera()
        cam.DeepCopy(base)
        cam.SetPosition(center)
        cam.SetOrientationFromAngleAxis(
            np.array([0.0, theta, 0.0]) + rng.normal(scale=0.02, size=3))
        gt[vid] = (np.asarray(cam.GetOrientationAsRotationMatrix()),
                   np.asarray(cam.GetPosition()))

    for _ in range(n_points):
        xyz = rng.uniform(-1.5, 1.5, size=3)
        obs = []
        for vid in recon.ViewIds():
            if rng.uniform() > keep_prob:
                continue
            depth, pixel = recon.View(vid).Camera().ProjectPoint(
                np.append(xyz, 1.0))
            if depth > 0 and 0 < pixel[0] < WIDTH and 0 < pixel[1] < HEIGHT:
                obs.append((vid, pixel + rng.normal(scale=pixel_noise, size=2)))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack()
        for vid, pixel in obs:
            recon.AddObservation(vid, tid, pt.sfm.Feature(pixel))

    view_graph = pt.sfm.ViewGraph()
    vids = sorted(recon.ViewIds())
    for a, i in enumerate(vids):
        for j in vids[a + 1:]:
            common = set(recon.View(i).TrackIds()) & set(recon.View(j).TrackIds())
            if len(common) < 20:
                continue
            info = pt.sfm.TwoViewInfo()
            info.rotation_2 = rotmat_to_aa(gt[j][0] @ gt[i][0].T)
            info.num_verified_matches = len(common)
            view_graph.AddEdge(i, j, info)
    return recon, view_graph, gt


def init_free_options(formulation=pt.sfm.OseFormulationType.POSE_ROT):
    opts = pt.sfm.InitFreeOptions()
    opts.formulation = formulation
    opts.alpha = 0.05
    opts.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
    opts.varpro.max_num_iterations = 300
    return opts


def projective_reprojection_errors(problem, solution):
    """|m - dehom(P [X; 1])| in normalized image coordinates."""
    cams = solution.cameras
    errs = []
    for k in range(problem.NumObservations()):
        P = cams[problem.observation_camera[k]]
        X = solution.points[:, problem.observation_point[k]]
        x = P @ np.append(X, 1.0)
        errs.append(np.linalg.norm(x[:2] / x[2] - problem.observations[:, k]))
    return np.array(errs)


def test_enum_and_options_are_exposed():
    assert pt.sfm.ReconstructionEstimatorType.INIT_FREE is not None
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.init_free_options.alpha = 0.2
    opts.init_free_options.varpro.max_num_iterations = 7
    assert opts.init_free_options.alpha == pytest.approx(0.2)
    assert opts.init_free_options.varpro.max_num_iterations == 7
    # The new enums must not clobber existing module-level names.
    assert pt.sfm.LINEAR == pt.sfm.GlobalRotationEstimatorType.LINEAR


def test_build_problem_layout():
    recon, view_graph, _ = make_scene()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph,
                                              init_free_options())
    assert ok, err
    assert problem.NumCameras() == recon.NumViews()
    assert problem.NumPoints() == recon.NumTracks()
    assert problem.observations.shape == (2, problem.NumObservations())
    offsets = np.asarray(problem.point_offsets)
    assert offsets[0] == 0 and offsets[-1] == problem.NumObservations()
    assert np.all(np.diff(offsets) >= 3)
    # One term per view graph edge plus one self term per camera.
    assert len(problem.rotation_terms) == (view_graph.NumEdges() +
                                           problem.NumCameras())


def test_pose_rot_requires_view_graph():
    recon, _, _ = make_scene()
    ok, _, err = pt.sfm.BuildOseProblem(recon, None, init_free_options())
    assert not ok and "view graph" in err


@pytest.mark.parametrize("formulation", [pt.sfm.OseFormulationType.POSE,
                                         pt.sfm.OseFormulationType.POSE_ROT])
def test_varpro_reaches_global_minimum_from_random_start(formulation):
    recon, view_graph, gt = make_scene()
    opts = init_free_options(formulation)
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init,
                                       opts.seed)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message
    assert sol.converged, sol.message
    assert np.all(np.diff(sol.cost_history) <= 0.0)

    # The random start reaches the same minimum as a start at ground truth.
    # pOSE's affine term is not zero at the optimum, so the cost is not 0.
    gt_cams = [np.hstack([gt[v][0], (-gt[v][0] @ gt[v][1])[:, None]])
               for v in problem.view_ids]
    ok, ref = pt.sfm.SolveOse(problem, opts, gt_cams)
    assert ok and ref.converged, ref.message
    assert sol.final_cost == pytest.approx(ref.final_cost, rel=1e-6)

    # The OSE minimum is close to a projective reconstruction; the remaining
    # sub-pixel bias comes from the alpha-weighted affine term.
    errs = projective_reprojection_errors(problem, sol)
    assert np.median(errs) < 1.0 / FOCAL


def test_estimator_recovers_noise_free_scene():
    recon, view_graph, gt = make_scene(seed=3)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options()
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    diag = estimator.Diagnostics()
    assert diag.metric_upgrade_success
    assert diag.fraction_in_front > 0.99

    vids = sorted(gt)
    assert all(recon.View(v).IsEstimated() for v in vids)
    est_R = {v: np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids}
    worst = 0.0
    for a, i in enumerate(vids):
        for j in vids[a + 1:]:
            rel_est = est_R[j] @ est_R[i].T
            rel_gt = gt[j][0] @ gt[i][0].T
            worst = max(worst, rotation_angle_deg(rel_est @ rel_gt.T))
    assert worst < 0.1, worst

    # Camera centers agree up to a similarity transform.
    est_c = np.array([recon.View(v).Camera().GetPosition() for v in vids])
    gt_c = np.array([gt[v][1] for v in vids])
    est0 = est_c - est_c.mean(0)
    gt0 = gt_c - gt_c.mean(0)
    U, S, Vt = np.linalg.svd(gt0.T @ est0)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / np.sum(est0 ** 2)
    residual = np.linalg.norm(s * est0 @ R.T - gt0, axis=1)
    assert residual.max() < 1e-3 * np.linalg.norm(gt0, axis=1).mean()


def test_unimplemented_choices_fail_cleanly():
    recon, view_graph, _ = make_scene()
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)

    opts.formulation = pt.sfm.OseFormulationType.EXPOSE
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert not ok and "not implemented" in sol.message

    opts = init_free_options()
    opts.varpro.linear_solver_type = pt.sfm.ReducedCameraSolverType.POWER_SERIES
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert not ok and "not implemented" in sol.message


@pytest.mark.skipif(not pt.sfm.CudaReducedCameraSolverAvailable(),
                    reason="built without THEIA_WITH_CUDA_SOLVERS or no GPU")
def test_cuda_dense_matches_cpu():
    recon, view_graph, _ = make_scene()
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 1)
    ok_cpu, cpu = pt.sfm.SolveOse(problem, opts, init)
    opts.varpro.linear_solver_type = (
        pt.sfm.ReducedCameraSolverType.CUDA_DENSE_CHOLESKY)
    ok_gpu, gpu = pt.sfm.SolveOse(problem, opts, init)
    assert ok_cpu and ok_gpu, gpu.message
    assert gpu.final_cost == pytest.approx(cpu.final_cost, rel=1e-6, abs=1e-12)
