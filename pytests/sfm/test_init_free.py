"""Tests for the experimental initialization-free (OSE / VarPro) module."""

import numpy as np
import pytest

import pytheia as pt

WIDTH, HEIGHT = 1200, 900
FOCAL = 800.0


def make_camera(focal=FOCAL):
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(pt.sfm.CameraIntrinsicsModelType.PINHOLE)
    cam.SetImageSize(WIDTH, HEIGHT)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(focal)
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
               seed=0, focal_lengths=None, translation=False):
    """Cameras on a ring looking at points around the origin.

    Returns the reconstruction (poses set to ground truth), a view graph with
    ground-truth relative rotations, and a dict of ground-truth poses.
    focal_lengths: optional per-view focal lengths in pixels.
    translation: pure translation instead. All cameras have the same
    orientation (no rotation) and their centres are spread over the plane
    z ~ -6. Varying focal lengths are then not determined by the views.
    """
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    base = make_camera()
    gt = {}
    for i in range(n_views):
        vid = recon.AddView(str(i), i, float(i))
        theta = 1.2 * np.pi * i / n_views  # partial ring
        radius = 6.0 + rng.uniform(-0.5, 0.5)
        cam = recon.MutableView(vid).MutableCamera()
        cam.DeepCopy(base if focal_lengths is None
                     else make_camera(focal_lengths[i]))
        if translation:
            center = np.array([rng.uniform(-3.0, 3.0), rng.uniform(-1.0, 1.0),
                               -6.0 + rng.uniform(-0.5, 0.5)])
            cam.SetPosition(center)
            cam.SetOrientationFromAngleAxis(np.zeros(3))
        else:
            center = np.array([radius * np.sin(theta), rng.uniform(-0.5, 0.5),
                               -radius * np.cos(theta)])
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
    assert opts.init_free_options.min_fraction_in_front == pytest.approx(0.9)
    assert opts.init_free_options.max_mean_orthogonality_error == (
        pytest.approx(0.3))
    assert opts.init_free_options.min_upgrade_conditioning == pytest.approx(3e-3)
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


def test_unsupported_choices_fail_cleanly():
    recon, view_graph, _ = make_scene()
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)

    with pytest.raises(ValueError):
        pt.sfm.ProjectiveRmsError(problem, init, np.zeros((2, 5)))

    # RPOSE cameras are radial (2x4): no projective refinement.
    opts = init_free_options(pt.sfm.OseFormulationType.RPOSE)
    opts.projective_refinement.max_num_iterations = 10
    ok, _, _, err = pt.sfm.RunProjectiveStages(problem, opts)
    assert not ok and "radial" in err


@pytest.mark.parametrize("solver", [
    pt.sfm.ReducedCameraSolverType.DENSE_CHOLESKY,
    pt.sfm.ReducedCameraSolverType.SPARSE_CHOLESKY,
    pt.sfm.ReducedCameraSolverType.POWER_SERIES])
def test_results_do_not_depend_on_thread_count(solver):
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=2)
    opts = init_free_options()
    opts.varpro.linear_solver_type = solver
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    results = []
    for num_threads in [1, 8]:
        opts.varpro.num_threads = num_threads
        ok, sol = pt.sfm.SolveOse(problem, opts, init)
        assert ok and sol.converged, sol.message
        results.append(sol)
    assert results[0].final_cost == results[1].final_cost
    assert np.array_equal(np.asarray(results[0].cameras),
                          np.asarray(results[1].cameras))


@pytest.mark.parametrize("formulation", [pt.sfm.OseFormulationType.POSE,
                                         pt.sfm.OseFormulationType.ROSE,
                                         pt.sfm.OseFormulationType.EXPOSE,
                                         pt.sfm.OseFormulationType.RPOSE])
def test_formulations_converge_on_noisy_scene(formulation):
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=2)
    opts = init_free_options(formulation)
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok and sol.converged, sol.message
    if formulation == pt.sfm.OseFormulationType.RPOSE:
        # Radial cameras: the first two rows reproject along the radial
        # direction of every observation.
        cams = sol.cameras
        cos = []
        for k in range(problem.NumObservations()):
            P = cams[problem.observation_camera[k]]
            x = P[:2] @ np.append(sol.points[:, problem.observation_point[k]],
                                  1.0)
            m = problem.observations[:, k]
            cos.append(abs(x @ m) / (np.linalg.norm(x) * np.linalg.norm(m)))
        assert np.median(cos) > 0.999
        return
    rms = np.sqrt(np.mean(projective_reprojection_errors(problem, sol) ** 2))
    if formulation == pt.sfm.OseFormulationType.ROSE:
        # rOSE's depth regularizer biases the projective solution strongly.
        assert rms * FOCAL < 20.0
    else:
        # pOSE ~1.4 px, expOSE ~1.3 px for 1 px noise.
        assert rms * FOCAL < 1.6


def test_cauchy_loss_suppresses_outliers():
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=2)
    rng = np.random.default_rng(7)
    bad = set()
    for tid in sorted(recon.TrackIds()):
        for vid in sorted(recon.Track(tid).ViewIds()):
            if rng.uniform() < 0.1:
                p = np.asarray(recon.View(vid).GetFeature(tid).point)
                p = p + rng.uniform(-150.0, 150.0, 2)
                recon.MutableView(vid).UpdateFeature(tid, pt.sfm.Feature(p))
                bad.add((vid, tid))
    opts = init_free_options(pt.sfm.OseFormulationType.POSE)
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    inlier = np.array([
        (problem.view_ids[problem.observation_camera[k]],
         problem.track_ids[problem.observation_point[k]]) not in bad
        for k in range(problem.NumObservations())])
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)

    medians = {}
    for loss in [pt.sfm.VarProRobustLossType.NONE,
                 pt.sfm.VarProRobustLossType.CAUCHY]:
        opts.varpro.robust_loss_type = loss
        opts.varpro.robust_loss_scale = 5.0 / FOCAL
        ok, sol = pt.sfm.SolveOse(problem, opts, init)
        assert ok and sol.converged, sol.message
        errs = projective_reprojection_errors(problem, sol) * FOCAL
        medians[loss] = np.median(errs[inlier])
    assert medians[pt.sfm.VarProRobustLossType.NONE] > 5.0
    assert medians[pt.sfm.VarProRobustLossType.CAUCHY] < 2.0


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


@pytest.mark.parametrize("given_translation", [True, False])
def test_two_view_rotation_weight(given_translation):
    recon, _, gt = make_scene(n_views=6, pixel_noise=0.5, seed=1)
    i, j = 0, 1  # camera b = i, camera a = j
    R = gt[j][0] @ gt[i][0].T
    t = gt[j][0] @ (gt[i][1] - gt[j][1])
    common = sorted(set(recon.View(i).TrackIds()) &
                    set(recon.View(j).TrackIds()))

    def normalized(v, tid):
        ray = recon.View(v).Camera().PixelToNormalizedCoordinates(
            np.asarray(recon.View(v).GetFeature(tid).point))
        return ray[:2] / ray[2]

    m_a = np.array([normalized(j, tid) for tid in common]).T
    m_b = np.array([normalized(i, tid) for tid in common]).T
    ok, w = pt.sfm.ComputeTwoViewRotationWeight(
        m_a, m_b, R, t if given_translation else np.zeros(3), 2.0, 10)
    assert ok
    assert w.num_points == len(common)
    t_err = np.degrees(np.arccos(np.clip(
        w.translation @ t / np.linalg.norm(t), -1.0, 1.0)))
    assert t_err < (1e-4 if given_translation else 0.5)

    L = np.asarray(w.sqrt_weight)
    W = L @ L
    H = np.asarray(w.rotation_information)
    assert np.allclose(L, L.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(H) > 0.0)
    # Tangent directions: the weight reproduces the rotation information.
    omega = np.array([0.3, -0.2, 0.5])
    v = (skew(omega) @ R).reshape(-1)
    assert v @ W @ v == pytest.approx(omega @ H @ omega, rel=1e-9)
    # Normal directions: scaled by normal_weight.
    S = np.array([[1.0, 2.0, 0.0], [2.0, 0.0, 1.0], [0.0, 1.0, 3.0]])
    n = (S @ R).reshape(-1)
    assert np.allclose(W @ n, 2.0 * n, atol=1e-9)

    ok, _ = pt.sfm.ComputeTwoViewRotationWeight(m_a[:, :5], m_b[:, :5], R, t,
                                                1.0, 10)
    assert not ok


def test_estimator_with_two_view_rotation_weights():
    recon, view_graph, gt = make_scene(n_views=15, pixel_noise=1.0, seed=6)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options()
    opts.init_free_options.rotation_weighting = (
        pt.sfm.RotationWeightingType.TWO_VIEW_INFORMATION)
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10
    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    assert estimator.Diagnostics().fraction_in_front > 0.99
    vids = sorted(gt)
    est_R = {v: np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids}
    worst = max(rotation_angle_deg((est_R[j] @ est_R[i].T) @
                                   (gt[j][0] @ gt[i][0].T).T)
                for a, i in enumerate(vids) for j in vids[a + 1:])
    assert worst < 0.5, worst


@pytest.mark.parametrize("pixel_noise", [0.0, 1.0])
def test_power_series_matches_dense(pixel_noise):
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=pixel_noise,
                                      seed=2)
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, dense = pt.sfm.SolveOse(problem, opts, init)
    assert ok and dense.converged, dense.message

    opts.varpro.linear_solver_type = pt.sfm.ReducedCameraSolverType.POWER_SERIES
    opts.varpro.power_series_max_iterations = 20
    ok, ps = pt.sfm.SolveOse(problem, opts, init)
    assert ok and ps.converged, ps.message
    assert np.all(np.diff(ps.cost_history) <= 0.0)
    assert ps.final_cost == pytest.approx(dense.final_cost, rel=1e-8)

    opts.varpro.power_series_max_iterations = 0
    ok, ps = pt.sfm.SolveOse(problem, opts, init)
    assert not ok and "at least 1" in ps.message


@pytest.mark.parametrize("formulation", [pt.sfm.OseFormulationType.POSE,
                                         pt.sfm.OseFormulationType.POSE_ROT])
def test_sparse_cholesky_matches_dense(formulation):
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=2)
    opts = init_free_options(formulation)
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, dense = pt.sfm.SolveOse(problem, opts, init)
    assert ok and dense.converged, dense.message
    opts.varpro.linear_solver_type = (
        pt.sfm.ReducedCameraSolverType.SPARSE_CHOLESKY)
    ok, sparse = pt.sfm.SolveOse(problem, opts, init)
    assert ok and sparse.converged, sparse.message
    assert len(sparse.cost_history) == len(dense.cost_history)
    assert sparse.final_cost == pytest.approx(dense.final_cost, rel=1e-10)


@pytest.mark.parametrize("pixel_noise", [0.0, 1.0])
def test_projective_refinement_removes_ose_bias(pixel_noise):
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=pixel_noise,
                                      seed=2)
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message
    ose_rms = pt.sfm.ProjectiveRmsError(problem, sol.cameras, sol.points)

    refine_opts = pt.sfm.ProjectiveRefinementOptions()
    refine_opts.max_num_iterations = 50
    # 3 x N and 4 x N points give the same result.
    ok, res = pt.sfm.RefineProjectiveReconstruction(
        problem, sol.cameras, sol.points, refine_opts)
    assert ok, res.message
    homogeneous = np.vstack([sol.points, np.ones((1, sol.points.shape[1]))])
    ok, res4 = pt.sfm.RefineProjectiveReconstruction(
        problem, sol.cameras, homogeneous, refine_opts)
    assert ok, res4.message
    assert res4.final_rms_error == pytest.approx(res.final_rms_error,
                                                 rel=1e-6, abs=1e-12)

    assert res.initial_rms_error == pytest.approx(ose_rms, rel=1e-9)
    assert res.final_rms_error < res.initial_rms_error
    assert res.points.shape == (4, problem.NumPoints())
    assert pt.sfm.ProjectiveRmsError(problem, res.cameras, res.points) == (
        pytest.approx(res.final_rms_error, rel=1e-9))
    if pixel_noise == 0.0:
        assert res.final_rms_error * FOCAL < 1e-4
    else:
        # The 2D RMS of 1 px isotropic noise is about sqrt(2) px.
        assert res.final_rms_error * FOCAL < 1.5 * pixel_noise


def test_run_projective_stages_selects_restart():
    recon, view_graph, _ = make_scene(n_views=12, pixel_noise=0.5, seed=4)
    opts = init_free_options()
    opts.num_random_restarts = 3
    opts.projective_refinement.max_num_iterations = 30
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err

    for selection in [pt.sfm.InitFreeRestartSelection.REPROJECTION_ERROR,
                      pt.sfm.InitFreeRestartSelection.OSE_COST]:
        opts.restart_selection = selection
        ok, best, diag, err = pt.sfm.RunProjectiveStages(problem, opts)
        assert ok, err
        assert len(diag.restarts) == 3
        assert [r.seed for r in diag.restarts] == [opts.seed + k
                                                   for k in range(3)]
        assert all(r.refined_rms_error >= 0.0 for r in diag.restarts)
        if selection == pt.sfm.InitFreeRestartSelection.OSE_COST:
            scores = [r.final_cost for r in diag.restarts]
        else:
            scores = [r.refined_rms_error for r in diag.restarts]
        assert diag.best_restart == int(np.argmin(scores))
        assert best.points.shape == (4, problem.NumPoints())
        assert pt.sfm.ProjectiveRmsError(problem, best.cameras, best.points) == (
            pytest.approx(diag.restarts[diag.best_restart].refined_rms_error,
                          rel=1e-9))

    # Without refinement the refined RMS is reported as -1.
    opts.projective_refinement.max_num_iterations = 0
    ok, best, diag, err = pt.sfm.RunProjectiveStages(problem, opts)
    assert ok, err
    assert all(r.refined_rms_error == -1.0 for r in diag.restarts)


def test_metric_upgrade_accepts_cartesian_and_homogeneous_points():
    recon, view_graph, _ = make_scene()
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message
    ok3, up3, err3 = pt.sfm.UpgradeToMetricCalibrated(problem, sol.cameras,
                                                       sol.points)
    # Scaling a homogeneous column must not change the result.
    homogeneous = np.vstack([sol.points, np.ones((1, sol.points.shape[1]))])
    homogeneous *= np.linspace(0.5, 2.0, homogeneous.shape[1])
    ok4, up4, err4 = pt.sfm.UpgradeToMetricCalibrated(problem, sol.cameras,
                                                       homogeneous)
    assert ok3 and ok4, (err3, err4)
    assert up4.fraction_in_front == pytest.approx(up3.fraction_in_front)
    assert np.allclose(up4.points, up3.points, atol=1e-8)


def test_estimator_with_projective_refinement_on_noisy_scene():
    recon, view_graph, gt = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options()
    opts.init_free_options.num_random_restarts = 2
    opts.init_free_options.projective_refinement.max_num_iterations = 30
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    assert "after refinement" in summary.message
    diag = estimator.Diagnostics()
    assert len(diag.restarts) == 2
    best = diag.restarts[diag.best_restart]
    assert 0.0 <= best.refined_rms_error < best.ose_rms_error
    assert diag.fraction_in_front > 0.99

    vids = sorted(gt)
    est_R = {v: np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids}
    worst = 0.0
    for a, i in enumerate(vids):
        for j in vids[a + 1:]:
            rel_err = (est_R[j] @ est_R[i].T) @ (gt[j][0] @ gt[i][0].T).T
            worst = max(worst, rotation_angle_deg(rel_err))
    assert worst < 0.5, worst


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


@pytest.mark.skipif(not pt.sfm.CudaReducedCameraSolverAvailable(),
                    reason="built without THEIA_WITH_CUDA_SOLVERS or no GPU")
def test_cuda_power_series_matches_cpu():
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=2)
    opts = init_free_options()
    opts.varpro.linear_solver_type = pt.sfm.ReducedCameraSolverType.POWER_SERIES
    opts.varpro.power_series_max_iterations = 20
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok_cpu, cpu = pt.sfm.SolveOse(problem, opts, init)
    opts.varpro.linear_solver_type = (
        pt.sfm.ReducedCameraSolverType.CUDA_POWER_SERIES)
    ok_gpu, gpu = pt.sfm.SolveOse(problem, opts, init)
    assert ok_cpu and ok_gpu, gpu.message
    assert gpu.converged
    assert gpu.final_cost == pytest.approx(cpu.final_cost, rel=1e-8)


def uncalibrated_options():
    opts = init_free_options(pt.sfm.OseFormulationType.POSE)
    opts.observation_normalization = (
        pt.sfm.OseObservationNormalization.PIXEL_ISOTROPIC)
    opts.projective_refinement.max_num_iterations = 50
    return opts


@pytest.mark.parametrize("pixel_noise", [0.0, 0.5])
def test_uncalibrated_upgrade_recovers_focal_lengths(pixel_noise):
    focals = np.random.default_rng(0).uniform(650.0, 1000.0, 15)
    recon, view_graph, gt = make_scene(n_views=15, n_points=200,
                                       pixel_noise=pixel_noise, seed=3,
                                       focal_lengths=focals)
    opts = uncalibrated_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    ok, best, _, err = pt.sfm.RunProjectiveStages(problem, opts)
    assert ok, err
    scale = np.array([np.asarray(T)[0, 0] for T in problem.normalizations])
    true_f = focals[problem.view_ids]

    def rotation_errors(result):
        R, vids = result.rotations, problem.view_ids
        return np.array([
            rotation_angle_deg((R[b] @ R[a].T) @
                               (gt[vids[b]][0] @ gt[vids[a]][0].T).T)
            for a in range(len(vids)) for b in range(a + 1, len(vids))])

    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    ok, linear, err = pt.sfm.UpgradeToMetric(problem, best.cameras,
                                             best.points, upgrade_opts)
    assert ok, err
    assert not linear.nonlinear_refinement_run
    linear_f_err = np.abs(np.array(linear.focal_lengths) / scale / true_f - 1)
    # The priors of the linear equations bias the focal lengths (~1 %).
    assert linear_f_err.max() < 0.03

    upgrade_opts.nonlinear_refinement = True
    ok, refined, err = pt.sfm.UpgradeToMetric(problem, best.cameras,
                                              best.points, upgrade_opts)
    assert ok, err
    assert refined.nonlinear_refinement_run
    assert refined.fraction_in_front > 0.99
    f_err = np.abs(np.array(refined.focal_lengths) / scale / true_f - 1)
    assert (refined.mean_orthogonality_error <
            0.5 * refined.linear_mean_orthogonality_error)
    if pixel_noise == 0.0:
        assert f_err.max() < 1e-5
        assert rotation_errors(refined).max() < 1e-3
    else:
        assert f_err.max() < 0.01
        assert (rotation_errors(refined).mean() <
                rotation_errors(linear).mean())


@pytest.mark.parametrize("shared", [False, True])
def test_estimator_self_calibrates(shared):
    rng = np.random.default_rng(0)
    focals = (np.full(15, 900.0) if shared
              else rng.uniform(650.0, 1000.0, 15))
    recon, view_graph, gt = make_scene(n_views=15, n_points=200,
                                       pixel_noise=0.5, seed=3,
                                       focal_lengths=focals)
    for vid in recon.ViewIds():  # wrong initial focal lengths
        recon.MutableView(vid).MutableCamera().SetFocalLength(500.0)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = uncalibrated_options()
    opts.init_free_options.metric_upgrade = (
        pt.sfm.MetricUpgradeType.LINEAR_AND_NONLINEAR)
    opts.init_free_options.shared_focal_length = shared
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH
    opts.min_num_two_view_inliers = 10
    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message

    vids = sorted(gt)
    diag = estimator.Diagnostics()
    upgrade_f = np.array(diag.focal_lengths)
    assert np.abs(upgrade_f / focals[vids] - 1).max() < 0.01
    if shared:
        assert np.ptp(upgrade_f) == 0.0
    final_f = np.array([recon.View(v).Camera().FocalLength() for v in vids])
    assert np.abs(final_f / focals[vids] - 1).max() < 0.01
    assert recon.View(vids[0]).Camera().PrincipalPointX() == WIDTH / 2.0
    est_R = {v: np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids}
    worst = max(rotation_angle_deg((est_R[j] @ est_R[i].T) @
                                   (gt[j][0] @ gt[i][0].T).T)
                for a, i in enumerate(vids) for j in vids[a + 1:])
    assert worst < 0.3, worst


def test_calibrated_nonlinear_upgrade():
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = init_free_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    ok, best, _, err = pt.sfm.RunProjectiveStages(problem, opts)
    assert ok, err
    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    upgrade_opts.nonlinear_refinement = True
    ok, refined, err = pt.sfm.UpgradeToMetric(problem, best.cameras,
                                              best.points, upgrade_opts)
    assert ok, err
    assert refined.focal_lengths == [1.0] * problem.NumCameras()
    assert (refined.mean_orthogonality_error <=
            refined.linear_mean_orthogonality_error)
    ok, linear, err = pt.sfm.UpgradeToMetricCalibrated(problem, best.cameras,
                                                        best.points)
    assert ok, err
    assert linear.mean_orthogonality_error == pytest.approx(
        refined.linear_mean_orthogonality_error)


def radial_options(normalization=pt.sfm.OseObservationNormalization.CALIBRATED_RAYS,
                   alpha=0.05):
    opts = init_free_options(pt.sfm.OseFormulationType.RPOSE)
    opts.alpha = alpha
    opts.observation_normalization = normalization
    return opts


def pairwise_rotation_errors(R, vids, gt):
    """Relative rotation errors (deg) of all camera pairs against ground truth."""
    errs = []
    for a in range(len(vids)):
        for b in range(a + 1, len(vids)):
            rel_est = R[b] @ R[a].T
            rel_gt = gt[vids[b]][0] @ gt[vids[a]][0].T
            errs.append(rotation_angle_deg(rel_est @ rel_gt.T))
    return np.array(errs)


def test_radial_upgrade_stage_wise():
    recon, view_graph, gt = make_scene(n_views=15, pixel_noise=0.0, seed=2)
    opts = radial_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message

    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    upgrade_opts.radial_cameras = True
    ok, linear, err = pt.sfm.UpgradeToMetric(problem, sol.cameras, sol.points,
                                             upgrade_opts)
    assert ok, err
    assert not linear.nonlinear_refinement_run
    assert linear.fraction_in_front > 0.99
    # The RPOSE affine term (alpha = 0.05) biases the OSE solution; the linear
    # upgrade measures 13 deg worst case here, the refinement 1.3 deg.
    linear_err = pairwise_rotation_errors(np.array(linear.rotations),
                                          problem.view_ids, gt)
    assert linear_err.max() < 20.0, linear_err.max()

    upgrade_opts.nonlinear_refinement = True
    ok, refined, err = pt.sfm.UpgradeToMetric(problem, sol.cameras, sol.points,
                                              upgrade_opts)
    assert ok, err
    assert refined.nonlinear_refinement_run
    assert refined.fraction_in_front > 0.99
    assert refined.mean_orthogonality_error < (
        0.5 * refined.linear_mean_orthogonality_error)
    refined_err = pairwise_rotation_errors(np.array(refined.rotations),
                                           problem.view_ids, gt)
    assert refined_err.max() < 3.0, refined_err.max()


def test_radial_upgrade_needs_five_cameras():
    recon, view_graph, _ = make_scene(n_views=4, pixel_noise=0.0, seed=2)
    opts = radial_options()
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message
    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    upgrade_opts.radial_cameras = True
    ok, _, err = pt.sfm.UpgradeToMetric(problem, sol.cameras, sol.points,
                                        upgrade_opts)
    assert not ok and "5 cameras" in err


@pytest.mark.parametrize("alpha, focal_tol", [(0.05, 0.3), (0.001, 0.02)])
def test_radial_upgrade_recovers_focal_lengths(alpha, focal_tol):
    focals = np.random.default_rng(0).uniform(650.0, 1000.0, 15)
    recon, view_graph, gt = make_scene(n_views=15, n_points=200,
                                       pixel_noise=0.0, seed=3,
                                       focal_lengths=focals)
    opts = radial_options(pt.sfm.OseObservationNormalization.PIXEL_ISOTROPIC,
                          alpha)
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    init = pt.sfm.SampleInitialCameras(problem.NumCameras(), opts.camera_init, 0)
    ok, sol = pt.sfm.SolveOse(problem, opts, init)
    assert ok, sol.message

    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    upgrade_opts.radial_cameras = True
    upgrade_opts.nonlinear_refinement = True
    ok, refined, err = pt.sfm.UpgradeToMetric(problem, sol.cameras, sol.points,
                                              upgrade_opts)
    assert ok, err
    assert refined.fraction_in_front > 0.99
    scale = np.array([np.asarray(T)[0, 0] for T in problem.normalizations])
    true_f = focals[problem.view_ids]
    f_err = np.abs(np.array(refined.focal_lengths) / scale / true_f - 1)
    # Measured max error: 16 % at alpha 0.05, 0.9 % at alpha 0.001; the error
    # is the alpha bias and vanishes as alpha -> 0.
    assert f_err.max() < focal_tol, f_err.max()


def test_estimator_rpose_end_to_end_on_noisy_scene():
    recon, view_graph, gt = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options(pt.sfm.OseFormulationType.RPOSE)
    opts.init_free_options.num_random_restarts = 2
    # Ignored for RPOSE (no projective refinement), kept to check that it is.
    opts.init_free_options.projective_refinement.max_num_iterations = 30
    # The linear upgrade alone is biased for RPOSE (see the stage-wise test).
    opts.init_free_options.metric_upgrade = (
        pt.sfm.MetricUpgradeType.LINEAR_AND_NONLINEAR)
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    diag = estimator.Diagnostics()
    assert diag.metric_upgrade_success
    assert diag.fraction_in_front > 0.99
    best = diag.restarts[diag.best_restart]
    assert best.refined_rms_error < 0.0

    vids = sorted(gt)
    est_R = {v: np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids}
    worst = pairwise_rotation_errors([est_R[v] for v in vids], vids, gt).max()
    # Measured 0.25 deg after the metric bundle adjustment.
    assert worst < 0.5, worst


def test_estimator_records_upgrade_of_used_restart():
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options()
    opts.init_free_options.num_random_restarts = 3
    opts.init_free_options.projective_refinement.max_num_iterations = 30
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    diag = estimator.Diagnostics()
    assert len(diag.restarts) == 3
    used = diag.restarts[diag.best_restart]
    assert used.upgrade_attempted and used.upgrade_success
    assert used.upgrade_message == "ok"
    assert used.upgrade_fraction_in_front == pytest.approx(diag.fraction_in_front)
    assert used.upgrade_mean_orthogonality_error == pytest.approx(
        diag.mean_orthogonality_error)
    assert used.upgrade_conditioning >= (
        opts.init_free_options.min_upgrade_conditioning)
    assert used.upgrade_fraction_in_front >= opts.init_free_options.min_fraction_in_front
    assert (used.upgrade_mean_orthogonality_error <=
            opts.init_free_options.max_mean_orthogonality_error)
    assert "restart %d used" % diag.best_restart in summary.message


def test_estimator_fails_when_no_restart_passes_checks():
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options()
    opts.init_free_options.num_random_restarts = 3
    opts.init_free_options.projective_refinement.max_num_iterations = 30
    # The orthogonality error of this noisy scene is about 0.011: no restart
    # can pass.
    opts.init_free_options.max_mean_orthogonality_error = 1e-6
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert not summary.success
    assert "no restart passed the metric upgrade checks" in summary.message
    assert "max_mean_orthogonality_error 1e-06" in summary.message
    diag = estimator.Diagnostics()
    assert diag.best_restart == -1
    assert not diag.metric_upgrade_success
    # Every restart was upgraded and rejected, and nothing was written.
    assert len(diag.restarts) == 3
    assert all(r.upgrade_attempted and not r.upgrade_success
               for r in diag.restarts)
    assert all("mean orthogonality error" in r.upgrade_message
               for r in diag.restarts)
    assert all(r.upgrade_fraction_in_front == pytest.approx(1.0)
               for r in diag.restarts)
    assert all(r.upgrade_mean_orthogonality_error > 1e-6 for r in diag.restarts)
    assert not any(recon.View(v).IsEstimated() for v in recon.ViewIds())


def test_estimator_rpose_linear_upgrade_is_nonlinear_and_correct():
    recon, view_graph, gt = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    opts = pt.sfm.ReconstructionEstimatorOptions()
    opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
    opts.init_free_options = init_free_options(pt.sfm.OseFormulationType.RPOSE)
    opts.init_free_options.num_random_restarts = 2
    # Requested as LINEAR: the radial estimator always runs the nonlinear
    # refinement, because the linear radial estimate is biased.
    opts.init_free_options.metric_upgrade = pt.sfm.MetricUpgradeType.LINEAR
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.min_num_two_view_inliers = 10

    estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
    summary = estimator.Estimate(view_graph, recon)
    assert summary.success, summary.message
    diag = estimator.Diagnostics()
    assert diag.metric_upgrade_success
    assert diag.restarts[diag.best_restart].upgrade_success
    assert diag.fraction_in_front > 0.99
    assert diag.mean_orthogonality_error < (
        0.5 * diag.linear_mean_orthogonality_error)

    vids = sorted(gt)
    est_R = [np.asarray(recon.View(v).Camera().GetOrientationAsRotationMatrix())
             for v in vids]
    worst = pairwise_rotation_errors(est_R, vids, gt).max()
    assert worst < 0.5, worst


def upgrade_scene(recon, view_graph, opts, nonlinear=True, radial=False,
                  shared=False):
    """Stage-wise OSE and metric upgrade of a scene (MetricUpgradeResult)."""
    ok, problem, err = pt.sfm.BuildOseProblem(recon, view_graph, opts)
    assert ok, err
    if radial:
        init = pt.sfm.SampleInitialCameras(problem.NumCameras(),
                                           opts.camera_init, 0)
        ok, sol = pt.sfm.SolveOse(problem, opts, init)
        assert ok, sol.message
        cameras, points = sol.cameras, sol.points
    else:
        ok, best, _, err = pt.sfm.RunProjectiveStages(problem, opts)
        assert ok, err
        cameras, points = best.cameras, best.points
    upgrade_opts = pt.sfm.MetricUpgradeOptions()
    upgrade_opts.nonlinear_refinement = nonlinear
    upgrade_opts.shared_focal_length = shared
    upgrade_opts.radial_cameras = radial
    ok, result, err = pt.sfm.UpgradeToMetric(problem, cameras, points,
                                             upgrade_opts)
    assert ok, err
    return problem, result


def focal_errors(problem, result, focals):
    """Relative errors of the upgraded focal lengths (pixels)."""
    scale = np.array([np.asarray(T)[0, 0] for T in problem.normalizations])
    return np.abs(np.array(result.focal_lengths) / scale /
                  focals[problem.view_ids] - 1)


def test_conditioning_is_well_posed_on_existing_scenes():
    # Measured: calibrated 0.56, varying focal 0.044, shared focal 0.16,
    # radial 0.018 to 0.037 (smallest 0.018 at 1 px).
    recon, view_graph, _ = make_scene(n_views=15, pixel_noise=1.0, seed=5)
    _, result = upgrade_scene(recon, view_graph, init_free_options())
    assert result.conditioning > 0.01, result.conditioning

    focals = np.random.default_rng(0).uniform(650.0, 1000.0, 15)
    for noise in (0.0, 0.5):
        recon, view_graph, _ = make_scene(n_views=15, n_points=200,
                                          pixel_noise=noise, seed=3,
                                          focal_lengths=focals)
        _, result = upgrade_scene(recon, view_graph, uncalibrated_options())
        assert result.conditioning > 0.01, result.conditioning

    shared = np.full(15, 900.0)
    recon, view_graph, _ = make_scene(n_views=15, n_points=200,
                                      pixel_noise=0.5, seed=3,
                                      focal_lengths=shared)
    _, result = upgrade_scene(recon, view_graph, uncalibrated_options(),
                              shared=True)
    assert result.conditioning > 0.01, result.conditioning

    recon, view_graph, _ = make_scene(n_views=15, n_points=200,
                                      pixel_noise=0.0, seed=3,
                                      focal_lengths=focals)
    opts = radial_options(pt.sfm.OseObservationNormalization.PIXEL_ISOTROPIC,
                          0.001)
    _, result = upgrade_scene(recon, view_graph, opts, radial=True)
    assert result.conditioning > 0.01, result.conditioning


@pytest.mark.parametrize("pixel_noise", [0.0, 0.5])
def test_pure_translation_is_flagged_as_ill_conditioned(pixel_noise):
    # Measured: conditioning 1e-8 (noise free) to 6e-4 (2 px), focal lengths
    # off by 20 to 60 %, while the orthogonality error stays below 0.01.
    focals = np.random.default_rng(0).uniform(650.0, 1000.0, 15)
    recon, view_graph, _ = make_scene(n_views=15, n_points=200,
                                      pixel_noise=pixel_noise, seed=3,
                                      focal_lengths=focals, translation=True)
    problem, result = upgrade_scene(recon, view_graph, uncalibrated_options())
    assert result.nonlinear_refinement_run
    assert result.conditioning < 1e-3, result.conditioning
    assert focal_errors(problem, result, focals).max() > 0.1
    assert result.mean_orthogonality_error < 0.3


def test_estimator_rejects_ill_conditioned_upgrade():
    focals = np.random.default_rng(0).uniform(650.0, 1000.0, 15)

    def estimate(min_conditioning):
        recon, view_graph, gt = make_scene(n_views=15, n_points=200,
                                           pixel_noise=0.5, seed=3,
                                           focal_lengths=focals,
                                           translation=True)
        opts = pt.sfm.ReconstructionEstimatorOptions()
        opts.reconstruction_estimator_type = (
            pt.sfm.ReconstructionEstimatorType.INIT_FREE)
        opts.init_free_options = uncalibrated_options()
        opts.init_free_options.metric_upgrade = (
            pt.sfm.MetricUpgradeType.LINEAR_AND_NONLINEAR)
        opts.init_free_options.min_upgrade_conditioning = min_conditioning
        opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH
        opts.min_num_two_view_inliers = 10
        estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
        summary = estimator.Estimate(view_graph, recon)
        return recon, summary, estimator.Diagnostics(), sorted(gt)

    # With the default threshold the critical upgrade is rejected cleanly.
    recon, summary, diag, _ = estimate(3e-3)
    assert not summary.success
    assert "ill-conditioned self-calibration" in summary.message
    assert diag.best_restart == -1 and not diag.metric_upgrade_success
    restart = diag.restarts[0]
    assert restart.upgrade_attempted and not restart.upgrade_success
    assert restart.upgrade_message.startswith(
        "rejected: ill-conditioned self-calibration")
    assert restart.upgrade_conditioning < 3e-3
    assert not any(recon.View(v).IsEstimated() for v in recon.ViewIds())

    # Without the check (0) the same scene is accepted, with wrong focal
    # lengths: the orthogonality error does not reveal them.
    recon, summary, diag, vids = estimate(0.0)
    assert summary.success, summary.message
    upgrade_f = np.array(diag.focal_lengths)
    assert np.abs(upgrade_f / focals[vids] - 1).max() > 0.1
    assert diag.mean_orthogonality_error < 0.3

