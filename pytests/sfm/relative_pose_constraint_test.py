import numpy as np
import pytheia as pt


def _build_collinear_reconstruction(positions):
    recon = pt.sfm.Reconstruction()
    cam_prior = pt.sfm.CameraIntrinsicsPrior()
    cam_prior.focal_length.value = np.array([900.0], dtype=np.float64)
    cam_prior.principal_point.value = np.array([720.0, 540.0], dtype=np.float64)
    cam_prior.aspect_ratio.value = np.array([1.0], dtype=np.float64)
    cam_prior.camera_intrinsics_model_type = "PINHOLE"
    cam_prior.image_width = 1440
    cam_prior.image_height = 1080
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(cam_prior)

    view_ids = []
    for i, pos in enumerate(positions):
        vid = recon.AddView(str(i), 0, i)
        view = recon.View(vid)
        m_cam = view.MutableCamera()
        m_cam.DeepCopy(camera)
        m_cam.SetPosition(np.asarray(pos, dtype=np.float64))
        m_cam.SetOrientationFromAngleAxis(np.zeros(3, dtype=np.float64))
        view.SetIsEstimated(True)
        view_ids.append(vid)

    points = [
        np.array([0.5, 0.5, 5.0, 1.0]),
        np.array([0.5, -0.5, 5.0, 1.0]),
        np.array([-0.5, 0.5, 5.0, 1.0]),
        np.array([-0.5, -0.5, 5.0, 1.0]),
    ]
    for p in points:
        tid = recon.AddTrack()
        track = recon.MutableTrack(tid)
        track.SetPoint(np.asarray(p, dtype=np.float64))
        track.SetIsEstimated(True)
        for vid in view_ids:
            cam = recon.View(vid).Camera()
            obs = cam.ProjectPoint(p)
            if obs[0] and 0 <= obs[1][0] <= 1440 and 0 <= obs[1][1] <= 1080:
                recon.AddObservation(vid, tid, pt.sfm.Feature(obs[1]))

    return recon, view_ids


def _relative_edges(view_ids, translation_weight, rotation_weight, scale_invariant):
    edges = []
    for i, j in zip(view_ids[:-1], view_ids[1:]):
        edge = pt.sfm.RelativePoseConstraint()
        edge.view_id_i = i
        edge.view_id_j = j
        edge.translation_sqrt_weight = translation_weight
        edge.rotation_sqrt_weight = rotation_weight
        edge.scale_invariant_translation = scale_invariant
        if scale_invariant:
            edge.translation_direction_sqrt_weight = translation_weight
            edge.translation_magnitude_sqrt_weight = 0.0
        edges.append(edge)
    return edges


def _set_position_priors(recon, view_ids, target_positions, weight):
    info = weight * np.eye(3, dtype=np.float64)
    for vid, pos in zip(view_ids, target_positions):
        recon.View(vid).SetPositionPrior(np.asarray(pos, dtype=np.float64), info)


def _mean_position_error(recon, view_ids, target_positions):
    errs = []
    for vid, pos in zip(view_ids, target_positions):
        errs.append(np.linalg.norm(recon.View(vid).Camera().GetPosition() - pos))
    return float(np.mean(errs))


def _run_ba_with_edges(positions, target_positions, scale_invariant):
    recon, view_ids = _build_collinear_reconstruction(positions)
    _set_position_priors(recon, view_ids, target_positions, weight=80.0)
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.use_position_priors = True
    opts.max_num_iterations = 80
    opts.verbose = False
    opts.loss_function_type = pt.sfm.LossFunctionType(0)
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    opts.use_homogeneous_point_parametrization = True
    opts.linear_solver_type = pt.sfm.LinearSolverType.SPARSE_SCHUR
    edges = _relative_edges(
        view_ids,
        translation_weight=60.0,
        rotation_weight=60.0,
        scale_invariant=scale_invariant,
    )
    summary = pt.sfm.BundleAdjustReconstructionWithRelativePoseEdges(
        opts, edges, recon
    )
    assert summary.success
    return _mean_position_error(recon, view_ids, target_positions)


def test_relative_pose_constraint_scale_invariant_fields():
    edge = pt.sfm.RelativePoseConstraint()
    edge.scale_invariant_translation = True
    edge.translation_direction_sqrt_weight = 12.0
    edge.translation_magnitude_sqrt_weight = 0.0
    assert edge.scale_invariant_translation is True
    assert edge.translation_direction_sqrt_weight == 12.0
    assert edge.translation_magnitude_sqrt_weight == 0.0


def test_scale_invariant_relative_edges_allow_scale_correction():
    target_positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([2.0, 0.0, 0.0]),
    ]
    drifted_positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([3.0, 0.0, 0.0]),
    ]

    rigid_err = _run_ba_with_edges(drifted_positions, target_positions, False)
    scale_inv_err = _run_ba_with_edges(
        drifted_positions, target_positions, True
    )

    assert scale_inv_err < rigid_err
    assert scale_inv_err < 0.2
