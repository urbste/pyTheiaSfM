"""Tests for capture view graph + GlobalRigReconstructor wiring."""

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")


def test_build_capture_view_graph_strips_extrinsics():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)

    c0 = recon.AddRigCapture(rid, 0.0, {left: "l0.png", right: "r0.png"})
    c1 = recon.AddRigCapture(rid, 1.0, {left: "l1.png", right: "r1.png"})
    v_l0 = recon.GetRigCapture(c0).ViewIdForCamera(left)
    v_l1 = recon.GetRigCapture(c1).ViewIdForCamera(left)

    vg = pt.sfm.ViewGraph()
    info = pt.sfm.TwoViewInfo()
    info.rotation_2 = np.array([0.0, 0.1, 0.0])
    info.position_2 = np.array([0.2, 0.0, 1.0])
    info.num_verified_matches = 100
    info.focal_length_1 = 500.0
    info.focal_length_2 = 500.0
    vg.AddEdge(v_l0, v_l1, info)

    # Intra-capture stereo edge should be ignored for capture graph motion.
    stereo = pt.sfm.TwoViewInfo()
    stereo.rotation_2 = np.zeros(3)
    stereo.position_2 = np.array([0.1, 0.0, 0.0])
    stereo.num_verified_matches = 200
    v_r0 = recon.GetRigCapture(c0).ViewIdForCamera(right)
    vg.AddEdge(v_l0, v_r0, stereo)

    cg = pt.sfm.ViewGraph()
    assert pt.sfm.BuildCaptureViewGraph(recon, vg, cg)
    assert cg.NumEdges() == 1


def test_global_rig_reconstructor_options_expose_averaging_backends():
    opts = pt.sfm.GlobalRigReconstructorOptions()
    assert (
        opts.sfm_options.global_rotation_estimator_type
        == pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2
    )
    assert (
        opts.sfm_options.global_position_estimator_type
        == pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
    )
    assert opts.rescale_positions_to_metric_edges is True
    assert not hasattr(opts, "refine_positions_with_tracks")
    opts.sfm_options.least_unsquared_deviation_position_estimator_options.use_scale_estimates = (
        True
    )
    _ = pt.sfm.GlobalRigReconstructor(opts)


def test_strip_capture_edges_have_no_metric_scale():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)

    c0 = recon.AddRigCapture(rid, 0.0, {left: "l0.png", right: "r0.png"})
    c1 = recon.AddRigCapture(rid, 1.0, {left: "l1.png", right: "r1.png"})
    v_l0 = recon.GetRigCapture(c0).ViewIdForCamera(left)
    v_l1 = recon.GetRigCapture(c1).ViewIdForCamera(left)

    vg = pt.sfm.ViewGraph()
    info = pt.sfm.TwoViewInfo()
    info.rotation_2 = np.array([0.0, 0.1, 0.0])
    info.position_2 = np.array([0.2, 0.0, 1.0])
    info.num_verified_matches = 100
    vg.AddEdge(v_l0, v_l1, info)

    opts = pt.sfm.BuildCaptureViewGraphOptions()
    opts.use_metric_relative_rig_pose = False
    opts.fallback_to_twoview_strip = True
    cg = pt.sfm.ViewGraph()
    assert pt.sfm.BuildCaptureViewGraph(recon, vg, cg, opts)
    edge = cg.GetEdge(c0, c1)
    assert edge is not None
    assert edge.scale_estimate < 0.0


def test_metric_pose_only_for_viewgraph_pairs():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)
    recon.AddRigCapture(rid, 0.0, {left: "l0.png", right: "r0.png"})
    recon.AddRigCapture(rid, 1.0, {left: "l1.png", right: "r1.png"})

    opts = pt.sfm.BuildCaptureViewGraphOptions()
    assert opts.metric_only_for_viewgraph_pairs is True
    opts.use_metric_relative_rig_pose = True
    opts.fallback_to_twoview_strip = False
    cg = pt.sfm.ViewGraph()
    vg = pt.sfm.ViewGraph()
    assert not pt.sfm.BuildCaptureViewGraph(recon, vg, cg, opts)
    assert cg.NumEdges() == 0


def test_essential_from_rig_sensors_and_sampson_filter():
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.06, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.06, 0.0, 0.0]), np.zeros(3))
    E = pt.sfm.EssentialMatrixFromRigSensors(
        rig.GetSensor(left), rig.GetSensor(right)
    )
    X = np.array([0.2, -0.1, 4.0])
    x1 = (X - np.array([-0.06, 0.0, 0.0]))
    x1 = x1 / x1[2]
    x2 = (X - np.array([0.06, 0.0, 0.0]))
    x2 = x2 / x2[2]
    assert abs(x2 @ E @ x1) < 1e-8

    cam = pt.sfm.Camera()
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [500.0]
    prior.focal_length.is_set = True
    prior.principal_point.value = [320.0, 240.0]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [1.0]
    prior.aspect_ratio.is_set = True
    prior.image_width = 640
    prior.image_height = 480
    prior.camera_intrinsics_model_type = "PINHOLE"
    cam.SetFromCameraIntrinsicsPriors(prior)

    def _pix(xn):
        return np.array(
            [500.0 * xn[0] / xn[2] + 320.0, 500.0 * xn[1] / xn[2] + 240.0]
        )

    p1 = _pix(x1)
    p2 = _pix(x2)
    good = pt.matching.FeatureCorrespondence(pt.sfm.Feature(p1), pt.sfm.Feature(p2))
    bad = pt.matching.FeatureCorrespondence(
        pt.sfm.Feature(p1), pt.sfm.Feature(np.array([10.0, 10.0]))
    )
    inliers = pt.sfm.FilterCorrespondencesWithEssential(E, cam, cam, [good, bad], 2.0)
    assert 0 in inliers
    assert 1 not in inliers


def test_rig_ba_preserves_calibrated_baseline():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)
    cap = recon.AddRigCapture(
        rid, 0.0, {left: "l.png", right: "r.png"}
    )
    capture = recon.MutableRigCapture(cap)
    capture.SetPosition(np.zeros(3))
    capture.SetOrientationFromAngleAxis(np.zeros(3))
    capture.SetEstimated(True)
    assert pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [800.0]
    prior.focal_length.is_set = True
    prior.principal_point.value = [400.0, 300.0]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [1.0]
    prior.aspect_ratio.is_set = True
    prior.image_width = 800
    prior.image_height = 600
    prior.camera_intrinsics_model_type = "PINHOLE"
    for vid in recon.ViewIds():
        view = recon.MutableView(vid)
        view.SetCameraIntrinsicsPrior(prior)
        view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)

    pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    points = [
        np.array([0.4, 0.3, 5.0, 1.0]),
        np.array([-0.3, 0.2, 4.5, 1.0]),
        np.array([0.1, -0.4, 6.0, 1.0]),
        np.array([-0.5, -0.2, 5.5, 1.0]),
        np.array([0.0, 0.0, 4.0, 1.0]),
        np.array([0.6, -0.1, 7.0, 1.0]),
    ]
    for p in points:
        tid = recon.AddTrack()
        track = recon.MutableTrack(tid)
        track.SetPoint(p.tolist())
        track.SetIsEstimated(True)
        for vid in recon.ViewIds():
            cam = recon.View(vid).Camera()
            depth, xy = cam.ProjectPoint(p)
            if depth > 0:
                recon.AddObservation(vid, tid, pt.sfm.Feature(xy))

    left_view = recon.MutableView(capture.ViewIdForCamera(left))
    left_view.MutableCamera().SetPosition(np.array([-0.08, 0.02, 0.01]))
    right_view = recon.View(capture.ViewIdForCamera(right))
    baseline_before = np.linalg.norm(
        right_view.Camera().GetPosition() - left_view.Camera().GetPosition()
    )
    assert abs(baseline_before - 0.1) > 0.02

    opts = pt.sfm.BundleAdjustmentOptions()
    opts.use_rig_constraints = True
    opts.constant_camera_orientation = False
    opts.num_threads = 1
    opts.use_inner_iterations = False
    summary = pt.sfm.BundleAdjustReconstruction(opts, recon)
    assert summary.success
    left_pos = recon.View(capture.ViewIdForCamera(left)).Camera().GetPosition()
    right_pos = recon.View(capture.ViewIdForCamera(right)).Camera().GetPosition()
    np.testing.assert_allclose(
        np.linalg.norm(right_pos - left_pos), 0.1, atol=1e-4
    )


def test_incremental_rig_seeds_with_empty_captures():
    """Empty captures before the first stereo track must not SIGSEGV in SeedInitialCapture."""
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [500.0]
    prior.focal_length.is_set = True
    prior.principal_point.value = [320.0, 240.0]
    prior.principal_point.is_set = True
    prior.image_width = 640
    prior.image_height = 480
    prior.camera_intrinsics_model_type = "PINHOLE"

    left_views = []
    right_views = []
    for i in range(8):
        cap = recon.AddRigCapture(
            rid, float(i), {left: f"l{i}.png", right: f"r{i}.png"}
        )
        capture = recon.GetRigCapture(cap)
        vl = capture.ViewIdForCamera(left)
        vr = capture.ViewIdForCamera(right)
        left_views.append(vl)
        right_views.append(vr)
        for vid in (vl, vr):
            view = recon.MutableView(vid)
            view.SetCameraIntrinsicsPrior(prior)
            view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)

    recon.AddTrack(
        [
            (left_views[5], pt.sfm.Feature(np.array([330.0, 240.0]))),
            (right_views[5], pt.sfm.Feature(np.array([310.0, 240.0]))),
        ]
    )
    vg = pt.sfm.ViewGraph()
    opts = pt.sfm.IncrementalRigReconstructorOptions()
    opts.ba_options.num_threads = 1
    opts.ba_options.use_inner_iterations = False
    summary = pt.sfm.IncrementalRigReconstructor(opts).Estimate(vg, recon)
    assert summary is not None
    assert any(
        recon.GetRigCapture(cid).IsEstimated() for cid in recon.CaptureIds()
    )


def _pinhole_prior(focal=500.0, width=640, height=480):
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [focal]
    prior.focal_length.is_set = True
    prior.principal_point.value = [width * 0.5, height * 0.5]
    prior.principal_point.is_set = True
    prior.aspect_ratio.value = [1.0]
    prior.aspect_ratio.is_set = True
    prior.image_width = width
    prior.image_height = height
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def test_twoview_info_from_relative_rig_is_unit_direction():
    info = pt.sfm.RelativeRigInfo()
    info.rotation = np.eye(3)
    info.position = np.array([0.0, 0.0, 2.5])
    info.translation = -info.rotation.T @ info.position
    twoview = info.ToTwoViewInfo()
    np.testing.assert_allclose(np.linalg.norm(twoview.position_2), 1.0, atol=1e-12)
    np.testing.assert_allclose(twoview.position_2, np.array([0.0, 0.0, 1.0]), atol=1e-12)
    np.testing.assert_allclose(twoview.scale_estimate, 2.5, atol=1e-12)


def test_partial_ba_skips_deleted_track_ids():
    """Partial BA used to prepend dummy TrackId 0; CHECK-fails if track 0 was pruned."""
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)
    cap = recon.AddRigCapture(rid, 0.0, {left: "l.png", right: "r.png"})
    capture = recon.MutableRigCapture(cap)
    capture.SetPosition(np.zeros(3))
    capture.SetOrientationFromAngleAxis(np.zeros(3))
    capture.SetEstimated(True)
    prior = _pinhole_prior()
    for vid in recon.ViewIds():
        view = recon.MutableView(vid)
        view.SetCameraIntrinsicsPrior(prior)
        view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)
    assert pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    points = [
        np.array([0.2, 0.1, 4.0, 1.0]),
        np.array([-0.3, 0.2, 5.0, 1.0]),
        np.array([0.0, -0.2, 4.5, 1.0]),
        np.array([0.4, -0.1, 6.0, 1.0]),
    ]
    track_ids = []
    for p in points:
        tid = recon.AddTrack()
        track = recon.MutableTrack(tid)
        track.SetPoint(p.tolist())
        track.SetIsEstimated(True)
        for vid in recon.ViewIds():
            depth, xy = recon.View(vid).Camera().ProjectPoint(p)
            if depth > 0:
                recon.AddObservation(vid, tid, pt.sfm.Feature(xy))
        track_ids.append(tid)

    assert track_ids[0] == 0
    assert recon.RemoveTrack(track_ids[0])

    opts = pt.sfm.BundleAdjustmentOptions()
    opts.use_rig_constraints = True
    opts.num_threads = 1
    opts.use_inner_iterations = False
    summary = pt.sfm.BundleAdjustPartialReconstruction(
        opts, set(recon.ViewIds()), set(track_ids[1:]), recon
    )
    assert summary is not None


def test_ba_with_estimated_views_and_no_tracks_does_not_abort():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)
    cap = recon.AddRigCapture(rid, 0.0, {left: "l.png", right: "r.png"})
    capture = recon.MutableRigCapture(cap)
    capture.SetPosition(np.zeros(3))
    capture.SetOrientationFromAngleAxis(np.zeros(3))
    capture.SetEstimated(True)
    prior = _pinhole_prior()
    for vid in recon.ViewIds():
        view = recon.MutableView(vid)
        view.SetCameraIntrinsicsPrior(prior)
        view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)
    assert pt.sfm.PropagateCameraPosesForCapture(cap, recon)

    opts = pt.sfm.BundleAdjustmentOptions()
    opts.use_rig_constraints = True
    opts.num_threads = 1
    opts.use_inner_iterations = False
    summary = pt.sfm.BundleAdjustReconstruction(opts, recon)
    assert summary is not None


def test_global_rig_reconstructor_lud_recovers_metric_forward_motion():
    recon = pt.sfm.Reconstruction()
    rig = pt.sfm.CameraRig("stereo")
    left = rig.AddSensor("left", np.array([-0.05, 0.0, 0.0]), np.zeros(3))
    right = rig.AddSensor("right", np.array([0.05, 0.0, 0.0]), np.zeros(3))
    rid = recon.AddCameraRig(rig)
    prior = _pinhole_prior()
    n_captures = 6
    step = 0.5
    capture_ids = []
    left_views = []
    for i in range(n_captures):
        cap = recon.AddRigCapture(
            rid, float(i), {left: f"l{i}.png", right: f"r{i}.png"}
        )
        capture = recon.MutableRigCapture(cap)
        capture.SetPosition(np.array([0.0, 0.0, i * step]))
        capture.SetOrientationFromAngleAxis(np.zeros(3))
        capture.SetEstimated(True)
        for vid in capture.GetViewIds():
            view = recon.MutableView(vid)
            view.SetCameraIntrinsicsPrior(prior)
            view.MutableCamera().SetFromCameraIntrinsicsPriors(prior)
        assert pt.sfm.PropagateCameraPosesForCapture(cap, recon)
        capture_ids.append(cap)
        left_views.append(capture.ViewIdForCamera(left))

    rng = np.random.default_rng(0)
    for _ in range(48):
        p = np.array(
            [
                rng.uniform(-0.8, 0.8),
                rng.uniform(-0.5, 0.5),
                rng.uniform(3.5, 7.0),
                1.0,
            ]
        )
        observations = []
        for vid in recon.ViewIds():
            cam = recon.View(vid).Camera()
            depth, xy = cam.ProjectPoint(p)
            if depth <= 0.5:
                continue
            if 2.0 < xy[0] < prior.image_width - 2.0 and 2.0 < xy[1] < prior.image_height - 2.0:
                observations.append((vid, pt.sfm.Feature(xy)))
        if len(observations) >= 4:
            recon.AddTrack(observations)

    assert len(recon.TrackIds()) >= 20

    vg = pt.sfm.ViewGraph()
    for i in range(n_captures - 1):
        info = pt.sfm.TwoViewInfo()
        info.rotation_2 = np.zeros(3)
        info.position_2 = np.array([0.0, 0.0, 1.0])
        info.num_verified_matches = 80
        vg.AddEdge(left_views[i], left_views[i + 1], info)

    for cid in capture_ids:
        recon.MutableRigCapture(cid).SetEstimated(False)
        for vid in recon.GetRigCapture(cid).GetViewIds():
            recon.MutableView(vid).SetIsEstimated(False)
    for tid in recon.TrackIds():
        recon.MutableTrack(tid).SetIsEstimated(False)

    gro = pt.sfm.GlobalRigReconstructorOptions()
    gro.sfm_options.filter_relative_translations_with_1dsfm = False
    gro.sfm_options.min_triangulation_angle_degrees = 0.5
    gro.sfm_options.num_retriangulation_iterations = 0
    gro.sfm_options.num_threads = 1
    gro.sfm_options.use_inner_iterations = False
    gro.rescale_positions_to_metric_edges = True
    summary = pt.sfm.GlobalRigReconstructor(gro).Estimate(vg, recon)
    assert summary.success
    assert len(summary.estimated_tracks) >= 10

    gt = [np.array([0.0, 0.0, i * step]) for i in range(n_captures)]
    est = [recon.GetRigCapture(cid).GetPosition() for cid in capture_ids]
    _, _, scale = pt.sfm.AlignPointCloudsUmeyama(est, gt)
    assert 0.5 < scale < 2.0, f"Umeyama scale {scale} is not metric"
