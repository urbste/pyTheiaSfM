"""Regression tests for bundle adjustment with the inverse depth parametrization.

Inverse depth BA used to produce NaN residuals when the reference bearing and
the inverse depth had not been initialized, used a depth/bearing convention that
only held for pinhole cameras, optimized stale anchors, ignored the requested
view subset and overwrote optimized points after covariance queries. These tests
compare the inverse depth solution with the XYZ solution for several camera
models, on clean and noisy synthetic data.
"""

import numpy as np
import pytest

import pytheia as pt

WIDTH, HEIGHT = 1200, 900

MODELS = [
    "PINHOLE",
    "PINHOLE_RADIAL_TANGENTIAL",
    "FISHEYE",
    "FOV",
    "DIVISION_UNDISTORTION",
    "DOUBLE_SPHERE",
    "EXTENDED_UNIFIED",
    "FULL_OPENCV",
    "RADIAL_FISHEYE",
    "THIN_PRISM_FISHEYE",
    "RAD_TAN_THIN_PRISM_FISHEYE",
]
# Models whose distortion terms start at parameter index 4 in COLMAP order.
COLMAP_ORDER = {"FULL_OPENCV", "RADIAL_FISHEYE", "THIN_PRISM_FISHEYE", "RAD_TAN_THIN_PRISM_FISHEYE"}


def make_camera(model):
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(getattr(pt.sfm.CameraIntrinsicsModelType, model))
    cam.SetImageSize(WIDTH, HEIGHT)
    intr = cam.CameraIntrinsics()
    intr.SetFocalLength(700.0)
    for setter, value in (("SetAspectRatio", 1.0), ("SetSkew", 0.0)):
        if model != "RADIAL_FISHEYE" and hasattr(intr, setter):
            getattr(intr, setter)(value)
    intr.SetPrincipalPoint(WIDTH / 2.0, HEIGHT / 2.0)
    if model in COLMAP_ORDER:
        for i, v in enumerate([0.02, -0.005, 0.0005, -0.0005]):
            if 4 + i < intr.NumParameters():
                intr.SetParameter(4 + i, v)
    return cam


def make_recon(model="PINHOLE", n_views=8, n_tracks=120, pixel_noise=0.0, seed=0):
    """Cameras on an arc looking at a point cloud in front of them."""
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    base = make_camera(model)
    for i in range(n_views):
        vid = recon.AddView(str(i), i, float(i))
        view = recon.View(vid)
        cam = recon.MutableView(vid).MutableCamera()
        cam.DeepCopy(base)
        cam.SetPosition(np.array([rng.uniform(-1.5, 1.5), rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.0)]))
        cam.SetOrientationFromAngleAxis(rng.uniform(-0.05, 0.05, size=3))
        recon.MutableView(vid).SetIsEstimated(True)
    truth = {}
    for _ in range(n_tracks):
        xyz = np.array([rng.uniform(-2.0, 2.0), rng.uniform(-1.5, 1.5), rng.uniform(4.0, 9.0)])
        obs = []
        for vid in recon.ViewIds():
            depth, pixel = recon.View(vid).Camera().ProjectPoint(np.append(xyz, 1.0))
            if depth > 0 and 0 < pixel[0] < WIDTH and 0 < pixel[1] < HEIGHT:
                obs.append((vid, pixel + rng.normal(size=2) * pixel_noise))
        if len(obs) < 3:
            continue
        tid = recon.AddTrack()
        for vid, pixel in obs:
            recon.AddObservation(vid, tid, pt.sfm.Feature(pixel))
        track = recon.MutableTrack(tid)
        track.SetPoint(np.append(xyz, 1.0))
        track.SetIsEstimated(True)
        truth[tid] = xyz
    return recon, truth


def options(inverse, fix_cameras=False, fix_intrinsics=True, **kwargs):
    o = pt.sfm.BundleAdjustmentOptions()
    o.num_threads = 1
    o.verbose = False
    o.loss_function_type = pt.sfm.LossFunctionType.TRIVIAL
    o.use_inner_iterations = False
    o.max_num_iterations = 200
    o.use_inverse_depth_parametrization = inverse
    o.constant_camera_orientation = fix_cameras
    o.constant_camera_position = fix_cameras
    if fix_intrinsics:
        o.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    for k, v in kwargs.items():
        setattr(o, k, v)
    return o


def euclidean(point4):
    p = np.asarray(point4, dtype=float)
    return p[:3] / p[3]


def perturb_points(recon, sigma, seed=1):
    rng = np.random.default_rng(seed)
    for tid in recon.TrackIds():
        p = euclidean(recon.Track(tid).Point()) + rng.normal(size=3) * sigma
        recon.MutableTrack(tid).SetPoint(np.append(p, 1.0))


def perturb_poses(recon, sigma_pos, sigma_rot, seed=2):
    rng = np.random.default_rng(seed)
    for vid in recon.ViewIds():
        cam = recon.MutableView(vid).MutableCamera()
        cam.SetPosition(cam.GetPosition() + rng.normal(size=3) * sigma_pos)
        cam.SetOrientationFromAngleAxis(cam.GetOrientationAsAngleAxis() + rng.normal(size=3) * sigma_rot)


def reprojection_rms(recon):
    sq, n = 0.0, 0
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        if not track.IsEstimated():
            continue
        point = np.asarray(track.Point(), dtype=float)
        for vid in track.ViewIds():
            view = recon.View(vid)
            _, pixel = view.Camera().ProjectPoint(point)
            sq += float(np.sum((np.asarray(pixel) - np.asarray(view.GetFeature(tid).point)) ** 2))
            n += 1
    return np.sqrt(sq / n)


def point_error(recon, truth):
    errs = [np.linalg.norm(euclidean(recon.Track(t).Point()) - xyz) for t, xyz in truth.items()]
    return float(np.mean(errs)), float(np.max(errs))


# ---------------------------------------------------------------------------
# Initialization and the original NaN symptom
# ---------------------------------------------------------------------------


def test_ba_without_initialize_inverse_depth_is_finite():
    """Inverse depths / bearings were never set: used to evaluate 0 / 0."""
    recon, truth = make_recon("PINHOLE")
    perturb_points(recon, 0.05)
    assert all(recon.Track(t).InverseDepth() == 0.0 for t in recon.TrackIds())
    summary = pt.sfm.BundleAdjustReconstruction(options(True), recon)
    assert summary.success
    assert np.isfinite(summary.initial_cost) and np.isfinite(summary.final_cost)
    assert summary.final_cost < 1e-6
    for tid in recon.TrackIds():
        assert np.all(np.isfinite(recon.Track(tid).Point()))
        assert recon.Track(tid).InverseDepth() > 0.0


def test_random_recon_gen_recon_with_inverse_depth():
    import os
    import sys

    sys.path.insert(0, os.path.dirname(__file__))
    from random_recon_gen import RandomReconGenerator

    gen = RandomReconGenerator(seed=1)
    recon = gen.generate_random_recon(nr_views=20, nr_tracks=500)
    gen.add_noise_to_tracks(1e-2)
    summary = pt.sfm.BundleAdjustReconstruction(options(True, fix_intrinsics=False), recon)
    assert summary.success
    assert np.isfinite(summary.final_cost)
    assert summary.final_cost < 1e-6


@pytest.mark.parametrize("model", MODELS)
def test_initialization_convention(model):
    """Unit bearing and inverse depth = 1 / distance to the reference camera."""
    recon, _ = make_recon(model)
    recon.InitializeInverseDepth()
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        ref = recon.View(track.ReferenceViewId()).Camera()
        bearing = np.asarray(track.ReferenceBearingVector())
        assert np.linalg.norm(bearing) == pytest.approx(1.0, abs=1e-12)
        x = euclidean(track.Point())
        cam_point = ref.GetOrientationAsRotationMatrix() @ (x - ref.GetPosition())
        assert track.InverseDepth() == pytest.approx(1.0 / np.linalg.norm(cam_point), rel=1e-12)
        # The bearing is the viewing ray of the reference observation.
        np.testing.assert_allclose(bearing, cam_point / np.linalg.norm(cam_point), atol=1e-5)


def test_initialization_picks_estimated_reference_and_skips_unrepresentable():
    recon, _ = make_recon("PINHOLE")
    tid = recon.TrackIds()[0]
    track = recon.MutableTrack(tid)
    ref = track.ReferenceViewId()
    recon.MutableView(ref).SetIsEstimated(False)
    assert recon.InitializeTrackInverseDepth(tid)
    assert track.ReferenceViewId() != ref
    assert recon.View(track.ReferenceViewId()).IsEstimated()

    # A point behind the reference camera can not be represented.
    cam = recon.View(track.ReferenceViewId()).Camera()
    behind = cam.GetPosition() - cam.GetOrientationAsRotationMatrix().T @ np.array([0.0, 0.0, 3.0])
    track.SetPoint(np.append(behind, 1.0))
    track.SetInverseDepth(0.123)
    assert not recon.InitializeTrackInverseDepth(tid)
    assert track.InverseDepth() == 0.123


def test_remove_view_keeps_reference_view():
    track = pt.sfm.Track()
    for vid in (3, 5, 9):
        track.AddView(vid)
    ref = track.ReferenceViewId()
    other = [v for v in (3, 5, 9) if v != ref][0]
    assert track.RemoveView(other)
    assert track.ReferenceViewId() == ref
    assert not track.RemoveView(1234)
    assert track.ReferenceViewId() == ref
    assert track.RemoveView(ref)
    assert track.ReferenceViewId() in track.ViewIds()


# ---------------------------------------------------------------------------
# Residual: inverse depth cost == XYZ cost at the same 3D point
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("model", MODELS)
def test_initial_cost_matches_reprojection_error_at_anchored_point(model):
    recon, _ = make_recon(model, pixel_noise=0.5)
    perturb_points(recon, 0.05)
    recon.InitializeInverseDepth()
    # 3D point implied by (reference view, bearing, inverse depth).
    expected = 0.0
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        ref_id = track.ReferenceViewId()
        ref = recon.View(ref_id).Camera()
        x = ref.GetPosition() + ref.GetOrientationAsRotationMatrix().T @ (
            np.asarray(track.ReferenceBearingVector()) / track.InverseDepth())
        for vid in track.ViewIds():
            if vid == ref_id:
                continue
            view = recon.View(vid)
            _, pixel = view.Camera().ProjectPoint(np.append(x, 1.0))
            expected += 0.5 * float(np.sum((np.asarray(pixel) - np.asarray(view.GetFeature(tid).point)) ** 2))
    o = options(True, fix_cameras=True, max_num_iterations=0)
    summary = pt.sfm.BundleAdjustReconstruction(o, recon)
    assert summary.initial_cost == pytest.approx(expected, rel=1e-8)


# ---------------------------------------------------------------------------
# Inverse depth vs XYZ
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("model", MODELS)
def test_clean_structure_only_recovers_points(model):
    results = {}
    for inverse in (False, True):
        recon, truth = make_recon(model)
        perturb_points(recon, 0.05)
        summary = pt.sfm.BundleAdjustReconstruction(options(inverse, fix_cameras=True), recon)
        assert summary.success
        results[inverse] = point_error(recon, truth)
        assert reprojection_rms(recon) < 1e-3, (model, inverse)
    assert results[True][1] < 5e-3, results
    assert results[False][1] < 5e-3, results


@pytest.mark.parametrize("model", MODELS)
def test_clean_full_ba_matches_xyz(model):
    errors = {}
    for inverse in (False, True):
        recon, truth = make_recon(model)
        perturb_points(recon, 0.03)
        perturb_poses(recon, 0.01, 0.002)
        summary = pt.sfm.BundleAdjustReconstruction(options(inverse), recon)
        assert summary.success
        errors[inverse] = reprojection_rms(recon)
    assert errors[False] < 5e-2, errors
    assert errors[True] < 5e-2, errors


@pytest.mark.parametrize("model", ["PINHOLE", "FISHEYE", "DOUBLE_SPHERE", "FULL_OPENCV"])
def test_noisy_structure_only_close_to_xyz(model):
    results = {}
    for inverse in (False, True):
        recon, truth = make_recon(model, pixel_noise=0.5, seed=3)
        perturb_points(recon, 0.05)
        summary = pt.sfm.BundleAdjustReconstruction(options(inverse, fix_cameras=True), recon)
        assert summary.success
        results[inverse] = (reprojection_rms(recon), point_error(recon, truth)[0])
    # The inverse depth point is restricted to the reference ray (1 instead of 3
    # degrees of freedom), so it can not fit better, but must not be far off.
    assert results[True][0] < 1.6 * results[False][0] + 0.05, results
    assert results[True][1] < 2.0 * results[False][1] + 0.02, results


@pytest.mark.parametrize("model", ["PINHOLE", "FISHEYE", "DOUBLE_SPHERE", "FULL_OPENCV"])
def test_noisy_full_ba_close_to_xyz(model):
    results = {}
    for inverse in (False, True):
        recon, truth = make_recon(model, pixel_noise=0.5, seed=4)
        perturb_points(recon, 0.03)
        perturb_poses(recon, 0.005, 0.001)
        summary = pt.sfm.BundleAdjustReconstruction(options(inverse), recon)
        assert summary.success
        results[inverse] = reprojection_rms(recon)
    assert results[True] < 1.6 * results[False] + 0.05, results


def test_points_stay_homogeneous_with_unit_w():
    recon, _ = make_recon("PINHOLE")
    perturb_points(recon, 0.05)
    pt.sfm.BundleAdjustReconstruction(options(True), recon)
    for tid in recon.TrackIds():
        assert recon.Track(tid).Point()[3] == pytest.approx(1.0)


def test_inverse_depth_is_refreshed_after_xyz_ba():
    recon, truth = make_recon("PINHOLE")
    perturb_points(recon, 0.05)
    pt.sfm.BundleAdjustReconstruction(options(False), recon)
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        ref = recon.View(track.ReferenceViewId()).Camera()
        cam_point = ref.GetOrientationAsRotationMatrix() @ (euclidean(track.Point()) - ref.GetPosition())
        assert track.InverseDepth() == pytest.approx(1.0 / np.linalg.norm(cam_point), rel=1e-9)
        assert np.linalg.norm(track.ReferenceBearingVector()) == pytest.approx(1.0)


def test_stale_inverse_depth_after_pose_change_is_resynchronized():
    recon, _ = make_recon("PINHOLE")
    recon.InitializeInverseDepth()
    # Move all cameras after the initialization: the stored anchors are stale.
    perturb_poses(recon, 0.05, 0.01)
    # Re-observe from the new poses so the data is consistent with the points.
    summary = pt.sfm.BundleAdjustReconstruction(options(True, fix_cameras=True, max_num_iterations=0), recon)
    assert np.isfinite(summary.initial_cost)
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        ref = recon.View(track.ReferenceViewId()).Camera()
        cam_point = ref.GetOrientationAsRotationMatrix() @ (euclidean(track.Point()) - ref.GetPosition())
        assert track.InverseDepth() == pytest.approx(1.0 / np.linalg.norm(cam_point), rel=1e-6)


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------


def _positions(recon):
    return {v: np.array(recon.View(v).Camera().GetPosition()) for v in recon.ViewIds()}


def test_partial_reconstruction_keeps_other_views_constant():
    recon, _ = make_recon("PINHOLE")
    perturb_points(recon, 0.03)
    perturb_poses(recon, 0.01, 0.001)
    view_ids = list(recon.ViewIds())
    variable = set(view_ids[: len(view_ids) // 2])
    before = _positions(recon)
    summary = pt.sfm.BundleAdjustPartialReconstruction(
        options(True), variable, set(recon.TrackIds()), recon)
    assert summary.success
    after = _positions(recon)
    for vid in view_ids:
        if vid in variable:
            assert np.linalg.norm(after[vid] - before[vid]) > 1e-9
        else:
            np.testing.assert_array_equal(after[vid], before[vid])


def test_partial_views_constant_keeps_unlisted_views_constant():
    recon, _ = make_recon("PINHOLE")
    perturb_points(recon, 0.03)
    perturb_poses(recon, 0.01, 0.001)
    view_ids = list(recon.ViewIds())
    var_ids, const_ids, others = view_ids[:3], view_ids[3:5], view_ids[5:]
    before = _positions(recon)
    summary = pt.sfm.BundleAdjustPartialViewsConstant(options(True), var_ids, const_ids, recon)
    assert summary.success
    after = _positions(recon)
    for vid in const_ids + others:
        np.testing.assert_array_equal(after[vid], before[vid])
    assert any(np.linalg.norm(after[v] - before[v]) > 1e-9 for v in var_ids)


def test_constant_tracks_with_inverse_depth_option_does_not_crash():
    recon, truth = make_recon("PINHOLE")
    perturb_poses(recon, 0.01, 0.001)
    constant = list(recon.TrackIds())[:20]
    summary = pt.sfm.BundleAdjustReconstructionWithConstantTracks(options(True), constant, recon)
    assert np.isfinite(summary.final_cost)
    for tid in constant:
        np.testing.assert_allclose(euclidean(recon.Track(tid).Point()), truth[tid], atol=1e-12)


def test_track_ba_does_not_change_cameras_or_intrinsics():
    recon, _ = make_recon("PINHOLE")
    perturb_points(recon, 0.05)
    tids = list(recon.TrackIds())[:10]
    before_pos = _positions(recon)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    focal_before = cam.FocalLength()
    pt.sfm.BundleAdjustTracks(recon, options(True, fix_intrinsics=False), tids)
    pt.sfm.BundleAdjustTrack(recon, options(True, fix_intrinsics=False), tids[0])
    assert cam.FocalLength() == focal_before
    for v, p in _positions(recon).items():
        np.testing.assert_array_equal(p, before_pos[v])


def test_cov_overloads_do_not_overwrite_points_with_stale_inverse_depth():
    """The WithCov overloads always optimize XYZ points; the inverse depths of a
    previous initialization must not be written back over the optimized points."""
    solutions = {}
    for inverse in (False, True):
        recon, truth = make_recon("PINHOLE", pixel_noise=0.3)
        recon.InitializeInverseDepth()  # anchors from the true points
        perturb_points(recon, 0.05)  # now stale
        tids = list(recon.TrackIds())[:5]
        pt.sfm.BundleAdjustTracksWithCov(recon, options(inverse), tids)
        pt.sfm.BundleAdjustTrackWithCov(recon, options(inverse), tids[0])
        solutions[inverse] = [euclidean(recon.Track(t).Point()) for t in tids]
        for tid in tids:
            track = recon.Track(tid)
            ref = recon.View(track.ReferenceViewId()).Camera()
            cam_point = ref.GetOrientationAsRotationMatrix() @ (euclidean(track.Point()) - ref.GetPosition())
            assert track.InverseDepth() == pytest.approx(1.0 / np.linalg.norm(cam_point), rel=1e-6)
    np.testing.assert_allclose(solutions[True], solutions[False], atol=1e-9)


def test_untriangulable_track_is_left_untouched():
    recon, truth = make_recon("PINHOLE")
    tid = list(recon.TrackIds())[0]
    track = recon.MutableTrack(tid)
    cam = recon.View(track.ReferenceViewId()).Camera()
    behind = cam.GetPosition() - cam.GetOrientationAsRotationMatrix().T @ np.array([0.0, 0.0, 3.0])
    track.SetPoint(np.append(behind, 1.0))
    summary = pt.sfm.BundleAdjustReconstruction(options(True), recon)
    assert np.isfinite(summary.final_cost)
    np.testing.assert_allclose(euclidean(recon.Track(tid).Point()), behind, atol=1e-12)
