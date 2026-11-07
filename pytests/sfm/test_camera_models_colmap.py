"""COLMAP camera models in pyTheia: projection, unprojection, priors and bundle adjustment.

The numpy functions below re-implement the COLMAP projections from
colmap/src/colmap/sensor/models.h (ImgFromCam). pyTheia's models must agree with
them for random parameters and points. The inverse (pixel -> ray) must invert
the projection, and bundle adjustment must recover intrinsics from a perturbed
initial guess, which exercises the ceres::Jet cost functions.
"""

import math

import numpy as np
import pytest

import pytheia as pt

WIDTH = 1200
HEIGHT = 900


# --------------------------------------------------------------------------- #
# numpy reference: COLMAP ImgFromCam for the models in models.h
# --------------------------------------------------------------------------- #


def _fisheye_from_normal(u, v):
    r = math.hypot(u, v)
    if r > 1e-12:
        scale = math.atan(r) / r
        return u * scale, v * scale
    return u, v


def colmap_full_opencv(p, X):
    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6 = p
    u, v = X[0] / X[2], X[1] / X[2]
    r2 = u * u + v * v
    r4 = r2 * r2
    r6 = r4 * r2
    radial = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6)
    du = u * radial + 2 * p1 * u * v + p2 * (r2 + 2 * u * u) - u
    dv = v * radial + 2 * p2 * u * v + p1 * (r2 + 2 * v * v) - v
    return np.array([fx * (u + du) + cx, fy * (v + dv) + cy])


def colmap_opencv_fisheye(p, X):
    fx, fy, cx, cy, k1, k2, k3, k4 = p
    uu, vv = _fisheye_from_normal(X[0] / X[2], X[1] / X[2])
    t2 = uu * uu + vv * vv
    radial = k1 * t2 + k2 * t2**2 + k3 * t2**3 + k4 * t2**4
    return np.array([fx * (uu + uu * radial) + cx, fy * (vv + vv * radial) + cy])


def colmap_fov(p, X):
    fx, fy, cx, cy, omega = p
    u, v = X[0] / X[2], X[1] / X[2]
    r = math.hypot(u, v)
    factor = 1.0 if r < 1e-12 else math.atan(2 * r * math.tan(omega / 2)) / (r * omega)
    return np.array([fx * u * factor + cx, fy * v * factor + cy])


def colmap_radial_fisheye(p, X):
    f, cx, cy, k1, k2 = p
    uu, vv = _fisheye_from_normal(X[0] / X[2], X[1] / X[2])
    t2 = uu * uu + vv * vv
    radial = k1 * t2 + k2 * t2**2
    return np.array([f * (uu + uu * radial) + cx, f * (vv + vv * radial) + cy])


def colmap_thin_prism_fisheye(p, X):
    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1 = p
    uu, vv = _fisheye_from_normal(X[0] / X[2], X[1] / X[2])
    r2 = uu * uu + vv * vv
    radial = k1 * r2 + k2 * r2**2 + k3 * r2**3 + k4 * r2**4
    du = uu * radial + 2 * p1 * uu * vv + p2 * (r2 + 2 * uu * uu) + sx1 * r2
    dv = vv * radial + 2 * p2 * uu * vv + p1 * (r2 + 2 * vv * vv) + sy1 * r2
    return np.array([fx * (uu + du) + cx, fy * (vv + dv) + cy])


def colmap_rad_tan_thin_prism_fisheye(p, X):
    fx, fy, cx, cy = p[:4]
    radial_coeffs = p[4:10]
    p0, p1 = p[10], p[11]
    s0, s1, s2, s3 = p[12:16]
    uu, vv = _fisheye_from_normal(X[0] / X[2], X[1] / X[2])
    theta2 = uu * uu + vv * vv
    th_radial = 1.0
    power = 1.0
    for k in radial_coeffs:
        power *= theta2
        th_radial += k * power
    x, y = th_radial * uu, th_radial * vv
    x2, y2, xy = x * x, y * y, x * y
    r2 = x2 + y2
    r4 = r2 * r2
    xd = x + 2 * p1 * xy + p0 * (r2 + 2 * x2) + s0 * r2 + s1 * r4
    yd = y + 2 * p0 * xy + p1 * (r2 + 2 * y2) + s2 * r2 + s3 * r4
    return np.array([fx * xd + cx, fy * yd + cy])


# --------------------------------------------------------------------------- #
# pyTheia model construction from COLMAP-ordered parameters
# --------------------------------------------------------------------------- #


def _fx_fy_cx_cy(p):
    fx, fy, cx, cy = p[:4]
    return fx, fy, cx, cy


def make_full_opencv(p):
    m = pt.sfm.FullOpenCVCameraModel()
    fx, fy, cx, cy = _fx_fy_cx_cy(p)
    m.SetFocalLength(fx)
    m.SetAspectRatio(fy / fx)
    m.SetPrincipalPoint(cx, cy)
    m.SetRadialDistortion(p[4], p[5], p[8], p[9], p[10], p[11])
    m.SetTangentialDistortion(p[6], p[7])
    return m


def make_radial_fisheye(p):
    f, cx, cy, k1, k2 = p
    m = pt.sfm.RadialFisheyeCameraModel()
    m.SetFocalLength(f)
    m.SetAspectRatio(1.0)
    m.SetPrincipalPoint(cx, cy)
    m.SetRadialDistortion(k1, k2)
    return m


def make_thin_prism_fisheye(p):
    fx, fy, cx, cy = _fx_fy_cx_cy(p)
    m = pt.sfm.ThinPrismFisheyeCameraModel()
    m.SetFocalLength(fx)
    m.SetAspectRatio(fy / fx)
    m.SetPrincipalPoint(cx, cy)
    m.SetRadialDistortion(p[4], p[5], p[8], p[9])
    m.SetTangentialDistortion(p[6], p[7])
    m.SetThinPrismDistortion(p[10], p[11])
    return m


def make_rad_tan_thin_prism_fisheye(p):
    fx, fy, cx, cy = _fx_fy_cx_cy(p)
    m = pt.sfm.RadTanThinPrismFisheyeCameraModel()
    m.SetFocalLength(fx)
    m.SetAspectRatio(fy / fx)
    m.SetPrincipalPoint(cx, cy)
    m.SetRadialDistortion(*p[4:10])
    m.SetTangentialDistortion(p[10], p[11])
    m.SetThinPrismDistortion(*p[12:16])
    return m


# --------------------------------------------------------------------------- #
# Random parameters and points
# --------------------------------------------------------------------------- #


def _random_camera_params(rng, model, scale=1.0):
    fx = rng.uniform(400.0, 1100.0)
    fy = fx * rng.uniform(0.97, 1.03)
    cx = rng.uniform(0.35, 0.65) * WIDTH
    cy = rng.uniform(0.35, 0.65) * HEIGHT
    if model == "FULL_OPENCV":
        d = [rng.uniform(-0.05, 0.05) * scale, rng.uniform(-0.02, 0.02) * scale,
             rng.uniform(-0.002, 0.002), rng.uniform(-0.002, 0.002),
             rng.uniform(-0.01, 0.01), rng.uniform(-0.05, 0.05) * scale,
             rng.uniform(-0.02, 0.02) * scale, rng.uniform(-0.01, 0.01) * scale]
        return [fx, fy, cx, cy] + d
    if model == "RADIAL_FISHEYE":
        return [fx, cx, cy, rng.uniform(-0.1, 0.1) * scale, rng.uniform(-0.05, 0.05) * scale]
    if model == "THIN_PRISM_FISHEYE":
        d = [rng.uniform(-0.1, 0.1) * scale, rng.uniform(-0.05, 0.05) * scale,
             rng.uniform(-0.002, 0.002), rng.uniform(-0.002, 0.002),
             rng.uniform(-0.02, 0.02) * scale, rng.uniform(-0.01, 0.01) * scale,
             rng.uniform(-0.002, 0.002), rng.uniform(-0.002, 0.002)]
        return [fx, fy, cx, cy] + d
    if model == "RAD_TAN_THIN_PRISM_FISHEYE":
        d = [rng.uniform(-0.1, 0.1) * scale, rng.uniform(-0.05, 0.05) * scale,
             rng.uniform(-0.02, 0.02) * scale, rng.uniform(-0.01, 0.01) * scale,
             rng.uniform(-0.005, 0.005) * scale, rng.uniform(-0.002, 0.002) * scale,
             rng.uniform(-0.002, 0.002), rng.uniform(-0.002, 0.002),
             rng.uniform(-0.001, 0.001), rng.uniform(-0.001, 0.001),
             rng.uniform(-0.001, 0.001), rng.uniform(-0.001, 0.001)]
        return [fx, fy, cx, cy] + d
    raise ValueError(model)


def _random_points(rng, n, max_angle_tan, model):
    """Points in front of the camera with |x / z|, |y / z| below max_angle_tan."""
    pts = []
    for _ in range(n):
        z = rng.uniform(1.0, 8.0)
        x = rng.uniform(-max_angle_tan, max_angle_tan) * z
        y = rng.uniform(-max_angle_tan, max_angle_tan) * z
        pts.append([x, y, z])
    return np.array(pts)


MODELS = {
    "FULL_OPENCV": (make_full_opencv, colmap_full_opencv, 12),
    "RADIAL_FISHEYE": (make_radial_fisheye, colmap_radial_fisheye, 5),
    "THIN_PRISM_FISHEYE": (make_thin_prism_fisheye, colmap_thin_prism_fisheye, 12),
    "RAD_TAN_THIN_PRISM_FISHEYE": (make_rad_tan_thin_prism_fisheye, colmap_rad_tan_thin_prism_fisheye, 16),
}

# Fisheye models accept larger angles than the pinhole-type FULL_OPENCV.
MAX_TAN = {"FULL_OPENCV": 0.6, "RADIAL_FISHEYE": 1.2, "THIN_PRISM_FISHEYE": 1.2,
           "RAD_TAN_THIN_PRISM_FISHEYE": 1.2}


def _split(name, p):
    """(fx, fy, cx, cy, distortion) of COLMAP-ordered parameters.

    RADIAL_FISHEYE has one focal length and is stored as f, cx, cy, k1, k2.
    """
    if name == "RADIAL_FISHEYE":
        return p[0], p[0], p[1], p[2], list(p[3:])
    return p[0], p[1], p[2], p[3], list(p[4:])


def _join(name, fx, fy, cx, cy, distortion):
    """Inverse of _split."""
    if name == "RADIAL_FISHEYE":
        return [fx, cx, cy] + list(distortion)
    return [fx, fy, cx, cy] + list(distortion)


def test_model_types_and_parameter_counts():
    expected = {
        "FULL_OPENCV": (pt.sfm.FullOpenCVCameraModel, 12),
        "RADIAL_FISHEYE": (pt.sfm.RadialFisheyeCameraModel, 6),
        "THIN_PRISM_FISHEYE": (pt.sfm.ThinPrismFisheyeCameraModel, 12),
        "RAD_TAN_THIN_PRISM_FISHEYE": (pt.sfm.RadTanThinPrismFisheyeCameraModel, 16),
    }
    for name, (cls, n) in expected.items():
        model = cls()
        assert model.Type() == getattr(pt.sfm.CameraIntrinsicsModelType, name)
        assert model.NumParameters() == n
        assert model.kIntrinsicsSize == n


@pytest.mark.parametrize("name", list(MODELS))
def test_projection_matches_colmap_formulas(name):
    make, reference, _ = MODELS[name]
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(40):
        p = _random_camera_params(rng, name)
        model = make(p)
        for X in _random_points(rng, 25, MAX_TAN[name], name):
            got = np.array(model.CameraToImageCoordinates(X))
            worst = max(worst, np.abs(got - reference(p, X)).max())
    assert worst < 1e-6, "max deviation from COLMAP: %g px" % worst


@pytest.mark.parametrize("name", list(MODELS))
def test_pixel_to_ray_inverts_projection(name):
    make, _, _ = MODELS[name]
    rng = np.random.default_rng(2)
    for _ in range(30):
        p = _random_camera_params(rng, name)
        model = make(p)
        for X in _random_points(rng, 20, MAX_TAN[name], name):
            pixel = np.array(model.CameraToImageCoordinates(X))
            ray = np.array(model.ImageToCameraCoordinates(pixel))
            # Same direction: compare the normalized points.
            np.testing.assert_allclose(ray[:2] / ray[2], X[:2] / X[2], atol=1e-7)
            np.testing.assert_allclose(np.array(model.CameraToImageCoordinates(ray)), pixel, atol=1e-6)


@pytest.mark.parametrize("name", list(MODELS))
def test_camera_wrapper_dispatches_to_new_models(name):
    make, reference, _ = MODELS[name]
    rng = np.random.default_rng(3)
    p = _random_camera_params(rng, name)
    cam = pt.sfm.Camera()
    cam.SetCameraIntrinsicsModelType(getattr(pt.sfm.CameraIntrinsicsModelType, name))
    cam.SetImageSize(WIDTH, HEIGHT)
    model = cam.CameraIntrinsics()
    fx, fy, cx, cy, distortion = _split(name, p)
    model.SetFocalLength(fx)
    model.SetAspectRatio(fy / fx)
    model.SetPrincipalPoint(cx, cy)
    # The distortion terms use the generic parameter setter, in COLMAP order from
    # index 4 (RADIAL_FISHEYE: k1, k2 at indices 4 and 5).
    for i, value in enumerate(distortion):
        model.SetParameter(4 + i, value)
    X = _random_points(rng, 1, 0.5, name)[0]
    depth, pixel = cam.ProjectPoint(X)
    assert depth > 0
    np.testing.assert_allclose(pixel, reference(p, X), atol=1e-6)


@pytest.mark.parametrize("name", list(MODELS))
def test_prior_round_trip_keeps_parameters(name):
    make, _, _ = MODELS[name]
    model_class = {
        "FULL_OPENCV": pt.sfm.FullOpenCVCameraModel,
        "RADIAL_FISHEYE": pt.sfm.RadialFisheyeCameraModel,
        "THIN_PRISM_FISHEYE": pt.sfm.ThinPrismFisheyeCameraModel,
        "RAD_TAN_THIN_PRISM_FISHEYE": pt.sfm.RadTanThinPrismFisheyeCameraModel,
    }[name]
    rng = np.random.default_rng(4)
    p = _random_camera_params(rng, name)
    model = make(p)
    prior = model.CameraIntrinsicsPriorFromIntrinsics()
    assert prior.camera_intrinsics_model_type == name
    copy = model_class()
    copy.SetFromCameraIntrinsicsPriors(prior)
    for i in range(model.NumParameters()):
        assert copy.GetParameter(i) == pytest.approx(model.GetParameter(i), abs=0.0), i


def test_fisheye_and_fov_forward_match_colmap():
    """pyTheia FISHEYE and FOV project like COLMAP's OPENCV_FISHEYE and FOV."""
    rng = np.random.default_rng(5)
    worst_fisheye = worst_fov = 0.0
    for _ in range(40):
        fx = rng.uniform(400.0, 1100.0)
        fy = fx * rng.uniform(0.97, 1.03)
        cx, cy = rng.uniform(300, 900), rng.uniform(200, 700)
        k = rng.uniform(-0.05, 0.05, size=4)
        fisheye = pt.sfm.FisheyeCameraModel()
        fisheye.SetFocalLength(fx)
        fisheye.SetAspectRatio(fy / fx)
        fisheye.SetSkew(0.0)
        fisheye.SetPrincipalPoint(cx, cy)
        fisheye.SetRadialDistortion(*k)
        p = [fx, fy, cx, cy, *k]
        omega = rng.uniform(0.3, 1.2)
        fov = pt.sfm.FOVCameraModel()
        fov.SetFocalLength(fx)
        fov.SetAspectRatio(fy / fx)
        fov.SetPrincipalPoint(cx, cy)
        fov.SetRadialDistortion(omega)
        q = [fx, fy, cx, cy, omega]
        for X in _random_points(rng, 20, 1.0, "OPENCV_FISHEYE"):
            worst_fisheye = max(worst_fisheye,
                                np.abs(np.array(fisheye.CameraToImageCoordinates(X)) - colmap_opencv_fisheye(p, X)).max())
            worst_fov = max(worst_fov, np.abs(np.array(fov.CameraToImageCoordinates(X)) - colmap_fov(q, X)).max())
    assert worst_fisheye < 1e-6, worst_fisheye
    assert worst_fov < 1e-5, worst_fov


@pytest.mark.xfail(
    strict=True,
    reason="pyTheia FISHEYE UndistortPoint is a fixed-point iteration (100 steps); "
           "it does not converge at ray angles near 80 degrees with |k| ~ 0.05.",
)
def test_fisheye_unprojection_converges_at_large_angles():
    k = [0.0337, -0.0485, 0.0218, -0.0102]
    fisheye = pt.sfm.FisheyeCameraModel()
    fisheye.SetFocalLength(800.0)
    fisheye.SetAspectRatio(1.0)
    fisheye.SetSkew(0.0)
    fisheye.SetPrincipalPoint(600.0, 400.0)
    fisheye.SetRadialDistortion(*k)
    X = np.array([math.sin(1.4), 0.0, math.cos(1.4)])
    pixel = np.array(fisheye.CameraToImageCoordinates(X))
    ray = np.array(fisheye.ImageToCameraCoordinates(pixel))
    np.testing.assert_allclose(ray[0] / ray[2], X[0] / X[2], atol=1e-6)


def _scene(name, params, rng, n_views=5, n_points=60):
    """Ground-truth poses, points and pixel observations for one model."""
    make, _, _ = MODELS[name]
    truth = make(params)
    views = []
    for _ in range(n_views):
        axis = rng.normal(size=3)
        axis = 0.15 * axis / np.linalg.norm(axis) * rng.uniform(0.0, 1.0)
        R = _rodrigues(axis)
        C = rng.normal(scale=0.3, size=3)
        views.append((R, C))
    points = []
    for _ in range(n_points):
        points.append(rng.uniform([-1.0, -0.8, 4.0], [1.0, 0.8, 7.0]))
    points = np.array(points)
    observations = []  # (view index, point index, pixel)
    for vi, (R, C) in enumerate(views):
        for pi, X in enumerate(points):
            Xc = R @ (X - C)
            if Xc[2] <= 0.5:
                continue
            if abs(Xc[0] / Xc[2]) > MAX_TAN[name] or abs(Xc[1] / Xc[2]) > MAX_TAN[name]:
                continue
            pixel = np.array(truth.CameraToImageCoordinates(Xc))
            if 0 <= pixel[0] < WIDTH and 0 <= pixel[1] < HEIGHT:
                observations.append((vi, pi, pixel))
    return views, points, observations


def _rodrigues(axis_angle):
    theta = np.linalg.norm(axis_angle)
    if theta < 1e-12:
        return np.eye(3)
    k = axis_angle / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K


def _build_recon(name, params, views, points, observations):
    recon = pt.sfm.Reconstruction()
    view_ids = []
    for vi, (R, C) in enumerate(views):
        vid = recon.AddView("view_%d.png" % vi, 0, float(vi))
        cam = recon.MutableView(vid).MutableCamera()
        cam.SetCameraIntrinsicsModelType(getattr(pt.sfm.CameraIntrinsicsModelType, name))
        cam.SetImageSize(WIDTH, HEIGHT)
        model = cam.CameraIntrinsics()
        fx, fy, cx, cy, distortion = _split(name, params)
        cam.SetFocalLength(fx)
        model.SetAspectRatio(fy / fx)
        cam.SetPrincipalPoint(cx, cy)
        for i, value in enumerate(distortion):
            model.SetParameter(4 + i, value)
        cam.SetOrientationFromRotationMatrix(R)
        cam.SetPosition(C)
        recon.MutableView(vid).SetIsEstimated(True)
        view_ids.append(vid)

    # One track per 3D point with all of its observations.
    tracks = {}
    for vi, pi, pixel in observations:
        tracks.setdefault(pi, []).append((view_ids[vi], pt.sfm.Feature(pixel)))
    for pi, features in sorted(tracks.items()):
        if len(features) < 2:
            continue
        tid = recon.AddTrack(features)
        recon.MutableTrack(tid).SetPoint(np.append(points[pi], 1.0))
        recon.MutableTrack(tid).SetIsEstimated(True)
    return recon


def _rms_reprojection(recon):
    errors = []
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        point = track.Point()[:3]
        for vid in track.ViewIds():
            view = recon.View(vid)
            feature = np.asarray(view.GetFeature(tid).point)
            _, projected = view.Camera().ProjectPoint(point)
            errors.append(np.linalg.norm(np.asarray(projected) - feature))
    return math.sqrt(np.mean(np.square(errors)))


@pytest.mark.parametrize("name", list(MODELS))
def test_bundle_adjustment_recovers_intrinsics(name):
    # The reprojection cost is built from the templated CameraToPixelCoordinates
    # with ceres::Jet, so this exercises the autodiff path of each model.
    make, _, n = MODELS[name]
    rng = np.random.default_rng(6)
    truth_params = _random_camera_params(rng, name, scale=0.5)
    views, points, observations = _scene(name, truth_params, rng)

    fx, fy, cx, cy, distortion = _split(name, truth_params)
    init = _join(
        name,
        fx * 1.02,  # focal length
        fy * 1.02,
        cx + 3.0,  # principal point
        cy - 3.0,
        [0.7 * d for d in distortion],  # distortion
    )

    recon = _build_recon(name, init, views, points, observations)
    initial_rms = _rms_reprojection(recon)

    options = pt.sfm.BundleAdjustmentOptions()
    options.verbose = False
    options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.ALL
    pt.sfm.BundleAdjustReconstruction(options, recon)

    final_rms = _rms_reprojection(recon)
    assert initial_rms > 0.5, "initial error too small to test anything: %g px" % initial_rms
    assert final_rms < 0.05, "BA did not converge: %g px (initial %g px)" % (final_rms, initial_rms)
    assert final_rms < 0.1 * initial_rms
    recovered = recon.View(recon.ViewIds()[0]).Camera()
    assert recovered.FocalLength() == pytest.approx(truth_params[0], rel=5e-3)
