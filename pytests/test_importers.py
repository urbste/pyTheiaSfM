"""Tests for pytheia.importers: COLMAP models, OpenMVG sfm_data.json, COLMAP database.

Synthetic reconstructions and fixtures are built inside the tests, so every
projected observation is known. The real-data smoke tests run only when the
environment variables below are set:

* ``PYTHEIA_COLMAP_SPARSE``: a COLMAP sparse folder (binary or text)
* ``PYTHEIA_COLMAP_DB``: a COLMAP ``database.db``
* ``PYTHEIA_OPENMVG_JSON``: an OpenMVG ``sfm_data.json``
"""

import json
import math
import os
import sqlite3
import struct
import warnings

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")

from pytheia.importers import (  # noqa: E402
    read_colmap_database,
    read_colmap_model,
    read_openmvg_sfm_data,
)

WIDTH = 640
HEIGHT = 480
FOCAL = 500.0
CX = 320.0
CY = 240.0
VIEW_NAMES = ["img_0.png", "sub/img_1.png", "img_2.png"]


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #


def _rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _project_openmvg(R, C, X, f, cx, cy, k=(0.0, 0.0, 0.0)):
    """Projection with the OpenMVG radial convention: x_d = x (1 + k1 r^2 + k2 r^4 + k3 r^6)."""
    xc = R @ (X - C)
    xn, yn = xc[0] / xc[2], xc[1] / xc[2]
    r2 = xn * xn + yn * yn
    d = 1.0 + k[0] * r2 + k[1] * r2 * r2 + k[2] * r2 * r2 * r2
    return np.array([f * xn * d + cx, f * yn * d + cy])


def _set_colmap_params(cam, model, params):
    """Sets a camera from COLMAP-ordered parameters (fx, fy, cx, cy, extra...).

    RADIAL_FISHEYE has one focal length: f, cx, cy, k1, k2.
    """
    model_type = getattr(pt.sfm.CameraIntrinsicsModelType, model)
    cam.SetCameraIntrinsicsModelType(model_type)
    intrinsics = cam.CameraIntrinsics()
    if model == "RADIAL_FISHEYE":
        fx = fy = params[0]
        cx, cy = params[1], params[2]
        extra = params[3:]
    else:
        fx, fy, cx, cy = params[:4]
        extra = params[4:]
    cam.SetFocalLength(float(fx))
    cam.SetPrincipalPoint(float(cx), float(cy))
    intrinsics.SetAspectRatio(float(fy) / float(fx))
    for i, value in enumerate(extra):
        intrinsics.SetParameter(4 + i, float(value))


def _make_recon(k1=0.0, k2=0.0, n_points=40, seed=0, model=None, model_params=None):
    """Three views (two intrinsics groups) with projected tracks, all estimated.

    With ``model`` set, the cameras use that model with ``model_params`` (COLMAP
    order) instead of the radial PINHOLE camera.
    """
    rng = np.random.default_rng(seed)
    recon = pt.sfm.Reconstruction()
    poses = [
        (np.eye(3), np.zeros(3)),
        (_rot_y(0.1), np.array([0.5, 0.05, -0.2])),
        (_rot_x(-0.05), np.array([-0.4, 0.1, 0.1])),
    ]
    view_ids = []
    for idx, (name, (rotation, center)) in enumerate(zip(VIEW_NAMES, poses)):
        group = 0 if idx < 2 else 1
        vid = recon.AddView(name, group, float(idx))
        cam = recon.MutableView(vid).MutableCamera()
        cam.SetImageSize(WIDTH, HEIGHT)
        if model is None:
            cam.SetFocalLength(FOCAL)
            cam.SetPrincipalPoint(CX, CY)
            cam.CameraIntrinsics().SetRadialDistortion(k1, k2)
        else:
            _set_colmap_params(cam, model, model_params)
        cam.SetOrientationFromRotationMatrix(rotation)
        cam.SetPosition(center)
        recon.MutableView(vid).SetIsEstimated(True)
        view_ids.append(vid)

    cameras = {vid: recon.View(vid).Camera() for vid in view_ids}
    for _ in range(n_points):
        X = np.array([rng.uniform(-1.0, 1.0), rng.uniform(-0.8, 0.8), rng.uniform(3.0, 6.0)])
        features = []
        for vid in view_ids:
            depth, pixel = cameras[vid].ProjectPoint(X)
            if depth > 0 and 0 <= pixel[0] < WIDTH and 0 <= pixel[1] < HEIGHT:
                features.append((vid, pt.sfm.Feature(np.asarray(pixel, dtype=float))))
        if len(features) < 2:
            continue
        tid = recon.AddTrack(features)
        recon.MutableTrack(tid).SetPoint(np.append(X, 1.0))
        recon.MutableTrack(tid).SetIsEstimated(True)
        recon.MutableTrack(tid).SetColor(np.array(rng.integers(0, 256, size=3), dtype=np.uint8))
    return recon


def _summary(recon):
    """Per-view camera data and per-track points, keyed by view name."""
    views = {}
    for vid in recon.ViewIds():
        view = recon.View(vid)
        cam = view.Camera()
        views[view.Name()] = dict(
            center=np.asarray(cam.GetPosition()),
            rotation=np.asarray(cam.GetOrientationAsRotationMatrix()),
            estimated=view.IsEstimated(),
        )
    points = sorted(tuple(np.round(recon.Track(t).Point()[:3], 9)) for t in recon.TrackIds())
    return views, points


# --------------------------------------------------------------------------- #
# COLMAP fixture writers (text and binary, following the COLMAP documentation)
# --------------------------------------------------------------------------- #


def _write_colmap_text(directory, cameras, images, points):
    """cameras: [(id, model, w, h, params)]; images: [(id, qvec, tvec, cam, name, [(x, y, pid)])];
    points: [(pid, xyz, rgb, error, [(image_id, idx)])]."""
    directory.mkdir(parents=True, exist_ok=True)
    with open(directory / "cameras.txt", "w", encoding="utf-8") as f:
        f.write("# Camera list with one line of data per camera:\n")
        for cid, model, w, h, params in cameras:
            f.write("%d %s %d %d %s\n" % (cid, model, w, h, " ".join(repr(float(p)) for p in params)))
    with open(directory / "images.txt", "w", encoding="utf-8") as f:
        f.write("# Image list with two lines of data per image:\n")
        for iid, q, t, cid, name, pts2d in images:
            f.write(
                "%d %s %s %d %s\n"
                % (
                    iid,
                    " ".join(repr(float(v)) for v in q),
                    " ".join(repr(float(v)) for v in t),
                    cid,
                    name,
                )
            )
            f.write(" ".join("%r %r %d" % (float(x), float(y), pid) for x, y, pid in pts2d) + "\n")
    with open(directory / "points3D.txt", "w", encoding="utf-8") as f:
        f.write("# 3D point list with one line of data per point:\n")
        for pid, xyz, rgb, err, track in points:
            track_str = " ".join("%d %d" % (i, k) for i, k in track)
            f.write(
                "%d %s %d %d %d %r %s\n"
                % (pid, " ".join(repr(float(v)) for v in xyz), rgb[0], rgb[1], rgb[2], err, track_str)
            )


def _write_colmap_binary(directory, cameras, images, points):
    directory.mkdir(parents=True, exist_ok=True)
    with open(directory / "cameras.bin", "wb") as f:
        f.write(struct.pack("<Q", len(cameras)))
        for cid, model_id, w, h, params in cameras:
            f.write(struct.pack("<IiQQ", cid, model_id, w, h))
            f.write(struct.pack("<%dd" % len(params), *params))
    with open(directory / "images.bin", "wb") as f:
        f.write(struct.pack("<Q", len(images)))
        for iid, q, t, cid, name, pts2d in images:
            f.write(struct.pack("<I4d3dI", iid, *q, *t, cid))
            f.write(name.encode("utf-8") + b"\x00")
            f.write(struct.pack("<Q", len(pts2d)))
            for x, y, pid in pts2d:
                f.write(struct.pack("<ddq", x, y, pid))
    with open(directory / "points3D.bin", "wb") as f:
        f.write(struct.pack("<Q", len(points)))
        for pid, xyz, rgb, err, track in points:
            f.write(struct.pack("<Q3d3Bd", pid, *xyz, *rgb, err))
            f.write(struct.pack("<Q", len(track)))
            for iid, k in track:
                f.write(struct.pack("<II", iid, k))


# --------------------------------------------------------------------------- #
# COLMAP round trips through pt.io.WriteColmapFiles
# --------------------------------------------------------------------------- #


def _check_roundtrip(recon, read, tol=1e-9):
    """Checks cameras, points and reprojection after a WriteColmapFiles round trip.

    WriteColmapFiles writes pyTheia pixel coordinates unchanged and the reader
    keeps them unchanged by default (``pixel_offset=0``), so the round trip is
    exact.

    ``tol`` is 1e-9 for binary files. The text writer prints 6 significant
    digits, so text round trips pass a looser tolerance.
    """
    assert read.NumViews() == recon.NumViews()
    assert read.NumTracks() == recon.NumTracks()
    assert read.NumCameraIntrinsicGroups() == recon.NumCameraIntrinsicGroups()

    orig_views, orig_points = _summary(recon)
    read_views, read_points = _summary(read)
    assert set(read_views) == set(orig_views) == set(VIEW_NAMES)
    for name in VIEW_NAMES:
        np.testing.assert_allclose(read_views[name]["center"], orig_views[name]["center"], atol=tol)
        np.testing.assert_allclose(read_views[name]["rotation"], orig_views[name]["rotation"], atol=tol)
        assert read_views[name]["estimated"] is True
    np.testing.assert_allclose(read_points, orig_points, atol=tol)

    # Reprojection of every observation with the read camera equals the read
    # observation, and the observation is the original shifted by -0.5 px.
    for read_tid in read.TrackIds():
        track = read.Track(read_tid)
        point = track.Point()[:3]
        orig_tid = next(
            t for t in recon.TrackIds()
            if np.allclose(recon.Track(t).Point()[:3], point, atol=10 * tol)
        )
        orig_track = recon.Track(orig_tid)
        assert sorted(track.ViewIds()) == sorted(
            read.ViewIdFromName(recon.View(v).Name()) for v in orig_track.ViewIds()
        )
        for vid in track.ViewIds():
            name = read.View(vid).Name()
            read_obs = np.asarray(read.View(vid).GetFeature(read_tid).point)
            depth, projected = read.View(vid).Camera().ProjectPoint(point)
            assert depth > 0
            np.testing.assert_allclose(projected, read_obs, atol=max(1e-6, 10 * tol))
            orig_vid = recon.ViewIdFromName(name)
            orig_obs = np.asarray(recon.View(orig_vid).GetFeature(orig_tid).point)
            np.testing.assert_allclose(read_obs, orig_obs, atol=max(1e-6, 100 * tol))


def test_colmap_binary_roundtrip(tmp_path):
    recon = _make_recon()
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=True)
    read = read_colmap_model(str(tmp_path))
    assert isinstance(read, pt.sfm.Reconstruction)
    _check_roundtrip(recon, read, tol=1e-9)


def test_colmap_text_roundtrip(tmp_path):
    recon = _make_recon()
    assert pt.io.WriteColmapFiles(recon, str(tmp_path))
    read = read_colmap_model(str(tmp_path))
    # Text output has 6 significant digits; 1e-4 covers that for these scales.
    _check_roundtrip(recon, read, tol=1e-4)


def test_colmap_text_and_binary_agree(tmp_path):
    recon = _make_recon(k1=-0.05, k2=0.01)
    txt_dir = tmp_path / "txt"
    bin_dir = tmp_path / "bin"
    txt_dir.mkdir()
    bin_dir.mkdir()
    assert pt.io.WriteColmapFiles(recon, str(txt_dir))
    assert pt.io.WriteColmapFiles(recon, str(bin_dir), binary=True)
    a = read_colmap_model(str(txt_dir))
    b = read_colmap_model(str(bin_dir))
    # The text writer emits RADIAL (read as PINHOLE) and the binary writer emits
    # OPENCV (read as PINHOLE_RADIAL_TANGENTIAL). The models differ, but both
    # must project the same points.
    point = np.array([0.3, -0.2, 4.0])
    for name in VIEW_NAMES:
        ca = a.View(a.ViewIdFromName(name)).Camera()
        cb = b.View(b.ViewIdFromName(name)).Camera()
        np.testing.assert_allclose(ca.GetPosition(), cb.GetPosition(), atol=1e-4)
        _, pa = ca.ProjectPoint(point)
        _, pb = cb.ProjectPoint(point)
        np.testing.assert_allclose(pa, pb, atol=1e-3)


def test_colmap_distorted_roundtrip_keeps_projection(tmp_path):
    # The binary writer uses OPENCV for distorted cameras, which reads back as
    # PINHOLE_RADIAL_TANGENTIAL with the same k1, k2 and zero tangential terms.
    k1, k2 = -0.05, 0.01
    recon = _make_recon(k1=k1, k2=k2)
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=True)
    read = read_colmap_model(str(tmp_path))
    vid = read.ViewIdFromName("img_0.png")
    model = read.View(vid).Camera().CameraIntrinsics()
    assert read.View(vid).Camera().GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL
    assert model.GetParameter(5) == pytest.approx(k1)
    assert model.GetParameter(6) == pytest.approx(k2)
    assert model.GetParameter(8) == pytest.approx(0.0)
    assert model.GetParameter(9) == pytest.approx(0.0)
    _check_roundtrip(recon, read)


_WRITER_EXTENDED_CASES = [
    ("FULL_OPENCV", [FOCAL, FOCAL * 1.01, CX, CY, 0.03, -0.004, 0.001, -0.002, 0.0005, 0.1, -0.2, 0.3]),
    ("RADIAL_FISHEYE", [FOCAL, CX, CY, 0.1, -0.02]),
    ("THIN_PRISM_FISHEYE",
     [FOCAL, FOCAL * 1.01, CX, CY, 0.1, -0.02, 0.001, -0.002, 0.003, -0.004, 0.0005, -0.0006]),
    ("RAD_TAN_THIN_PRISM_FISHEYE",
     [FOCAL, FOCAL * 1.01, CX, CY, 0.1, -0.02, 0.03, -0.004, 0.005, -0.006, 0.001, -0.002,
      0.0003, -0.0004, 0.0005, -0.0006]),
]


@pytest.mark.parametrize("binary", [False, True])
@pytest.mark.parametrize("model,params", _WRITER_EXTENDED_CASES, ids=[c[0] for c in _WRITER_EXTENDED_CASES])
def test_colmap_writer_extended_models_roundtrip(tmp_path, model, params, binary):
    # The writer emits the COLMAP model with every parameter; reading it back
    # reprojects every observation of the original cameras.
    recon = _make_recon(model=model, model_params=params)
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=binary)
    read = read_colmap_model(str(tmp_path))
    cam = read.View(read.ViewIdFromName(VIEW_NAMES[0])).Camera()
    assert cam.GetCameraIntrinsicsModelType() == getattr(pt.sfm.CameraIntrinsicsModelType, model)
    _check_roundtrip(recon, read, tol=1e-9 if binary else 1e-4)


# --------------------------------------------------------------------------- #
# COLMAP model variants and error handling (hand-written fixtures)
# --------------------------------------------------------------------------- #

IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]
ZERO_T = [0.0, 0.0, 0.0]


def _single_view_model(directory, model, params, binary):
    cameras = [(1, model, 800, 600, params)]
    images = [(1, IDENTITY_Q, ZERO_T, 1, "a.jpg", [(100.5, 50.5, -1)])]
    points = []
    if binary:
        ids = {"SIMPLE_PINHOLE": 0, "PINHOLE": 1, "SIMPLE_RADIAL": 2, "RADIAL": 3, "OPENCV": 4,
               "OPENCV_FISHEYE": 5, "FULL_OPENCV": 6, "FOV": 7, "SIMPLE_RADIAL_FISHEYE": 8,
               "RADIAL_FISHEYE": 9, "THIN_PRISM_FISHEYE": 10, "RAD_TAN_THIN_PRISM_FISHEYE": 11}
        _write_colmap_binary(
            directory, [(1, ids[model], 800, 600, params)],
            [(1, IDENTITY_Q, ZERO_T, 1, "a.jpg", [(100.5, 50.5, -1)])], points,
        )
    else:
        _write_colmap_text(directory, cameras, images, points)
    return read_colmap_model(str(directory))


@pytest.mark.parametrize("binary", [False, True])
def test_colmap_model_mappings(tmp_path, binary):
    # SIMPLE_PINHOLE -> PINHOLE with square pixels and no distortion.
    recon = _single_view_model(tmp_path / "sp", "SIMPLE_PINHOLE", [400.0, 300.0, 200.0], binary)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE
    assert cam.FocalLength() == pytest.approx(400.0)
    assert cam.PrincipalPointX() == pytest.approx(300.0)  # kept as in COLMAP (pixel_offset=0)
    assert cam.PrincipalPointY() == pytest.approx(200.0)
    assert cam.ImageWidth() == 800 and cam.ImageHeight() == 600

    # PINHOLE -> PINHOLE with aspect ratio fy / fx.
    recon = _single_view_model(tmp_path / "p", "PINHOLE", [400.0, 404.0, 300.0, 200.0], binary)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE
    assert cam.CameraIntrinsics().GetParameter(1) == pytest.approx(404.0 / 400.0)

    # SIMPLE_RADIAL -> PINHOLE with k1.
    recon = _single_view_model(tmp_path / "sr", "SIMPLE_RADIAL", [400.0, 300.0, 200.0, 0.03], binary)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.CameraIntrinsics().GetParameter(5) == pytest.approx(0.03)
    assert cam.CameraIntrinsics().GetParameter(6) == pytest.approx(0.0)

    # RADIAL -> PINHOLE with k1, k2.
    recon = _single_view_model(tmp_path / "r", "RADIAL", [400.0, 300.0, 200.0, 0.03, -0.004], binary)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE
    assert cam.CameraIntrinsics().GetParameter(5) == pytest.approx(0.03)
    assert cam.CameraIntrinsics().GetParameter(6) == pytest.approx(-0.004)

    # OPENCV -> PINHOLE_RADIAL_TANGENTIAL with k1, k2, p1, p2.
    recon = _single_view_model(
        tmp_path / "o", "OPENCV", [400.0, 401.0, 300.0, 200.0, 0.03, -0.004, 0.001, -0.002], binary
    )
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL
    model = cam.CameraIntrinsics()
    assert model.GetParameter(1) == pytest.approx(401.0 / 400.0)
    assert model.GetParameter(5) == pytest.approx(0.03)
    assert model.GetParameter(6) == pytest.approx(-0.004)
    assert model.GetParameter(7) == pytest.approx(0.0)
    assert model.GetParameter(8) == pytest.approx(0.001)
    assert model.GetParameter(9) == pytest.approx(-0.002)


def _colmap_reference_project(model, params, X):
    """numpy version of the COLMAP projection (colmap/src/colmap/sensor/models.h)."""
    X = np.asarray(X, dtype=float)
    u, v = X[0] / X[2], X[1] / X[2]
    if model in ("FULL_OPENCV",):
        fx, fy, cx, cy = params[:4]
        k1, k2, p1, p2, k3, k4, k5, k6 = params[4:12]
        r2 = u * u + v * v
        radial = (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3) / (1 + k4 * r2 + k5 * r2**2 + k6 * r2**3)
        x = u * radial + 2 * p1 * u * v + p2 * (r2 + 2 * u * u)
        y = v * radial + 2 * p2 * u * v + p1 * (r2 + 2 * v * v)
        return np.array([fx * x + cx, fy * y + cy])
    if model == "FOV":
        fx, fy, cx, cy, omega = params
        r = math.sqrt(u * u + v * v)
        factor = 1.0 if r < 1e-12 else math.atan(2 * r * math.tan(omega / 2)) / (r * omega)
        return np.array([fx * u * factor + cx, fy * v * factor + cy])
    # Fisheye models: the angle scaled point, then the model distortion.
    if model in ("OPENCV_FISHEYE", "RADIAL_FISHEYE", "THIN_PRISM_FISHEYE", "RAD_TAN_THIN_PRISM_FISHEYE"):
        r = math.sqrt(u * u + v * v)
        scale = math.atan(r) / r if r > 1e-8 else 1.0
        uu, vv = u * scale, v * scale
        t2 = uu * uu + vv * vv
        if model == "OPENCV_FISHEYE":
            fx, fy, cx, cy = params[:4]
            k1, k2, k3, k4 = params[4:8]
            radial = k1 * t2 + k2 * t2**2 + k3 * t2**3 + k4 * t2**4
            x, y = uu + uu * radial, vv + vv * radial
        elif model == "RADIAL_FISHEYE":
            f, cx, cy = params[:3]
            fx = fy = f
            k1, k2 = params[3:5]
            radial = k1 * t2 + k2 * t2**2
            x, y = uu + uu * radial, vv + vv * radial
        elif model == "THIN_PRISM_FISHEYE":
            fx, fy, cx, cy = params[:4]
            k1, k2, p1, p2, k3, k4, sx1, sy1 = params[4:12]
            radial = k1 * t2 + k2 * t2**2 + k3 * t2**3 + k4 * t2**4
            x = uu + uu * radial + 2 * p1 * uu * vv + p2 * (t2 + 2 * uu * uu) + sx1 * t2
            y = vv + vv * radial + 2 * p2 * uu * vv + p1 * (t2 + 2 * vv * vv) + sy1 * t2
        else:
            fx, fy, cx, cy = params[:4]
            th = 1.0
            power = 1.0
            for i in range(6):
                power *= t2
                th += params[4 + i] * power
            xr, yr = th * uu, th * vv
            p0, p1 = params[10], params[11]
            s0, s1, s2, s3 = params[12:16]
            x2, y2, xy = xr * xr, yr * yr, xr * yr
            r2 = x2 + y2
            r4 = r2 * r2
            x = xr + 2 * p1 * xy + p0 * (r2 + 2 * x2) + s0 * r2 + s1 * r4
            y = yr + 2 * p0 * xy + p1 * (r2 + 2 * y2) + s2 * r2 + s3 * r4
        return np.array([fx * x + cx, fy * y + cy])
    raise ValueError(model)


# COLMAP models that pyTheia imports with every parameter (all pyTheia models
# besides FULL_OPENCV, FISHEYE and FOV are reached through this list too).
_EXTENDED_COLMAP_CASES = [
    ("OPENCV_FISHEYE", "FISHEYE", [400.0, 402.0, 300.0, 200.0, 0.1, -0.02, 0.003, -0.0004]),
    ("FOV", "FOV", [400.0, 404.0, 300.0, 200.0, 0.8]),
    ("FULL_OPENCV", "FULL_OPENCV",
     [400.0, 401.0, 300.0, 200.0, 0.03, -0.004, 0.001, -0.002, 0.0005, 0.1, -0.2, 0.3]),
    ("SIMPLE_RADIAL_FISHEYE", "RADIAL_FISHEYE", [400.0, 300.0, 200.0, 0.1]),
    ("RADIAL_FISHEYE", "RADIAL_FISHEYE", [400.0, 300.0, 200.0, 0.1, -0.02]),
    ("THIN_PRISM_FISHEYE", "THIN_PRISM_FISHEYE",
     [400.0, 401.0, 300.0, 200.0, 0.1, -0.02, 0.001, -0.002, 0.003, -0.004, 0.0005, -0.0006]),
    ("RAD_TAN_THIN_PRISM_FISHEYE", "RAD_TAN_THIN_PRISM_FISHEYE",
     [400.0, 401.0, 300.0, 200.0, 0.1, -0.02, 0.03, -0.004, 0.005, -0.006, 0.001, -0.002,
      0.0003, -0.0004, 0.0005, -0.0006]),
]


@pytest.mark.parametrize("binary", [False, True])
@pytest.mark.parametrize("colmap_model,pytheia_model,params", _EXTENDED_COLMAP_CASES,
                         ids=[c[0] for c in _EXTENDED_COLMAP_CASES])
def test_colmap_extended_models_keep_all_terms(tmp_path, colmap_model, pytheia_model, params, binary):
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        recon = _single_view_model(tmp_path / colmap_model, colmap_model, params, binary)
    cam = recon.View(recon.ViewIds()[0]).Camera()
    assert cam.GetCameraIntrinsicsModelType() == getattr(pt.sfm.CameraIntrinsicsModelType, pytheia_model)
    # COLMAP and pyTheia must project a set of points identically: a dropped term
    # would change the projection of points away from the principal point.
    # SIMPLE_RADIAL_FISHEYE is RADIAL_FISHEYE with k2 = 0.
    if colmap_model == "SIMPLE_RADIAL_FISHEYE":
        reference_model, reference_params = "RADIAL_FISHEYE", list(params) + [0.0]
    else:
        reference_model, reference_params = colmap_model, params
    for X in ([0.3, -0.2, 2.0], [-0.9, 0.7, 1.5], [0.1, 0.05, 4.0]):
        _, pixel = cam.ProjectPoint(np.array(X))
        expected = _colmap_reference_project(reference_model, reference_params, X)
        np.testing.assert_allclose(pixel, expected, atol=1e-6)


def test_colmap_full_opencv_warns_nothing(tmp_path):
    params = [400.0, 400.0, 300.0, 200.0, 0.03, -0.004, 0.001, -0.002, 0.0005, 0.1, 0.0, 0.0]
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        recon = _single_view_model(tmp_path / "f", "FULL_OPENCV", params, binary=False)
    model = recon.View(recon.ViewIds()[0]).Camera().CameraIntrinsics()
    assert model.GetParameter(8) == pytest.approx(0.0005)  # k3 (r^6 term)
    assert model.GetParameter(9) == pytest.approx(0.1)  # k4 kept


def test_colmap_unknown_model_raises(tmp_path):
    with pytest.raises(ValueError, match="not supported"):
        _single_view_model(tmp_path / "bogus", "BOGUS_MODEL", [400.0, 300.0, 200.0], binary=False)


def test_colmap_auto_detects_nested_sparse_folder(tmp_path):
    recon = _make_recon()
    sparse = tmp_path / "scene" / "sparse" / "0"
    sparse.mkdir(parents=True)
    assert pt.io.WriteColmapFiles(recon, str(sparse), binary=True)
    # Pass the scene root, the sparse folder, and a folder above both.
    for path in (tmp_path / "scene", sparse, tmp_path):
        read = read_colmap_model(str(path))
        assert read.NumViews() == 3


def test_colmap_missing_model_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        read_colmap_model(str(tmp_path))
    with pytest.raises(FileNotFoundError):
        read_colmap_model(str(tmp_path / "does_not_exist"))


def test_colmap_text_handles_empty_points_line_and_spaces_in_names(tmp_path):
    # An image without 2D points has an empty second line; names may contain spaces.
    d = tmp_path / "sparse"
    _write_colmap_text(
        d,
        [(1, "PINHOLE", 640, 480, [500.0, 500.0, 320.0, 240.0])],
        [
            (1, IDENTITY_Q, ZERO_T, 1, "my photo.jpg", []),
            (2, IDENTITY_Q, [0.0, 0.0, 1.0], 1, "b.jpg", [(320.5, 240.5, 0)]),
        ],
        [(0, [0.0, 0.0, 5.0], [9, 8, 7], 0.5, [(2, 0)])],
    )
    recon = read_colmap_model(str(d))
    assert recon.NumViews() == 2
    assert recon.ViewIdFromName("my photo.jpg") != pt.sfm.kInvalidViewId
    tid = recon.TrackIds()[0]
    assert list(recon.Track(tid).Color()) == [9, 8, 7]
    vid = recon.ViewIdFromName("b.jpg")
    np.testing.assert_allclose(recon.View(vid).GetFeature(tid).point, [320.5, 240.5])
    # Every image in images.txt is a registered view, so both are estimated.
    assert recon.View(recon.ViewIdFromName("my photo.jpg")).IsEstimated()


def test_colmap_binary_trailing_bytes_raise(tmp_path):
    recon = _make_recon()
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=True)
    with open(tmp_path / "points3D.bin", "ab") as f:
        f.write(b"\x00")
    with pytest.raises(ValueError, match="trailing"):
        read_colmap_model(str(tmp_path))


# --------------------------------------------------------------------------- #
# OpenMVG sfm_data.json
# --------------------------------------------------------------------------- #

OPENMVG_K3 = (0.02, -0.003, 0.0004)


def _openmvg_json(extra_structure=True):
    R0 = _rot_y(0.1)
    C0 = np.array([0.5, 0.05, -0.2])
    R1 = _rot_x(-0.05)
    C1 = np.array([-0.4, 0.1, 0.1])
    # View 2 has no pose; it is kept but not estimated.

    intrinsics = [
        {
            "key": 0,
            "value": {
                "polymorphic_id": 2147483649,
                "polymorphic_name": "pinhole",
                "ptr_wrapper": {
                    "id": 2147483660,
                    "data": {"width": WIDTH, "height": HEIGHT, "focal_length": FOCAL,
                             "principal_point": [CX, CY]},
                },
            },
        },
        {
            "key": 1,
            "value": {
                "polymorphic_id": 2147483651,
                "polymorphic_name": "pinhole_radial_k3",
                "ptr_wrapper": {
                    "id": 2147483661,
                    "data": {"width": WIDTH, "height": HEIGHT, "focal_length": FOCAL,
                             "principal_point": [CX, CY], "disto_k3": list(OPENMVG_K3)},
                },
            },
        },
        # Same pointer as intrinsics 1, written as a reference only.
        {"key": 2, "value": {"polymorphic_id": 2147483651, "ptr_wrapper": {"id": 2147483661}}},
    ]
    views = [
        {
            "key": 0,
            "value": {
                "polymorphic_id": 1073741824,
                "ptr_wrapper": {
                    "id": 2147483649,
                    "data": {"local_path": "", "filename": "img_0.png", "width": WIDTH,
                             "height": HEIGHT, "id_view": 0, "id_intrinsic": 0, "id_pose": 0},
                },
            },
        },
        {
            "key": 1,
            "value": {
                "polymorphic_id": 1073741824,
                "ptr_wrapper": {
                    "id": 2147483650,
                    "data": {"local_path": "sub", "filename": "img_1.png", "width": WIDTH,
                             "height": HEIGHT, "id_view": 1, "id_intrinsic": 1, "id_pose": 1},
                },
            },
        },
        {
            "key": 2,
            "value": {
                "polymorphic_id": 1073741824,
                "ptr_wrapper": {
                    "id": 2147483651,
                    "data": {"local_path": "", "filename": "img_2.png", "width": WIDTH,
                             "height": HEIGHT, "id_view": 2, "id_intrinsic": 2, "id_pose": 2},
                },
            },
        },
    ]
    extrinsics = [
        {"key": 0, "value": {"rotation": R0.tolist(), "center": C0.tolist()}},
        {"key": 1, "value": {"rotation": R1.tolist(), "center": C1.tolist()}},
    ]
    rng = np.random.default_rng(7)
    structure = []
    points = []
    for j in range(12):
        X = np.array([rng.uniform(-1, 1), rng.uniform(-0.8, 0.8), rng.uniform(3, 6)])
        points.append(X)
        observations = []
        p0 = _project_openmvg(R0, C0, X, FOCAL, CX, CY)
        p1 = _project_openmvg(R1, C1, X, FOCAL, CX, CY, OPENMVG_K3)
        observations.append({"key": 0, "value": {"id_feat": j, "x": p0.tolist()}})
        observations.append({"key": 1, "value": {"id_feat": j, "x": p1.tolist()}})
        if j == 0:
            # Observation of a view that is not in the file: dropped with a warning.
            observations.append({"key": 9, "value": {"id_feat": 0, "x": [1.0, 2.0]}})
        value = {"X": X.tolist(), "observations": observations}
        if j == 1:
            value["color"] = [12, 200, 33]
        structure.append({"key": j, "value": value})
    data = {
        "sfm_data_version": "0.3",
        "root_path": "/nonexistent/images",
        "views": views,
        "intrinsics": intrinsics,
        "extrinsics": extrinsics,
        "structure": structure if extra_structure else [],
        "control_points": [],
    }
    return data, points, (R0, C0), (R1, C1)


def test_openmvg_sfm_data_reads_views_poses_and_structure(tmp_path):
    data, points, (R0, C0), (R1, C1) = _openmvg_json()
    path = tmp_path / "sfm_data.json"
    path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.warns(UserWarning, match="unknown views"):
        recon = read_openmvg_sfm_data(str(path))
    assert isinstance(recon, pt.sfm.Reconstruction)
    assert recon.NumViews() == 3
    assert recon.NumTracks() == len(points)

    names = sorted(recon.View(v).Name() for v in recon.ViewIds())
    assert names == sorted(["img_0.png", "sub/img_1.png", "img_2.png"])

    v0 = recon.ViewIdFromName("img_0.png")
    v1 = recon.ViewIdFromName("sub/img_1.png")
    v2 = recon.ViewIdFromName("img_2.png")
    assert recon.View(v0).IsEstimated() and recon.View(v1).IsEstimated()
    assert not recon.View(v2).IsEstimated()
    np.testing.assert_allclose(recon.View(v0).Camera().GetPosition(), C0, atol=1e-12)
    np.testing.assert_allclose(recon.View(v1).Camera().GetPosition(), C1, atol=1e-12)
    np.testing.assert_allclose(recon.View(v1).Camera().GetOrientationAsRotationMatrix(), R1, atol=1e-12)

    # Intrinsics groups follow the OpenMVG intrinsic keys.
    assert recon.CameraIntrinsicsGroupIdFromViewId(v0) == 0
    assert recon.CameraIntrinsicsGroupIdFromViewId(v1) == 1
    assert recon.CameraIntrinsicsGroupIdFromViewId(v2) == 2
    cam0 = recon.View(v0).Camera()
    assert cam0.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE
    assert cam0.FocalLength() == pytest.approx(FOCAL)
    assert cam0.PrincipalPointX() == pytest.approx(CX)  # no pixel offset for OpenMVG
    cam1 = recon.View(v1).Camera()
    assert cam1.GetCameraIntrinsicsModelType() == pt.sfm.CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL
    assert cam1.CameraIntrinsics().GetParameter(7) == pytest.approx(OPENMVG_K3[2])
    # The reference-only intrinsics (key 2) resolved to the k3 data.
    assert recon.View(v2).Camera().CameraIntrinsics().GetParameter(7) == pytest.approx(OPENMVG_K3[2])

    # Observations reproject exactly with the read cameras (same convention).
    checked = 0
    for tid in recon.TrackIds():
        track = recon.Track(tid)
        for vid in track.ViewIds():
            if not recon.View(vid).IsEstimated():
                continue
            _, projected = recon.View(vid).Camera().ProjectPoint(track.Point()[:3])
            np.testing.assert_allclose(
                projected, recon.View(vid).GetFeature(tid).point, atol=1e-6
            )
            checked += 1
    assert checked == 2 * len(points)
    # Track ids follow the structure order (0, 1, ...).
    colored = recon.Track(1)
    assert list(colored.Color()) == [12, 200, 33]
    np.testing.assert_allclose(colored.Point()[:3], points[1], atol=1e-12)


def test_openmvg_fisheye_and_unknown_distortion_raise(tmp_path):
    data, _, _, _ = _openmvg_json()
    data["intrinsics"][0]["value"]["polymorphic_name"] = "fisheye"
    path = tmp_path / "fisheye.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="not supported"):
        read_openmvg_sfm_data(str(path))

    data, _, _, _ = _openmvg_json()
    data["intrinsics"][0]["value"]["ptr_wrapper"]["data"]["disto_k9"] = [0.1]
    path = tmp_path / "disto.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported distortion"):
        read_openmvg_sfm_data(str(path))


def test_openmvg_export_without_structure(tmp_path):
    # The openmvg_export stage writes intrinsics and views only.
    data, _, _, _ = _openmvg_json(extra_structure=False)
    path = tmp_path / "sfm_data.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    recon = read_openmvg_sfm_data(str(path))
    assert recon.NumViews() == 3
    assert recon.NumTracks() == 0


# --------------------------------------------------------------------------- #
# COLMAP database.db
# --------------------------------------------------------------------------- #


def _pair_id(id1, id2):
    return 2147483647 * id1 + id2


def _create_database(path):
    """Writes a database with the COLMAP table layout used by the reader.

    Keypoints are in COLMAP pixel coordinates (centre of the top-left pixel at
    0.5), so the principal point is shifted by 0.5 as well. The stored F is then
    exact for those keypoints. Returns (F, E, t, R).
    """
    rng = np.random.default_rng(11)
    Kc = np.array([[FOCAL, 0.0, CX + 0.5], [0.0, FOCAL, CY + 0.5], [0.0, 0.0, 1.0]])
    R = _rot_y(0.12)
    t = np.array([0.4, -0.05, 0.1])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R
    F = np.linalg.inv(Kc).T @ E @ np.linalg.inv(Kc)

    n = 25
    X = np.column_stack([rng.uniform(-1, 1, n), rng.uniform(-0.8, 0.8, n), rng.uniform(3, 6, n)])

    def _project(K, P):
        p = (K @ P.T).T
        return p[:, :2] / p[:, 2:3]

    x1 = _project(Kc, X)
    x2 = _project(Kc, (R @ X.T).T + t)
    # 6 columns: x, y, and the affine shape a11, a12, a21, a22.
    keypoints_1 = np.column_stack([x1, np.tile([1.0, 0.0, 0.0, 1.0], (n, 1))]).astype(np.float32)
    # 4 columns: x, y, scale, orientation.
    keypoints_2 = np.column_stack([x2, np.full(n, 2.5), np.full(n, 0.3)]).astype(np.float32)
    keypoints_3 = rng.uniform(0, 600, (7, 2)).astype(np.float32)  # 2 columns

    matches_12 = np.column_stack([np.arange(n), np.arange(n)]).astype(np.uint32)
    matches_23 = np.array([[0, 1], [2, 3], [4, 5], [6, 0], [1, 6]], dtype=np.uint32)

    con = sqlite3.connect(str(path))
    con.executescript(
        """
        CREATE TABLE images (image_id INTEGER PRIMARY KEY NOT NULL, name TEXT NOT NULL UNIQUE,
                             camera_id INTEGER NOT NULL);
        CREATE TABLE keypoints (image_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL,
                                cols INTEGER NOT NULL, data BLOB);
        CREATE TABLE matches (pair_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL,
                              cols INTEGER NOT NULL, data BLOB);
        CREATE TABLE two_view_geometries (pair_id INTEGER PRIMARY KEY NOT NULL, rows INTEGER NOT NULL,
                                          cols INTEGER NOT NULL, data BLOB, config INTEGER NOT NULL,
                                          F BLOB, E BLOB, H BLOB, qvec BLOB, tvec BLOB);
        """
    )
    con.executemany("INSERT INTO images VALUES (?, ?, ?)", [(1, "a.jpg", 1), (2, "b.jpg", 1), (3, "c.jpg", 1)])
    for image_id, kp in ((1, keypoints_1), (2, keypoints_2), (3, keypoints_3)):
        con.execute(
            "INSERT INTO keypoints VALUES (?, ?, ?, ?)",
            (image_id, kp.shape[0], kp.shape[1], kp.tobytes()),
        )
    # Pair (1, 2): calibrated two-view geometry with matrices (row-major, as COLMAP writes them).
    H = rng.normal(size=(3, 3))
    qvec = np.array([np.cos(0.06), 0.0, np.sin(0.06), 0.0])
    con.execute(
        "INSERT INTO matches VALUES (?, ?, ?, ?)",
        (_pair_id(1, 2), matches_12.shape[0], 2, matches_12.tobytes()),
    )
    con.execute(
        "INSERT INTO two_view_geometries VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (
            _pair_id(1, 2), matches_12.shape[0], 2, matches_12.tobytes(), 2,
            F.astype("<f8").tobytes(), E.astype("<f8").tobytes(), H.astype("<f8").tobytes(),
            qvec.astype("<f8").tobytes(), t.astype("<f8").tobytes(),
        ),
    )
    # Pair (2, 3): raw matches only, degenerate configuration with no matrices.
    con.execute(
        "INSERT INTO matches VALUES (?, ?, ?, ?)",
        (_pair_id(2, 3), matches_23.shape[0], 2, matches_23.tobytes()),
    )
    con.execute(
        "INSERT INTO two_view_geometries VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (_pair_id(2, 3), 0, 0, None, 1, None, None, None, None, None),
    )
    con.commit()
    con.close()
    return F, E, t, R


def test_colmap_database_reader(tmp_path):
    db = tmp_path / "database.db"
    F, E, t, R = _create_database(db)
    data = read_colmap_database(str(db))

    assert data["images"] == {1: "a.jpg", 2: "b.jpg", 3: "c.jpg"}
    assert data["keypoints"][1].shape == (25, 6) and data["keypoints"][1].dtype == np.float32
    assert data["keypoints"][2].shape == (25, 4)
    assert data["keypoints"][3].shape == (7, 2)

    assert set(data["matches"]) == {(1, 2), (2, 3)}
    assert data["matches"][(1, 2)].shape == (25, 2)
    assert data["matches"][(1, 2)].dtype == np.uint32
    np.testing.assert_array_equal(data["matches"][(2, 3)], [[0, 1], [2, 3], [4, 5], [6, 0], [1, 6]])

    tvg = data["two_view_geometries"]
    calibrated = tvg[(1, 2)]
    assert calibrated["config"] == 2 and calibrated["config_name"] == "CALIBRATED"
    np.testing.assert_allclose(calibrated["F"], F, atol=1e-12)
    np.testing.assert_allclose(calibrated["E"], E, atol=1e-12)
    assert calibrated["H"].shape == (3, 3)
    np.testing.assert_allclose(calibrated["tvec"], t, atol=1e-12)
    assert calibrated["qvec"].shape == (4,)
    assert calibrated["inlier_matches"].shape == (25, 2)

    empty = tvg[(2, 3)]
    assert empty["config_name"] == "DEGENERATE"
    assert empty["F"] is None and empty["E"] is None and empty["H"] is None
    assert empty["qvec"] is None and empty["tvec"] is None
    assert empty["inlier_matches"].shape == (0, 2)


def test_colmap_database_epipolar_constraint(tmp_path):
    # Independent check of the matrix layout: the stored F must satisfy x2^T F x1 = 0
    # for the true correspondences (row-major storage, as COLMAP writes it).
    db = tmp_path / "database.db"
    _create_database(db)
    data = read_colmap_database(str(db))
    F = data["two_view_geometries"][(1, 2)]["F"]
    m = data["two_view_geometries"][(1, 2)]["inlier_matches"]
    kp1 = data["keypoints"][1][m[:, 0], :2].astype(float)
    kp2 = data["keypoints"][2][m[:, 1], :2].astype(float)
    x1 = np.column_stack([kp1, np.ones(len(kp1))])
    x2 = np.column_stack([kp2, np.ones(len(kp2))])

    def _epipolar_px(F_matrix):
        # Distance (px) of x2 to the epipolar line F x1 in image 2.
        line = x1 @ F_matrix.T
        return np.abs(np.sum(x2 * line, axis=1)) / np.linalg.norm(line[:, :2], axis=1)

    assert np.max(_epipolar_px(F)) < 1e-3
    # The transpose must not satisfy it.
    assert np.median(_epipolar_px(F.T)) > 1.0


def test_colmap_database_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        read_colmap_database(str(tmp_path / "missing.db"))


# --------------------------------------------------------------------------- #
# Real-data smoke tests (opt-in through environment variables)
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(not os.environ.get("PYTHEIA_COLMAP_SPARSE"), reason="PYTHEIA_COLMAP_SPARSE not set")
def test_real_colmap_sparse_model():
    path = os.environ["PYTHEIA_COLMAP_SPARSE"]
    recon = read_colmap_model(path)
    assert recon.NumViews() > 0
    assert recon.NumTracks() > 0
    for vid in recon.ViewIds()[:3]:
        assert recon.View(vid).IsEstimated()
    # Reprojection residuals of the read model stay near the pixel scale.
    residuals = []
    for tid in recon.TrackIds()[:200]:
        track = recon.Track(tid)
        for vid in track.ViewIds():
            _, projected = recon.View(vid).Camera().ProjectPoint(track.Point()[:3])
            residuals.append(np.linalg.norm(np.asarray(projected) - np.asarray(recon.View(vid).GetFeature(tid).point)))
    assert np.median(residuals) < 5.0


@pytest.mark.skipif(not os.environ.get("PYTHEIA_COLMAP_DB"), reason="PYTHEIA_COLMAP_DB not set")
def test_real_colmap_database():
    data = read_colmap_database(os.environ["PYTHEIA_COLMAP_DB"])
    assert data["images"] and data["keypoints"]
    for (id1, id2), entry in data["two_view_geometries"].items():
        if entry["F"] is not None and len(entry["inlier_matches"]):
            kp1 = data["keypoints"][id1][entry["inlier_matches"][:, 0], :2].astype(float)
            kp2 = data["keypoints"][id2][entry["inlier_matches"][:, 1], :2].astype(float)
            x1 = np.column_stack([kp1, np.ones(len(kp1))])
            x2 = np.column_stack([kp2, np.ones(len(kp2))])
            line = x1 @ entry["F"].T
            dist = np.abs(np.sum(x2 * line, axis=1)) / np.linalg.norm(line[:, :2], axis=1)
            assert np.median(dist) < 3.0
            break


@pytest.mark.skipif(not os.environ.get("PYTHEIA_OPENMVG_JSON"), reason="PYTHEIA_OPENMVG_JSON not set")
def test_real_openmvg_sfm_data():
    recon = read_openmvg_sfm_data(os.environ["PYTHEIA_OPENMVG_JSON"])
    assert recon.NumViews() > 0
    assert recon.NumCameraIntrinsicGroups() > 0


def test_pixel_offset_shifts_observations_and_principal_point(tmp_path):
    recon = _make_recon()
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=True)
    a = read_colmap_model(str(tmp_path))
    b = read_colmap_model(str(tmp_path), pixel_offset=0.5)
    for tid in a.TrackIds():
        for vid in a.Track(tid).ViewIds():
            pa = np.asarray(a.View(vid).GetFeature(tid).point)
            pb = np.asarray(b.View(vid).GetFeature(tid).point)
            np.testing.assert_allclose(pb, pa - 0.5, atol=1e-9)
    vid = a.ViewIds()[0]
    ca, cb = a.View(vid).Camera(), b.View(vid).Camera()
    assert cb.PrincipalPointX() == pytest.approx(ca.PrincipalPointX() - 0.5)
    assert cb.PrincipalPointY() == pytest.approx(ca.PrincipalPointY() - 0.5)
