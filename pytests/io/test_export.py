"""Tests for the COLMAP binary writer, transforms.json, and the splatting export.

The reconstruction is generated in the test so that every projected feature is
known exactly. The checks parse the files written by pyTheia with ``struct``
and compare them with the Reconstruction and with the camera model.
"""

import json
import math
import os
import struct

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")

WIDTH = 640
HEIGHT = 480
FOCAL = 500.0
CX = 320.0
CY = 240.0
VIEW_NAMES = ["img_0.png", "sub/img_1.png", "img_2.png"]


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def _rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _quat_to_rot(q):
    """COLMAP quaternion (w, x, y, z) to a world-to-camera rotation matrix."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _make_recon(k1=0.0, k2=0.0, n_points=40, seed=0, add_bad_tracks=False):
    """Builds a reconstruction with three views and projected tracks.

    Returns (reconstruction, view_ids, cameras) where ``cameras`` maps view id
    to the pyTheia camera of the view (with the requested distortion).
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
        # Timestamps must be unique within a reconstruction.
        vid = recon.AddView(name, group, float(idx))
        cam = recon.MutableView(vid).MutableCamera()
        cam.SetImageSize(WIDTH, HEIGHT)
        cam.SetFocalLength(FOCAL)
        cam.SetPrincipalPoint(CX, CY)
        cam.CameraIntrinsics().SetRadialDistortion(k1, k2)
        cam.SetOrientationFromRotationMatrix(rotation)
        cam.SetPosition(center)
        recon.MutableView(vid).SetIsEstimated(True)
        view_ids.append(vid)

    cameras = {vid: recon.View(vid).Camera() for vid in view_ids}

    for _ in range(n_points):
        X = np.array(
            [
                rng.uniform(-1.0, 1.0),
                rng.uniform(-0.8, 0.8),
                rng.uniform(3.0, 6.0),
            ]
        )
        features = []
        for vid in view_ids:
            depth, pixel = cameras[vid].ProjectPoint(X)
            in_image = 0 <= pixel[0] < WIDTH and 0 <= pixel[1] < HEIGHT
            if depth > 0 and in_image:
                features.append((vid, pt.sfm.Feature(np.asarray(pixel, dtype=float))))
        if len(features) < 2:
            continue
        tid = recon.AddTrack(features)
        recon.MutableTrack(tid).SetPoint(np.append(X, 1.0))
        recon.MutableTrack(tid).SetIsEstimated(True)
        recon.MutableTrack(tid).SetColor(
            np.array(rng.integers(0, 256, size=3), dtype=np.uint8)
        )

    if add_bad_tracks:
        # Two-view track with a 40 px error on the second observation.
        X = np.array([0.0, 0.0, 4.0])
        _, p0 = cameras[view_ids[0]].ProjectPoint(X)
        _, p1 = cameras[view_ids[1]].ProjectPoint(X)
        tid = recon.AddTrack(
            [
                (view_ids[0], pt.sfm.Feature(np.asarray(p0, dtype=float))),
                (view_ids[1], pt.sfm.Feature(np.asarray(p1 + [40.0, 0.0], dtype=float))),
            ]
        )
        recon.MutableTrack(tid).SetPoint(np.append(X, 1.0))
        recon.MutableTrack(tid).SetIsEstimated(True)

    return recon, view_ids, cameras


def _read_cameras_bin(path):
    cameras = {}
    with open(path, "rb") as f:
        (num,) = struct.unpack("<Q", f.read(8))
        for _ in range(num):
            camera_id, model_id, width, height = struct.unpack("<IiQQ", f.read(24))
            num_params = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8}[model_id]
            params = struct.unpack("<%dd" % num_params, f.read(8 * num_params))
            cameras[camera_id] = dict(
                model_id=model_id, width=width, height=height, params=params
            )
    return cameras


def _read_images_bin(path):
    images = []
    with open(path, "rb") as f:
        (num,) = struct.unpack("<Q", f.read(8))
        for _ in range(num):
            (image_id,) = struct.unpack("<I", f.read(4))
            qvec = struct.unpack("<4d", f.read(32))
            tvec = struct.unpack("<3d", f.read(24))
            (camera_id,) = struct.unpack("<I", f.read(4))
            name = b""
            while True:
                c = f.read(1)
                assert c != b"", "unterminated image name"
                if c == b"\x00":
                    break
                name += c
            (num_points2d,) = struct.unpack("<Q", f.read(8))
            points2d = []
            for _ in range(num_points2d):
                x, y, point3d_id = struct.unpack("<ddq", f.read(24))
                points2d.append((x, y, point3d_id))
            images.append(
                dict(
                    image_id=image_id,
                    qvec=np.array(qvec),
                    tvec=np.array(tvec),
                    camera_id=camera_id,
                    name=name.decode("utf-8"),
                    points2d=points2d,
                )
            )
        assert f.read() == b"", "trailing bytes in images.bin"
    return images


def _read_points_bin(path):
    points = {}
    with open(path, "rb") as f:
        (num,) = struct.unpack("<Q", f.read(8))
        for _ in range(num):
            (point_id,) = struct.unpack("<Q", f.read(8))
            xyz = np.array(struct.unpack("<3d", f.read(24)))
            rgb = struct.unpack("<3B", f.read(3))
            (error,) = struct.unpack("<d", f.read(8))
            (track_length,) = struct.unpack("<Q", f.read(8))
            track = [struct.unpack("<II", f.read(8)) for _ in range(track_length)]
            points[point_id] = dict(xyz=xyz, rgb=rgb, error=error, track=track)
        assert f.read() == b"", "trailing bytes in points3D.bin"
    return points


def _read_text_model(directory):
    """Parses the COLMAP text files into the same shape as the binary readers."""
    cameras = {}
    with open(os.path.join(directory, "cameras.txt"), encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.split()
            cameras[int(parts[0])] = dict(
                width=int(parts[2]),
                height=int(parts[3]),
                params=tuple(float(v) for v in parts[4:]),
            )
    images = {}
    with open(os.path.join(directory, "images.txt"), encoding="utf-8") as f:
        lines = [line for line in f if not line.startswith("#")]
    for header in lines[0::2]:
        parts = header.split()
        images[parts[9]] = dict(
            qvec=np.array([float(v) for v in parts[1:5]]),
            tvec=np.array([float(v) for v in parts[5:8]]),
            camera_id=int(parts[8]),
        )
    return cameras, images


# --------------------------------------------------------------------------- #
# COLMAP binary writer
# --------------------------------------------------------------------------- #


def test_colmap_binary_roundtrip_counts_names_and_pose(tmp_path):
    recon, view_ids, _ = _make_recon()
    out = str(tmp_path)
    assert pt.io.WriteColmapFiles(recon, out, binary=True)
    for name in ("cameras.bin", "images.bin", "points3D.bin"):
        assert os.path.getsize(os.path.join(out, name)) > 0
    assert not os.path.exists(os.path.join(out, "cameras.txt"))

    cameras = _read_cameras_bin(os.path.join(out, "cameras.bin"))
    images = _read_images_bin(os.path.join(out, "images.bin"))
    points = _read_points_bin(os.path.join(out, "points3D.bin"))

    assert len(cameras) == recon.NumCameraIntrinsicGroups()
    assert len(images) == len(view_ids)
    assert len(points) == recon.NumTracks()
    assert sorted(img["name"] for img in images) == sorted(VIEW_NAMES)

    # Camera 0 has no distortion and square pixels: SIMPLE_PINHOLE (f, cx, cy).
    cam0 = cameras[0]
    assert cam0["model_id"] == 0
    assert cam0["width"] == WIDTH and cam0["height"] == HEIGHT
    assert cam0["params"] == pytest.approx((FOCAL, CX, CY))

    for img in images:
        vid = recon.ViewIdFromName(img["name"])
        cam = recon.View(vid).Camera()
        R = cam.GetOrientationAsRotationMatrix()
        C = cam.GetPosition()
        np.testing.assert_allclose(_quat_to_rot(img["qvec"]), R, atol=1e-9)
        np.testing.assert_allclose(img["tvec"], -R @ C, atol=1e-9)
        assert img["camera_id"] == recon.CameraIntrinsicsGroupIdFromViewId(vid)


def test_colmap_binary_projected_point_matches_camera(tmp_path):
    recon, view_ids, cameras = _make_recon()
    out = str(tmp_path)
    assert pt.io.WriteColmapFiles(recon, out, binary=True)
    images = {img["name"]: img for img in _read_images_bin(os.path.join(out, "images.bin"))}
    points = _read_points_bin(os.path.join(out, "points3D.bin"))

    # Pick one track and check its first observation.
    point_id, point = next(iter(points.items()))
    image_id, point2d_idx = point["track"][0]
    obs_image = next(
        img for img in images.values() if recon.ViewIdFromName(img["name"]) == image_id
    )
    x, y, p3d_id = obs_image["points2d"][point2d_idx]
    assert p3d_id == point_id

    # The stored 3D point projects onto the stored observation.
    vid = recon.ViewIdFromName(obs_image["name"])
    depth, pixel = recon.View(vid).Camera().ProjectPoint(point["xyz"])
    assert depth > 0
    np.testing.assert_allclose(pixel, [x, y], atol=1e-6)

    # Stored reprojection error is the mean over observations.
    errors = []
    for img_id, idx in point["track"]:
        name = recon.View(img_id).Name()
        ox, oy, _ = images[name]["points2d"][idx]
        _, px = recon.View(img_id).Camera().ProjectPoint(point["xyz"])
        errors.append(math.hypot(px[0] - ox, px[1] - oy))
    assert point["error"] == pytest.approx(np.mean(errors), abs=1e-6)
    assert point["rgb"] == tuple(int(c) for c in recon.Track(point_id).Color())


def test_colmap_binary_text_roundtrip_agrees(tmp_path):
    recon, _, _ = _make_recon()
    bin_dir = tmp_path / "bin"
    txt_dir = tmp_path / "txt"
    bin_dir.mkdir()
    txt_dir.mkdir()
    assert pt.io.WriteColmapFiles(recon, str(bin_dir), binary=True)
    assert pt.io.WriteColmapFiles(recon, str(txt_dir))

    bin_cams = _read_cameras_bin(str(bin_dir / "cameras.bin"))
    bin_images = _read_images_bin(str(bin_dir / "images.bin"))
    txt_cams, txt_images = _read_text_model(str(txt_dir))

    assert set(bin_cams) == set(txt_cams)
    for cid in bin_cams:
        assert bin_cams[cid]["width"] == txt_cams[cid]["width"]
        assert bin_cams[cid]["height"] == txt_cams[cid]["height"]
    for img in bin_images:
        txt = txt_images[img["name"]]
        assert txt["camera_id"] == img["camera_id"]
        np.testing.assert_allclose(txt["tvec"], img["tvec"], atol=1e-6)
        # Text qvec is not sign-normalised; compare rotations instead.
        np.testing.assert_allclose(
            _quat_to_rot(txt["qvec"]), _quat_to_rot(img["qvec"]), atol=1e-6
        )


def test_colmap_binary_distorted_camera_uses_opencv_model(tmp_path):
    recon, _, _ = _make_recon(k1=-0.05, k2=0.01)
    assert pt.io.WriteColmapFiles(recon, str(tmp_path), binary=True)
    cameras = _read_cameras_bin(str(tmp_path / "cameras.bin"))
    cam0 = cameras[0]
    assert cam0["model_id"] == 4  # OPENCV: fx, fy, cx, cy, k1, k2, p1, p2
    assert cam0["params"][0:4] == pytest.approx((FOCAL, FOCAL, CX, CY))
    assert cam0["params"][4:6] == pytest.approx((-0.05, 0.01))
    assert cam0["params"][6:8] == pytest.approx((0.0, 0.0))


def test_colmap_text_default_signature_still_works(tmp_path):
    recon, _, _ = _make_recon()
    assert pt.io.WriteColmapFiles(recon, str(tmp_path))
    assert os.path.exists(tmp_path / "cameras.txt")
    assert os.path.exists(tmp_path / "images.txt")
    assert os.path.exists(tmp_path / "points3D.txt")


# --------------------------------------------------------------------------- #
# transforms.json convention
# --------------------------------------------------------------------------- #


def test_transforms_json_convention(tmp_path):
    k1, k2 = -0.05, 0.01
    recon, view_ids, cameras = _make_recon(k1=k1, k2=k2)
    path = str(tmp_path / "transforms.json")
    assert pt.io.WriteTransformsJson("images", recon, path)

    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    # Two intrinsics groups: intrinsics are stored per frame.
    assert "camera_model" not in data
    frames = {fr["file_path"]: fr for fr in data["frames"]}
    assert set(frames) == {"images/" + name for name in VIEW_NAMES}

    for vid in view_ids:
        name = recon.View(vid).Name()
        frame = frames["images/" + name]
        c2w = np.array(frame["transform_matrix"])
        assert c2w.shape == (4, 4)
        assert frame["fl_x"] == pytest.approx(FOCAL)
        assert frame["fl_y"] == pytest.approx(FOCAL)
        assert frame["cx"] == pytest.approx(CX)
        assert frame["cy"] == pytest.approx(CY)
        assert frame["w"] == WIDTH and frame["h"] == HEIGHT
        assert frame["k1"] == pytest.approx(k1)
        assert frame["k2"] == pytest.approx(k2)
        assert frame["camera_model"] == "OPENCV"

        # Camera center is the translation of the camera-to-world matrix.
        cam = recon.View(vid).Camera()
        np.testing.assert_allclose(c2w[:3, 3], cam.GetPosition(), atol=1e-9)

        # Known world points: project them with the OpenGL convention and
        # compare with pyTheia's projection.
        w2c_gl = np.linalg.inv(c2w)
        for X in [np.array([0.2, -0.1, 4.0]), np.array([-0.5, 0.3, 5.0])]:
            _, expected = cam.ProjectPoint(X)
            x_gl = w2c_gl @ np.append(X, 1.0)
            # OpenGL camera looks along -z, y up. OpenCV: x right, y down, z forward.
            x_cv = np.array([x_gl[0], -x_gl[1], -x_gl[2]])
            assert x_cv[2] > 0
            xn, yn = x_cv[0] / x_cv[2], x_cv[1] / x_cv[2]
            r2 = xn * xn + yn * yn
            d = 1.0 + k1 * r2 + k2 * r2 * r2
            u = FOCAL * xn * d + CX
            v = FOCAL * yn * d + CY
            np.testing.assert_allclose([u, v], expected, atol=1e-6)


def test_transforms_json_pinhole_without_distortion(tmp_path):
    recon, _, _ = _make_recon()
    path = str(tmp_path / "transforms.json")
    assert pt.io.WriteTransformsJson("images", recon, path)
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    for frame in data["frames"]:
        assert frame["camera_model"] == "PINHOLE"
        assert frame["k1"] == 0.0 and frame["k2"] == 0.0


# --------------------------------------------------------------------------- #
# Gaussian splatting export
# --------------------------------------------------------------------------- #


def _write_random_images(image_dir, seed=1):
    PIL = pytest.importorskip("PIL.Image")
    rng = np.random.default_rng(seed)
    for name in VIEW_NAMES:
        path = os.path.join(image_dir, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = rng.integers(0, 256, size=(HEIGHT, WIDTH, 3), dtype=np.uint8)
        PIL.fromarray(data).save(path)


def _quat_rot_and_center(img):
    R = _quat_to_rot(img["qvec"])
    return R, -R.T @ img["tvec"]


def test_undistortion_map_matches_pytheia_camera(tmp_path):
    pytest.importorskip("cv2")
    from pytheia.export import undistortion_maps

    k1, k2 = -0.08, 0.02
    recon, view_ids, cameras = _make_recon(k1=k1, k2=k2)
    cam = cameras[view_ids[0]]
    K = cam.GetCalibrationMatrix()
    map1, map2 = undistortion_maps(WIDTH, HEIGHT, K, k1, k2)
    assert map1.shape == (HEIGHT, WIDTH)

    rng = np.random.default_rng(3)
    for _ in range(200):
        u = int(rng.integers(0, WIDTH))
        v = int(rng.integers(0, HEIGHT))
        src = np.array([float(map1[v, u]), float(map2[v, u])])
        # The source pixel, undistorted by pyTheia, gives the normalized
        # coordinates of the output pixel with the same (pinhole) intrinsics.
        normalized = cam.PixelToNormalizedCoordinates(src)
        np.testing.assert_allclose(
            normalized[:2], [(u - CX) / FOCAL, (v - CY) / FOCAL], atol=2e-4
        )


def test_export_for_gaussian_splatting_colmap(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("PIL.Image")
    from pytheia.export import export_for_gaussian_splatting

    k1, k2 = -0.05, 0.01
    recon, view_ids, _ = _make_recon(k1=k1, k2=k2, add_bad_tracks=True)
    image_dir = tmp_path / "input"
    _write_random_images(str(image_dir))
    out = tmp_path / "scene_gs"

    result = export_for_gaussian_splatting(
        recon,
        str(out),
        str(image_dir),
        undistort=True,
        max_image_size=-1,
        format="colmap",
        max_reprojection_error=4.0,
        min_track_length=2,
    )
    assert result["format"] == "colmap"
    assert result["num_images"] == len(view_ids)

    # Images exist with the same names and size (no resize requested).
    cv2 = pytest.importorskip("cv2")
    for name in VIEW_NAMES:
        image = cv2.imread(str(out / "images" / name))
        assert image is not None
        assert image.shape[:2] == (HEIGHT, WIDTH)

    sparse = out / "sparse" / "0"
    cameras = _read_cameras_bin(str(sparse / "cameras.bin"))
    images = _read_images_bin(str(sparse / "images.bin"))
    points = _read_points_bin(str(sparse / "points3D.bin"))
    assert len(images) == len(view_ids)
    for cam in cameras.values():
        # Undistorted: no distortion term in the model.
        assert cam["model_id"] in (0, 1)
    assert len(points) == result["num_points"]

    # The bad track (40 px error on one observation) is dropped; all others stay.
    assert len(points) == recon.NumTracks() - 1

    # Observations are in the undistorted image. Projecting the stored point
    # with the stored pinhole camera reproduces the observation.
    cam_by_id = cameras
    max_err = 0.0
    for img in images:
        R, C = _quat_rot_and_center(img)
        cam = cam_by_id[img["camera_id"]]
        f, cx, cy = cam["params"][0], cam["params"][-2], cam["params"][-1]
        for x, y, p3d_id in img["points2d"]:
            if p3d_id < 0:
                continue
            X = points[p3d_id]["xyz"]
            xc = R @ X + img["tvec"]
            u = f * xc[0] / xc[2] + cx
            v = f * xc[1] / xc[2] + cy
            max_err = max(max_err, math.hypot(u - x, v - y))
    assert max_err < 0.05

    # Seed point cloud matches the exported point count.
    ply = out / "points3d.ply"
    with open(ply, "rb") as f:
        header = f.read(400).split(b"end_header\n")[0].decode("ascii")
    assert f"element vertex {len(points)}" in header


def test_export_resizes_and_scales_intrinsics(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("PIL.Image")
    from pytheia.export import export_for_gaussian_splatting

    recon, _, _ = _make_recon(k1=-0.05, k2=0.01)
    image_dir = tmp_path / "input"
    _write_random_images(str(image_dir))
    out = tmp_path / "scene_small"
    export_for_gaussian_splatting(
        recon, str(out), str(image_dir), max_image_size=320, format="colmap"
    )
    cv2 = pytest.importorskip("cv2")
    assert cv2.imread(str(out / "images" / "img_0.png")).shape[:2] == (240, 320)
    cameras = _read_cameras_bin(str(out / "sparse" / "0" / "cameras.bin"))
    for cam in cameras.values():
        assert (cam["width"], cam["height"]) == (320, 240)
        assert cam["params"][0] == pytest.approx(FOCAL * 0.5)


def test_export_transforms_format(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("PIL.Image")
    from pytheia.export import export_for_gaussian_splatting

    recon, _, _ = _make_recon(k1=-0.05, k2=0.01)
    image_dir = tmp_path / "input"
    _write_random_images(str(image_dir))
    out = tmp_path / "scene_tf"
    export_for_gaussian_splatting(
        recon, str(out), str(image_dir), format="transforms", undistort=True
    )
    assert not (out / "sparse").exists()
    with open(out / "transforms.json", encoding="utf-8") as f:
        data = json.load(f)
    assert all(fr["camera_model"] == "PINHOLE" for fr in data["frames"])  # undistorted
    assert len(data["frames"]) == len(VIEW_NAMES)


def test_export_rejects_unknown_format(tmp_path):
    from pytheia.export import export_for_gaussian_splatting

    recon, _, _ = _make_recon()
    with pytest.raises(ValueError):
        export_for_gaussian_splatting(recon, str(tmp_path), str(tmp_path), format="ply")


def test_write_points_ply_layout(tmp_path):
    from pytheia.export import write_points_ply

    path = tmp_path / "points3d.ply"
    write_points_ply(str(path), [(0.0, 1.0, 2.0), (3.0, 4.0, 5.0)], [(1, 2, 3), (4, 5, 6)])
    raw = path.read_bytes()
    header, body = raw.split(b"end_header\n", 1)
    assert b"element vertex 2" in header
    assert len(body) == 2 * struct.calcsize("<6f3B")
    x, y, z, nx, ny, nz, r, g, b = struct.unpack("<6f3B", body[:27])
    assert (x, y, z) == (0.0, 1.0, 2.0)
    assert (r, g, b) == (1, 2, 3)
