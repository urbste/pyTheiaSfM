"""Tests for the pure-Python browser viewer (pytheia.viewer).

The reconstruction is synthetic: cameras on a ring looking at the origin, points
in a box, exact projections (or noisy ones for the error test). Run with
``PYTHONPATH=src`` when the installed wheel does not ship ``pytheia.viewer``.
"""

import base64
import json
import re
import urllib.request
from pathlib import Path

import numpy as np
import pytest

import pytheia as pt
from pytheia import viewer
from pytheia.viewer.compare import DegenerateAlignment, align_scene, umeyama_sim3
from pytheia.viewer.scene import decode_array, recon_to_scene, scene_to_payload

ASSETS = Path(viewer.__file__).resolve().parent / "assets"
WIDTH, HEIGHT, FOCAL = 640, 480, 500.0
NUM_VIEWS = 6  # the last view is left unregistered
NUM_POINTS = 80


def _look_at_rotation(center, target=np.zeros(3)):
    """World-to-camera rotation (OpenCV axes) for a camera at ``center`` looking at ``target``."""
    z = target - center
    z = z / np.linalg.norm(z)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.vstack([x, y, z])


def random_rotation(rng):
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q * np.sign(np.diag(r))
    if np.linalg.det(q) < 0:
        q[:, 2] *= -1.0
    return q


def make_reconstruction(noise=0.0, seed=0, num_points=NUM_POINTS, transform=None, name_fmt="img_{}.jpg"):
    """Return (reconstruction, ground truth dict) for a small synthetic scene.

    ``transform=(scale, R, t)`` maps the whole scene (cameras and points) with
    ``X' = scale * R X + t``; the pixel observations do not change.
    """
    rng = np.random.default_rng(seed)
    rec = pt.sfm.Reconstruction()
    centers, rotations = [], []
    for i in range(NUM_VIEWS):
        angle = 2.0 * np.pi * i / NUM_VIEWS
        c = np.array([5.0 * np.cos(angle), 5.0 * np.sin(angle), 0.3 * i])
        centers.append(c)
        rotations.append(_look_at_rotation(c))
    points = rng.uniform(-1.0, 1.0, size=(num_points, 3))
    if transform is not None:
        scale, rs, ts = transform
        centers = [scale * rs @ c + ts for c in centers]
        # X' - C' = scale R (X - C): the world-to-camera rotation becomes R_w2c R^T
        rotations = [r @ rs.T for r in rotations]
        points = scale * points @ rs.T + ts
    vids = []
    for i in range(NUM_VIEWS):
        vid = rec.AddView(name_fmt.format(i), i)
        cam = rec.MutableView(vid).MutableCamera()
        cam.SetImageSize(WIDTH, HEIGHT)
        cam.SetFocalLength(FOCAL)
        cam.SetPrincipalPoint(WIDTH / 2.0, HEIGHT / 2.0)
        cam.SetPosition(centers[i])
        cam.SetOrientationFromRotationMatrix(rotations[i])
        rec.MutableView(vid).SetIsEstimated(i < NUM_VIEWS - 1)
        vids.append(vid)

    colors = rng.integers(0, 256, size=(num_points, 3), dtype=np.uint8)
    K = np.array([[FOCAL, 0, WIDTH / 2.0], [0, FOCAL, HEIGHT / 2.0], [0, 0, 1.0]])
    track_ids = []
    for p in range(num_points):
        track = []
        for i, vid in enumerate(vids):
            x_cam = rotations[i] @ (points[p] - centers[i])
            px = K @ (x_cam / x_cam[2])
            px = px[:2] + rng.normal(0.0, noise, size=2)
            track.append((vid, pt.sfm.Feature(float(px[0]), float(px[1]))))
        # keep 4 views per track so that the tracks overlap but are not identical
        track = track[: 4 + (p % 3)] if p % 2 else track
        tid = rec.AddTrack(track)
        mt = rec.MutableTrack(tid)
        mt.SetPoint(np.append(points[p], 1.0))
        mt.SetColor(colors[p])
        mt.SetIsEstimated(True)
        track_ids.append(tid)

    truth = {
        "points": points,
        "colors": colors,
        "centers": np.asarray(centers),
        "rotations": np.asarray(rotations),
        "K": K,
        "track_ids": track_ids,
        "view_ids": vids,
    }
    return rec, truth


@pytest.fixture(scope="module")
def recon_and_truth():
    return make_reconstruction()


@pytest.fixture(scope="module")
def scene(recon_and_truth):
    rec, _ = recon_and_truth
    return recon_to_scene(rec)


# ---------------------------------------------------------------------------
# scene data
# ---------------------------------------------------------------------------


def test_scene_shapes(scene):
    pts = scene["points"]
    n = NUM_POINTS
    assert pts["xyz"].shape == (n, 3) and pts["xyz"].dtype == np.float32
    assert pts["rgb"].shape == (n, 3) and pts["rgb"].dtype == np.uint8
    for key in ("track_length", "reprojection_error", "track_id"):
        assert pts[key].shape == (n,) and pts[key].dtype == np.float32
    assert pts["obs_offsets"].shape == (n + 1,)

    obs = scene["observations"]
    total = int(pts["obs_offsets"][-1])
    assert obs["view"].shape == (total,)
    for key in ("x", "y", "residual"):
        assert obs[key].shape == (total,)

    cams = scene["cameras"]
    v = NUM_VIEWS
    assert cams["center"].shape == (v, 3)
    assert cams["rotation_w2c"].shape == (v, 3, 3)
    assert cams["intrinsics"].shape == (v, 4)
    assert cams["image_size"].shape == (v, 2)
    assert cams["frustum"].shape == (v, 4, 3)
    assert cams["estimated"].shape == (v,) and cams["estimated"].dtype == np.uint8
    assert len(cams["names"]) == v
    assert cams["names"][0] == "img_0.jpg"
    assert int(cams["estimated"].sum()) == NUM_VIEWS - 1
    assert scene["edges"] is None

    stats = scene["stats"]
    assert stats["num_points"] == n
    assert stats["num_estimated_views"] == NUM_VIEWS - 1
    assert stats["num_observations"] == total


def test_points_match_reconstruction(scene, recon_and_truth):
    _, truth = recon_and_truth
    xyz = scene["points"]["xyz"].astype(np.float64)
    # tracks come back in id order, which is the order they were added in
    np.testing.assert_allclose(xyz, truth["points"], atol=1e-5)
    np.testing.assert_array_equal(scene["points"]["rgb"], truth["colors"])


def test_zero_noise_reprojection_error_is_zero(scene):
    err = scene["points"]["reprojection_error"]
    assert np.all(np.isfinite(err))
    assert np.max(err) < 1e-3
    assert np.max(scene["observations"]["residual"]) < 1e-3


def test_projection_convention(scene, recon_and_truth):
    """Stored poses + intrinsics reproduce the stored observations: x_c = R (X - C), u = K x_c / z."""
    _, truth = recon_and_truth
    cams = scene["cameras"]
    pts = scene["points"]
    obs = scene["observations"]
    offsets = pts["obs_offsets"].astype(int)
    checked = 0
    for p in range(NUM_POINTS):
        for k in range(offsets[p], offsets[p + 1]):
            v = int(obs["view"][k])
            R = cams["rotation_w2c"][v].astype(np.float64)
            C = cams["center"][v].astype(np.float64)
            fx, fy, cx, cy = cams["intrinsics"][v].astype(np.float64)
            x_cam = R @ (pts["xyz"][p].astype(np.float64) - C)
            assert x_cam[2] > 0
            u = fx * x_cam[0] / x_cam[2] + cx
            w = fy * x_cam[1] / x_cam[2] + cy
            assert abs(u - obs["x"][k]) < 1e-2
            assert abs(w - obs["y"][k]) < 1e-2
            checked += 1
    assert checked > 0
    # stored pose equals the one given to pytheia
    for i in range(NUM_VIEWS):
        np.testing.assert_allclose(cams["center"][i], truth["centers"][i], atol=1e-5)
        np.testing.assert_allclose(cams["rotation_w2c"][i], truth["rotations"][i], atol=1e-5)


def test_frustum_corners_are_at_requested_depth(scene):
    cams = scene["cameras"]
    depth = scene["bounds"]["frustum_scale"]
    assert depth > 0
    for i in range(NUM_VIEWS):
        R = cams["rotation_w2c"][i].astype(np.float64)
        C = cams["center"][i].astype(np.float64)
        z = (cams["frustum"][i].astype(np.float64) - C) @ R.T
        np.testing.assert_allclose(z[:, 2], depth, rtol=1e-4)


def test_noisy_errors_are_positive_and_bounded():
    rec, _ = make_reconstruction(noise=0.5, seed=3)
    sc = recon_to_scene(rec)
    err = sc["points"]["reprojection_error"]
    assert np.all(np.isfinite(err))
    assert 0.05 < float(err.mean()) < 2.0
    assert sc["stats"]["mean_reprojection_error"] > 0.05


def test_max_points_subsamples():
    rec, _ = make_reconstruction()
    sc = recon_to_scene(rec, max_points=10, seed=1)
    assert sc["points"]["xyz"].shape == (10, 3)
    assert sc["stats"]["num_tracks_total"] == NUM_POINTS
    assert sc["stats"]["num_points"] == 10
    again = recon_to_scene(rec, max_points=10, seed=1)
    np.testing.assert_array_equal(sc["points"]["xyz"], again["points"]["xyz"])


def test_max_points_negative_raises(recon_and_truth):
    rec, _ = recon_and_truth
    with pytest.raises(ValueError):
        recon_to_scene(rec, max_points=-1)


def test_view_graph_edges(recon_and_truth):
    rec, _ = recon_and_truth
    sc = recon_to_scene(rec, include_view_graph=True)
    edges = sc["edges"]
    assert edges is not None
    assert edges["pairs"].shape[1] == 2 and edges["pairs"].shape[0] == edges["weight"].shape[0]
    assert edges["pairs"].shape[0] > 0
    assert np.all(edges["weight"] >= 1)
    # only registered views have edges
    assert int(edges["pairs"].max()) < NUM_VIEWS


def test_thumbnails_are_embedded(tmp_path, recon_and_truth):
    rec, _ = recon_and_truth
    png = b"\x89PNG\r\n\x1a\n" + b"fake-image-bytes"
    (tmp_path / "img_0.png").write_bytes(png)  # matched by stem, different extension
    sc = recon_to_scene(rec, thumbnails_dir=str(tmp_path))
    thumbs = sc["cameras"]["thumbnails"]
    assert thumbs[0].startswith("data:image/png;base64,")
    assert base64.b64decode(thumbs[0].split(",", 1)[1]) == png
    assert all(t is None for t in thumbs[1:])


def test_thumbnails_dir_must_exist(recon_and_truth, tmp_path):
    rec, _ = recon_and_truth
    with pytest.raises(FileNotFoundError):
        recon_to_scene(rec, thumbnails_dir=str(tmp_path / "missing"))


def test_payload_roundtrip(scene):
    payload = scene_to_payload(scene)
    assert payload["format"] == "pytheia-scene"
    arr = decode_array(payload["points"]["xyz"])
    np.testing.assert_array_equal(arr, scene["points"]["xyz"])
    arr = decode_array(payload["points"]["rgb"])
    np.testing.assert_array_equal(arr, scene["points"]["rgb"])
    assert payload["points"]["rgb"]["dtype"] == "uint8"
    assert payload["points"]["xyz"]["dtype"] == "float32"


# ---------------------------------------------------------------------------
# HTML
# ---------------------------------------------------------------------------

_SCENE_TAG = re.compile(r'<script type="application/json" id="pytheia-scene">(.*?)</script>', re.S)


def _embedded_scene(html_text):
    match = _SCENE_TAG.search(html_text)
    assert match, "scene JSON tag not found"
    # strict JSON: NaN/Infinity are not allowed
    return json.loads(match.group(1), parse_constant=lambda c: pytest.fail("non-strict JSON: " + c))


def test_export_html_written(tmp_path, recon_and_truth):
    rec, _ = recon_and_truth
    out = tmp_path / "scene.html"
    returned = viewer.export_html(rec, str(out), title="unit <test>")
    assert Path(returned).is_file()
    text = out.read_text(encoding="utf-8")
    assert out.stat().st_size > 10_000
    assert text.startswith("<!DOCTYPE html>")
    assert "<title>unit &lt;test&gt;</title>" in text
    # everything is inline: no external script or stylesheet
    assert "<script src=" not in text
    assert 'rel="stylesheet"' not in text
    # inlined three.js and OrbitControls
    three_b64 = base64.b64encode((ASSETS / "three.module.min.js").read_bytes()).decode("ascii")
    orbit_b64 = base64.b64encode((ASSETS / "OrbitControls.js").read_bytes()).decode("ascii")
    assert three_b64 in text
    assert orbit_b64 in text
    # no unreplaced placeholders
    assert "__PYTHEIA_" not in text


def test_embedded_json_parses_headless(tmp_path, recon_and_truth):
    rec, truth = recon_and_truth
    out = tmp_path / "scene.html"
    viewer.export_html(rec, str(out), include_view_graph=True)
    data = _embedded_scene(out.read_text(encoding="utf-8"))

    assert data["format"] == "pytheia-scene"
    assert data["version"] == 1
    n = data["points"]["xyz"]["shape"][0]
    assert n == NUM_POINTS
    xyz = decode_array(data["points"]["xyz"])
    assert xyz.shape == (n, 3)
    np.testing.assert_allclose(xyz, truth["points"], atol=1e-5)
    rgb = decode_array(data["points"]["rgb"])
    np.testing.assert_array_equal(rgb, truth["colors"])
    obs_offsets = decode_array(data["points"]["obs_offsets"])
    assert obs_offsets.shape == (n + 1,)
    assert int(obs_offsets[-1]) == data["observations"]["view"]["shape"][0]
    views = decode_array(data["observations"]["view"]).astype(int)
    assert views.min() >= 0 and views.max() < NUM_VIEWS
    centers = decode_array(data["cameras"]["center"])
    np.testing.assert_allclose(centers, truth["centers"], atol=1e-5)
    estimated = decode_array(data["cameras"]["estimated"])
    assert int(estimated.sum()) == NUM_VIEWS - 1
    assert data["cameras"]["names"][-1] == "img_{}.jpg".format(NUM_VIEWS - 1)
    assert data["edges"] is not None
    assert decode_array(data["edges"]["pairs"]).shape[1] == 2
    assert data["stats"]["num_points"] == n


def test_theme_validation(recon_and_truth):
    rec, _ = recon_and_truth
    with pytest.raises(ValueError):
        viewer.build_html(rec, theme="blue")
    doc = viewer.build_html(rec, theme="light")
    assert 'data-theme="light"' in doc


def test_show_returns_iframe(recon_and_truth):
    pytest.importorskip("IPython")
    from IPython.display import HTML

    rec, _ = recon_and_truth
    obj = viewer.show(rec, height=420)
    assert isinstance(obj, HTML)
    assert "srcdoc=" in obj.data
    assert 'height="420"' in obj.data


def test_serve_nonblocking_returns_page(recon_and_truth):
    rec, _ = recon_and_truth
    server = viewer.serve(rec, port=0, block=False)
    try:
        url = "http://127.0.0.1:{}/".format(server.server_address[1])
        with urllib.request.urlopen(url, timeout=30) as resp:
            body = resp.read().decode("utf-8")
            assert resp.status == 200
        assert "pytheia-scene" in body
        assert _embedded_scene(body)["stats"]["num_points"] == NUM_POINTS
        with pytest.raises(Exception):
            urllib.request.urlopen(url + "missing", timeout=30)
    finally:
        server.shutdown()
        server.server_close()


# ---------------------------------------------------------------------------
# compare mode: Sim(3) alignment on shared views
# ---------------------------------------------------------------------------

KNOWN_SCALE = 2.5
KNOWN_T = np.array([0.7, -1.2, 3.0])


def _known_transform():
    rng = np.random.default_rng(11)
    return KNOWN_SCALE, random_rotation(rng), KNOWN_T.copy()


def test_umeyama_recovers_known_sim3():
    rng = np.random.default_rng(5)
    src = rng.normal(size=(12, 3))
    rot = random_rotation(rng)
    dst = 0.4 * src @ rot.T + np.array([-2.0, 1.0, 0.5])
    scale, r_fit, t_fit = umeyama_sim3(src, dst)
    assert scale == pytest.approx(0.4, rel=1e-9)
    np.testing.assert_allclose(r_fit, rot, atol=1e-9)
    np.testing.assert_allclose(t_fit, [-2.0, 1.0, 0.5], atol=1e-9)
    assert np.linalg.det(r_fit) == pytest.approx(1.0)


def test_umeyama_handles_reflection_case():
    """A reflected target is not a rotation: the fit must return a proper rotation."""
    rng = np.random.default_rng(6)
    src = rng.normal(size=(8, 3))
    dst = src * np.array([1.0, 1.0, -1.0])
    _, r_fit, _ = umeyama_sim3(src, dst)
    assert np.linalg.det(r_fit) == pytest.approx(1.0)
    np.testing.assert_allclose(r_fit @ r_fit.T, np.eye(3), atol=1e-9)


def test_umeyama_rejects_degenerate_input():
    line = np.outer(np.arange(5.0), np.array([1.0, 2.0, 0.0]))
    with pytest.raises(DegenerateAlignment):
        umeyama_sim3(line, line + 1.0)
    with pytest.raises(DegenerateAlignment):
        umeyama_sim3(np.zeros((4, 3)), np.ones((4, 3)))
    with pytest.raises(DegenerateAlignment):
        umeyama_sim3(np.eye(3)[:2], np.eye(3)[:2])


def test_compare_recovers_known_transform():
    """B is A moved by a known Sim(3): the fit must map B back onto A."""
    scale, rot, trans = _known_transform()
    rec_a, truth_a = make_reconstruction()
    rec_b, _ = make_reconstruction(transform=(scale, rot, trans))
    cmp_scene = recon_to_scene(rec_a, compare_with=rec_b)["compare"]
    assert cmp_scene is not None
    assert cmp_scene["num_shared"] == NUM_VIEWS - 1  # the last view is unregistered in both
    # B = s R A + t  =>  A = (1/s) R^T (B - t)
    assert cmp_scene["sim3"]["scale"] == pytest.approx(1.0 / scale, rel=1e-6)
    np.testing.assert_allclose(np.asarray(cmp_scene["sim3"]["rotation"]), rot.T, atol=1e-6)
    aligned = cmp_scene["scene"]
    np.testing.assert_allclose(aligned["points"]["xyz"], truth_a["points"], atol=1e-4)
    np.testing.assert_allclose(aligned["cameras"]["center"], truth_a["centers"], atol=1e-4)
    errs = cmp_scene["cameras"]
    matched = errs["matched_b"]
    np.testing.assert_array_equal(matched[:-1], np.arange(NUM_VIEWS - 1))
    assert matched[-1] == -1
    assert np.all(errs["center_error"][:-1] < 1e-4)
    assert np.isnan(errs["center_error"][-1])
    assert np.all(errs["rotation_error_deg"][:-1] < 1e-3)
    assert cmp_scene["rms_center_error"] < 1e-4


def test_compare_reports_camera_error_after_alignment():
    """Moving one shared camera of B by a known offset shows up as its center error."""
    rec_a, _ = make_reconstruction()
    rec_b, _ = make_reconstruction()
    # shift one registered camera of B far away; the fit then has a residual for it
    vid = min(rec_b.ViewIds())  # view 0 is registered; the last view is not
    assert rec_b.View(vid).IsEstimated()
    cam = rec_b.MutableView(vid).MutableCamera()
    cam.SetPosition(np.asarray(cam.GetPosition()) + np.array([0.0, 0.0, 0.5]))
    cmp_scene = recon_to_scene(rec_a, compare_with=rec_b)["compare"]
    assert cmp_scene is not None
    cerr = cmp_scene["cameras"]["center_error"]
    assert np.all(np.isfinite(cerr[:-1]))
    assert cerr[:-1].max() > 1e-3


def test_compare_skips_with_fewer_than_three_shared_views():
    rec_a, _ = make_reconstruction()
    rec_b, _ = make_reconstruction(name_fmt="other_{}.jpg")  # no name in common
    with pytest.warns(UserWarning, match="at least 3"):
        scene = recon_to_scene(rec_a, compare_with=rec_b)
    assert scene["compare"] is None
    # two shared names are not enough either
    rec_c, _ = make_reconstruction()
    scene_b = recon_to_scene(rec_c)
    scene_b["cameras"]["names"] = ["img_0.jpg", "img_1.jpg", "x", "y", "z", "w"]
    with pytest.warns(UserWarning):
        assert align_scene(recon_to_scene(rec_a), scene_b) is None


def test_compare_html_contains_compare_data(tmp_path):
    rec_a, _ = make_reconstruction()
    scale, rot, trans = _known_transform()
    rec_b, _ = make_reconstruction(transform=(scale, rot, trans))
    out = tmp_path / "compare.html"
    viewer.export_html(rec_a, str(out), compare_with=rec_b, title="cmp")
    text = out.read_text(encoding="utf-8")
    assert "__PYTHEIA_" not in text
    data = _embedded_scene(text)
    cmp = data["compare"]
    assert cmp is not None
    assert cmp["num_shared"] == NUM_VIEWS - 1
    assert cmp["sim3"]["scale"] == pytest.approx(1.0 / scale, rel=1e-6)
    assert len(cmp["sim3"]["rotation"]) == 3 and len(cmp["sim3"]["translation"]) == 3
    assert cmp["rms_center_error"] < 1e-4
    # the second reconstruction is embedded as a full scene
    b_scene = cmp["scene"]
    assert b_scene["format"] == "pytheia-scene"
    assert b_scene["stats"]["num_points"] == NUM_POINTS
    assert b_scene["cameras"]["names"][0] == "img_0.jpg"
    b_xyz = decode_array(b_scene["points"]["xyz"])
    a_xyz = decode_array(data["points"]["xyz"])
    np.testing.assert_allclose(b_xyz, a_xyz, atol=1e-4)
    assert decode_array(cmp["cameras"]["matched_b"]).shape == (NUM_VIEWS,)
    # the page without compare keeps the key, null
    plain = _embedded_scene(viewer.build_html(rec_a))
    assert plain["compare"] is None


def test_compare_helper_writes_page(tmp_path, recon_and_truth):
    rec_a, _ = recon_and_truth
    rec_b, _ = make_reconstruction()
    out = viewer.compare(rec_a, rec_b, str(tmp_path / "pair.html"))
    assert Path(out).is_file()
    data = _embedded_scene(Path(out).read_text(encoding="utf-8"))
    assert data["compare"] is not None
    assert data["compare"]["num_shared"] == NUM_VIEWS - 1
