"""Tests for the ``pytheia`` command line interface (pytheia.cli)."""

import importlib.util
import json
import os
import subprocess
import sys

import numpy as np
import pytest

pt = pytest.importorskip("pytheia")
cli = pytest.importorskip("pytheia.cli")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGACY_RECON = os.path.join(REPO, "pytests", "data", "legacy_v1.recon")
SFM_TEST_DIR = os.path.join(REPO, "pytests", "sfm")
sys.path.insert(0, SFM_TEST_DIR)
from random_recon_gen import RandomReconGenerator  # noqa: E402


def _synthetic_scene(root, nr_views=8, nr_tracks=200, seed=3):
    """Write an SfM input directory (reconstruction.recon + view_graph.bin) and return (input_dir, GT path).

    Conventions (see docs/content/view_graph.md):

    * ``ViewGraph`` stores an edge under the key (min id, max id) and does not
      swap the ``TwoViewInfo`` it is given. The info must therefore describe
      the smaller view id as "view 1" and the larger one as "view 2", so the
      pairs are generated from the sorted view ids.
    * ``rotation_2 = R_2 R_1^T`` (``RelativeRotationFromTwoRotations(r1, r2)``
      with r1 = view 1 and r2 = view 2).
    * ``position_2 = normalize(R_1 (c_2 - c_1))``: the direction from camera 1
      to camera 2 in the frame of camera 1.
    * ``num_verified_matches`` is above the incremental initialization
      threshold (100 tracks) and the scene has enough tracks shared by a pair.
    """
    gen = RandomReconGenerator(seed=seed)
    gt = gen.generate_random_recon(nr_views=nr_views, nr_tracks=nr_tracks)
    gt_path = os.path.join(root, "gt.recon")
    assert pt.io.WriteReconstruction(gt, gt_path, True)

    view_ids = sorted(gt.ViewIds())
    graph = pt.sfm.ViewGraph()
    for i, a in enumerate(view_ids):
        cam_a = gt.View(a).Camera()
        rot_a = cam_a.GetOrientationAsRotationMatrix()
        for b in view_ids[i + 1:]:
            cam_b = gt.View(b).Camera()
            info = pt.sfm.TwoViewInfo()
            info.focal_length_1 = cam_a.FocalLength()
            info.focal_length_2 = cam_b.FocalLength()
            info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(
                cam_a.GetOrientationAsAngleAxis(), cam_b.GetOrientationAsAngleAxis())
            direction = rot_a @ (np.asarray(cam_b.GetPosition()) - np.asarray(cam_a.GetPosition()))
            info.position_2 = direction / np.linalg.norm(direction)
            info.num_verified_matches = 150
            graph.AddEdge(a, b, info)

    # The sfm command re-estimates all poses from the tracks and the graph.
    for tid in gt.TrackIds():
        gt.MutableTrack(tid).SetIsEstimated(False)
    for vid in view_ids:
        view = gt.MutableView(vid)
        view.SetIsEstimated(False)
        # Intrinsics priors are what the estimators use to (re)set cameras.
        # Without them the image size becomes 0 inside the incremental estimator.
        view.SetCameraIntrinsicsPrior(view.Camera().CameraIntrinsicsPriorFromIntrinsics())
    input_dir = os.path.join(root, "sfm_input")
    os.makedirs(input_dir)
    assert pt.io.WriteReconstruction(gt, os.path.join(input_dir, "reconstruction.recon"), True)
    assert graph.WriteToDisk(os.path.join(input_dir, "view_graph.bin"))
    return input_dir, gt_path


def _assert_centers_match_gt(recon_path, gt_path, tol=1e-6):
    """Camera centers of the estimated views must match GT up to a similarity."""
    ok, est = pt.io.ReadReconstruction(recon_path)
    assert ok
    ok, gt = pt.io.ReadReconstruction(gt_path)
    assert ok
    view_ids = [v for v in est.ViewIds() if est.View(v).IsEstimated()]
    assert len(view_ids) >= 6
    src = np.array([est.View(v).Camera().GetPosition() for v in view_ids], dtype=np.float64)
    dst = np.array([gt.View(v).Camera().GetPosition() for v in view_ids], dtype=np.float64)
    scale, rot, trans = cli.umeyama(src, dst)
    residual = np.linalg.norm(scale * src @ rot.T + trans - dst, axis=1)
    assert residual.max() < tol, residual


# ---------------------------------------------------------------------------
# Generic behaviour
# ---------------------------------------------------------------------------

def test_help_and_version_return_status(capsys):
    assert cli.main(["--help"]) == 0
    assert cli.main(["--version"]) == 0
    assert "pytheia" in capsys.readouterr().out


def test_unknown_command_is_usage_error(capsys):
    assert cli.main(["nosuchcommand"]) == 2


def test_python_dash_m_entry_point():
    out = subprocess.run(
        [sys.executable, "-m", "pytheia", "--version"],
        capture_output=True, text=True, check=False, cwd=REPO,
        env={**os.environ, "PYTHONPATH": os.path.join(REPO, "src")})
    assert out.returncode == 0, out.stderr
    assert out.stdout.startswith("pytheia ")


# ---------------------------------------------------------------------------
# info / convert / align
# ---------------------------------------------------------------------------

def test_info_text_and_json(capsys):
    assert cli.main(["info", LEGACY_RECON]) == 0
    text = capsys.readouterr().out
    assert "views:" in text and "4 (registered 4)" in text
    assert "tracks:" in text and "7 (estimated 7)" in text

    assert cli.main(["info", LEGACY_RECON, "--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["views"] == 4
    assert data["registered_views"] == 4
    assert data["tracks"] == 7
    assert data["mean_reprojection_error_px"] > 0


def test_info_missing_file_is_runtime_error(tmp_path, capsys):
    assert cli.main(["info", str(tmp_path / "missing.recon")]) == 1
    assert "not found" in capsys.readouterr().err


@pytest.mark.parametrize("fmt", ["colmap", "nvm", "bundler", "ply", "json"])
def test_convert_writes_output(tmp_path, fmt, capsys):
    out = tmp_path / fmt
    assert cli.main(["convert", LEGACY_RECON, str(out), "--format", fmt]) == 0
    assert os.path.exists(out)
    if fmt == "colmap":
        for name in ("cameras.txt", "images.txt", "points3D.txt"):
            assert (out / name).stat().st_size > 0
    if fmt == "bundler":
        assert (out / "bundle.out").stat().st_size > 0
        assert (out / "list.txt").stat().st_size > 0


def test_convert_colmap_bin(tmp_path, capsys):
    out = tmp_path / "colmap_bin"
    status = cli.main(["convert", LEGACY_RECON, str(out), "--format", "colmap-bin"])
    if status == 0:
        for name in ("cameras.bin", "images.bin", "points3D.bin"):
            assert (out / name).stat().st_size > 0
    else:
        # Older builds have no binary COLMAP writer; the message must say so.
        assert status == 1
        assert "binary" in capsys.readouterr().err


def test_convert_nerfstudio_needs_images(tmp_path, capsys):
    assert cli.main(["convert", LEGACY_RECON, str(tmp_path / "ns.json"),
                     "--format", "nerfstudio"]) == 2
    assert "--images" in capsys.readouterr().err


def test_convert_nerfstudio(tmp_path):
    out = tmp_path / "transforms.json"
    images = tmp_path / "images"
    images.mkdir()
    assert cli.main(["convert", LEGACY_RECON, str(out), "--format", "nerfstudio",
                     "--images", str(images)]) == 0
    assert out.stat().st_size > 0


def test_umeyama_recovers_similarity():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(20, 3))
    angle = 0.7
    rot_true = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0],
                         [0, 0, 1.0]])
    scale_true = 2.5
    t_true = np.array([1.0, -2.0, 0.5])
    dst = scale_true * src @ rot_true.T + t_true
    scale, rot, trans = cli.umeyama(src, dst)
    assert scale == pytest.approx(scale_true, rel=1e-9)
    assert np.allclose(rot, rot_true, atol=1e-9)
    assert np.allclose(trans, t_true, atol=1e-9)


def test_umeyama_recovers_half_turn_rotation():
    rng = np.random.default_rng(1)
    src = rng.normal(size=(10, 3))
    # Rotation by 180 degrees around z: a proper rotation, determinant +1.
    rot_true = np.diag([-1.0, -1.0, 1.0])
    dst = src @ rot_true.T
    scale, rot, trans = cli.umeyama(src, dst)
    assert scale == pytest.approx(1.0, rel=1e-9)
    assert np.linalg.det(rot) == pytest.approx(1.0)
    assert np.allclose(rot, rot_true, atol=1e-9)


def test_align_identical_reconstructions_is_exact(capsys):
    assert cli.main(["align", LEGACY_RECON, LEGACY_RECON, "--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["common_views"] == 4
    assert data["scale"] == pytest.approx(1.0)
    assert data["camera_center_error_max"] < 1e-8


def test_align_needs_three_common_views(tmp_path, capsys):
    input_dir, gt_path = _synthetic_scene(str(tmp_path), nr_views=2, nr_tracks=20)
    assert cli.main(["align", LEGACY_RECON, gt_path]) == 1
    assert "at least 3" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# view / export-splat (optional modules)
# ---------------------------------------------------------------------------

def _has_function(module_name, attr):
    """True if ``module_name`` imports and provides ``attr`` (pytheia.viewer / pytheia.export)."""
    try:
        module = importlib.import_module(module_name)
    except ImportError:
        return False
    return hasattr(module, attr)


@pytest.mark.skipif(not _has_function("pytheia.viewer", "export_html"),
                    reason="pytheia.viewer.export_html not available")
def test_view_writes_html(tmp_path):
    html = tmp_path / "scene.html"
    assert cli.main(["view", LEGACY_RECON, "--html", str(html)]) == 0
    assert html.stat().st_size > 0


@pytest.mark.skipif(_has_function("pytheia.viewer", "export_html"),
                    reason="pytheia.viewer.export_html is available")
def test_view_reports_missing_module(tmp_path, capsys):
    assert cli.main(["view", LEGACY_RECON, "--html", str(tmp_path / "x.html")]) == 1
    assert "pytheia.viewer" in capsys.readouterr().err


@pytest.mark.skipif(not _has_function("pytheia.export", "export_for_gaussian_splatting"),
                    reason="pytheia.export.export_for_gaussian_splatting not available")
def test_export_splat_runs(tmp_path):
    images = tmp_path / "images"
    images.mkdir()
    assert cli.main(["export-splat", LEGACY_RECON, "--images", str(images),
                     "--out", str(tmp_path / "splat"), "--no-undistort"]) in (0, 1)


@pytest.mark.skipif(_has_function("pytheia.export", "export_for_gaussian_splatting"),
                    reason="pytheia.export.export_for_gaussian_splatting is available")
def test_export_splat_reports_missing_module(tmp_path, capsys):
    assert cli.main(["export-splat", LEGACY_RECON, "--images", str(tmp_path),
                     "--out", str(tmp_path / "splat")]) == 1
    assert "pytheia.export" in capsys.readouterr().err


# ---------------------------------------------------------------------------
# sfm options
# ---------------------------------------------------------------------------

def test_list_options(capsys):
    assert cli.main(["sfm", "--list-options"]) == 0
    out = capsys.readouterr().out
    assert "bundle_adjustment_loss_function_type = " in out
    assert "ligt_position_estimator_options.use_scale_constraint = " in out
    assert "art_position_estimator_options.max_triplets_per_view = " in out
    assert "refine_rotations_with_rotation_only_error = " in out
    assert "rotation_only_refiner_options.tuple_gate_px = " in out


def test_set_overrides_typed_values():
    opts = cli.build_estimator_options(None, [
        "max_num_iterations=7",
        "bundle_adjustment_robust_loss_width=2.5",
        "bundle_adjust_tracks=false",
        "global_position_estimator_type=LIGT",
        "ligt_position_estimator_options.use_scale_constraint=false",
    ])
    assert opts.max_num_iterations == 7
    assert opts.bundle_adjustment_robust_loss_width == pytest.approx(2.5)
    assert opts.bundle_adjust_tracks is False
    assert opts.global_position_estimator_type == pt.sfm.GlobalPositionEstimatorType.LIGT
    assert opts.ligt_position_estimator_options.use_scale_constraint is False


def test_set_overrides_rotation_refiner():
    opts = cli.build_estimator_options(None, [
        "refine_rotations_with_rotation_only_error=true",
        "rotation_only_refiner_options.max_reference_candidates=5",
        "rotation_only_refiner_options.tuple_gate_px=4.5",
        "rotation_only_refiner_options.use_two_view_tuples=false",
    ])
    assert opts.refine_rotations_with_rotation_only_error is True
    assert opts.rotation_only_refiner_options.max_reference_candidates == 5
    assert opts.rotation_only_refiner_options.tuple_gate_px == pytest.approx(4.5)
    assert opts.rotation_only_refiner_options.use_two_view_tuples is False


def test_set_overrides_art_estimator():
    opts = cli.build_estimator_options(None, [
        "global_position_estimator_type=ART",
        "art_position_estimator_options.max_triplets_per_view=20",
    ])
    assert opts.global_position_estimator_type == pt.sfm.GlobalPositionEstimatorType.ART
    assert opts.art_position_estimator_options.max_triplets_per_view == 20


def test_config_file_then_set_override(tmp_path):
    config = tmp_path / "options.json"
    config.write_text(json.dumps({
        "max_num_iterations": 11,
        "bundle_adjustment_loss_function_type": "TRIVIAL",
        "ligt_position_estimator_options": {"use_scale_constraint": False},
    }), encoding="utf-8")
    opts = cli.build_estimator_options(str(config), ["max_num_iterations=3"])
    assert opts.max_num_iterations == 3  # --set wins over the file
    assert opts.bundle_adjustment_loss_function_type == pt.sfm.LossFunctionType.TRIVIAL
    assert opts.ligt_position_estimator_options.use_scale_constraint is False


@pytest.mark.parametrize("pair", ["nosuch=1", "ligt_position_estimator_options.nosuch=1"])
def test_unknown_option_is_clear_usage_error(tmp_path, capsys, pair):
    status = cli.main(["sfm", "--input", str(tmp_path), "--out", str(tmp_path / "o"),
                       "--set", pair])
    assert status == 2
    assert "unknown option" in capsys.readouterr().err


def test_invalid_value_is_usage_error(tmp_path, capsys):
    status = cli.main(["sfm", "--input", str(tmp_path), "--out", str(tmp_path / "o"),
                       "--set", "max_num_iterations=many"])
    assert status == 2
    assert "max_num_iterations" in capsys.readouterr().err


def test_bad_enum_name_is_usage_error(tmp_path, capsys):
    status = cli.main(["sfm", "--input", str(tmp_path), "--out", str(tmp_path / "o"),
                       "--set", "global_position_estimator_type=NOPE"])
    assert status == 2
    assert "expected one of" in capsys.readouterr().err


def test_set_requires_key_value():
    with pytest.raises(cli.CLIError):
        cli.build_estimator_options(None, ["max_num_iterations"])


# ---------------------------------------------------------------------------
# sfm end to end
# ---------------------------------------------------------------------------

def test_sfm_global_end_to_end(tmp_path, capsys):
    input_dir, gt_path = _synthetic_scene(str(tmp_path))
    out = tmp_path / "out"
    status = cli.main(["sfm", "--input", input_dir, "--out", str(out),
                       "--method", "global", "--seed", "1"])
    assert status == 0, capsys.readouterr().err
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["success"] is True
    assert summary["seed"] == 1
    assert summary["estimated_views"] >= 6
    assert summary["estimated_tracks"] > 0
    assert summary["mean_reprojection_error_px"] < 1.0
    assert (out / "reconstruction.recon").stat().st_size > 0
    assert (out / "reconstruction.ply").stat().st_size > 0

    # Noise-free synthetic data: the registered camera centers must match GT.
    _assert_centers_match_gt(str(out / "reconstruction.recon"), gt_path)

    # The written reconstruction loads and can be inspected.
    assert cli.main(["info", str(out / "reconstruction.recon")]) == 0
    assert cli.main(["align", str(out / "reconstruction.recon"), gt_path]) == 0


def test_sfm_incremental_end_to_end(tmp_path, capsys):
    input_dir, gt_path = _synthetic_scene(str(tmp_path))
    out = tmp_path / "out"
    status = cli.main(["sfm", "--input", input_dir, "--out", str(out),
                       "--method", "incremental", "--seed", "1"])
    assert status == 0, capsys.readouterr().err
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["success"] is True
    assert summary["estimated_views"] >= 6
    assert summary["mean_reprojection_error_px"] < 1.0
    _assert_centers_match_gt(str(out / "reconstruction.recon"), gt_path)


def test_sfm_missing_input_is_error(tmp_path, capsys):
    status = cli.main(["sfm", "--input", str(tmp_path / "nope"), "--out", str(tmp_path / "o")])
    assert status == 1
    assert "not found" in capsys.readouterr().err


def test_sfm_requires_input_and_out(capsys):
    assert cli.main(["sfm", "--out", "x"]) == 2
    assert cli.main(["sfm", "--input", "x"]) == 2
