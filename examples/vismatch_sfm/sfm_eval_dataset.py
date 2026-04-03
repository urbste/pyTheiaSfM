# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""Load SFM_EVAL-style scenes: images/, images/K.txt, gt_dense_cameras/<name>.camera."""

from __future__ import annotations

import glob
import os
import numpy as np


def list_eval_scenes(root: str) -> list[str]:
    """Subdirectory names under ``root`` that contain ``images`` and ``gt_dense_cameras``."""
    root = os.path.abspath(root)
    names: list[str] = []
    if not os.path.isdir(root):
        return names
    for name in sorted(os.listdir(root)):
        scene = os.path.join(root, name)
        if not os.path.isdir(scene):
            continue
        if os.path.isdir(os.path.join(scene, "images")) and os.path.isdir(
            os.path.join(scene, "gt_dense_cameras")
        ):
            names.append(name)
    return names


def resolve_scene_path(sfm_eval_root: str | None, dataset: str) -> str:
    """``dataset`` is either an absolute path to a scene folder or a name under ``sfm_eval_root``."""
    if os.path.isabs(dataset) and os.path.isdir(dataset):
        return os.path.abspath(dataset)
    if not sfm_eval_root:
        raise ValueError("sfm_eval_root is required when dataset is not an absolute path")
    scene = os.path.join(os.path.abspath(sfm_eval_root), dataset)
    if not os.path.isdir(scene):
        raise FileNotFoundError(f"Scene not found: {scene}")
    return scene


def _parse_float_line(line: str) -> list[float]:
    return [float(x) for x in line.split()]


def parse_dense_camera_file(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    """
    Parse ``*.camera`` text file: 3×3 K, separator, 3×3 R, translation t, image size.

    Assumes **COLMAP-style** extrinsics: ``X_cam = R @ X_world + t`` with pinhole ``x ~ K X_cam``.
    Theia uses camera center ``C`` and world-to-camera rotation ``R`` with ``X_cam = R @ (X - C)``,
    hence ``C = -R.T @ t``.
    """
    with open(path, encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]
    if len(lines) < 9:
        raise ValueError(f"Expected >= 9 non-empty lines in {path}, got {len(lines)}")

    K = np.array([_parse_float_line(lines[0]), _parse_float_line(lines[1]), _parse_float_line(lines[2])])
    R = np.array([_parse_float_line(lines[4]), _parse_float_line(lines[5]), _parse_float_line(lines[6])])
    t = np.array(_parse_float_line(lines[7]), dtype=np.float64).reshape(3)
    wh = _parse_float_line(lines[8])
    if len(wh) != 2:
        raise ValueError(f"Line 9 must be width height, got {lines[8]!r}")
    w_ref, h_ref = int(wh[0]), int(wh[1])
    return K, R, t, w_ref, h_ref


def load_K_txt(path: str) -> np.ndarray:
    data = np.loadtxt(path, dtype=np.float64)
    if data.shape != (3, 3):
        raise ValueError(f"K.txt must be 3×3, got {data.shape} in {path}")
    return data


def scale_intrinsics_K(K: np.ndarray, w0: int, h0: int, w1: int, h1: int) -> np.ndarray:
    """Scale calibration matrix when image resolution differs from reference (w0,h0)."""
    if w0 <= 0 or h0 <= 0:
        return K.copy()
    sx = float(w1) / float(w0)
    sy = float(h1) / float(h0)
    Ks = K.astype(np.float64).copy()
    Ks[0, 0] *= sx
    Ks[0, 2] *= sx
    Ks[1, 1] *= sy
    Ks[1, 2] *= sy
    return Ks


def prior_from_calibration_matrix(pt, K: np.ndarray, image_width: int, image_height: int):
    """Build ``CameraIntrinsicsPrior`` for PINHOLE from a 3×3 K matrix."""
    focal, skew, aspect, px, py = pt.sfm.CalibrationMatrixToIntrinsics(K)
    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [float(focal)]
    prior.aspect_ratio.value = [float(aspect)]
    prior.principal_point.value = [float(px), float(py)]
    prior.skew.value = [float(skew)]
    prior.image_width = int(image_width)
    prior.image_height = int(image_height)
    prior.camera_intrinsics_model_type = "PINHOLE"
    return prior


def collect_eval_image_paths(images_dir: str, img_ext: str) -> list[str]:
    pattern = os.path.join(images_dir, f"*.{img_ext}")
    return sorted(glob.glob(pattern))


def assert_uniform_resolution(image_paths: list[str]) -> tuple[int, int]:
    """Return (width, height) if all images share the same size; else raise."""
    import cv2

    wh: tuple[int, int] | None = None
    for p in image_paths:
        im = cv2.imread(p)
        if im is None:
            raise FileNotFoundError(f"Could not read image: {p}")
        w, h = im.shape[1], im.shape[0]
        if wh is None:
            wh = (w, h)
        elif (w, h) != wh:
            raise ValueError(
                f"Eval mode expects uniform image size; {p} is {w}×{h}, expected {wh[0]}×{wh[1]}"
            )
    if wh is None:
        raise ValueError("No images")
    return wh


def assert_gt_cameras_exist(image_paths: list[str], gt_dir: str) -> None:
    missing = []
    for p in image_paths:
        cam = os.path.join(gt_dir, os.path.basename(p) + ".camera")
        if not os.path.isfile(cam):
            missing.append(cam)
    if missing:
        raise FileNotFoundError(
            "Missing GT camera file(s) (expected `<basename>.camera` in gt_dense_cameras):\n"
            + "\n".join(missing[:20])
            + (f"\n… and {len(missing) - 20} more" if len(missing) > 20 else "")
        )


def build_gt_reconstruction(pt, image_paths: list[str], gt_dir: str, prior) -> object:
    """Ground truth reconstruction: calibrated, posed views from ``gt_dense_cameras`` (no tracks)."""
    recon = pt.sfm.Reconstruction()
    for idx, pth in enumerate(image_paths):
        name = os.path.basename(pth)
        cam_path = os.path.join(gt_dir, name + ".camera")
        _, R, C, _, _ = parse_dense_camera_file(cam_path)

        vid = recon.AddView(name, 0, float(idx))
        v = recon.MutableView(vid)
        v.SetCameraIntrinsicsPrior(prior)
        cam = v.MutableCamera()
        cam.SetFromCameraIntrinsicsPriors(prior)
        cam.SetOrientationFromRotationMatrix(R.astype(np.float64).T)
        cam.SetPosition(C.astype(np.float64))
        v.SetIsEstimated(True)
    return recon
