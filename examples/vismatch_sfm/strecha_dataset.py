# Copyright 2026 the pyTheiaSfM contributors. SPDX-License-Identifier: BSD-3-Clause
"""
Strecha scene root layout (as used by this pipeline)::

    <strecha_dir>/images/*.jpg
    <strecha_dir>/images/K.txt
    <strecha_dir>/gt_dense_cameras/*.camera
"""

from __future__ import annotations

import os


def layout_paths(strecha_root: str) -> tuple[str, str, str]:
    """Return (images_dir, gt_dense_cameras_dir, k_txt_path)."""
    root = os.path.abspath(strecha_root)
    images_dir = os.path.join(root, "images")
    gt_dir = os.path.join(root, "gt_dense_cameras")
    k_path = os.path.join(images_dir, "K.txt")
    return images_dir, gt_dir, k_path


def validate_layout(strecha_root: str) -> tuple[str, str, str]:
    """Ensure ``images/``, ``gt_dense_cameras/``, and ``images/K.txt`` exist."""
    images_dir, gt_dir, k_path = layout_paths(strecha_root)
    if not os.path.isdir(images_dir):
        raise FileNotFoundError(f"Missing images/ directory: {images_dir}")
    if not os.path.isdir(gt_dir):
        raise FileNotFoundError(f"Missing gt_dense_cameras/ directory: {gt_dir}")
    if not os.path.isfile(k_path):
        raise FileNotFoundError(f"Missing K.txt: {k_path}")
    return images_dir, gt_dir, k_path


def resolve_images_for_stems(
    images_dir: str,
    view_stems: list[str],
    img_ext: str = "jpg",
) -> dict[str, str]:
    """
    Map each GT view name to a file under ``images/`` (absolute path).

    ``ReadStrechaDataset`` view names match the camera filename without
    ``.camera`` (e.g. ``0000.jpg`` from ``0000.jpg.camera``). The raster is
    then ``images/0000.jpg``, not ``images/0000.jpg.<ext>``.
    """
    images_dir = os.path.abspath(images_dir)
    ext = img_ext.lstrip(".").lower()
    ext_with_dot = f".{ext}"
    resolved: dict[str, str] = {}
    missing: list[str] = []
    for stem in view_stems:
        candidates: list[str] = []
        candidates.append(os.path.join(images_dir, stem))
        if not stem.lower().endswith(ext_with_dot):
            candidates.append(os.path.join(images_dir, f"{stem}.{ext}"))
        path: str | None = None
        for c in candidates:
            if os.path.isfile(c):
                path = os.path.abspath(c)
                break
        if path is not None:
            resolved[stem] = path
        else:
            missing.append(candidates[-1])
    if missing:
        raise FileNotFoundError(
            "Missing image file(s) under images/ (expected one file per GT view stem):\n"
            + "\n".join(missing[:30])
            + (f"\n… and {len(missing) - 30} more" if len(missing) > 30 else "")
            + f"\n(images directory: {images_dir})"
        )
    return resolved
