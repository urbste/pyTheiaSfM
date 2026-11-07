"""Export pyTheia reconstructions for Gaussian splatting trainers.

Most splatting codebases (3D Gaussian Splatting, gsplat, Brush, OpenSplat,
Nerfstudio Splatfacto) read a COLMAP ``sparse/0`` folder with undistorted
PINHOLE / SIMPLE_PINHOLE cameras plus an ``images/`` folder. This module writes
that layout from a :class:`pytheia.sfm.Reconstruction`::

    import pytheia as pt
    from pytheia.export import export_for_gaussian_splatting

    export_for_gaussian_splatting(recon, "scene_gs", "/path/to/images")

Output layout (``format="colmap"``)::

    scene_gs/
        images/            undistorted (and optionally resized) images
        sparse/0/cameras.bin, images.bin, points3D.bin
        points3d.ply       seed point cloud (x y z nx ny nz r g b)

``format="transforms"`` writes ``transforms.json`` (see
:func:`pytheia.io.WriteTransformsJson`) instead of ``sparse/0``, and
``format="nerfstudio"`` writes the Nerfstudio ``transforms.json`` produced by
:func:`pytheia.io.WriteNerfStudio`.

Undistortion uses the radial model of the pyTheia PINHOLE camera
(``r_d = r_u (1 + k1 r^2 + k2 r^4)``, the same convention as OpenCV with
``k1, k2`` and no tangential terms). The remap is built with OpenCV, which needs
``numpy`` and ``opencv-python`` (or ``opencv-python-headless``). Images that
need no undistortion and no resizing are copied unchanged, so the optional
dependencies are only needed for undistortion and resizing. Resizing falls
back to Pillow when OpenCV is not installed.

Only PINHOLE cameras are supported. Other camera models raise ``ValueError``.
"""

from __future__ import annotations

import math
import os
import shutil
import struct
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

from pytheia.pytheia import io as _io
from pytheia.pytheia import sfm as _sfm

__all__ = [
    "export_for_gaussian_splatting",
    "undistortion_maps",
    "write_points_ply",
]

SUPPORTED_FORMATS = ("colmap", "transforms", "nerfstudio")


def _require_cv2():
    try:
        import cv2  # pylint: disable=import-outside-toplevel
    except ImportError as exc:
        raise ImportError(
            "pytheia.export needs OpenCV for undistortion and resizing. "
            "Install it with `pip install opencv-python-headless`, or pass "
            "undistort=False and max_image_size=-1 to only copy images."
        ) from exc
    return cv2


def undistortion_maps(width: int, height: int, K, k1: float, k2: float):
    """Builds remap tables that undistort an image to a pinhole image.

    The output image keeps the focal lengths and principal point ``K`` and has
    the same size as the input. For every output pixel ``(u, v)`` the tables
    give the distorted source pixel obtained by applying the pyTheia radial
    model to the normalized coordinates ``K^-1 [u, v, 1]``.

    Args:
        width: Image width in pixels.
        height: Image height in pixels.
        K: 3x3 calibration matrix (numpy array, float64).
        k1: First radial distortion coefficient.
        k2: Second radial distortion coefficient.

    Returns:
        ``(map1, map2)`` float32 arrays for ``cv2.remap``.
    """
    import numpy as np  # pylint: disable=import-outside-toplevel

    cv2 = _require_cv2()
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.array([k1, k2, 0.0, 0.0], dtype=np.float64)
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, None, K, (int(width), int(height)), cv2.CV_32FC1
    )
    return map1, map2


def write_points_ply(path: str, xyz: Sequence, rgb: Sequence) -> None:
    """Writes a binary little-endian PLY point cloud in the 3DGS seed layout.

    Each vertex has ``x y z nx ny nz red green blue``. Normals are written as
    zeros. This is the ``points3d.ply`` layout used to initialise Gaussian
    splatting training.

    Args:
        path: Output file path.
        xyz: Sequence of ``(x, y, z)`` points.
        rgb: Sequence of ``(r, g, b)`` colors in ``[0, 255]``, same length.
    """
    if len(xyz) != len(rgb):
        raise ValueError("xyz and rgb must have the same length.")
    record = struct.Struct("<6f3B")
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(xyz)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float nx\n"
        "property float ny\n"
        "property float nz\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for p, c in zip(xyz, rgb):
            f.write(
                record.pack(
                    float(p[0]),
                    float(p[1]),
                    float(p[2]),
                    0.0,
                    0.0,
                    0.0,
                    int(min(max(c[0], 0), 255)),
                    int(min(max(c[1], 0), 255)),
                    int(min(max(c[2], 0), 255)),
                )
            )


def _copy_reconstruction(reconstruction, scratch_dir: str):
    """Returns a deep copy of a reconstruction through the binary format."""
    os.makedirs(scratch_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(suffix=".recon", prefix=".export_", dir=scratch_dir)
    os.close(fd)
    try:
        if not _io.WriteReconstruction(
            reconstruction, tmp_path, write_full_reconstruction=True
        ):
            raise RuntimeError("Could not write a temporary copy of the reconstruction.")
        ok, copy = _io.ReadReconstruction(tmp_path)
        if not ok:
            raise RuntimeError("Could not read back the temporary reconstruction copy.")
        return copy
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def _collect_points(
    reconstruction,
    view_ids: List[int],
    max_reprojection_error: Optional[float],
    min_track_length: int,
) -> Dict[int, Tuple[Tuple[float, float, float], Tuple[int, int, int], float]]:
    """Returns the tracks that pass the filters.

    Maps track id to ``(xyz, rgb, mean_reprojection_error)``. The reprojection
    error is measured in pixels with the original (distorted) cameras.
    """
    estimated = set(view_ids)
    cameras = {}
    view_track_ids = {}
    for vid in view_ids:
        view = reconstruction.View(vid)
        cameras[vid] = view.Camera()
        view_track_ids[vid] = set(view.TrackIds())

    kept = {}
    for tid in reconstruction.TrackIds():
        track = reconstruction.Track(tid)
        point = track.Point()
        if point[3] == 0.0:
            continue
        xyz = (point[0] / point[3], point[1] / point[3], point[2] / point[3])

        errors = []
        for vid in track.ViewIds():
            if vid not in estimated or tid not in view_track_ids[vid]:
                continue
            observed = reconstruction.View(vid).GetFeature(tid).point
            depth, projected = cameras[vid].ProjectPoint(list(xyz))
            if depth <= 0.0:
                errors.append(float("inf"))
                continue
            errors.append(
                math.hypot(
                    float(projected[0]) - float(observed[0]),
                    float(projected[1]) - float(observed[1]),
                )
            )

        if len(errors) < min_track_length:
            continue
        mean_error = sum(errors) / len(errors)
        if max_reprojection_error is not None and mean_error > max_reprojection_error:
            continue
        color = track.Color()
        rgb = (int(color[0]), int(color[1]), int(color[2]))
        kept[tid] = (xyz, rgb, mean_error)
    return kept


class _PinholeState:
    """Original intrinsics of one view, read before any mutation."""

    def __init__(self, camera):
        if camera.GetCameraIntrinsicsModelType() != _sfm.CameraIntrinsicsModelType.PINHOLE:
            raise ValueError(
                "export_for_gaussian_splatting only supports PINHOLE cameras, "
                f"found {camera.GetCameraIntrinsicsModelType()}."
            )
        K = camera.GetCalibrationMatrix()
        intrinsics = camera.CameraIntrinsics()
        self.width = int(camera.ImageWidth())
        self.height = int(camera.ImageHeight())
        self.K = K
        self.fx = float(K[0, 0])
        self.fy = float(K[1, 1])
        self.cx = float(K[0, 2])
        self.cy = float(K[1, 2])
        self.k1 = float(intrinsics.RadialDistortion1())
        self.k2 = float(intrinsics.RadialDistortion2())

    @property
    def has_distortion(self) -> bool:
        return self.k1 != 0.0 or self.k2 != 0.0


def _rewrite_observations(view, state: "_PinholeState", scale: float, undistort: bool) -> None:
    """Moves the 2D observations of a view into the exported image frame.

    Observations are undistorted with the same pinhole intrinsics that are used
    for the undistorted image (``K``), then scaled by ``scale`` for resizing.
    The feature objects are updated in place so covariances and depth priors
    are kept.
    """
    import numpy as np  # pylint: disable=import-outside-toplevel

    track_ids = list(view.TrackIds())
    if not track_ids:
        return
    features = [view.GetFeature(tid) for tid in track_ids]
    points = np.array([f.point for f in features], dtype=np.float64).reshape(-1, 2)

    if undistort and state.has_distortion:
        cv2 = _require_cv2()
        K = np.ascontiguousarray(np.asarray(state.K, dtype=np.float64).reshape(3, 3))
        dist = np.array([state.k1, state.k2, 0.0, 0.0], dtype=np.float64)
        src = points.reshape(-1, 1, 2)
        if hasattr(cv2, "undistortPointsIter"):
            criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)
            out = cv2.undistortPointsIter(src, K, dist, None, K, criteria)
        else:
            out = cv2.undistortPoints(src, K, dist, None, K)
        points = np.asarray(out, dtype=np.float64).reshape(-1, 2)

    if scale != 1.0:
        points = points * scale
    for feature, point in zip(features, points):
        feature.point = point


def _scaled_size(width: int, height: int, scale: float) -> Tuple[int, int]:
    return max(1, int(round(width * scale))), max(1, int(round(height * scale)))


def _remap_image(src: str, dst: str, map1, map2, out_size: Tuple[int, int], expected_size) -> None:
    cv2 = _require_cv2()
    image = cv2.imread(src, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Could not read image {src}.")
    if (image.shape[1], image.shape[0]) != tuple(expected_size):
        raise ValueError(
            f"Image {src} has size {image.shape[1]}x{image.shape[0]} but the "
            f"camera model says {expected_size[0]}x{expected_size[1]}."
        )
    out = cv2.remap(
        image, map1, map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0
    )
    if out_size != tuple(expected_size):
        out = cv2.resize(out, out_size, interpolation=cv2.INTER_AREA)
    _write_image(dst, out)


def _resize_image(src: str, dst: str, out_size: Tuple[int, int]) -> None:
    try:
        import cv2  # pylint: disable=import-outside-toplevel
    except ImportError:
        cv2 = None
    if cv2 is not None:
        image = cv2.imread(src, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f"Could not read image {src}.")
        out = cv2.resize(image, out_size, interpolation=cv2.INTER_AREA)
        _write_image(dst, out)
        return

    try:
        from PIL import Image  # pylint: disable=import-outside-toplevel
    except ImportError as exc:
        raise ImportError(
            "Resizing images needs OpenCV or Pillow. Install one of them, or "
            "use max_image_size=-1."
        ) from exc
    with Image.open(src) as image:
        image.resize(out_size, Image.Resampling.LANCZOS).save(dst)


def _write_image(dst: str, image) -> None:
    cv2 = _require_cv2()
    if not cv2.imwrite(dst, image):
        raise OSError(f"Could not write image {dst}.")


def export_for_gaussian_splatting(
    reconstruction,
    output_dir: str,
    image_dir: str,
    undistort: bool = True,
    max_image_size: int = -1,
    format: str = "colmap",  # pylint: disable=redefined-builtin
    max_reprojection_error: Optional[float] = 4.0,
    min_track_length: int = 2,
    write_seed_points: bool = True,
) -> Dict[str, object]:
    """Exports a reconstruction as input for Gaussian splatting trainers.

    The input reconstruction is not modified. The export works on a copy.

    Args:
        reconstruction: ``pytheia.sfm.Reconstruction`` to export. Only
            estimated views and tracks are exported.
        output_dir: Output directory. Created if it does not exist.
        image_dir: Directory that holds the input images. Each view name is
            resolved relative to this directory.
        undistort: If True, images with radial distortion are undistorted to
            a pinhole model and the cameras are written without distortion.
            If False, distortion is kept in the exported camera model.
        max_image_size: If positive, images are scaled so that the longer side
            is at most this many pixels. The intrinsics are scaled to match.
            Use -1 (default) to keep the original resolution.
        format: One of ``"colmap"`` (``sparse/0`` binary model, default),
            ``"transforms"`` (``transforms.json`` in the instant-ngp / gsplat
            convention, see :func:`pytheia.io.WriteTransformsJson`), or
            ``"nerfstudio"`` (``transforms.json`` from
            :func:`pytheia.io.WriteNerfStudio`).
        max_reprojection_error: Drop tracks whose mean reprojection error in
            pixels (measured with the input cameras) is larger than this value.
            Set to None to disable the filter.
        min_track_length: Drop tracks observed in fewer estimated views.
        write_seed_points: If True, write ``points3d.ply`` with the points that
            pass the filters.

    Returns:
        A dictionary with ``output_dir``, ``format``, ``num_images``, and
        ``num_points`` (the number of exported tracks).

    Raises:
        ValueError: On unsupported formats, camera models, or missing views.
        FileNotFoundError: If an estimated view has no image in ``image_dir``.
        ImportError: If OpenCV is needed (undistortion, resizing) and missing.
    """
    if format not in SUPPORTED_FORMATS:
        raise ValueError(
            f"Unknown format {format!r}. Use one of {SUPPORTED_FORMATS}."
        )
    os.makedirs(output_dir, exist_ok=True)
    images_out = os.path.join(output_dir, "images")
    os.makedirs(images_out, exist_ok=True)

    work = _copy_reconstruction(reconstruction, output_dir)
    view_ids = [
        vid for vid in work.ViewIds() if work.View(vid).IsEstimated()
    ]
    if not view_ids:
        raise ValueError("Reconstruction has no estimated views to export.")

    # Filter tracks on the input cameras, then drop the rest from the copy.
    kept = _collect_points(work, view_ids, max_reprojection_error, min_track_length)
    for tid in list(work.TrackIds()):
        if tid not in kept:
            work.RemoveTrack(tid)

    # Read every view's intrinsics before mutating the copy. Views in one
    # intrinsics group share the same camera object, so mutation would hide
    # the original distortion from later views.
    originals = {vid: _PinholeState(work.View(vid).Camera()) for vid in view_ids}
    scales = {}
    for vid in view_ids:
        state = originals[vid]
        longest = max(state.width, state.height)
        if max_image_size is not None and max_image_size > 0 and longest > max_image_size:
            scales[vid] = max_image_size / float(longest)
        else:
            scales[vid] = 1.0

    # Rewrite the observations and cameras of the copy: observations follow
    # the undistorted and resized images, cameras become scaled pinhole
    # intrinsics without distortion when the images are undistorted.
    for vid in view_ids:
        state = originals[vid]
        scale = scales[vid]
        new_w, new_h = _scaled_size(state.width, state.height, scale)
        view = work.MutableView(vid)
        _rewrite_observations(view, state, scale, undistort)
        camera = view.MutableCamera()
        intrinsics = camera.CameraIntrinsics()
        camera.SetImageSize(new_w, new_h)
        camera.SetFocalLength(state.fx * scale)
        camera.SetPrincipalPoint(state.cx * scale, state.cy * scale)
        if undistort:
            intrinsics.SetRadialDistortion(0.0, 0.0)

    # Images.
    maps = {}
    for vid in view_ids:
        name = work.View(vid).Name()
        src = os.path.join(image_dir, name)
        if not os.path.isfile(src):
            raise FileNotFoundError(f"Image for view {name!r} not found: {src}")
        dst = os.path.join(images_out, name)
        os.makedirs(os.path.dirname(dst), exist_ok=True)

        state = originals[vid]
        scale = scales[vid]
        new_size = _scaled_size(state.width, state.height, scale)
        if undistort and state.has_distortion:
            group = work.CameraIntrinsicsGroupIdFromViewId(vid)
            if group not in maps:
                maps[group] = undistortion_maps(
                    state.width, state.height, state.K, state.k1, state.k2
                )
            map1, map2 = maps[group]
            _remap_image(src, dst, map1, map2, new_size, (state.width, state.height))
        elif scale < 1.0:
            _resize_image(src, dst, new_size)
        else:
            shutil.copyfile(src, dst)

    # Camera and point files.
    if format == "colmap":
        sparse_dir = os.path.join(output_dir, "sparse", "0")
        os.makedirs(sparse_dir, exist_ok=True)
        if not _io.WriteColmapFiles(work, sparse_dir, binary=True):
            raise RuntimeError(f"Could not write COLMAP binary files to {sparse_dir}.")
    elif format == "transforms":
        path = os.path.join(output_dir, "transforms.json")
        if not _io.WriteTransformsJson("images", work, path):
            raise RuntimeError(f"Could not write {path}.")
    else:
        path = os.path.join(output_dir, "transforms.json")
        if not _io.WriteNerfStudio("images", work, 16, path):
            raise RuntimeError(f"Could not write {path}.")

    if write_seed_points:
        tids = sorted(kept)
        write_points_ply(
            os.path.join(output_dir, "points3d.ply"),
            [kept[t][0] for t in tids],
            [kept[t][1] for t in tids],
        )

    return {
        "output_dir": output_dir,
        "format": format,
        "num_images": len(view_ids),
        "num_points": len(kept),
    }
