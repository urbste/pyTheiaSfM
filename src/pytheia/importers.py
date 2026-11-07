"""Import reconstructions and matches written by other SfM tools.

Pure Python (``numpy`` and the standard library only). Nothing here needs the
C++ library to be rebuilt.

Functions:

* :func:`read_colmap_model` reads a COLMAP sparse model (``cameras``,
  ``images``, ``points3D`` as ``.txt`` or ``.bin``) and returns a
  :class:`pytheia.sfm.Reconstruction`. GLOMAP writes the same format.
* :func:`read_openmvg_sfm_data` reads an OpenMVG ``sfm_data.json`` (views,
  intrinsics, extrinsics and structure) into a :class:`pytheia.sfm.Reconstruction`.
* :func:`read_colmap_database` reads keypoints, matches and two-view
  geometries from a COLMAP ``database.db`` as numpy arrays.

Conventions:

* Pixel coordinates. COLMAP puts the centre of the top-left pixel at
  ``(0.5, 0.5)``, OpenCV at ``(0, 0)``. pyTheia has no convention of its own: it
  uses whatever the observations and the principal point use. The model reader
  therefore keeps COLMAP's coordinates by default; ``pixel_offset=0.5``
  subtracts 0.5 from every observation and from the principal point to get the
  OpenCV convention. OpenMVG is read without an offset. The database
  reader returns COLMAP's raw keypoints.
* Poses. COLMAP and OpenMVG store a world-to-camera rotation and a camera
  center (COLMAP stores ``tvec = -R C`` with a quaternion ``qvec`` in
  ``(w, x, y, z)`` order). pyTheia stores the same rotation and center.
* Camera models. Every COLMAP camera model is imported with all of its
  parameters, under the pyTheia model of the same name where one exists:
  ``SIMPLE_PINHOLE``, ``PINHOLE``, ``SIMPLE_RADIAL`` and ``RADIAL`` map to
  ``PINHOLE`` (radial terms ``k1``, ``k2``). ``OPENCV`` maps to
  ``PINHOLE_RADIAL_TANGENTIAL`` (``k1``, ``k2``, ``p1``, ``p2``).
  ``FULL_OPENCV`` maps to ``FULL_OPENCV`` (``k1`` to ``k6``, ``p1``, ``p2``).
  ``OPENCV_FISHEYE`` maps to ``FISHEYE`` (``k1`` to ``k4``). ``FOV`` maps to
  ``FOV`` (``omega``). ``RADIAL_FISHEYE`` maps to ``RADIAL_FISHEYE`` (``k1``,
  ``k2``) and ``SIMPLE_RADIAL_FISHEYE`` maps to the same model with ``k2 = 0``.
  ``THIN_PRISM_FISHEYE`` and ``RAD_TAN_THIN_PRISM_FISHEYE`` map to the pyTheia
  models of the same name. COLMAP's fisheye models use the same projection
  as pyTheia's ``FISHEYE``, ``FOV`` and related models, so the projections
  agree with COLMAP.
* Views are added in COLMAP image-id order (or OpenMVG view-key order). The
  COLMAP camera id (or OpenMVG intrinsic key) is used as the camera
  intrinsics group id.

Example::

    import pytheia as pt
    from pytheia.importers import read_colmap_model

    recon = read_colmap_model("scene/sparse/0")
    print(recon.NumViews(), recon.NumTracks())
"""

from __future__ import annotations

import json
import os
import sqlite3
import struct
import warnings
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from pytheia.pytheia import sfm as _sfm

__all__ = [
    "read_colmap_model",
    "read_colmap_database",
    "read_openmvg_sfm_data",
]

PIXEL_OFFSET_COLMAP = 0.5
"""COLMAP pixel centers are at +0.5 relative to OpenCV pixel centers. Pass this as
``pixel_offset`` to convert to the OpenCV convention; the default (0) keeps
COLMAP's coordinates unchanged."""

# COLMAP camera models: name -> (model id, number of parameters).
# Ids and parameter counts from colmap/src/colmap/sensor/models.h.
_COLMAP_MODELS: Dict[str, Tuple[int, int]] = {
    "SIMPLE_PINHOLE": (0, 3),
    "PINHOLE": (1, 4),
    "SIMPLE_RADIAL": (2, 4),
    "RADIAL": (3, 5),
    "OPENCV": (4, 8),
    "OPENCV_FISHEYE": (5, 8),
    "FULL_OPENCV": (6, 12),
    "FOV": (7, 5),
    "SIMPLE_RADIAL_FISHEYE": (8, 4),
    "RADIAL_FISHEYE": (9, 5),
    "THIN_PRISM_FISHEYE": (10, 12),
    "RAD_TAN_THIN_PRISM_FISHEYE": (11, 16),
}
_COLMAP_MODEL_BY_ID = {mid: name for name, (mid, _) in _COLMAP_MODELS.items()}
_COLMAP_SUPPORTED = tuple(_COLMAP_MODELS)

# pyTheia camera model used for each COLMAP model.
_COLMAP_PYTHEIA_MODEL: Dict[str, str] = {
    "SIMPLE_PINHOLE": "PINHOLE",
    "PINHOLE": "PINHOLE",
    "SIMPLE_RADIAL": "PINHOLE",
    "RADIAL": "PINHOLE",
    "OPENCV": "PINHOLE_RADIAL_TANGENTIAL",
    "OPENCV_FISHEYE": "FISHEYE",
    "FULL_OPENCV": "FULL_OPENCV",
    "FOV": "FOV",
    "SIMPLE_RADIAL_FISHEYE": "RADIAL_FISHEYE",
    "RADIAL_FISHEYE": "RADIAL_FISHEYE",
    "THIN_PRISM_FISHEYE": "THIN_PRISM_FISHEYE",
    "RAD_TAN_THIN_PRISM_FISHEYE": "RAD_TAN_THIN_PRISM_FISHEYE",
}

_SPARSE_FILES = ("cameras", "images", "points3D")
_SPARSE_SEARCH_DEPTH = 4

# COLMAP two-view geometry configurations (colmap/src/colmap/scene/two_view_geometry.h).
TWO_VIEW_CONFIGS = {
    0: "UNDEFINED",
    1: "DEGENERATE",
    2: "CALIBRATED",
    3: "UNCALIBRATED",
    4: "PLANAR",
    5: "PANORAMIC",
    6: "PLANAR_OR_PANORAMIC",
    7: "WATERMARK",
    8: "MULTIPLE",
}

# COLMAP stores image pairs as kMaxNumImages * id1 + id2 (see Database::ImagePairToPairId).
_COLMAP_MAX_NUM_IMAGES = 2147483647


def _warn(message: str) -> None:
    warnings.warn(message, UserWarning, stacklevel=3)


def _model_type(name: str):
    return getattr(_sfm.CameraIntrinsicsModelType, name)


# --------------------------------------------------------------------------- #
# Internal specifications shared by all readers
# --------------------------------------------------------------------------- #


@dataclass
class _Intrinsics:
    """Camera intrinsics in pyTheia pixel convention."""

    model_type: object
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    # Distortion terms of the models that are not pinhole models, in COLMAP's
    # order (for example FISHEYE: k1..k4, FOV: omega, FULL_OPENCV: k1, k2, p1,
    # p2, k3, k4, k5, k6). Empty for the pinhole models.
    extra: Tuple[float, ...] = ()


@dataclass
class _ViewSpec:
    name: str
    group_id: int
    intrinsics: _Intrinsics
    rotation: Optional[np.ndarray] = None  # world-to-camera, None if not registered
    center: Optional[np.ndarray] = None


@dataclass
class _TrackSpec:
    xyz: np.ndarray
    rgb: Optional[np.ndarray] = None
    # (index into the view list, x, y) with x, y in pyTheia pixel convention.
    observations: List[Tuple[int, float, float]] = field(default_factory=list)


def _set_extended_distortion(model, model_type, extra: Sequence[float]) -> None:
    """Sets the distortion of a non-pinhole model from COLMAP-ordered terms."""
    d = [float(v) for v in extra]
    if model_type == _model_type("FISHEYE"):
        model.SetRadialDistortion(d[0], d[1], d[2], d[3])
    elif model_type == _model_type("FOV"):
        model.SetRadialDistortion(d[0])
    elif model_type == _model_type("FULL_OPENCV"):
        # COLMAP order: k1, k2, p1, p2, k3, k4, k5, k6.
        model.SetRadialDistortion(d[0], d[1], d[4], d[5], d[6], d[7])
        model.SetTangentialDistortion(d[2], d[3])
    elif model_type == _model_type("RADIAL_FISHEYE"):
        model.SetRadialDistortion(d[0], d[1])
    elif model_type == _model_type("THIN_PRISM_FISHEYE"):
        # COLMAP order: k1, k2, p1, p2, k3, k4, sx1, sy1.
        model.SetRadialDistortion(d[0], d[1], d[4], d[5])
        model.SetTangentialDistortion(d[2], d[3])
        model.SetThinPrismDistortion(d[6], d[7])
    elif model_type == _model_type("RAD_TAN_THIN_PRISM_FISHEYE"):
        # COLMAP order: k0..k5, p0, p1, s0..s3.
        model.SetRadialDistortion(d[0], d[1], d[2], d[3], d[4], d[5])
        model.SetTangentialDistortion(d[6], d[7])
        model.SetThinPrismDistortion(d[8], d[9], d[10], d[11])
    else:
        raise ValueError("No distortion mapping for camera model type %s." % (model_type,))


def _configure_camera(camera, intr: _Intrinsics, rotation, center) -> None:
    # The model type must be set first: changing it creates a new model with
    # default parameters.
    camera.SetCameraIntrinsicsModelType(intr.model_type)
    camera.SetImageSize(int(intr.width), int(intr.height))
    camera.SetFocalLength(float(intr.fx))
    camera.SetPrincipalPoint(float(intr.cx), float(intr.cy))
    model = camera.CameraIntrinsics()
    model.SetAspectRatio(float(intr.fy) / float(intr.fx))
    if hasattr(model, "SetSkew"):
        model.SetSkew(0.0)
    if intr.model_type == _model_type("PINHOLE_RADIAL_TANGENTIAL"):
        model.SetRadialDistortion(float(intr.k1), float(intr.k2), float(intr.k3))
        model.SetTangentialDistortion(float(intr.p1), float(intr.p2))
    elif intr.model_type == _model_type("PINHOLE"):
        model.SetRadialDistortion(float(intr.k1), float(intr.k2))
    else:
        _set_extended_distortion(model, intr.model_type, intr.extra)
    if rotation is not None:
        camera.SetOrientationFromRotationMatrix(np.asarray(rotation, dtype=float))
        camera.SetPosition(np.asarray(center, dtype=float))


def _build_reconstruction(views: Sequence[_ViewSpec], tracks: Sequence[_TrackSpec]):
    recon = _sfm.Reconstruction()
    view_ids = []
    for index, spec in enumerate(views):
        # Timestamps must be unique: use the view index.
        view_id = recon.AddView(spec.name, int(spec.group_id), float(index))
        if view_id == _sfm.kInvalidViewId:
            raise ValueError(
                "Could not add view %r (duplicate or empty image name)." % spec.name
            )
        view = recon.MutableView(view_id)
        _configure_camera(
            view.MutableCamera(), spec.intrinsics, spec.rotation, spec.center
        )
        view.SetIsEstimated(spec.rotation is not None)
        view_ids.append(view_id)

    duplicates = 0
    for spec in tracks:
        track_id = recon.AddTrack()
        track = recon.MutableTrack(track_id)
        track.SetPoint(np.array([spec.xyz[0], spec.xyz[1], spec.xyz[2], 1.0]))
        if spec.rgb is not None:
            track.SetColor(np.asarray(spec.rgb, dtype=np.uint8))
        track.SetIsEstimated(True)
        seen_views = set()
        for view_index, x, y in spec.observations:
            # A track holds at most one observation per view. Some tools (COLMAP
            # can, for duplicated features) write more; keep the first.
            if view_index in seen_views:
                duplicates += 1
                continue
            seen_views.add(view_index)
            recon.AddObservation(
                view_ids[view_index],
                track_id,
                _sfm.Feature(np.array([x, y], dtype=float)),
            )
    if duplicates:
        _warn(
            "%d observations repeat a view already observing the same track; "
            "only the first was kept." % duplicates
        )
    return recon


def _quat_to_rotation(qvec: np.ndarray) -> np.ndarray:
    """COLMAP quaternion (w, x, y, z) to a world-to-camera rotation matrix."""
    w, x, y, z = np.asarray(qvec, dtype=float) / np.linalg.norm(qvec)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# --------------------------------------------------------------------------- #
# COLMAP sparse model
# --------------------------------------------------------------------------- #


@dataclass
class _ColmapCamera:
    camera_id: int
    model: str
    width: int
    height: int
    params: Tuple[float, ...]


@dataclass
class _ColmapImage:
    image_id: int
    qvec: np.ndarray
    tvec: np.ndarray
    camera_id: int
    name: str
    xy: np.ndarray  # (N, 2) COLMAP pixel coordinates
    point3d_ids: np.ndarray  # (N,) int, -1 when not triangulated


@dataclass
class _ColmapPoint:
    xyz: np.ndarray
    rgb: np.ndarray
    error: float


def _find_colmap_sparse_dir(path: str) -> Tuple[Path, str]:
    """Returns (directory, "bin" or "txt") for a COLMAP sparse model.

    Searches ``path`` itself, then ``sparse/0``, ``sparse``, ``0`` and finally
    the subfolders of ``path`` (breadth first, up to a few levels). A folder
    qualifies when it holds ``cameras``, ``images`` and ``points3D`` in one
    format. Binary files are preferred when both formats are present.
    """

    def _kind(directory: Path) -> Optional[str]:
        for ext in ("bin", "txt"):
            if all((directory / (name + "." + ext)).is_file() for name in _SPARSE_FILES):
                return ext
        return None

    root = Path(path)
    if not root.is_dir():
        raise FileNotFoundError("COLMAP model directory not found: %s" % path)

    for candidate in (root, root / "sparse" / "0", root / "sparse", root / "0"):
        kind = _kind(candidate)
        if kind:
            return candidate, kind

    for dirpath, dirnames, _ in os.walk(root):
        depth = len(Path(dirpath).relative_to(root).parts)
        if depth >= _SPARSE_SEARCH_DEPTH:
            dirnames[:] = []
        dirnames.sort()
        kind = _kind(Path(dirpath))
        if kind:
            return Path(dirpath), kind

    raise FileNotFoundError(
        "No COLMAP sparse model (cameras, images, points3D as .txt or .bin) "
        "found under %s" % path
    )


def _read_text_lines(path: Path) -> List[str]:
    with open(path, encoding="utf-8") as f:
        return [line.rstrip("\r\n") for line in f if not line.startswith("#")]


def _parse_cameras_txt(path: Path) -> Dict[int, _ColmapCamera]:
    cameras = {}
    for line in _read_text_lines(path):
        if not line.strip():
            continue
        parts = line.split()
        camera_id, model = int(parts[0]), parts[1]
        width, height = int(float(parts[2])), int(float(parts[3]))
        params = tuple(float(v) for v in parts[4:])
        cameras[camera_id] = _ColmapCamera(camera_id, model, width, height, params)
    return cameras


def _parse_images_txt(path: Path) -> List[_ColmapImage]:
    # Each image is two lines: a header and the 2D points. The points line can
    # be empty, so blank lines are kept and consumed in pairs.
    lines = _read_text_lines(path)
    images = []
    i = 0
    while i < len(lines):
        if not lines[i].strip():
            i += 1
            continue
        header = lines[i].split(None, 9)
        points_line = lines[i + 1] if i + 1 < len(lines) else ""
        i += 2
        if len(header) < 10:
            raise ValueError("Malformed images.txt header: %r" % header)
        tokens = points_line.split()
        if len(tokens) % 3 != 0:
            raise ValueError("Malformed 2D point list for image %s" % header[9])
        xy = np.array(
            [[float(v) for v in tokens[0::3]], [float(v) for v in tokens[1::3]]],
            dtype=float,
        ).T.reshape(-1, 2)
        point3d_ids = np.array([int(v) for v in tokens[2::3]], dtype=np.int64)
        images.append(
            _ColmapImage(
                image_id=int(header[0]),
                qvec=np.array([float(v) for v in header[1:5]]),
                tvec=np.array([float(v) for v in header[5:8]]),
                camera_id=int(header[8]),
                name=header[9],
                xy=xy,
                point3d_ids=point3d_ids,
            )
        )
    return images


def _parse_points_txt(path: Path) -> Dict[int, _ColmapPoint]:
    points = {}
    for line in _read_text_lines(path):
        if not line.strip():
            continue
        parts = line.split()
        point_id = int(parts[0])
        points[point_id] = _ColmapPoint(
            xyz=np.array([float(v) for v in parts[1:4]]),
            rgb=np.array([int(v) for v in parts[4:7]], dtype=np.uint8),
            error=float(parts[7]),
        )
    return points


def _read_colmap_bin(directory: Path):
    cameras: Dict[int, _ColmapCamera] = {}
    data = (directory / "cameras.bin").read_bytes()
    (num,) = struct.unpack_from("<Q", data, 0)
    offset = 8
    for _ in range(num):
        camera_id, model_id, width, height = struct.unpack_from("<IiQQ", data, offset)
        offset += 24
        if model_id not in _COLMAP_MODEL_BY_ID:
            raise ValueError("Unknown COLMAP camera model id %d" % model_id)
        model = _COLMAP_MODEL_BY_ID[model_id]
        n = _COLMAP_MODELS[model][1]
        params = struct.unpack_from("<%dd" % n, data, offset)
        offset += 8 * n
        cameras[camera_id] = _ColmapCamera(camera_id, model, int(width), int(height), params)
    if offset != len(data):
        raise ValueError("Unexpected trailing bytes in %s" % (directory / "cameras.bin"))

    images: List[_ColmapImage] = []
    data = (directory / "images.bin").read_bytes()
    (num,) = struct.unpack_from("<Q", data, 0)
    offset = 8
    for _ in range(num):
        image_id = struct.unpack_from("<I", data, offset)[0]
        qvec = np.array(struct.unpack_from("<4d", data, offset + 4))
        tvec = np.array(struct.unpack_from("<3d", data, offset + 36))
        camera_id = struct.unpack_from("<I", data, offset + 60)[0]
        name_end = data.index(b"\x00", offset + 64)
        name = data[offset + 64:name_end].decode("utf-8")
        offset = name_end + 1
        (num_points,) = struct.unpack_from("<Q", data, offset)
        offset += 8
        points = np.frombuffer(
            data, dtype=np.dtype([("x", "<f8"), ("y", "<f8"), ("id", "<i8")]),
            count=num_points, offset=offset,
        )
        offset += 24 * num_points
        images.append(
            _ColmapImage(
                image_id=int(image_id),
                qvec=qvec,
                tvec=tvec,
                camera_id=int(camera_id),
                name=name,
                xy=np.stack([points["x"], points["y"]], axis=1).astype(float),
                point3d_ids=points["id"].astype(np.int64),
            )
        )
    if offset != len(data):
        raise ValueError("Unexpected trailing bytes in %s" % (directory / "images.bin"))

    points3d: Dict[int, _ColmapPoint] = {}
    data = (directory / "points3D.bin").read_bytes()
    (num,) = struct.unpack_from("<Q", data, 0)
    offset = 8
    header = struct.Struct("<Q3d3BdQ")
    for _ in range(num):
        point_id, x, y, z, r, g, b, error, track_len = header.unpack_from(data, offset)
        offset += header.size + 8 * track_len  # each track element is (uint32, uint32)
        points3d[int(point_id)] = _ColmapPoint(
            xyz=np.array([x, y, z]),
            rgb=np.array([r, g, b], dtype=np.uint8),
            error=float(error),
        )
    if offset != len(data):
        raise ValueError("Unexpected trailing bytes in %s" % (directory / "points3D.bin"))
    return cameras, images, points3d


def _colmap_intrinsics(camera: _ColmapCamera, pixel_offset: float = 0.0) -> _Intrinsics:
    """Maps a COLMAP camera to pyTheia intrinsics, shifting the principal point by ``pixel_offset``."""
    if camera.model not in _COLMAP_SUPPORTED:
        raise ValueError(
            "COLMAP camera model %s (camera %d) is not supported by pytheia.importers. "
            "Supported: %s." % (camera.model, camera.camera_id, ", ".join(_COLMAP_SUPPORTED))
        )
    p = [float(v) for v in camera.params]
    name = camera.model
    k1 = k2 = k3 = p1 = p2 = 0.0
    extra: Tuple[float, ...] = ()
    if name == "SIMPLE_PINHOLE":
        fx = fy = p[0]
        cx, cy = p[1], p[2]
    elif name == "PINHOLE":
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
    elif name == "SIMPLE_RADIAL":
        fx = fy = p[0]
        cx, cy = p[1], p[2]
        k1 = p[3]
    elif name == "RADIAL":
        fx = fy = p[0]
        cx, cy = p[1], p[2]
        k1, k2 = p[3], p[4]
    elif name == "OPENCV":
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        k1, k2, p1, p2 = p[4], p[5], p[6], p[7]
    elif name == "SIMPLE_RADIAL_FISHEYE":
        # COLMAP: f, cx, cy, k. The same model as RADIAL_FISHEYE with k2 = 0.
        fx = fy = p[0]
        cx, cy = p[1], p[2]
        extra = (p[3], 0.0)
    elif name == "RADIAL_FISHEYE":
        # COLMAP: f, cx, cy, k1, k2.
        fx = fy = p[0]
        cx, cy = p[1], p[2]
        extra = (p[3], p[4])
    elif name == "FOV":
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        extra = (p[4],)
    else:
        # OPENCV_FISHEYE, FULL_OPENCV, THIN_PRISM_FISHEYE and
        # RAD_TAN_THIN_PRISM_FISHEYE: fx, fy, cx, cy followed by the distortion terms.
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
        extra = tuple(p[4:])

    if name in ("SIMPLE_PINHOLE", "PINHOLE", "SIMPLE_RADIAL", "RADIAL"):
        model_type = _model_type("PINHOLE")
    elif name == "OPENCV":
        model_type = _model_type("PINHOLE_RADIAL_TANGENTIAL")
    else:
        model_type = _model_type(_COLMAP_PYTHEIA_MODEL[name])

    return _Intrinsics(
        model_type=model_type,
        width=camera.width,
        height=camera.height,
        fx=fx,
        fy=fy,
        cx=cx - pixel_offset,
        cy=cy - pixel_offset,
        k1=k1,
        k2=k2,
        k3=k3,
        p1=p1,
        p2=p2,
        extra=extra,
    )


def read_colmap_model(path: str, pixel_offset: float = 0.0):
    """Reads a COLMAP sparse model into a :class:`pytheia.sfm.Reconstruction`.

    Args:
        path: The sparse model folder, or a folder above it. The folder is
            found by checking ``path``, ``path/sparse/0``, ``path/sparse``,
            ``path/0`` and then the subfolders (up to a few levels). A folder
            qualifies when it contains ``cameras``, ``images`` and
            ``points3D`` in ``.txt`` or ``.bin`` format.
        pixel_offset: Subtracted from every observation and from the principal
            point. The default 0 keeps COLMAP's pixel coordinates (centre of the
            top-left pixel at 0.5), which is what pyTheia needs when the
            features come from the COLMAP database. Pass
            ``PIXEL_OFFSET_COLMAP`` (0.5) for the OpenCV convention.

    Returns:
        A reconstruction with one view per registered image (estimated, with
        pose), one track per 3D point (with position and color), and one
        observation per 2D point that refers to a 3D point. View ids follow
        the COLMAP image order. The COLMAP camera id is the camera
        intrinsics group id.

    Raises:
        FileNotFoundError: No sparse model was found.
        ValueError: The files are malformed or use an unsupported camera model.
    """
    directory, kind = _find_colmap_sparse_dir(path)
    if kind == "bin":
        cameras, images, points3d = _read_colmap_bin(directory)
    else:
        cameras = _parse_cameras_txt(directory / "cameras.txt")
        images = _parse_images_txt(directory / "images.txt")
        points3d = _parse_points_txt(directory / "points3D.txt")

    images = sorted(images, key=lambda im: im.image_id)
    intrinsics_cache: Dict[int, _Intrinsics] = {}
    views: List[_ViewSpec] = []
    image_index: Dict[int, int] = {}
    for image in images:
        if image.camera_id not in cameras:
            raise ValueError(
                "Image %s refers to unknown camera %d" % (image.name, image.camera_id)
            )
        if image.camera_id not in intrinsics_cache:
            intrinsics_cache[image.camera_id] = _colmap_intrinsics(cameras[image.camera_id], pixel_offset)
        rotation = _quat_to_rotation(image.qvec)
        center = -rotation.T @ image.tvec
        image_index[image.image_id] = len(views)
        views.append(
            _ViewSpec(
                name=image.name,
                group_id=image.camera_id,
                intrinsics=intrinsics_cache[image.camera_id],
                rotation=rotation,
                center=center,
            )
        )

    point_ids = sorted(points3d)
    track_index = {point_id: i for i, point_id in enumerate(point_ids)}
    tracks = [
        _TrackSpec(xyz=points3d[pid].xyz, rgb=points3d[pid].rgb) for pid in point_ids
    ]
    dropped = 0
    for image in images:
        view_index = image_index[image.image_id]
        for (x, y), point_id in zip(image.xy, image.point3d_ids):
            if point_id < 0:
                continue
            if point_id not in track_index:
                dropped += 1
                continue
            tracks[track_index[point_id]].observations.append(
                (view_index, x - pixel_offset, y - pixel_offset)
            )
    if dropped:
        _warn(
            "%d 2D points refer to 3D point ids missing from points3D; they were skipped."
            % dropped
        )

    return _build_reconstruction(views, tracks)


# --------------------------------------------------------------------------- #
# OpenMVG sfm_data.json
# --------------------------------------------------------------------------- #

# Distortion keys written by OpenMVG for the pinhole intrinsics that are read.
# disto_k1 (radial, one coefficient), disto_k3 ([k1, k2, k3]) and
# disto_t2 ([k1, k2, k3, d1, d2, t1, t2], Brown). The layout of disto_t2 is
# not verified against OpenMVG source here, see docs/content/importers.md.
_OPENMVG_DISTO_KEYS = ("disto_k1", "disto_k3", "disto_t2")


def _collect_ptr_wrappers(node, registry: Dict[int, Tuple[str, dict]]) -> None:
    """Records every cereal ``ptr_wrapper`` definition (one that has ``data``)."""
    stack = [node]
    while stack:
        current = stack.pop()
        if isinstance(current, dict):
            wrapper = current.get("ptr_wrapper")
            if isinstance(wrapper, dict) and "data" in wrapper and "id" in wrapper:
                registry[wrapper["id"]] = (current.get("polymorphic_name", ""), wrapper["data"])
            stack.extend(current.values())
        elif isinstance(current, list):
            stack.extend(current)


def _unwrap(value, registry: Dict[int, Tuple[str, dict]]) -> Tuple[str, dict]:
    """Resolves an OpenMVG ``{"polymorphic_id", "ptr_wrapper": {...}}`` value.

    Returns ``(polymorphic_name, data)``. A ``ptr_wrapper`` that holds only an
    ``id`` refers to an earlier definition, which is looked up in ``registry``.
    """
    if isinstance(value, dict) and "ptr_wrapper" in value:
        wrapper = value["ptr_wrapper"]
        if "data" in wrapper:
            return value.get("polymorphic_name", ""), wrapper["data"]
        ref = wrapper.get("id")
        if ref not in registry:
            raise ValueError("OpenMVG sfm_data: unresolved pointer id %r" % (ref,))
        return registry[ref]
    return "", value


def _openmvg_intrinsics(name: str, data: dict) -> _Intrinsics:
    lname = name.lower()
    if "fisheye" in lname or "spherical" in lname or "disto_k4" in data:
        raise ValueError(
            "OpenMVG intrinsics %r (%s) are not supported by pytheia.importers."
            % (name or "unnamed", sorted(data))
        )
    unknown = [k for k in data if k.startswith("disto_") and k not in _OPENMVG_DISTO_KEYS]
    if unknown:
        raise ValueError(
            "OpenMVG intrinsics use unsupported distortion keys %s." % unknown
        )

    focal = float(data["focal_length"])
    cx, cy = (float(v) for v in data["principal_point"])
    k1 = k2 = k3 = p1 = p2 = 0.0
    if "disto_k1" in data:
        k1 = float(np.ravel(data["disto_k1"])[0])
    if "disto_k3" in data:
        coeffs = np.ravel(np.asarray(data["disto_k3"], dtype=float))
        if coeffs.size != 3:
            raise ValueError("OpenMVG disto_k3 must hold 3 values, got %d" % coeffs.size)
        k1, k2, k3 = coeffs
    if "disto_t2" in data:
        coeffs = np.ravel(np.asarray(data["disto_t2"], dtype=float))
        if coeffs.size != 7:
            raise ValueError("OpenMVG disto_t2 must hold 7 values, got %d" % coeffs.size)
        k1, k2, k3, d1, d2, p1, p2 = coeffs
        if d1 != 0.0 or d2 != 0.0:
            _warn("OpenMVG Brown intrinsics: decentering terms d1, d2 were dropped.")

    model = "PINHOLE_RADIAL_TANGENTIAL" if (k3 != 0.0 or p1 != 0.0 or p2 != 0.0) else "PINHOLE"
    return _Intrinsics(
        model_type=_model_type(model),
        width=int(data.get("width", 0) or 0),
        height=int(data.get("height", 0) or 0),
        fx=focal,
        fy=focal,
        cx=cx,
        cy=cy,
        k1=float(k1),
        k2=float(k2),
        k3=float(k3),
        p1=float(p1),
        p2=float(p2),
    )


def read_openmvg_sfm_data(json_path: str):
    """Reads an OpenMVG ``sfm_data.json`` into a :class:`pytheia.sfm.Reconstruction`.

    Handles the ``ptr_wrapper`` encoding (a pointer is defined once with
    ``data`` and may be referenced later by ``id``) and the ``polymorphic_id``
    fields. ``root_path`` is not needed for the reconstruction, so view names
    are the image paths relative to it (``local_path/filename``).

    Views become estimated when they have an extrinsic (pose) entry. Structure
    entries become tracks with their observations. Observations are read in
    pixel coordinates without an offset.

    Args:
        json_path: Path to ``sfm_data.json`` (the text format, not the cereal
            ``sfm_data.bin``).

    Returns:
        The reconstruction. The OpenMVG intrinsic key is the camera intrinsics
        group id.

    Raises:
        ValueError: The file is malformed or uses an unsupported camera model
            (fisheye, spherical, unknown distortion keys).
    """
    with open(json_path, encoding="utf-8") as f:
        data = json.load(f)

    registry: Dict[int, Tuple[str, dict]] = {}
    _collect_ptr_wrappers(data, registry)

    intrinsics: Dict[int, _Intrinsics] = {}
    for entry in data.get("intrinsics", []):
        name, payload = _unwrap(entry["value"], registry)
        intrinsics[int(entry["key"])] = _openmvg_intrinsics(name, payload)

    extrinsics: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for entry in data.get("extrinsics", []):
        value = entry["value"]
        rotation = np.asarray(value["rotation"], dtype=float).reshape(3, 3)
        center = np.asarray(value["center"], dtype=float).reshape(3)
        extrinsics[int(entry["key"])] = (rotation, center)

    raw_views = []
    for entry in data.get("views", []):
        _, payload = _unwrap(entry["value"], registry)
        raw_views.append((int(entry["key"]), payload))
    raw_views.sort(key=lambda item: item[0])

    views: List[_ViewSpec] = []
    view_index: Dict[int, int] = {}
    for key, payload in raw_views:
        view_id = int(payload.get("id_view", key))
        intrinsic_key = int(payload["id_intrinsic"])
        if intrinsic_key not in intrinsics:
            raise ValueError(
                "OpenMVG view %s refers to missing intrinsics %d" % (payload.get("filename"), intrinsic_key)
            )
        width = int(payload.get("width", 0) or 0)
        height = int(payload.get("height", 0) or 0)
        intr = intrinsics[intrinsic_key]
        intr = replace(
            intr,
            width=intr.width or width,
            height=intr.height or height,
        )
        local_path = payload.get("local_path", "") or ""
        filename = payload["filename"]
        name = "/".join(part for part in (local_path, filename) if part)

        rotation = center = None
        pose_key = payload.get("id_pose")
        if pose_key is not None and int(pose_key) in extrinsics:
            rotation, center = extrinsics[int(pose_key)]

        view_index[view_id] = len(views)
        views.append(
            _ViewSpec(
                name=name,
                group_id=intrinsic_key,
                intrinsics=intr,
                rotation=rotation,
                center=center,
            )
        )

    tracks: List[_TrackSpec] = []
    dropped = 0
    for entry in data.get("structure", []):
        value = entry["value"]
        observations = []
        for obs in value.get("observations", []):
            obs_value = obs["value"]
            obs_view = int(obs_value.get("id_view", obs["key"]))
            if obs_view not in view_index:
                dropped += 1
                continue
            x, y = (float(v) for v in obs_value["x"])
            observations.append((view_index[obs_view], x, y))
        color = value.get("color")
        tracks.append(
            _TrackSpec(
                xyz=np.asarray(value["X"], dtype=float).reshape(3),
                rgb=None if color is None else np.clip(np.round(np.asarray(color, dtype=float)), 0, 255).astype(np.uint8),
                observations=observations,
            )
        )
    if dropped:
        _warn("%d OpenMVG observations refer to unknown views; they were skipped." % dropped)

    return _build_reconstruction(views, tracks)


# --------------------------------------------------------------------------- #
# COLMAP database.db
# --------------------------------------------------------------------------- #


def _decode_pair_id(pair_id: int) -> Tuple[int, int]:
    id2 = pair_id % _COLMAP_MAX_NUM_IMAGES
    id1 = (pair_id - id2) // _COLMAP_MAX_NUM_IMAGES
    return int(id1), int(id2)


def _blob_array(blob, rows, cols, dtype: str) -> np.ndarray:
    """Decodes a blob of ``rows * cols`` values stored in row-major order."""
    dt = np.dtype(dtype)
    if blob is None or rows == 0:
        return np.zeros((0, cols), dtype=dt)
    expected = rows * cols * dt.itemsize
    if len(blob) != expected:
        raise ValueError("Blob has %d bytes, expected %d" % (len(blob), expected))
    return np.frombuffer(blob, dtype=dt).reshape(rows, cols).copy()


def read_colmap_database(path: str) -> Dict[str, object]:
    """Reads keypoints, matches and two-view geometries from a COLMAP ``database.db``.

    The file is opened read-only. Blobs are decoded as COLMAP writes them:

    * keypoints: ``float32`` matrix with ``rows x cols``, ``cols`` 2
      (``x, y``), 4 (``x, y, scale, orientation``) or 6 (``x, y`` and the
      affine shape ``a11, a12, a21, a22``). Values are in COLMAP pixel
      coordinates (centre of the top-left pixel at 0.5).
    * matches: ``uint32`` pairs of keypoint indices.
    * two-view geometries: ``F``, ``E`` and ``H`` are ``float64`` 3x3
      matrices stored row-major, ``qvec`` (``w, x, y, z``) and ``tvec`` are
      the relative pose, and ``data`` holds the inlier matches.

    Args:
        path: Path to ``database.db``.

    Returns:
        A dict with these keys:

        * ``"images"``: ``{image_id: name}``
        * ``"keypoints"``: ``{image_id: float32 array (N, cols)}``
        * ``"matches"``: ``{(image_id1, image_id2): uint32 array (M, 2)}``
        * ``"two_view_geometries"``: ``{(image_id1, image_id2): dict}``. Each
          dict has ``config`` (int), ``config_name`` (str), ``F``, ``E``,
          ``H`` (3x3 float64 or ``None`` when empty), ``qvec`` and ``tvec``
          (or ``None``), and ``inlier_matches`` (uint32 array (K, 2)).

        Pair keys are ordered as COLMAP stores them (``image_id1 < image_id2``).

    Raises:
        FileNotFoundError: The database does not exist.
        ValueError: A blob size does not match its row and column counts.
    """
    db_path = Path(path)
    if not db_path.is_file():
        raise FileNotFoundError("COLMAP database not found: %s" % path)

    uri = db_path.resolve().as_uri() + "?mode=ro"
    con = sqlite3.connect(uri, uri=True)
    try:
        images = {int(i): str(name) for i, name in con.execute("SELECT image_id, name FROM images")}

        keypoints = {}
        for image_id, rows, cols, blob in con.execute(
            "SELECT image_id, rows, cols, data FROM keypoints"
        ):
            keypoints[int(image_id)] = _blob_array(blob, rows, cols, "<f4")

        matches = {}
        for pair_id, rows, cols, blob in con.execute(
            "SELECT pair_id, rows, cols, data FROM matches"
        ):
            matches[_decode_pair_id(pair_id)] = _blob_array(blob, rows, 2, "<u4")

        two_view = {}
        query = (
            "SELECT pair_id, rows, cols, data, config, F, E, H, qvec, tvec "
            "FROM two_view_geometries"
        )
        for pair_id, rows, cols, data, config, f_blob, e_blob, h_blob, q_blob, t_blob in con.execute(query):
            config = int(config)
            entry = {
                "config": config,
                "config_name": TWO_VIEW_CONFIGS.get(config, "UNKNOWN"),
                "inlier_matches": _blob_array(data, rows, 2, "<u4"),
            }
            for key, blob in (("F", f_blob), ("E", e_blob), ("H", h_blob)):
                # COLMAP writes these matrices row-major (it transposes them
                # before storing), so a plain reshape gives the matrix.
                arr = _blob_array(blob, 3, 3, "<f8") if blob else None
                entry[key] = None if arr is None or not np.any(arr) else arr
            entry["qvec"] = np.frombuffer(q_blob, dtype="<f8").copy() if q_blob else None
            entry["tvec"] = np.frombuffer(t_blob, dtype="<f8").copy() if t_blob else None
            two_view[_decode_pair_id(pair_id)] = entry
    finally:
        con.close()

    return {
        "images": images,
        "keypoints": keypoints,
        "matches": matches,
        "two_view_geometries": two_view,
    }
