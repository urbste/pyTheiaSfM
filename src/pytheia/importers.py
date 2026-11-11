"""Import reconstructions and matches written by other SfM tools.

Functions:

* :func:`read_colmap_model` reads a COLMAP sparse model (``cameras``,
  ``images``, ``points3D`` as ``.txt`` or ``.bin``) and returns a
  :class:`pytheia.sfm.Reconstruction`. GLOMAP writes the same format.
* :func:`read_openmvg_sfm_data` reads an OpenMVG ``sfm_data.json`` (views,
  intrinsics, extrinsics and structure) into a :class:`pytheia.sfm.Reconstruction`.
* :func:`read_colmap_database` reads keypoints, matches and two-view
  geometries from a COLMAP ``database.db`` as numpy arrays.

The two reconstruction readers wrap the C++ readers ``theia::ReadColmapFiles``
and ``theia::ReadOpenMVGSfMData`` (``pt.io.ReadColmapFiles``,
``pt.io.ReadOpenMVGSfMData``) and turn their messages into exceptions and
warnings. The database reader is pure Python (``sqlite3`` and ``numpy``), so
the C++ library does not depend on SQLite.

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

import sqlite3
import warnings
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

from pytheia.pytheia import io as _io

__all__ = [
    "read_colmap_model",
    "read_colmap_database",
    "read_openmvg_sfm_data",
]

PIXEL_OFFSET_COLMAP = 0.5
"""COLMAP pixel centers are at +0.5 relative to OpenCV pixel centers. Pass this as
``pixel_offset`` to convert to the OpenCV convention; the default (0) keeps
COLMAP's coordinates unchanged."""

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


def _check_import(result):
    """Returns the reconstruction of a C++ reader, or raises on failure.

    The C++ readers return ``(success, reconstruction, messages)``. Warnings
    become :class:`UserWarning`; a failure becomes :class:`FileNotFoundError`
    when the input is missing and :class:`ValueError` otherwise.
    """
    success, recon, messages = result
    for message in messages.warnings:
        warnings.warn(message, UserWarning, stacklevel=3)
    if not success:
        if messages.file_not_found:
            raise FileNotFoundError(messages.error)
        raise ValueError(messages.error)
    return recon


# --------------------------------------------------------------------------- #
# COLMAP sparse model and OpenMVG sfm_data.json (C++ readers)
# --------------------------------------------------------------------------- #


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
    return _check_import(_io.ReadColmapFiles(str(path), float(pixel_offset)))


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
        FileNotFoundError: The file does not exist.
        ValueError: The file is malformed or uses an unsupported camera model
            (fisheye, spherical, unknown distortion keys).
    """
    return _check_import(_io.ReadOpenMVGSfMData(str(json_path)))


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
