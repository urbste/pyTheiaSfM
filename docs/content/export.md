# Gaussian splatting export

pyTheia writes reconstructions in the layouts that Gaussian splatting trainers read. Three pieces are involved:

| Piece | Where | What it writes |
|-------|-------|----------------|
| `WriteColmapFiles(..., binary=True)` | `pt.io` | COLMAP binary model: `cameras.bin`, `images.bin`, `points3D.bin` |
| `WriteTransformsJson(...)` | `pt.io` | `transforms.json` (instant-ngp / gsplat / Nerfstudio convention) |
| `export_for_gaussian_splatting(...)` | `pytheia.export` | Undistorted `images/`, `sparse/0/`, and a seed `points3d.ply` in one call |

Feature detection and matching stay outside the library. The exporter only needs a reconstruction and the folder with the input images.

## One-call export

```python
import pytheia as pt
from pytheia.export import export_for_gaussian_splatting

# recon: a pt.sfm.Reconstruction after SfM
info = export_for_gaussian_splatting(
    recon,
    output_dir="scene_gs",
    image_dir="/path/to/images",
    undistort=True,          # undistort radial distortion to a pinhole model
    max_image_size=-1,       # e.g. 1600 scales the longer side down to 1600 px
    format="colmap",         # "colmap" | "transforms" | "nerfstudio"
    max_reprojection_error=4.0,  # px, mean over the track; None disables
    min_track_length=2,      # minimum number of estimated views per track
    write_seed_points=True,  # write points3d.ply
)
print(info)  # {'output_dir': ..., 'format': 'colmap', 'num_images': ..., 'num_points': ...}
```

The input reconstruction is not modified. The exporter works on a copy.

### Output layout

With `format="colmap"` (default):

```text
scene_gs/
    images/                  undistorted (and resized) images, same relative names
    sparse/0/cameras.bin     PINHOLE / SIMPLE_PINHOLE (no distortion when undistort=True)
    sparse/0/images.bin      poses (world-to-camera) and observations in the exported images
    sparse/0/points3D.bin    points that pass the filters, with mean reprojection error
    points3d.ply             seed cloud: x y z nx ny nz red green blue (binary little-endian)
```

With `format="transforms"`, the `sparse/` folder is replaced by `transforms.json`. With `format="nerfstudio"`, the file is written by `pt.io.WriteNerfStudio` (see the note below).

### What the exporter does

1. Copies the reconstruction (through the binary `.recon` format) and keeps estimated views and tracks.
2. Drops tracks whose mean reprojection error (measured with the input cameras) is above `max_reprojection_error`, or that have fewer than `min_track_length` estimated observations.
3. Undistorts the observations with the same pinhole intrinsics as the undistorted image. The features are updated in place, so they stay consistent with `images/`.
4. Scales the intrinsics if `max_image_size` is set. Images are resized with area interpolation.
5. Writes the cameras, images, points, and seed PLY.

Undistortion uses the pyTheia PINHOLE model: `r_d = r_u (1 + k1 r^2 + k2 r^4)` on normalized coordinates, with no tangential terms. This is the OpenCV model with `k1, k2` only. Cameras with zero `k1, k2` are copied without remapping.

### Requirements

- Only **PINHOLE** cameras are supported. Other camera models raise `ValueError`.
- Undistortion and resizing need `numpy` and OpenCV (`opencv-python-headless`). Resizing falls back to Pillow if OpenCV is missing. Pure copies need nothing extra.

## Lower-level writers

### COLMAP binary

```python
import os
import pytheia as pt

os.makedirs("sparse/0", exist_ok=True)   # the writer does not create the folder
pt.io.WriteColmapFiles(recon, "sparse/0", binary=True)   # cameras.bin, images.bin, points3D.bin
pt.io.WriteColmapFiles(recon, "sparse/0")                # text: cameras.txt, images.txt, points3D.txt
```

`binary` defaults to `False`, so existing calls keep the text output.

The binary files follow the COLMAP format and are little-endian:

- `cameras.bin`: `uint64 num_cameras`, then per camera `uint32 camera_id`, `int32 model_id`, `uint64 width`, `uint64 height`, and `double params[]`. Models: `SIMPLE_PINHOLE` (`f, cx, cy`) and `PINHOLE` (`fx, fy, cx, cy`) when there is no distortion, otherwise `OPENCV` (`fx, fy, cx, cy, k1, k2, 0, 0`).
- `images.bin`: `uint64 num_images`, then per image `uint32 image_id`, `double qvec[4]` (w, x, y, z), `double tvec[3]`, `uint32 camera_id`, the name with a null terminator, `uint64 num_points2D`, and per point `double x`, `double y`, `int64 point3D_id` (-1 if none).
- `points3D.bin`: `uint64 num_points`, then per point `uint64 point3D_id`, `double xyz[3]`, `uint8 rgb[3]`, `double error` (mean reprojection error in px), `uint64 track_length`, and `(uint32 image_id, uint32 point2D_idx)` per observation.

Image and point ids are the pyTheia view and track ids, as in the text output.

### transforms.json convention {#transforms-json-convention}

`pt.io.WriteTransformsJson(path_to_images, reconstruction, out_json_file)` writes:

- `transform_matrix`: a 4x4 **camera-to-world** matrix `[R_c2w | C; 0 0 0 1]` in the world frame of the reconstruction. `C` is the camera center. `R_c2w` is the transpose of the world-to-camera rotation, so `x_cam = R (X - C)` for a world point `X`.
- **OpenGL camera axes**: x points right, y up, and z backwards (the camera looks along `-z`). This is the OpenCV camera-to-world matrix with its 2nd and 3rd columns negated.
- Per frame: `file_path` (`path_to_images + "/" + view name`), `w`, `h`, `fl_x`, `fl_y`, `cx`, `cy`, `k1`, `k2`, `p1`, `p2`, and `camera_model` (`"PINHOLE"` or `"OPENCV"`). The top level also has the intrinsics when all views share one camera group.

To project a world point into a frame, invert the matrix and flip the axes back to OpenCV:

```python
import numpy as np

c2w = np.array(frame["transform_matrix"])
x_gl = np.linalg.inv(c2w) @ np.append(X, 1.0)
x_cv = np.array([x_gl[0], -x_gl[1], -x_gl[2]])       # OpenCV camera frame
u = frame["fl_x"] * x_cv[0] / x_cv[2] + frame["cx"]  # (plus distortion if k1, k2 != 0)
v = frame["fl_y"] * x_cv[1] / x_cv[2] + frame["cy"]
```

The test suite checks this convention against `ProjectPoint` (`pytests/io/test_export.py`).

Note: `pt.io.WriteNerfStudio` applies Nerfstudio's additional COLMAP-to-Nerfstudio world permutation. Its matrices therefore differ from `WriteTransformsJson` by that world transform. Use `WriteTransformsJson` for instant-ngp and gsplat, and `WriteNerfStudio` when you need Nerfstudio's own frame.

### Seed point cloud

`pytheia.export.write_points_ply(path, xyz, rgb)` writes a binary PLY with `x y z nx ny nz red green blue`. Normals are zero. This is the `points3d.ply` layout used to initialise 3D Gaussian splatting.

## Known limitations

- PINHOLE cameras only. Fisheye, FOV, and other models are not exported.
- Skew is not representable in the COLMAP and transforms formats and is ignored with a warning.
- Undistortion uses OpenCV's remap, so the output is an approximation of the original image at the pixel level (half-pixel conventions of the resize are not corrected).
