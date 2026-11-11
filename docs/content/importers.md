# Importers

`pytheia.importers` reads the output of other SfM tools, so that pyTheia tools (the [viewer](viewer.md), [`pytheia info`](cli.md), the evaluation scripts) can work on it. The COLMAP model and OpenMVG readers are implemented in C++ (`theia/io/read_colmap_files.h`, `theia/io/read_openmvg_sfm_data.h`) and also available as `pt.io.ReadColmapFiles` and `pt.io.ReadOpenMVGSfMData`; the Python functions wrap them and raise exceptions and warnings. The COLMAP database reader is pure Python (`sqlite3`), so the C++ library does not depend on SQLite.

| Function | Input | Returns |
|----------|-------|---------|
| `read_colmap_model(path)` | COLMAP sparse model (`cameras`, `images`, `points3D`, `.txt` or `.bin`). GLOMAP writes the same format. | `pt.sfm.Reconstruction` |
| `read_openmvg_sfm_data(json_path)` | OpenMVG `sfm_data.json` (text JSON, not the cereal `sfm_data.bin`) | `pt.sfm.Reconstruction` |
| `read_colmap_database(path)` | COLMAP `database.db` | dict of numpy arrays (keypoints, matches, two-view geometries) |

```python
import pytheia as pt
from pytheia.importers import read_colmap_model, read_colmap_database, read_openmvg_sfm_data

recon = read_colmap_model("scene/sparse/0")          # or the folder above it
recon = read_openmvg_sfm_data("out/sfm_data.json")

db = read_colmap_database("scene/database.db")
db["keypoints"][1]                 # float32 (N, 2|4|6) for image id 1
db["matches"][(1, 2)]              # uint32 (M, 2) keypoint index pairs
db["two_view_geometries"][(1, 2)]["F"]   # 3x3 float64, or None
```

From C++, or from Python without exceptions:

```cpp
#include <theia/theia.h>

theia::Reconstruction recon;
theia::ImportMessages messages;
if (!theia::ReadColmapFiles("scene", &recon, /*pixel_offset=*/0.0, &messages)) {
  LOG(ERROR) << messages.error;  // messages.file_not_found tells a missing model apart
}
theia::ReadOpenMVGSfMData("out/sfm_data.json", &recon, &messages);
```

```python
success, recon, messages = pt.io.ReadColmapFiles("scene", pixel_offset=0.0)
success, recon, messages = pt.io.ReadOpenMVGSfMData("out/sfm_data.json")
print(messages.error, messages.warnings)
```

## COLMAP sparse model

`read_colmap_model(path)` looks for a folder that holds `cameras`, `images` and `points3D` in one format. It checks `path`, `path/sparse/0`, `path/sparse`, `path/0` and then the subfolders (up to four levels). Binary files are used when both formats are present.

What is read:

- **Views**: one per image in `images`, named by the COLMAP image name, estimated, with the pose from the quaternion `(w, x, y, z)` and translation. The camera center is `C = -R^T t`. The COLMAP camera id is the camera intrinsics group id.
- **Tracks**: one per 3D point, with position, color and estimated flag. The reprojection error stored by COLMAP is not kept.
- **Observations**: one per entry of `images` whose point id refers to a point in `points3D`.

Camera models:

| COLMAP model | pyTheia model | Notes |
|--------------|---------------|-------|
| `SIMPLE_PINHOLE`, `PINHOLE` | `PINHOLE` | `PINHOLE` sets the aspect ratio `fy / fx` |
| `SIMPLE_RADIAL`, `RADIAL` | `PINHOLE` | radial `k1`, `k2` |
| `OPENCV` | `PINHOLE_RADIAL_TANGENTIAL` | `k1`, `k2`, `p1`, `p2` |
| `FULL_OPENCV` | `FULL_OPENCV` | `k1`..`k6`, `p1`, `p2`; no term is dropped |
| `OPENCV_FISHEYE` | `FISHEYE` | `k1`..`k4` |
| `FOV` | `FOV` | `omega` |
| `RADIAL_FISHEYE` | `RADIAL_FISHEYE` | `k1`, `k2` |
| `SIMPLE_RADIAL_FISHEYE` | `RADIAL_FISHEYE` | `k1`; `k2` is set to 0 |
| `THIN_PRISM_FISHEYE` | `THIN_PRISM_FISHEYE` | `k1`..`k4`, `p1`, `p2`, `sx1`, `sy1` |
| `RAD_TAN_THIN_PRISM_FISHEYE` | `RAD_TAN_THIN_PRISM_FISHEYE` | `k0`..`k5`, `p0`, `p1`, `s0`..`s3` |

Every COLMAP model is imported with all of its parameters, so no warning is raised for a valid COLMAP file. Only an unknown model name raises `ValueError`. The parameter order of each COLMAP model is in `colmap/sensor/models.h`; the pyTheia models keep the same order after the focal length, aspect ratio and principal point (see [Camera models](cameras.md)).

### Pixel convention

COLMAP puts the center of the top-left pixel at `(0.5, 0.5)`, OpenCV at `(0, 0)`. pyTheia has no convention of its own: it uses whatever the observations and the principal point use, as long as both agree. `read_colmap_model` therefore keeps COLMAP's coordinates by default, which also matches `read_colmap_database` and `WriteColmapFiles`, so a round trip is exact. Pass `pixel_offset=0.5` (`PIXEL_OFFSET_COLMAP`) to subtract 0.5 from every observation and from the principal point and get the OpenCV convention.

## OpenMVG `sfm_data.json`

`read_openmvg_sfm_data(json_path)` reads the text JSON that OpenMVG writes (`sfm_data_version` 0.3 or 2.0):

- `views`: each view becomes a view named `local_path/filename`, relative to `root_path`. The intrinsics key `id_intrinsic` is the group id. A view is estimated when its `id_pose` has an entry in `extrinsics`.
- `intrinsics`: `pinhole` (no distortion) becomes `PINHOLE`. `disto_k1` and `disto_k3` (`[k1, k2, k3]`) become `PINHOLE` when `k3 = 0` and `PINHOLE_RADIAL_TANGENTIAL` otherwise. `disto_t2` (`[k1, k2, k3, d1, d2, t1, t2]`) becomes `PINHOLE_RADIAL_TANGENTIAL`, and the decentering terms `d1`, `d2` are dropped with a warning. The `disto_t2` layout is not checked against OpenMVG's source, so treat Brown-model files with care. Fisheye and spherical intrinsics, and any other `disto_*` key, raise `ValueError`.
- `extrinsics`: `rotation` (world-to-camera, as nested rows) and `center`.
- `structure`: each entry becomes a track (`X`, and `color` if present). Each observation is taken from its `key` (the view id) or `value.id_view`, with `value.x` as the pixel. Observations of unknown views are skipped with a warning.

Shared data is written with cereal pointers. The first occurrence has `ptr_wrapper` with `id` and `data`, and later references have only `ptr_wrapper` with `id`. The reader resolves both forms.

OpenMVG pixels are read without an offset.

!!! note "Export stage files"
    The `openmvg_export` stage of a pipeline writes intrinsics and views, with empty `extrinsics` and `structure`. Such a file reads as views with no poses and no tracks. Poses and structure come from the reconstruction stage, which writes `sfm_data.bin` (cereal binary). That file is not read here. To get the poses and tracks, run OpenMVG's `openMVG_main_openMVG2Colmap` on it (the benchmark scripts use it) and read the COLMAP text output with `read_colmap_model`.

## COLMAP `database.db`

`read_colmap_database(path)` opens the file read-only with the standard library `sqlite3` module and returns a dict:

| Key | Value |
|-----|-------|
| `"images"` | `{image_id: name}` |
| `"keypoints"` | `{image_id: float32 array (N, cols)}`. `cols` is 2 (`x, y`), 4 (`x, y, scale, orientation`) or 6 (`x, y` and the affine shape `a11, a12, a21, a22`). Raw COLMAP coordinates (centre at 0.5). |
| `"matches"` | `{(image_id1, image_id2): uint32 array (M, 2)}` keypoint index pairs |
| `"two_view_geometries"` | `{(image_id1, image_id2): dict}` with `config`, `config_name`, `F`, `E`, `H` (3x3 float64 or `None`), `qvec` (`w, x, y, z`), `tvec`, and `inlier_matches` (`uint32 (K, 2)`) |

Pair ids are `2147483647 * image_id1 + image_id2`, so the keys are `(image_id1, image_id2)` with `image_id1 < image_id2`. The config names are COLMAP's: `UNDEFINED`, `DEGENERATE`, `CALIBRATED`, `UNCALIBRATED`, `PLANAR`, `PANORAMIC`, `PLANAR_OR_PANORAMIC`, `WATERMARK`, `MULTIPLE`.

Blobs are row-major. COLMAP transposes `F`, `E` and `H` before storing them, so a plain `reshape(3, 3)` gives the matrix. Keypoint coordinates are returned unchanged. To get pyTheia pixel centers, subtract 0.5. Images without keypoints or pairs without matches are simply absent from the dict.

## Known limits

- Reconstructions are read as they are. Reprojection errors and the COLMAP `points3D` track lists are not stored; observations come from `images`.
- COLMAP can hold two 2D points of one image on the same 3D point. pyTheia allows one observation per view and track, so the reader keeps the first and warns with the count. The `pipes__xf` COLMAP model in the benchmark set has 413 such duplicates.
- Points in `points3D` that no image refers to are read as tracks with no observations.
- Other formats are not read: COLMAP `rigs.bin`/`frames.bin` (ignored), OpenMVG cereal `sfm_data.bin`, and MVS or NVM files.
