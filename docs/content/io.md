# I/O Utilities {#chapter-io}

Python bindings for [`src/theia/io`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/io): reconstruction import/export, Bundler/NVM/COLMAP-style writers, and NeRF/SDF studio JSON. All symbols below live on **`pytheia.io`** (submodule defined in [`io.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/pytheia/io/io.cc)).

Convention: **read** helpers exposed from [`io_wrapper.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/io_wrapper.h) return a leading **`bool` success** flag, then loaded objects. **Write** functions return **`bool`** directly (except where noted).

---

## Classes

### `BundlerObservation`

One 2D observation of a Bundler 3D point: `camera_index`, `feature_index`, `x`, `y`.

### `BundlerPoint`

Bundler 3D point: `position`, `color`, `view_list` (list of `BundlerObservation`).

### `BundlerFileReader(lists_file, bundle_file)`

Low-level Bundler parser: call `ParseListsFile()` / `ParseBundleFile()` then inspect `cameras()`, `points()`, `img_entries()`.

### `BundlerCamera`

Fields: `translation`, `rotation`, `focal_length`, `radial_coeff_1`, `radial_coeff_2`.

### `ListImgEntry`

List-file row: `filename`, `second_entry`, `focal_length`.

---

## Reading / importing

| Function | Returns | Description |
|----------|---------|-------------|
| `ReadReconstruction(input_file)` | `(ok, reconstruction)` | Binary Theia reconstruction ([`reconstruction_reader.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/reconstruction_reader.h)). |
| `ReadBundlerFiles(lists_file, bundle_file)` | `(ok, reconstruction)` | Noah Snavely Bundler lists + bundle. |
| `ImportNVMFile(nvm_filepath)` | `(ok, reconstruction)` | NVM_V3 text bundle (`.nvm`). |
| `Read1DSFM(dataset_directory)` | `(ok, reconstruction, view_graph)` | [1DSfM](http://www.cs.cornell.edu/projects/1dsfm/) layout: sparse geometry + view graph. |
| `ReadStrechaDataset(dataset_directory)` | `(ok, reconstruction)` | Strecha multi-view benchmark style data. |
| `PopulateImageSizesAndPrincipalPoints(image_directory)` | `(ok, reconstruction)` | Wrapper around [`populate_image_sizes.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/populate_image_sizes.h): loads image files named by each view and sets camera size and **center principal point**. The Python binding passes a **fresh** empty `Reconstruction` into the C++ call (see [`io_wrapper.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/io_wrapper.cc)); it succeeds trivially if there are no views. After `ReadBundlerFiles`, you typically need image dimensions filled on that same reconstruction—use C++ or extend the binding if the empty-reconstruction path is insufficient. |

---

## Writing reconstructions and exchange formats

| Function | Returns | Description |
|----------|---------|-------------|
| `WriteReconstruction(reconstruction, output_file)` | `bool` | Binary Theia format ([`reconstruction_writer.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/reconstruction_writer.h)). |
| `WriteReconstructionJson(reconstruction, output_json_file)` | `bool` | JSON export of estimated views and tracks. |
| `WritePlyFile(ply_file, reconstruction, camera_color, min_num_observations_per_point)` | `bool` | PLY point cloud for MeshLab-style viewers ([`write_ply_file.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_ply_file.h)). `camera_color` is an `Eigen::Vector3i` (RGB); use a NumPy length-3 integer array from Python. |
| `WriteBundlerFiles(reconstruction, lists_file, bundle_file)` | `bool` | Bundler lists + bundle from a reconstruction. |
| `WriteNVMFile(nvm_filepath, reconstruction)` | `bool` | NVM text. |
| `WriteColmapFiles(reconstruction, output_directory)` | `bool` | COLMAP-compatible text under `output_directory`. |
| `WriteNerfStudio(path_to_images, reconstruction, aabb_scale, out_json_file)` | `bool` | [`transforms.json`](https://docs.nerf.studio/en/latest/quickstart/data_conventions.html)-style export (PINHOLE / FISHEYE mapping per [`write_nerfstudio.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_nerfstudio.h)). |
| `WriteSdfStudio(path_to_images, reconstruction, nearfar, radius)` | `bool` | SDFStudio JSON; **PINHOLE**, undistorted images ([`write_sdfstudio.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_sdfstudio.h)). `nearfar` is `(near, far)` floats. |

---

## Export to Nerfstudio

Writes a `transforms.json` compatible with Nerfstudio.

``` python
import pytheia as pt
ok = pt.io.WriteNerfStudio("/path/to/images", recon, 16, "/path/to/out/transforms.json")
# aabb_scale is typically 2, 4, 8, or 16
```

Notes: Supported camera models are mapped internally (PINHOLE and FISHEYE). Images are referenced as `path_to_images/view_image_name`.

---

## Export to SDFStudio

Writes JSON compatible with SDFStudio. Images must be undistorted and PINHOLE.

``` python
import pytheia as pt
ok = pt.io.WriteSdfStudio("/path/to/images", recon, (2.0, 6.0), 1.0)
# nearfar is (near, far); radius controls scene radius
```

---

## Not exposed in Python

The C++ tree also includes helpers such as **`WriteMatchesAndGeometry`** and **`WriteCalibration`** ([`write_matches.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_matches.h), [`write_calibration.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/write_calibration.h)) without pybind entries in `io.cc`. Use the C++ API or extend the bindings if you need those.
