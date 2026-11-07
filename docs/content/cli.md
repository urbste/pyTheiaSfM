# Command line interface

pyTheia ships a `pytheia` command (also `python -m pytheia`). It wraps the Python API for common tasks: inspecting a reconstruction, converting formats, running SfM from a prepared input, aligning two reconstructions and exporting data for viewers and splatting tools.

```bash
pytheia --help
python -m pytheia --help
```

Exit status: `0` on success, `1` on a runtime failure (missing file, failed estimation, unreadable input), `2` on a usage error (unknown option, bad value, missing argument).

Reconstructions are read and written in the binary `.recon` format (`pytheia.io.WriteReconstruction`). JSON reconstructions cannot be read back.

## Subcommands

| Command | Purpose |
|---------|---------|
| `pytheia info RECON` | Views (registered / total), tracks, observations, mean reprojection error. `--json` for machine output. |
| `pytheia convert IN OUT --format FMT` | Write `colmap`, `colmap-bin`, `nvm`, `bundler`, `ply`, `nerfstudio` or `json`. |
| `pytheia view RECON [--html OUT.html]` | Self-contained HTML viewer. Needs `pytheia.viewer`. |
| `pytheia export-splat RECON --images DIR --out DIR` | COLMAP-style input for Gaussian splatting. Needs `pytheia.export`. |
| `pytheia align A B` | Sim(3) alignment of A onto B using views with the same name, and camera-center error. |
| `pytheia sfm --input DIR --out DIR` | Global, incremental or hybrid SfM from tracks and a view graph. |

### info

```bash
pytheia info reconstruction.recon
```

Prints the view count with the number of registered views, track count with estimated tracks, observation count and the mean reprojection error in pixels.

### convert

```bash
pytheia convert reconstruction.recon out/colmap --format colmap
pytheia convert reconstruction.recon out/bundler --format bundler     # bundle.out, list.txt
pytheia convert reconstruction.recon cloud.ply --format ply --min-observations 2
pytheia convert reconstruction.recon scene.nvm --format nvm
pytheia convert reconstruction.recon transforms.json --format nerfstudio --images images/ --aabb-scale 16
pytheia convert reconstruction.recon scene.json --format json
```

- `colmap` writes `cameras.txt`, `images.txt` and `points3D.txt` into the output directory.
- `colmap-bin` writes the binary files. It needs a build whose `pytheia.io.WriteColmapFiles` accepts `binary=`; otherwise the command reports that the writer is missing.
- `nerfstudio` needs `--images` (the image directory recorded in the JSON).

### view

```bash
pytheia view reconstruction.recon --html scene.html
```

Without `--html` the file is written as `<name>.html` in the current directory.

### export-splat

```bash
pytheia export-splat reconstruction.recon --images images/ --out splat_input/
pytheia export-splat reconstruction.recon --images images/ --out splat_input/ --no-undistort --max-image-size 1600
```

Writes the reconstruction for Gaussian splatting tools. Undistortion is on by default; `--no-undistort` keeps the original images and camera models. `--max-image-size N` limits the longer image side to `N` pixels (`-1`, the default, keeps the original size).

### align

```bash
pytheia align estimate.recon reference.recon
pytheia align estimate.recon gt.recon --json
```

Finds the views that are registered in both files with the same name, estimates the similarity transform (scale, rotation, translation; Umeyama) that maps the camera centers of A onto B, and reports the camera-center error in the units of B: mean, median, RMSE and max. At least three common views are required.

### sfm

`sfm` does not detect features or match images. Feature detection and matching stay in your application (see the [Agent usage guide](agent_usage.md)). `sfm` runs estimation on an input directory that contains:

| File | Content |
|------|---------|
| `reconstruction.recon` | Views (names, camera intrinsics priors) and tracks with features, written by `WriteReconstruction(recon, path, write_full_reconstruction=True)`. |
| `view_graph.bin` | Two-view geometries for the view pairs, written by `ViewGraph.WriteToDisk`. |

The existing poses in the reconstruction are discarded; estimation starts from the tracks and the view graph. The reference pipeline that builds both files is `examples/vismatch_sfm/pipeline.py`: its track builder and `view_graph.AddEdge` calls produce the data, and the pipeline's final `WriteReconstruction` call can be extended with `ViewGraph.WriteToDisk` to produce the input directory.

```bash
pytheia sfm --input scene_input/ --out result/ --method global --seed 1
pytheia sfm --input scene_input/ --out result/ --method incremental \
    --config options.json --set max_num_iterations=50
```

Outputs in `--out`:

- `reconstruction.recon` (estimated views and tracks),
- `reconstruction.ply`,
- `summary.json` (method, seed, success, message, estimated views and tracks, mean reprojection error, wall time).

`--method` is `global` (default), `incremental` or `hybrid`. `--seed N` calls `pytheia.solvers.SetDefaultRandomSeed(N)` before estimation, so repeated runs draw the same samples (see [RANSAC](ransac.md)).

#### Options

Every field of `pytheia.sfm.ReconstructionEstimatorOptions` can be set in two ways. Later sources override earlier ones: defaults, then `--config`, then `--set`.

- `--config FILE.json`: a JSON object. Keys are option names. Nested option groups are JSON objects, and enum values are given by name:

    ```json
    {
      "max_num_iterations": 50,
      "bundle_adjustment_loss_function_type": "HUBER",
      "bundle_adjustment_robust_loss_width": 1.0,
      "ligt_position_estimator_options": {"use_scale_constraint": true}
    }
    ```

- `--set key=value` (repeatable): one option. Nested options use dots, for example `--set ligt_position_estimator_options.use_scale_constraint=false`. Values are converted to the type of the current default: booleans accept `true/false/1/0/yes/no/on/off`, enums accept the member name (for example `LIGT`).

Unknown names and values of the wrong type are usage errors (exit status 2) that name the option. YAML config files are not supported.

To list every option with its default value:

```bash
pytheia sfm --list-options
```

## Python entry points

The command is a thin layer over:

```python
from pytheia.cli import main, build_estimator_options, umeyama

status = main(["info", "reconstruction.recon"])          # returns the exit status
opts = build_estimator_options("options.json", ["max_num_iterations=50"])
```

`main(argv)` takes the same arguments as the shell command and returns the status instead of exiting.

## Not included

- Native C++ command line tools (`theia_global_sfm` and others) are a separate, planned feature and are not part of the wheel.
- Feature detection and matching are not part of the command. Use your own pipeline to write the `sfm` input directory.
- The benchmark hook (`pytheia bench`) is planned.
