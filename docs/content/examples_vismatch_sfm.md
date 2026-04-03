# vismatch + pyTheia SfM {#examples-vismatch-sfm}

End-to-end demo under [`examples/vismatch_sfm/`](https://github.com/urbste/pyTheiaSfM/tree/master/examples/vismatch_sfm):

1. **[vismatch](https://github.com/gmberton/vismatch)** extracts putative correspondences (many matchers; default **SuperPoint + LightGlue**).
2. **pyTheia** runs two-view geometry, builds tracks, and reconstructs with **incremental**, **global**, or **hybrid** estimators.
3. With a **Strecha-style scene root** (`images/` + `K.txt`, `gt_dense_cameras/*.camera`), the pipeline loads **ground-truth cameras**, **Sim3-aligns** the estimated model to GT, prints **average camera-center error**, optionally writes **match figures**, and can open **[Rerun](https://rerun.io/)**.

For lighter OpenCV-only flows, use [`pyexamples/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples) instead ([Examples overview](examples_showcase.md)).

## Install

Build or install **pyTheia** first ([Building](building.md)), then example extras from the repository root:

```bash
pip install ".[examples]"
```

This pulls in `vismatch`, `torch`, `opencv-python`, `rerun-sdk`, etc. (see [`pyproject.toml`](https://github.com/urbste/pyTheiaSfM/blob/master/pyproject.toml) `[project.optional-dependencies] examples`).

!!! note "CUDA"

    A GPU is recommended for matchers; CPU often works but is much slower.

## Strecha scene root (`--strecha_dir`)

Point **`--strecha_dir`** at the **scene root** (not at `images/` alone). The pipeline validates:

| Path | Contents |
|------|-----------|
| `<scene>/images/` | RGB images (default **`*.jpg`**, or `--img_ext`) and **`K.txt`** (must exist; layout sanity check). |
| `<scene>/gt_dense_cameras/` | One **`*.camera`** per view. |

Ground truth is read with **`ReadStrechaDataset`** on `gt_dense_cameras/`. Images are resolved under `images/` by **view name** (basename of each `.camera` file without the suffix). If the name already ends with an extension (e.g. **`0000.jpg`** from **`0000.jpg.camera`**), the raster is **`images/0000.jpg`**; otherwise **`images/<stem>.<ext>`** with default `ext` from `--img_ext`.

Intrinsics for the running reconstruction come from **`.camera`** priors, not from `K.txt` in this script.

!!! warning "Pair count"

    The script tries **all** image pairs: cost grows as \(O(n^2)\). Start with a small set or extend the script with pair scheduling.

### Command line

Full matcher list groups and defaults are in **`python examples/vismatch_sfm/pipeline.py --help`**. Example:

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --img_ext jpg \
  --matcher superpoint-lightglue \
  --device cuda \
  --reconstruction incremental \
  --output_dir /path/to/out
```

Default **`--resize`**, **`--min_inliers_twoview`**, and **`--num_threads`** match the current `pipeline.py`; prefer `--help` over hard-coding values here.

Optional 3D viewer after reconstruction:

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --rerun
```

Robust Sim3 alignment (camera-center threshold in **meters**):

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --align_robust 0.05 \
  --rerun
```

Debug correspondences (`match_plots/` under the output directory):

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --plot_matches
```

### Outputs and metrics

Under **`--output_dir`** (or **`--strecha_dir`** if unset):

- **`reconstruction.ply`** / **`reconstruction.recon`** — estimate **after** Sim3 alignment to GT.
- **`gt_reconstruction.ply`** — ground-truth export for comparison.

After alignment, the log includes **average camera center error**: mean Euclidean distance between aligned estimated and GT camera centers over views with the same name (scene units; typically meters on benchmark datasets).

Progress lines summarize preloading, pairwise matching, two-view verification, track count, and reconstruction time.

### Rerun layers (`--rerun`)

| Entity prefix | Meaning |
|---------------|---------|
| `world/gt/` | Ground-truth cameras (centers + pinhole frustums) |
| `world/estimated/aligned/` | Triangulated points and cameras **after Sim3** to GT |

Only GT and the **aligned** estimate are shown (no pre-alignment layer). Frustum geometry uses an enlarged depth for visibility in the viewer.

### Pose convention in `.camera` files

The loader assumes **COLMAP-style** extrinsics: \(X_\mathrm{cam} = R\,X_\mathrm{world} + t\) with \(x \sim K X_\mathrm{cam}\). That is converted to **Theia** camera center and world-to-camera rotation. If your files use another convention, alignment will be wrong until the parser is adjusted.

### Requirements

- Every GT view must have a matching raster under **`images/`** (see naming rules above).
- **`images/K.txt`** must exist.

## Related docs

- [Examples overview](examples_showcase.md) — `pyexamples/` vs `examples/`
- [Matching](matching.md) — correspondences and view graph
- [IO](io.md) — PLY / reconstruction files
- [Transformations](transformations.md) — `AlignReconstructions`, Sim3 helpers
