# vismatch + pyTheia SfM {#examples-vismatch-sfm}

End-to-end demo under [`examples/vismatch_sfm/`](https://github.com/urbste/pyTheiaSfM/tree/master/examples/vismatch_sfm):

1. **[vismatch](https://github.com/gmberton/vismatch)** extracts putative correspondences (many matchers; default **SuperPoint + LightGlue**).
2. **pyTheia** runs two-view geometry, builds tracks, and reconstructs with **incremental**, **global**, or **hybrid** estimators.
3. With a **Strecha-style scene root** (`images/` + `K.txt`, `gt_dense_cameras/*.camera`), the pipeline loads **ground-truth cameras**, optionally **Sim3-aligns** the estimated model to GT, and can open **[Rerun](https://rerun.io/)** for 3D visualization.

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

Point **`--strecha_dir`** at the **scene root directory** (not at `images/` alone). The pipeline validates and loads:

| Path | Contents |
|------|-----------|
| `<scene>/images/` | RGB images (default **`*.jpg`**, or set `--img_ext`) and **`K.txt`** (must exist; used as a layout sanity check). |
| `<scene>/gt_dense_cameras/` | One **`*.camera`** per view (same basename stem as the image file). |

Ground truth is read with **`ReadStrechaDataset`** on `gt_dense_cameras/`. Image paths are **`images/<stem>.<ext>`** for each stem taken from the `.camera` basenames.

Intrinsics used when building the **running** reconstruction are taken from the **GT cameras** after loading `.camera` files (via Theia priors), not by parsing `K.txt` in this script.

!!! warning "Pair count"

    The script tries **all** image pairs: cost grows as \(O(n^2)\). Start with a small set or extend the script with pair scheduling.

### Run one scene

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --img_ext jpg \
  --matcher superpoint-lightglue \
  --device cuda \
  --reconstruction incremental \
  --resize 512 \
  --output_dir /path/to/out
```

Optional 3D viewer after reconstruction:

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --rerun
```

Outputs include `reconstruction.ply`, `reconstruction.recon`, and **`gt_reconstruction.ply`** under `--output_dir` (defaults to the Strecha root if omitted).

Robust Sim3 alignment (camera-center threshold in **meters**; tune for outliers):

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --align_robust 0.05 \
  --rerun
```

Debug correspondences (full-resolution side-by-side plots under `match_plots/` in the output directory):

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/scene \
  --plot_matches
```

### Rerun layers

With `--rerun`, entities are split for comparison:

| Entity prefix | Meaning |
|---------------|---------|
| `world/gt/` | Ground-truth cameras (frustums / centers) |
| `world/estimated/aligned/` | Triangulated points and cameras **after Sim3** alignment to GT |

GT provides **cameras** from `.camera` files; **triangulated** points come from the estimated reconstruction.

### Pose convention in `.camera` files

The loader assumes **COLMAP-style** extrinsics in each `.camera` file: \(X_\mathrm{cam} = R\,X_\mathrm{world} + t\) with pinhole \(x \sim K X_\mathrm{cam}\). That is converted to **Theia** camera center and world-to-camera rotation. If your files use another convention, alignment will be wrong until the parser is adjusted.

### Requirements

- Every view in `gt_dense_cameras/` must have **`images/<stem>.<ext>`** (default `jpg`).
- **`images/K.txt`** must exist (layout check).

## Related docs

- [Examples overview](examples_showcase.md) — `pyexamples/` vs `examples/`
- [Matching](matching.md) — correspondences and view graph
- [IO](io.md) — PLY / reconstruction files
- [Transformations](transformations.md) — `AlignReconstructions`, Sim3 helpers
