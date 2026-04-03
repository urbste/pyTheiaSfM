# vismatch + pyTheia SfM (Strecha layout)

**Learned matchers** from [vismatch](https://github.com/gmberton/vismatch) produce pixel correspondences; **pyTheia** estimates two-view geometry, builds tracks, and runs a **global / incremental / hybrid** reconstruction estimator.

Ground-truth cameras come from **Strecha-style** `.camera` files via Theia [`ReadStrechaDataset`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/read_strecha_dataset.cc). RGB images live under **`images/`** and are paired to GT by view name (see [Dataset layout](#dataset-layout)).

## Requirements

See [`../requirements.txt`](../requirements.txt). You need **pytheia** installed (built from this repository or a wheel).

## Dataset layout

Pass **`--strecha_dir`** to the **scene root** (not to `images/` alone).

| Path | Contents |
|------|-----------|
| `<strecha_dir>/images/` | Rasters (default **`*.jpg`**, or set `--img_ext`) and **`K.txt`** (must exist; layout check only in this script). |
| `<strecha_dir>/gt_dense_cameras/` | One **`*.camera`** file per view. |

**View names** are the camera filename with the `.camera` suffix removed. Examples:

- `gt_dense_cameras/0000.camera` → view `0000` → image `images/0000.jpg` (with default `--img_ext jpg`).
- `gt_dense_cameras/0000.jpg.camera` → view `0000.jpg` → image **`images/0000.jpg`** (full basename; the script does not append `.jpg` twice).

**Intrinsics** for SfM priors come from the **`.camera`** files (via the GT reconstruction), not from parsing `K.txt` in Python.

## How to run

From the repository root (with optional `[examples]` env installed):

```bash
python examples/vismatch_sfm/pipeline.py --help   # all flags + matcher name groups
```

Typical run:

```bash
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/strecha_scene \
  --matcher superpoint-lightglue \
  --device cuda \
  --reconstruction incremental \
  --output_dir /path/to/out
```

Useful options:

| Flag | Role |
|------|------|
| `--resize` | vismatch `load_image` resize (pixels); default matches `pipeline.py` (see `--help`). |
| `--min_inliers_twoview` | Putative matches + RANSAC two-view gate; default in `pipeline.py`. |
| `--align_robust METERS` | Sim3 with `AlignReconstructionsRobust` and camera-center outlier threshold. |
| `--plot_matches` | Writes `match_plots/<a>__<b>.png` under `--output_dir` (optional `--plot_matches_max_lines`). |
| `--rerun` | Opens [Rerun](https://rerun.io/) after export (needs `rerun-sdk`). |

**Note:** The script tries **all** unordered view pairs — cost is \(O(n^2)\) in the number of views.

## Console output

Besides per-stage **progress** (preload, matching, two-view summary, tracks, reconstruction time), after **Sim3 alignment** you get:

- Scale and translation of the alignment.
- **Average camera center error** vs GT (mean Euclidean distance over views that exist in both reconstructions, **after** alignment; units follow the scene, typically **meters** for standard benchmark layouts).

## Outputs

Written under **`--output_dir`** if set, otherwise under **`--strecha_dir`**:

| File | Content |
|------|--------|
| `reconstruction.ply` | Aligned sparse points (Sim3 to GT), red coloring |
| `reconstruction.recon` | Same reconstruction, native Theia format |
| `gt_reconstruction.ply` | Ground-truth cameras / structure export, green coloring |
| `match_plots/` | Only if `--plot_matches` |

## Rerun (`--rerun`)

The eval layout shows **ground truth** (`world/gt/...`) and the **Sim3-aligned estimate** (`world/estimated/aligned/...`) — points, camera centers, and **pinhole frustums**. Pre-alignment geometry is not logged. Frustum depth in the viewer uses an internal scale factor for visibility.

## Matcher names

See the [vismatch README](https://github.com/gmberton/vismatch#available-models); names are also grouped in `python examples/vismatch_sfm/pipeline.py --help`.

## Limitations

- **All pairs** matching does not scale to huge image sets without changing the script.
- Two-view thresholds may need tuning per scene or matcher.
