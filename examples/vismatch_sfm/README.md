# vismatch + pyTheia SfM (Strecha layout)

**Learned matchers** from [vismatch](https://github.com/gmberton/vismatch) produce pixel correspondences; **pyTheia** estimates two-view geometry, builds tracks, and runs a **global / incremental / hybrid** reconstruction estimator.

Ground-truth cameras and intrinsics come from **Strecha-style** `.camera` files loaded via Theia [`ReadStrechaDataset`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/io/read_strecha_dataset.cc). Rasters are resolved under **`images/`** by matching each GT view stem to **`images/<stem>.<ext>`** (default extension **`jpg`**).

## Requirements

See [`../requirements.txt`](../requirements.txt). You need `pytheia` installed (built from this repository or a wheel).

## Dataset layout

Pass **`--strecha_dir`** to the **scene root**. The pipeline expects:

| Path | Contents |
|------|-----------|
| `<strecha_dir>/images/` | **JPEG** rasters named **`<view_stem>.jpg`** (or another extension via `--img_ext`), plus **`K.txt`** (must be present; 3×3 intrinsics text file used as a layout marker). |
| `<strecha_dir>/gt_dense_cameras/` | One **`*.camera`** per view (same **stem** as the image basename). |

View names are the **basename** of each `.camera` file (without the `.camera` suffix). The running reconstruction still gets **camera intrinsics** from the **GT reconstruction** built from the `.camera` files (`CameraIntrinsicsPriorFromIntrinsics` on those cameras), not by parsing `K.txt` in Python (that file is only required to exist today).

## Run

```bash
# From repo root, with examples env activated
python examples/vismatch_sfm/pipeline.py \
  --strecha_dir /path/to/strecha_scene \
  --matcher superpoint-lightglue \
  --device cuda \
  --reconstruction incremental \
  --resize 512 \
  --output_dir /path/to/out
```

Optional live 3D view (after reconstruction):

```bash
python examples/vismatch_sfm/pipeline.py ... --strecha_dir /path/to/scene --rerun
```

## Matcher names

See the [vismatch README](https://github.com/gmberton/vismatch#available-models) for the full list (e.g. `superpoint-lightglue`, `loftr`, `roma`).

## Limitations

- **All pairs** of views are tried: cost grows as \(O(n^2)\). Use a small image set first, or extend the script with **pair selection**.
- Geometric verification uses **pyTheia** `EstimateTwoViewInfo` on vismatch correspondences; thresholds may need tuning per scene.
