# vismatch + pyTheia SfM

End-to-end sketch: **learned matchers** from [vismatch](https://github.com/gmberton/vismatch) produce pixel correspondences; **pyTheia** estimates two-view geometry, builds tracks, and runs a **global / incremental / hybrid** reconstruction estimator.

## Requirements

See [`../requirements.txt`](../requirements.txt). You need `pytheia` installed (built from this repository or a wheel).

## Dataset

Any folder of images with a shared extension works. For a classic benchmark, use the **fountain** sequence (e.g. from [VisionMM](https://github.com/Vislearn/VisMM) or similar public SfM datasets) and pass `--image_dir` to that folder.

Default **intrinsics** in `pipeline.py` use a rough pinhole guess (focal \(\approx 1.2 \times \max(w,h)\), principal point at the image center). For real data, pass `--focal_length` and dimensions consistent with EXIF or calibration.

## Run

```bash
# From repo root, with examples env activated
python examples/vismatch_sfm/pipeline.py \
  --image_dir /path/to/images \
  --img_ext jpg \
  --matcher superpoint-lightglue \
  --device cuda \
  --reconstruction incremental \
  --resize 512 \
  --output_dir /path/to/out
```

Optional live 3D view (after reconstruction):

```bash
python examples/vismatch_sfm/pipeline.py ... --rerun
```

## Matcher names

See the [vismatch README](https://github.com/gmberton/vismatch#available-models) for the full list (e.g. `superpoint-lightglue`, `loftr`, `roma`).

## Limitations

- **All pairs** of views are tried: cost grows as \(O(n^2)\). Use a small image set first, or extend the script with **pair selection** (e.g. sequential neighbors, vocabulary tree, or `pt.matching.GraphMatch` scheduling).
- Geometric verification uses **pyTheia** `EstimateTwoViewInfo` on vismatch correspondences; thresholds may need tuning per scene.
