# pyexamples/

Lightweight Python scripts that exercise the **pyTheia** API. Heavier Torch /
vismatch / Rerun demos live under [`examples/`](../examples/).

## Layout

| Folder | Contents |
|--------|----------|
| [`common/`](common/) | Shared helpers (`image_utils`, geometry plots) |
| [`preprocess/`](preprocess/) | Frame extraction (video, image sequence, **ZED SVO**) |
| [`sfm/`](sfm/) | Monocular SfM pipelines (OpenCV, Kornia, LoFTR) |
| [`stereo/`](stereo/) | Calibrated multi-camera / stereo rig reconstruction |
| [`twoview/`](twoview/) | Two-view estimators (e.g. monodepth-assisted) |
| [`transform/`](transform/) | Sim(3) alignment demos |
| [`io/`](io/) | NeRFStudio / SDFStudio export |
| [`mvs/`](mvs/) | Multi-view stereo + PLY viewer |

## Quick start

```bash
# After building / installing pytheia:
export PYTHONPATH=src:$PYTHONPATH   # if using an in-tree build

pip install -r pyexamples/requirements.txt   # opencv / kornia / open3d / …

# Stereo from a ZED recording
python pyexamples/preprocess/zed_svo_extract_stereo.py \
  --svo /data/capture.svo2 --out_dir /data/zed_frames --every 5

python pyexamples/stereo/stereo_rig_reconstruction.py \
  --left_dir /data/zed_frames/left --right_dir /data/zed_frames/right \
  --baseline … --focal … --cx … --cy … --matcher edm --method global
```

`zed_svo_extract_stereo.py` needs the Stereolabs **ZED SDK** Python API (`pyzed`),
not listed in `requirements.txt`.

## Status notes

- Scripts that import Torch / OpenCV / Kornia at module top level need those
  packages installed even for `--help`.
- Dataset paths inside older fountain / Aqualoc scripts are machine-local —
  pass your own directories via argparse where available.
- Prefer [`examples/vismatch_sfm/`](../examples/vismatch_sfm/) for a maintained
  Strecha + vismatch showcase.

See also: [Examples showcase](../docs/content/examples_showcase.md), [Rigs](../docs/content/rigs.md).
