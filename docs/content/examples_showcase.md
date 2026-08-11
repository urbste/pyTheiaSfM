# Examples showcase {#examples-showcase}

The repository ships two levels of Python examples: **lightweight** scripts under **`pyexamples/`**, and **optional showcase** workflows under **`examples/`** that pull in larger third-party stacks (Torch, vismatch, Rerun).

## `pyexamples/` (fewer dependencies)

Organized by topic (see [`pyexamples/README.md`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/README.md)):

| Area | Path |
|------|------|
| Preprocess / ZED SVO | [`pyexamples/preprocess/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/preprocess) — video/sequence extractors; [`zed_svo_extract_stereo.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/preprocess/zed_svo_extract_stereo.py) writes rectified left/right + `rig_calibration.json` (needs `pyzed`) |
| Stereo / calibrated rig | [`pyexamples/stereo/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/stereo_rig_reconstruction.py) — baseline + intrinsics, vismatch (`edm`), `GlobalRigReconstructor` / `IncrementalRigReconstructor` (see [Rigs](rigs.md)) |
| Monocular SfM | [`pyexamples/sfm/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/sfm) — OpenCV fountain, Kornia deep matching, LoFTR + Aqualoc |
| Two-view / Sim(3) / IO / MVS | [`twoview/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/twoview), [`transform/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/transform), [`io/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/io), [`mvs/`](https://github.com/urbste/pyTheiaSfM/tree/master/pyexamples/mvs) |

Typical extras: `pip install -r pyexamples/requirements.txt`. ZED extraction needs the Stereolabs SDK Python API.

Use these when you want a quick sanity check without installing Torch-based matchers (OpenCV fountain / IO exporters need only light deps).

## `examples/` (showcase)

- **[vismatch + SfM](examples_vismatch_sfm.md)** — **Strecha scene root** (`images/` + `K.txt`, `gt_dense_cameras/*.camera`), Sim3 alignment, **average camera-center error** vs GT, optional **Rerun**, PLY/recon exports. Sources: [`examples/vismatch_sfm/`](https://github.com/urbste/pyTheiaSfM/tree/master/examples/vismatch_sfm) ([vismatch](https://github.com/gmberton/vismatch), optional [Rerun](https://rerun.io/)).
- **Nerfstudio / 3DGS export** — use [`pyexamples/io/nerfstudio_export_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/io/nerfstudio_export_reconstruction.py) to write `transforms.json` from a `pt.sfm.Reconstruction`; train Splatfacto or other 3D Gaussian splatting pipelines in external projects (e.g. Nerfstudio).

Install optional dependencies:

```bash
pip install ".[examples]"
```

See [`examples/README.md`](https://github.com/urbste/pyTheiaSfM/blob/master/examples/README.md) for setup, licenses (vismatch wraps many models), and CUDA notes.

## Related manual chapters

- [Building](building.md) — compile / install pyTheia  
- [Python API overview](python_wrapper.md) — imports and stubs  
- [Matching](matching.md) — `FeatureCorrespondence`, `ImagePairMatch`, pairing helpers  
- [IO](io.md) — `WriteNerfStudio`, `WriteColmapFiles`, PLY  
