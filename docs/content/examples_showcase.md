# Examples showcase {#examples-showcase}

The repository ships two levels of Python examples: **lightweight** scripts under **`pyexamples/`**, and **optional showcase** workflows under **`examples/`** that pull in larger third-party stacks (Torch, vismatch, Rerun).

## `pyexamples/` (fewer dependencies)

- Classic **OpenCV** features (SIFT / AKAZE) and **BFMatcher**, wired to **pyTheia** two-view estimation and **incremental / global / hybrid** reconstruction (e.g. fountain scene scripts).
- **Stereo / calibrated rig** — [`pyexamples/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo_rig_reconstruction.py) sets baseline + intrinsics, matches with **vismatch** (`edm` default), runs `GlobalRigReconstructor` or `IncrementalRigReconstructor` (see [Rigs](rigs.md)).
- Utilities: **NeRFStudio** / **SDFStudio** export, **MVS** / **Open3D** stubs, video frame extraction.
- Typical extras: `opencv-python`, `kornia` (see [`pyexamples/requirements.txt`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/requirements.txt)).

Use these when you want a quick sanity check without installing Torch-based matchers.

## `examples/` (showcase)

- **[vismatch + SfM](examples_vismatch_sfm.md)** — **Strecha scene root** (`images/` + `K.txt`, `gt_dense_cameras/*.camera`), Sim3 alignment, **average camera-center error** vs GT, optional **Rerun**, PLY/recon exports. Sources: [`examples/vismatch_sfm/`](https://github.com/urbste/pyTheiaSfM/tree/master/examples/vismatch_sfm) ([vismatch](https://github.com/gmberton/vismatch), optional [Rerun](https://rerun.io/)).
- **Nerfstudio / 3DGS export** — use [`pyexamples/nerfstudio_export_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/nerfstudio_export_reconstruction.py) to write `transforms.json` from a `pt.sfm.Reconstruction`; train Splatfacto or other 3D Gaussian splatting pipelines in external projects (e.g. Nerfstudio).

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
