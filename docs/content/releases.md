# Releases {#chapter-releases}

This page records **pyTheia** releases. It no longer mirrors the historical [TheiaSfM](http://www.theia-sfm.org) upstream changelog; older version notes referred to upstream tarballs and features that this fork no longer ships.

---

## 1.0.x (current: 1.0.9)

Stable Python packaging line with MkDocs documentation, cross-run alignment APIs, and continued binding coverage for cameras, BA, pose solvers, view graphs, and I/O (COLMAP, Bundler, NVM, PLY, Nerfstudio, SDFStudio, etc.).

### Highlights since 0.6.0

- **Versioning** aligned across root `VERSION`, CMake `project()`, and `pyproject.toml`.
- **Documentation** maintained as MkDocs under `docs/content/` (Sphinx-era pages removed).
- **Cross-run alignment** APIs documented in [Cross-run alignment](cross_run_alignment.md) and [Transformations](transformations.md) (BA relative pose edges, Sim(3) pose graph, scale-invariant odometry).
- **Examples:** lightweight scripts in `pyexamples/`; showcase `examples/vismatch_sfm/`; Nerfstudio export via `pyexamples/nerfstudio_export_reconstruction.py`.
- **Wheels:** manylinux builds for Python 3.8–3.13 via Docker (`urbste/pytheia_base`).
- **Two-view / PnP speed:** Sturm 5-pt and monodepth 3-pt minimal solvers (PoseLib-adapted); dense LM RANSAC local optimization (`use_lo`) for calibrated relative pose, monodepth relative pose, and calibrated absolute pose — see [RANSAC](ransac.md#ransac-local-optimization) and [Performance](performance.md).

### Known limitations

- C++ unit tests exist but are mostly unwired in CMake; CI builds wheels without running pytest or CTest.
- Feature detection / descriptor matching in C++ is deprecated in favor of Python-side pipelines; see 0.6.0 notes below.

---

## 0.6.0

Major cleanup: smaller surface area, Python-first imaging and features, fewer vendored or optional dependencies in the default build path.

### Removed or deprecated in this line

- **Raster / EXIF in C++:** The `theia/image` stack and **OpenImageIO** integration are gone. Images and EXIF are expected in Python (OpenCV, Pillow, etc.). C++ keeps geometry, reconstructions, cameras, BA, and solvers.
- **Feature and descriptor I/O:** Reading/writing bundled keypoint–descriptor files and **SIFT** key formats has been removed from the library and from Python bindings. Feature detection and matching stay outside the core package; you pass correspondences and use `Feature`, `FeatureCorrespondence`, and estimators as needed.
- **Python API trim:** Descriptor-centric matching helpers (e.g. `KeypointsAndDescriptors`, brute-force / cascade hash matchers, Fisher-vector globals, two-view verification types that depended on that stack) are no longer exposed on `pytheia.matching` / `pytheia.sfm`. **`FeaturesAndMatchesDatabase`**, **`InMemoryFeaturesAndMatchesDatabase`**, **`ReconstructionBuilder`**, and **`ReconstructionBuilderOptions`** are also removed from Python (C++ Theia still uses the match database inside its own reconstruction-builder path).
- **Exports and extras:** Legacy **PMVS** export and other dead or redundant tooling paths have been removed or stripped from docs and CMake where applicable.
- **Documentation:** Sphinx-era pages were replaced with **MkDocs**; narrative docs focus on what pyTheia actually exposes and how to wire Python-side features into SfM.

### Cross-run alignment additions (0.6.x)

New capabilities for aligning two independent reconstructions (segment + run). Narrative docs: [Cross-run alignment](cross_run_alignment.md).

- **`BundleAdjustReconstructionWithRelativePoseEdges`** — full BA plus SE(3) pose-to-pose edges that snapshot run odometry at setup time.
- **`RelativePoseConstraint.scale_invariant_translation`** — optional **scale-invariant** odometry (translation direction + optional log-magnitude) via `ScaledRelativePoseError`.
- **`BundleAdjustReconstructionWithConstantTracks`** — control-point BA: fixed map tracks, optimized run cameras.
- **Sim(3) view helpers** — `GetSim3LieFromView`, `RelativeSim3BetweenViews`, `SetViewCameraFromSim3Lie`.
- **Cross-reconstruction Sim(3) pose graph** — `AlignReconstructionsWithPoseGraph`, `CrossReconstructionSim3PoseGraphOptimizer`, sequential / anchor / scale-smoothness edges (`CrossViewAnchorEdge`, `SequentialSim3Edge`, …). Documented under [Transformations → pose graph](transformations.md#transformations-pose-graph).

### Still here

- Core **SfM** pipelines, **bundle adjustment**, **cameras**, **pose / triangulation** solvers, **view graph**, **I/O** for reconstructions (binary/JSON, Bundler, NVM, COLMAP text, PLY, Nerfstudio, SDFStudio, etc.), and **pybind11** Python API for the remaining types and functions.

If you need the old upstream release notes for academic comparison, see the [TheiaSfM repository](https://github.com/sweeneychris/TheiaSfM) tags and history.
