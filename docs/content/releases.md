# Releases {#chapter-releases}

This page records **pyTheia** releases. It no longer mirrors the historical [TheiaSfM](http://www.theia-sfm.org) upstream changelog; older version notes referred to upstream tarballs and features that this fork no longer ships.

---

## 1.1.0 (current) {#release-1-1-0}

Bulk calibrated two-view estimation now carries monodepth depth priors and returns relative depth-map scale.

### Bulk two-view API

- **`BulkEstimateTwoViewInfo`** accepts optional per-correspondence `depth_i` / `depth_j` (1-D float arrays, length \(N\)) and sets `Feature.depth_prior` when building flat correspondences. With `EstimateTwoViewInfoOptions.use_monodepth=True` and ≥95% valid depths, each pair uses the calibrated monodepth 3-pt path (same as scalar `EstimateTwoViewInfo`).
- Return dict gains **`scales`**: per-pair `TwoViewInfo.scale_estimate` (\(-1\) when unset / unused). Existing keys (`success`, `orientations`, `positions`, `inlier_offsets`, `inlier_indices`) are unchanged.
- Smoke test: `pytests/sfm/test_bulk_monodepth_two_view.py`.

Narrative: [RANSAC — monocular-depth-assisted two-view](ransac.md#monocular-depth-assisted-two-view-estimation), [Performance — GIL / bulk](performance.md#python--pybind11-boundary), [Estimators — relative pose](estimators.md#estimators-relative).

---

## 1.0.9 {#release-1-0-9}

Faster two-view / PnP RANSAC paths: PoseLib-adapted minimal solvers plus dense LM local optimization that replaces Ceres in the LO hot path.

### Two-view estimation

- **Sturm-sequence 5-point** essential-matrix solver ([`FivePointRelativePoseSturm`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/five_point_relative_pose_sturm.h), [`theia/math/sturm.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/math/sturm.h)), adapted from [PoseLib](bibliography.md#LarssonPoseLib) / [Nistér](bibliography.md#Nister). Default for `EstimateRelativePose` / essential-matrix RANSAC via `RansacParameters.use_sturm_5pt` (default `true`).
- **RANSAC-loop speedups** for relative pose: Sampson-only scoring with a single final essential-matrix decomposition, vectorized Sampson residuals, less allocation churn in `SampleConsensusEstimator`.
- **Monodepth 3-point** relative-pose solvers and estimators ([`relative_pose_monodepth_3pt`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/relative_pose_monodepth_3pt.h), [`EstimateMonoDepthRelativePose`](estimators.md#estimate-monodepth-relative-pose) + shared-/varying-focal variants), adapted from PoseLib RePoseD [DingRePoseD2025](bibliography.md#DingRePoseD2025). Wired into `EstimateTwoViewInfo` via `use_monodepth` / `monodepth_shared_focal` when correspondences carry `Feature.depth_prior`.

Narrative: [RANSAC](ransac.md), [Performance](performance.md), [Pose — Sturm 5-pt](pose.md#section-five_point_essential_matrix_sturm), [Pose — monodepth 3-pt](pose.md#section-monodepth_relative_pose), [Pose — dense LM refiners](pose.md#section-dense-lm-pose-refinement), [Math — Sturm](math.md#section-sturm), [Math — lmlsq](math.md#section-dense-lm-lmlsq).

### Dense LM RANSAC local optimization (`use_lo`)

New tiny Levenberg–Marquardt stack under [`theia/math/lmlsq/`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/math/lmlsq) (truncated loss + dense normal equations). Model-specific refiners (PoseLib-style Jacobians, Theia types) replace Ceres problem setup on every LO call:

| Estimator | Refiner | Residual / DoF |
|-----------|---------|----------------|
| `EstimateRelativePose` | [`refine_relative_pose`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_relative_pose.h) | Sampson; **5** |
| `EstimateMonoDepthRelativePose` (+ shared/varying focal) | [`refine_monodepth_relative_pose`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_monodepth_relative_pose.h) | Sampson (+ \(F\)) and depth reprojection; **7–9** |
| `EstimateCalibratedAbsolutePose` | [`refine_absolute_pose`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_absolute_pose.h) | Reprojection; **6** |

Full-scene / multi-view bundle adjustment is unchanged and still uses Ceres. Details: [RANSAC — local optimization](ransac.md#ransac-local-optimization), [Pose — dense LM refiners](pose.md#section-dense-lm-pose-refinement).

### Docs and licensing

- MkDocs pages updated for the new solvers and LO path ([RANSAC](ransac.md), [Estimators](estimators.md), [Performance](performance.md), [Math](math.md), [Bundle adjustment](bundle_adjustment.md), [Pose](pose.md)).
- PoseLib BSD-3-Clause attribution extended to the ported refiners ([License](license.md#third-party-code-poselib), [README Acknowledgements](https://github.com/urbste/pyTheiaSfM#acknowledgements)).

---

## 1.0.x

Stable Python packaging line with MkDocs documentation, cross-run alignment APIs, and continued binding coverage for cameras, BA, pose solvers, view graphs, and I/O (COLMAP, Bundler, NVM, PLY, Nerfstudio, SDFStudio, etc.).

### Highlights since 0.6.0

- **Versioning** aligned across root `VERSION`, CMake `project()`, and `pyproject.toml`.
- **Documentation** maintained as MkDocs under `docs/content/` (Sphinx-era pages removed).
- **Cross-run alignment** APIs documented in [Cross-run alignment](cross_run_alignment.md) and [Transformations](transformations.md) (BA relative pose edges, Sim(3) pose graph, scale-invariant odometry).
- **Examples:** lightweight scripts in `pyexamples/`; showcase `examples/vismatch_sfm/`; Nerfstudio export via `pyexamples/nerfstudio_export_reconstruction.py`.
- **Wheels:** manylinux builds for Python 3.8–3.13 via Docker (`urbste/pytheia_base`).
- **1.0.9:** Sturm 5-pt + monodepth 3-pt solvers; dense LM RANSAC LO for relative / monodepth / calibrated absolute pose — see [1.0.9 notes](#release-1-0-9).
- **1.1.0:** `BulkEstimateTwoViewInfo` depth priors + `scales` — see [1.1.0 notes](#release-1-1-0).

### Known limitations

- C++ unit tests exist but are mostly unwired in CMake; CI builds wheels without running pytest or CTest. A growing subset of pose / estimator / `lmlsq` tests can be enabled with `BUILD_TESTING=ON`.
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
