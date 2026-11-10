# Releases {#chapter-releases}

This page records **pyTheia** releases. It no longer mirrors the historical [TheiaSfM](http://www.theia-sfm.org) upstream changelog; older version notes referred to upstream tarballs and features that this fork no longer ships.

---

## 1.4.1 (unreleased) {#release-1-4-1}

### Features

- **Analytic Jacobians for bundle adjustment** (`BundleAdjustmentOptions.analytic_jacobians`, default `True`). Hand-derived Jacobians replace Ceres autodiff for the reprojection residual of `PINHOLE`, `PINHOLE_RADIAL_TANGENTIAL`, `FISHEYE`, `DOUBLE_SPHERE`, `EXTENDED_UNIFIED`, `FOV`, `DIVISION_UNDISTORTION`, `FULL_OPENCV`, `RADIAL_FISHEYE`, `THIN_PRISM_FISHEYE` and `RAD_TAN_THIN_PRISM_FISHEYE`, for both the XYZ and the inverse-depth parametrization and in `BundleAdjustTwoViews`. They agree with autodiff to about 1e-8 relative and make the Jacobian phase about 3x faster (solve time 1.2-4x lower on small and medium problems, more with inverse depth, tending to 1.1x on large problems where the linear solve dominates). `ORTHOGRAPHIC` and `Sim3` residuals stay on autodiff; set `analytic_jacobians = False` to use autodiff everywhere. SuiteSparse (GPL) is intentionally not used. See [Bundle adjustment](bundle_adjustment.md#ba-analytic-jacobians).

- **3D point priors** (ground control points, LiDAR or other known points). A `Track` can carry a prior position and a sqrt information (`Track.SetPositionPrior`) or a covariance (`Track.SetPositionPriorCovariance`, returns `False` if it is not positive definite), with accessors `HasPositionPrior`, `PositionPrior`, `PositionPriorSqrtInformation` and `RemovePositionPrior`. With `BundleAdjustmentOptions.use_point_priors = True` each optimized track with a prior adds the residual `sqrt_information * (X - prior)`; `robust_loss_width_point_prior` (default 0, no robust loss) optionally makes it robust. Tracks held constant get no prior, and the inverse-depth parametrization does not support point priors (they are ignored with a warning). Tracks are serialized as version 2 of the `Track` archive; existing `.recon` files still load (without priors). See [Bundle adjustment](bundle_adjustment.md#ba-point-priors).

- **View graph consistency filters** (opt-in, both default `0` = off): `ReconstructionEstimatorOptions.view_graph_min_relative_inliers` (drop edges with fewer verified matches than this fraction of the strongest edge of either endpoint view; also `pytheia.sfm.FilterViewGraphByRelativeInliers`) and `view_graph_max_cycle_rotation_error_degrees` (triplet rotation-loop filter, as `FilterViewGraphCyclesByRotation`). Applied by the global, hybrid and incremental estimators before estimation. In our DISK+LightGlue (8192 keypoints) benchmark, both together removed the single-scene failures (mean-of-means over 13 scenes for nonlinear global SfM dropped from 159 to 0.04 and for incremental SfM from 0.41 to 0.05, with similar medians); on SIFT the relative-inlier filter lost registered views, so enable it for dense or learned-feature matches only. After filtering, the estimators also prune the tracks to the remaining edges (an observation is kept only if its view still has an edge to another view of the track; also available as `pytheia.sfm.PruneTracksToViewGraph`). This is only a partial safeguard: a wrong edge may already have merged two true tracks, which cannot be undone afterwards. For the full effect, filter the view graph (`FilterViewGraphByRelativeInliers`, `FilterViewGraphCyclesByRotation`) BEFORE building the tracks and only add the correspondences of the surviving edges to the `TrackBuilder`.

### Changed defaults

- **`EstimateTwoViewInfoOptions` now follows COLMAP's two-view geometry defaults:** `max_sampson_error_pixels = 4`, `expected_ransac_confidence = 0.999`, `min_ransac_iterations = 100`, `max_ransac_iterations = 10000`, new `min_inlier_ratio = 0.25`, and `use_lo = True` (was off). The Sampson threshold is no longer rescaled with the image size (it used to be defined for a 1024 pixel wide image and scaled by `max(w, h) / 1024`); set `scale_threshold_with_resolution = True` for the old behavior. `ReconstructionEstimatorOptions.ransac_min_iterations / ransac_max_iterations` are now 100 / 10000.

### Fixes

- **Docs and checks for RANSAC thresholds:** `RansacParameters.error_thresh` is the squared error in normalized (`inv(K)`) coordinates, so a pixel threshold is `(px / f) ** 2`; this is now stated in the header, binding docstring and docs. `FourPointsPoseFocalLengthRadialDistortion` and the radial absolute pose estimators now raise `ValueError` for invalid distortion bounds (both must be <= 0, `max_radial_distortion` is the lower bound) instead of aborting the process.
- `pt.sfm.SQPnP` is callable again (the `PnPType` enum value no longer shadows the solver; use `PnPType.SQPnP`).
- `EstimateTriangulation` no longer reports success with zero inliers (and returns a zero point instead of uninitialized memory).
- `ColorizeReconstruction` raised no error but left all tracks black (the implementation was a stub); it now raises `ValueError`. Set colors with `Track.SetColor`.
- `View.Set{Position,Orientation,Gravity}Prior`: the matrix argument is the SQUARE ROOT of the information matrix (documented; argument names unchanged for compatibility).
- `EstimateTwoViewInfo` aborted the process (glog `CHECK` in the RANSAC sampler) for image pairs with fewer correspondences than a minimal sample; it now returns `False` for fewer than 5 (calibrated) or 8 (uncalibrated) correspondences.
- **Angular epipolar error** (`AngularEpipolarError`, used by `BundleAdjustTwoViewsAngular` and the refinement in `EstimateUncalibratedRelativePose`): the quadratic term mixed `R * f2` with `R^T * f2`, so for most correct matches the square-root argument was negative and the residual fell back to its constant 1000; the refinement therefore did not move (or moved in an arbitrary direction). Both rays are now rotated into camera 1 consistently and normalized, and the residual is the angular epipolar error itself (the square root of the former squared-sine expression, so least squares no longer minimizes fourth powers of the angles); invalid configurations get a residual of 1 instead of 1000. Robust loss widths for this residual are therefore in radians. `TwoViewInfo.position_2` is, as documented, the position of camera 2 in the frame of camera 1.
- **Hybrid SfM (`HybridReconstructionEstimator`)** was considerably less accurate than the global and incremental pipelines and failed on weakly textured scenes. Five fixes: (1) the camera orientations from global rotation averaging were held constant in every bundle adjustment and never refined; the estimator now ends with a re-triangulation and a full bundle adjustment with free orientations. (2) When an initial-pair candidate was rejected, the first camera of the next candidate kept the position from the previous attempt, so most of its triangulations failed; it is now reset to the origin, and the known-orientation initialization requires the orientations of both views (it accepted pairs with only one). (3) The initial pair needs 100 triangulated tracks instead of 200, as in the incremental estimator. (4) Candidate initial pairs were ranked by the angle between the two principal rays, which favors pure rotations without baseline; they are now ranked by the median parallax of the matched features (angle between the world-frame rays, using the known orientations), as in the original TheiaSfM code. (5) Like the global estimator, it now reconstructs only the largest connected component of the view graph; a disconnected view graph made rotation averaging abort (`Map key not found` in `FindOrDie`). On 13 ETH3D scenes with SIFT matches and calibrated intrinsics the median camera-center error dropped to the level of global SfM (e.g. courtyard 45 -> 5.3 mm), and `meadow` and `office` no longer fail. With the parallax criterion the initial pair is also independent of the random seed.

- **Inverse-depth bundle adjustment** (`use_inverse_depth_parametrization = True`): it produced NaN residuals when the inverse depths had not been initialized, and several entry points optimized stale anchors or the wrong state. The reference view, the unit bearing and the inverse depth are now derived from the current 3D point of each track right before the track is added (`Reconstruction.InitializeTrackInverseDepth`, also bound for single tracks), so `Reconstruction.InitializeInverseDepth()` is optional and poses that moved cannot leave stale anchors. The inverse depth is `w / range` along a unit bearing (it was `1 / z` with a bearing that was not unit for the fisheye and sphere models), so wide-angle models work. The residual is evaluated without dividing by the inverse depth and matches the XYZ residual at the same point for all central camera models; the inverse depth is bounded below by 0. The reference-view residual no longer carries the pose, tracks that cannot be represented are skipped instead of corrupting the solve (and their points are no longer overwritten from stale inverse depths), `BundleAdjustPartialReconstruction` and `BundleAdjustPartialViewsConstant` now keep views outside the requested set constant, `BundleAdjustTrack` and `BundleAdjustTracks` no longer optimize the intrinsics, the covariance overloads no longer overwrite the optimized point with a stale inverse depth, `BundleAdjustReconstructionWithConstantTracks` no longer crashes when the option is set, and `Track::RemoveView` keeps the reference view when the removed view is not the reference. New: `Track.SetReferenceViewId`, `BundleAdjuster.AddInvTrack`. See [Bundle adjustment](bundle_adjustment.md#ba-inverse-depth).

---

## 1.4.0 (unreleased) {#release-1-4-0}

Tooling around the library: command line, browser viewer and exporters for Gaussian splatting.

### Features

- **Gaussian splatting export.** `WriteColmapFiles(..., binary=True)` writes the COLMAP binary model (`cameras.bin`, `images.bin`, `points3D.bin`). `WriteTransformsJson` writes `transforms.json` with OpenGL camera-to-world matrices. The new `pytheia.export.export_for_gaussian_splatting` writes undistorted images, `sparse/0` and a seed `points3d.ply` in one call. See [Gaussian splatting export](export.md).
- **Browser viewer.** New `pytheia.viewer` package: `export_html(recon, path)` writes one self-contained HTML file (three.js r160 and OrbitControls inlined, scene data as base64 buffers, works offline); `show(recon)` embeds it in a notebook and `serve(recon, port)` serves it locally. Orbit controls, camera frusta, point size, RGB / track-length / reprojection-error colors, error and track-length filters, click-to-inspect cameras (thumbnails) and points (track residuals), per-view table, unregistered-view highlight, dark/light theme, PNG screenshot. Also: compare mode (`viewer.compare(a, b, path)`) aligns a second reconstruction onto the first with a Sim(3) fit on shared view names and shows per-camera errors; depth coloring, camera trajectory, a two-click measure tool and a WASD fly mode. Only numpy is required. See [Browser viewer](viewer.md).
- **Command line interface.** New `pytheia` command (also `python -m pytheia`) with `info`, `convert`, `view`, `export-splat`, `align` and `sfm` subcommands. `sfm` takes every `ReconstructionEstimatorOptions` field from a JSON config and `--set key=value` overrides, and `--seed` for reproducible runs. See [Command line interface](cli.md).
- **Native applications.** Built with `-DBUILD_APPS=ON` (off by default, not in the wheel): `theia_global_sfm`, `theia_incremental_sfm` and `theia_hybrid_sfm` read the same `reconstruction.recon` + `view_graph.bin` input directory as `pytheia sfm` and take the same `--config` / `--set` options; `theia_info` reports a reconstruction and `theia_convert` writes COLMAP text or binary, NVM, Bundler, PLY, nerfstudio, `transforms.json` and JSON. See [Native applications](cli.md#native-applications).
- **COLMAP camera models.** New intrinsics models `FULL_OPENCV`, `RADIAL_FISHEYE`, `THIN_PRISM_FISHEYE` and `RAD_TAN_THIN_PRISM_FISHEYE` (`FullOpenCVCameraModel`, `RadialFisheyeCameraModel`, `ThinPrismFisheyeCameraModel`, `RadTanThinPrismFisheyeCameraModel`), with autodiff for bundle adjustment and a Newton inverse. `read_colmap_model` imports every COLMAP model (including `OPENCV_FISHEYE`, `FOV`, `RADIAL_FISHEYE`, `SIMPLE_RADIAL_FISHEYE`) without dropping terms, and `WriteColmapFiles` writes the four new models back as their COLMAP models, in text and binary. `CameraIntrinsicsPrior` gained `radial_distortion_6` and `thin_prism_distortion` (serialization version 5).
- **Importers.** New pure-Python `pytheia.importers` module: `read_colmap_model(path)` reads a COLMAP or GLOMAP sparse model in text or binary form (`SIMPLE_PINHOLE`, `PINHOLE`, `SIMPLE_RADIAL`, `RADIAL`, `OPENCV`, `FULL_OPENCV` and the fisheye and FOV models, all without dropped terms) into a `pt.sfm.Reconstruction`. `read_openmvg_sfm_data(path)` reads an OpenMVG `sfm_data.json` (views, pinhole and radial intrinsics, poses, structure). `read_colmap_database(path)` returns keypoints, matches and two-view geometries of a COLMAP `database.db` as numpy arrays. See [Importers](importers.md).

### Fixes

- **`WriteColmapFiles` text cameras**: the second radial term was written as `k1` (`... k1 k1`). It now writes `k2`.
- **`FISHEYE` unprojection**: `FisheyeCameraModel::UndistortPoint` used a fixed-point iteration that did not converge at large ray angles (about 0.8 px round-trip error at 80 degrees) while still returning success. It now uses the same Newton inverse as the other COLMAP fisheye models, with round-trip tests up to 85 degrees.
- **Bundle adjustment with unobserved views**: estimated views without any observation made `BundleAdjustReconstruction` and the other `BundleAdjust*` entry points fail with "The program has N parameter blocks, but the parameter block ordering has N+4". Such views are now left out of the linear solver ordering, and the covariance queries no longer abort on blocks that are not in the problem.
- **`BundleAdjustPartialReconstruction`**: the track ids used to refresh the inverse depths were padded with zeros. Only the requested tracks are now updated.
- **`NonlinearPositionEstimatorOptions`**: `min_num_points_per_view`, `point_to_camera_weight` and `max_power_iterations` were bound to the wrong C++ members (for example `min_num_points_per_view` wrote `num_threads`). They now map correctly; `max_num_iterations` and `robust_loss_width` were added, and the old names `max_power_iterations` and `eigensolver_threshold` stay as aliases.
- **Incremental SfM**: the initial pair now resets camera 1 to the identity pose, so inputs that already contain poses (such as a `.recon` file) no longer fail triangulation of the initial pair. Views without an image size are skipped with an error message instead of aborting on a `CHECK`.
- **`ViewGraph::AddEdge`**: documented that `TwoViewInfo` is stored unchanged and read as larger id relative to smaller id (use `SwapCameras` for reversed ids).

---

## 1.3.2 {#release-1-3-2}

Robustness: a robust default bundle-adjustment loss, and fixes for the `LINEAR_TRIPLET` and `LIGT` global position estimators.

### Defaults

- **BA loss is now `HUBER` with a 1 px width** (was `TRIVIAL` / pure L2 with a 10 px width) in both `ReconstructionEstimatorOptions` and `BundleAdjustmentOptions`. A plain L2 loss is dominated by the few wrong correspondences that survive geometric verification. On ETH3D (12 scenes, SIFT, calibrated, global SfM) the median camera-center error drops from 4.5 to 3.5 mm and the mean error from 37 to 18 mm. A Huber width of 10 px is practically identical to `TRIVIAL`, so the width matters as much as the loss. Set `LossFunctionType.TRIVIAL` to get the old behavior.

### Features

- **Keyword arguments everywhere.** All pybind11 bindings (`sfm`, `solvers`, `math`, `io`, `matching`, `mvs`) now name their arguments after the C++ parameters, so `pt.sfm.Feature(point=..., covariance=...)` works. Positional calls are unchanged. Stubs show the names.
- **Feature uncertainty in bundle adjustment.** The reprojection residual is whitened with the full 2x2 `Feature.covariance` (it used only the diagonal before). New `Feature.set_isotropic_sigma(sigma)` and `Feature.sqrt_information()`. The comment on `covariance_` said "standard deviation"; it is a covariance in px^2.

### Fixes

- **`LINEAR_TRIPLET`**: triplets whose baseline ratios could not be computed (too few well-conditioned common tracks) were kept with baseline `(0, 0, 0)`, which turns the ratios into `NaN`, poisons the constraint matrix and made the eigensolver throw (`TridiagEigen: eigen decomposition failed`). Such triplets are now discarded. Registered views on ETH3D rise from 16 % to 64 % (scenes that work: 2/13 to 9/13).
- **`LIGT`**: the smallest eigenvector of the constraint matrix only fixes `||x|| = 1`, so a weakly constrained view can take almost all of the norm and the rest collapse (registered 0 of 45 views on ETH3D `electro`, 0 of 76 on `facade`). The scale is now fixed with a ShapeFit-style constraint that sums the two-view translation directions (`LiGTPositionEstimatorOptions.use_scale_constraint`, default `True`; one sparse solve, falls back to the eigenvector). The system is restricted to the largest connected set of views, with the best constrained view fixed at the origin. Constraints are also row-normalized and re-weighted with Huber IRLS (`normalize_constraints`, `num_irls_iterations`, `irls_scale_multiplier`). On 13 ETH3D scenes with SIFT matches, every scene now registers at least 44 of 45 views (previously 4 scenes failed); median errors are 1-12 mm except `meadow` (56 mm). Also removed a stray debug `std::cout` print.

### Known limitations

- With deep matchers (SuperPoint/ALIKED + LightGlue) on repetitive scenes up to half of the view-graph edges can be wrong in a mutually consistent way, so triplet-loop, rotation and translation-direction filters cannot remove them; pyTheia registers fewer views there than COLMAP/GLOMAP.

---

## 1.3.1 {#release-1-3-1}

Opt-in reproducible RNG seeding and a bundle-adjuster fix.

### Reproducible runs

- **Fixed seed for default-constructed RNGs.** RANSAC samplers and global estimators that are not handed a `RandomNumberGenerator` seed from the clock by default. They can now use a fixed seed, so repeated runs draw identical sample sequences.
    - Python: `pt.solvers.SetDefaultRandomSeed(seed)`, `pt.solvers.ClearDefaultRandomSeed()`, `pt.solvers.HasDefaultRandomSeed()`.
    - C++: the same names in the `theia::` namespace (`theia/util/random.h`).
    - Environment: `THEIA_RANDOM_SEED=<unsigned decimal>` sets the seed at process start. An invalid value is ignored and the clock is used.
    - The seed is process-wide and re-applied on every default construction, so each RANSAC / estimator run restarts from the same state. Without a seed, behavior is unchanged.

### Fixes

- `BundleAdjuster::SetCameraExtrinsicsConstant` no longer aborts in Ceres for a view that has no residual block (all observations removed, no gravity prior). The extrinsics block is added before it is marked constant.

### Maintenance

- Type stubs regenerated with pybind11-stubgen 3.0.0; they now include the seed functions and previously missing entries such as `io.WriteRigPlyFile`.
- Bumped vulnerable pins in the vendored pybind11 `requirements.txt` files (no effect on the built package).

Docs: [RANSAC](ransac.md).

---

## 1.3.0 {#release-1-3-0}

Compact binary format for reconstructions.

### I/O

- **Reconstruction format version 2.** `ReadReconstruction` / `WriteReconstruction` write a compact, lossless layout: about 27 B/observation instead of 191 on a synthetic 50-view / 2000-track set.
- Older files (Reconstruction version 0 or 1) still load exactly. Files written by 1.3.x cannot be read by 1.2.x.
- Tests and fixtures: `pytests/io/test_compact_reconstruction_format.py` (legacy load, lossless round-trip, size).

Docs: [IO — binary `.recon` format](io.md#binary-recon-format-compact).

---

## 1.2.1 {#release-1-2-1}

Rig pipeline hardening on top of 1.2.0: metric capture–capture relative pose, clearer global-rig logging, rig PLY export, and example cleanup.

### Metric relative rig pose

- In-tree **5+1 generalized relative pose** (`FivePointOnePointGeneralizedRelativePose`) and upright 4-pt wrapper; RANSAC via **`EstimateRelativeRigInfo`** / **`EstimateRelativeRigInfoUpright`**.
- **`BuildCaptureViewGraph`** defaults to metric edges from tracks (lift rays → 5+1); View–View strip is fallback only for missing pairs.
- **`GlobalRigReconstructor`**: `capture_graph_options`, optional **`rescale_positions_to_metric_edges`**, and **INFO** logging for capture-graph / rotation / position / triangulation stages (same style as monocular global SfM).

### I/O and examples

- **`WriteRigPlyFile`** — tracks + capture trajectory + intra-capture sensor baselines (densely sampled polylines for MeshLab / Open3D).
- Reorganized **`pyexamples/`** (`preprocess/`, `stereo/`, `sfm/`, …); ZED SVO extract (`zed_svo_extract_stereo.py`); stereo example `--frames_dir`, default **xfeat**, rig PLY + mean reprojection stats.
- **`IncrementalRigReconstructorOptions`**: default BA `num_threads=1` (avoids SIGSEGV when Ceres OpenMP runs after Torch CUDA matching); stereo example clears the GPU matcher before Estimate.
- Bindings: `ReconstructionEstimatorOptions.triangulation_max_reprojection_error_in_pixels`.
- Fixes: Sim3 alignment setters API, deep-matching None deref, LoFTR confidence filtering, Open3D PLY viewer stub.

Docs: [Rigs](rigs.md), [Pose — generalized relative](pose.md#section-generalized-relative-pose), [IO](io.md), [Examples showcase](examples_showcase.md).

---

## 1.2.0 {#release-1-2-0}

Calibrated multi-camera / stereo **rig** support: abstract body frame, synchronized captures, incremental and global reconstructors, docs, and a vismatch stereo example.

### Camera rigs

- **`CameraRig` / `RigSensor` / `RigCapture`** on `Reconstruction` — one capture = one timestamp = one body pose; Views keep features/matches; sensor extrinsics live in the abstract rig CS (default body at identity).
- **`PropagateCameraPosesForCapture`** / **`BuildCaptureViewGraph`** — compose View cameras from body ⊕ calibrated sensors; derive capture–capture edges from View–View `TwoViewInfo`.
- **`IncrementalRigReconstructor`** — seed at identity, triangulate (intra-rig metric), localize later captures (generalized 2D–3D / single-view fallback).
- **`GlobalRigReconstructor`** (MGSfM-inspired) — capture graph + selectable Theia averaging backends (`GlobalRotationEstimatorType` / `GlobalPositionEstimatorType` via `sfm_options`). Extrinsics assumed **calibrated** (not optimized in averaging).
- Docs: [Rigs](rigs.md). Example: `pyexamples/stereo/stereo_rig_reconstruction.py` (baseline / intrinsics, vismatch `edm`, global or incremental).

### Tests

- `pytests/test_camera_rig.py`, `pytests/test_global_rig_reconstructor.py`.

---

## 1.1.0 {#release-1-1-0}

Bulk calibrated two-view estimation now carries monodepth depth priors and returns relative depth-map scale.

### Bulk two-view API

- **`BulkEstimateTwoViewInfo`** accepts optional per-correspondence `depth_i` / `depth_j` (1-D float arrays, length \(N\)) and sets `Feature.depth_prior` when building flat correspondences. With `EstimateTwoViewInfoOptions.use_monodepth=True` and ≥95% valid depths, each pair uses the calibrated monodepth 3-pt path (same as scalar `EstimateTwoViewInfo`).
- Return dict gains **`scales`**: per-pair `TwoViewInfo.scale_estimate` (\(-1\) when unset / unused). Existing keys (`success`, `orientations`, `positions`, `inlier_offsets`, `inlier_indices`) are unchanged.
- Smoke test: `pytests/sfm/test_bulk_monodepth_two_view.py`.

Narrative: [RANSAC — monocular-depth-assisted two-view](ransac.md#monocular-depth-assisted-two-view-estimation), [Performance — GIL / bulk](performance.md#python-pybind11-boundary), [Estimators — relative pose](estimators.md#estimators-relative).

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
- **Examples:** lightweight scripts in `pyexamples/`; showcase `examples/vismatch_sfm/`; Nerfstudio export via `pyexamples/io/nerfstudio_export_reconstruction.py`.
- **Wheels:** manylinux builds for Python 3.8–3.13 via Docker (`urbste/pytheia_base`).
- **1.0.9:** Sturm 5-pt + monodepth 3-pt solvers; dense LM RANSAC LO for relative / monodepth / calibrated absolute pose — see [1.0.9 notes](#release-1-0-9).
- **1.1.0:** `BulkEstimateTwoViewInfo` depth priors + `scales` — see [1.1.0 notes](#release-1-1-0).
- **1.2.0:** Calibrated stereo / multi-camera rigs (`CameraRig`, incremental + global reconstructors) — see [1.2.0 notes](#release-1-2-0).
- **1.2.1:** Metric 5+1 capture edges, `WriteRigPlyFile`, example / logging fixes — see [1.2.1 notes](#release-1-2-1).

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
