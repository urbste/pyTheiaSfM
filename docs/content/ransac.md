# RANSAC and robust estimation {#documentation-ransac}

[Random sample consensus](https://en.wikipedia.org/wiki/RANSAC) is the usual outer loop around minimal **pose** and **two-view** solvers in Theia. pyTheia does **not** expose the full C++ `Estimator` / `SampleConsensusEstimator` template API; you typically call **`pytheia.sfm`** helpers that take **`pytheia.solvers.RansacParameters`** and **`pytheia.sfm.RansacType`**, and return **`(success, model, pytheia.solvers.RansacSummary)`**. See [Geometric estimators](estimators.md).

## Variants used by SfM estimators (`RansacType`)

The factory in [`create_and_initialize_ransac_variant.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/create_and_initialize_ransac_variant.h) selects the sample-consensus implementation:

| `pytheia.sfm.RansacType` | C++ class | Role |
|--------------------------|-----------|------|
| **`RANSAC`** | [`Ransac`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/ransac.h) | Uniform random minimal samples ([Fischler](bibliography.md#Fischler)). Default for most call sites. |
| **`PROSAC`** | [`Prosac`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/prosac.h) | Progressive sampling ([Chum](bibliography.md#Chum)). **Input correspondences must be sorted best-first** (e.g. by match score). |
| **`LMED`** | [`LMed`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/lmed.h) | Least median of squares ([Rousseeuw](bibliography.md#Rousseeuw)); automatic inlier thresholding, at most ~50% outliers. |
| **`EXHAUSTIVE`** | [`ExhaustiveRansac`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/exhaustive_ransac.h) | Enumerates minimal samples (only feasible for very small pools; uses [`ExhaustiveSampler`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/exhaustive_sampler.h) with **sample size 2**). |

Older forks of Theia documented **ARRSAC** (preemptive / real-time RANSAC) and **EVSAC** (match-score modeling). **Those are not hooked into this factory** and are **not** selectable via `RansacType`. **EVSAC** and its vendored stats/optimization dependencies have been removed from this tree; **geometric estimators bundled in pyTheia use only the four types above**.

## `RansacParameters` (`pytheia.solvers`)

Aligned with [`RansacParameters`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/sample_consensus_estimator.h) in C++:

| Field | Notes |
|-------|--------|
| **`rng`** | Optional `pytheia.solvers.RandomNumberGenerator`. **Set this to a seeded generator for reproducible/deterministic RANSAC runs** (e.g. in tests); if left unset, a time-seeded generator is created internally and results will vary run to run. |
| **`error_thresh`** | Inlier threshold for the estimator’s residuals (often squared error). **You should set this** for meaningful results. |
| **`failure_probability`** | Target failure probability (default `0.01`). |
| **`min_inlier_ratio`** | Lower bound on inlier fraction used to cap iteration count (default `0`). |
| **`min_iterations`** / **`max_iterations`** | Iteration bounds. |
| **`use_mle`** | Prefer MLE-style scoring ([Torr](bibliography.md#Torr)) instead of raw inlier count. |
| **`use_lo`** / **`lo_start_iterations`** | LO-RANSAC-style local refinement when the estimator implements `RefineModel` (see [Local optimization](#ransac-local-optimization) below). |
| **`use_Tdd_test`** | Reserved for the Td,d test ([ChumRandomizedRansac](bibliography.md#ChumRandomizedRansac)); **not implemented** in the current loop—leave `false`. |
| **`use_sturm_5pt`** | Default `true`. Use the Sturm-sequence-based 5-point minimal solver ([`FivePointRelativePoseSturm`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/five_point_relative_pose_sturm.h), ported from [PoseLib](bibliography.md#LarssonPoseLib), [Nistér](bibliography.md#Nister)) instead of theia's own Stewénius-style eigendecomposition solver ([Stewenius5pt](bibliography.md#Stewenius5pt)) inside the two relative-pose/essential-matrix RANSAC estimators. Faster since it never forms a 10x10 eigendecomposition; only applies to exactly-5-point minimal samples (non-minimal calls, e.g. `n > 5`, always use the original solver). See [Pose — relative pose](pose.md) and [Performance](performance.md). |

`RansacSummary` exposes **`inliers`**, **`num_iterations`**, **`confidence`**, **`num_lo_iterations`**, etc., after a run.

## Local optimization (`use_lo`) {#ransac-local-optimization}

When **`use_lo=true`**, after a new best model is found (and again on the final inlier set) the estimator’s `RefineModel` runs on the current inliers. Several hot-path estimators implement this with a **dense Levenberg–Marquardt** stack under [`theia/math/lmlsq/`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/math/lmlsq/) (truncated robust loss + normal equations on a tiny parameter block) rather than building a Ceres problem per LO call:

| Estimator | Refiner (C++) | Residual / DoF |
|-----------|---------------|----------------|
| `EstimateRelativePose` | [`refine_relative_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_relative_pose.h) | Sampson on \(E\); **5** (rotation + translation tangent) |
| `EstimateMonoDepthRelativePose` (+ shared/varying focal) | [`refine_monodepth_relative_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_monodepth_relative_pose.h) | Sampson (+ \(F\) for uncalibrated) and monodepth reprojection; **7–9** |
| `EstimateCalibratedAbsolutePose` | [`refine_absolute_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/refine_absolute_pose.h) | Reprojection; **6** |

Jacobians / steps follow [PoseLib](bibliography.md#LarssonPoseLib)’s robust optimizers (adapted into Theia types; Theia’s RANSAC loop is unchanged). Truncation width is `error_thresh` in the same units as the estimator’s residuals (typically **squared** Sampson or reprojection error). Full-scene bundle adjustment still uses Ceres — see [Bundle adjustment](bundle_adjustment.md).

## Monocular-depth-assisted two-view estimation

`pytheia.sfm.EstimateTwoViewInfoOptions.use_monodepth` (default `false`) switches `EstimateTwoViewInfo` to a **3-point** minimal solver family ([`MonoDepthRelativePose3pt`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/relative_pose_monodepth_3pt.h) and its shared-/varying-focal variants, ported from [PoseLib](bibliography.md#LarssonPoseLib)'s RePoseD, [DingRePoseD2025](bibliography.md#DingRePoseD2025)) whenever each correspondence's `Feature.depth_prior` (e.g. from a monocular depth network) is set on both sides. Dropping the minimal sample size from 5 (or 8, uncalibrated) to 3 sharply reduces the number of RANSAC iterations needed for a given inlier ratio and confidence — see the benchmark in `dev/benchmark_two_view_estimation.py`. `EstimateTwoViewInfoOptions.monodepth_shared_focal` (default `true`) selects between the shared- and varying-focal uncalibrated variants. If fewer than 95% of correspondences carry a valid depth prior, `EstimateTwoViewInfo` logs a warning once and falls back to the standard (depth-free) estimator. The recovered relative scale between the two (possibly differently-scaled) depth maps is written to `TwoViewInfo.scale_estimate`.

## SPRT and other internals

Global RANSAC acceleration via **sequential probability ratio tests** (SPRT) appears in the math layer ([Matas](bibliography.md#Matas)); see [Math — SPRT](math.md#section-sprt). That is separate from the `RansacType` switch above.

## C++: custom estimators

If you extend Theia in C++, the pattern is still: subclass [`Estimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/estimator.h), then instantiate **`Ransac<YourEstimator>`**, **`Prosac<>`**, **`LMed<>`**, or **`ExhaustiveRansac<>`** and call **`Estimate`**. New variants plug in by subclassing [`SampleConsensusEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/sample_consensus_estimator.h) with a custom [`Sampler`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/sampler.h) / quality metric—then wire them into your own code paths (the core SfM factory only knows the four `RansacType` values listed here).
