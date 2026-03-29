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
| **`error_thresh`** | Inlier threshold for the estimator’s residuals (often squared error). **You should set this** for meaningful results. |
| **`failure_probability`** | Target failure probability (default `0.01`). |
| **`min_inlier_ratio`** | Lower bound on inlier fraction used to cap iteration count (default `0`). |
| **`min_iterations`** / **`max_iterations`** | Iteration bounds. |
| **`use_mle`** | Prefer MLE-style scoring ([Torr](bibliography.md#Torr)) instead of raw inlier count. |
| **`use_lo`** / **`lo_start_iterations`** | LO-RANSAC-style local refinement when the estimator implements `RefineModel`. |
| **`use_Tdd_test`** | Reserved for the Td,d test ([ChumRandomizedRansac](bibliography.md#ChumRandomizedRansac)); **not implemented** in the current loop—leave `false`. |

`RansacSummary` exposes **`inliers`**, **`num_iterations`**, **`confidence`**, **`num_lo_iterations`**, etc., after a run.

## SPRT and other internals

Global RANSAC acceleration via **sequential probability ratio tests** (SPRT) appears in the math layer ([Matas](bibliography.md#Matas)); see [Math — SPRT](math.md#section-sprt). That is separate from the `RansacType` switch above.

## C++: custom estimators

If you extend Theia in C++, the pattern is still: subclass [`Estimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/estimator.h), then instantiate **`Ransac<YourEstimator>`**, **`Prosac<>`**, **`LMed<>`**, or **`ExhaustiveRansac<>`** and call **`Estimate`**. New variants plug in by subclassing [`SampleConsensusEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/sample_consensus_estimator.h) with a custom [`Sampler`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/solvers/sampler.h) / quality metric—then wire them into your own code paths (the core SfM factory only knows the four `RansacType` values listed here).
