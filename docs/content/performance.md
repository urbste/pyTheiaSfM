# Performance {#chapter-performance}

This page describes practical performance considerations for **pyTheia** (the Python fork). Historical Strecha / 1DSfM benchmark tables from upstream TheiaSfM (2016) are **not** representative of this fork's Python-first API and are omitted here.

## Threading

- **`ReconstructionEstimatorOptions.num_threads`** defaults to **1**. Raise it for incremental, hybrid, or global reconstruction when running large scenes on multi-core hosts.
- Some internal paths (e.g. nested bundle adjustment inside incremental estimation) may still cap parallelism for numerical stability.
- **OpenMP pragmas** exist in parts of the C++ tree, but OpenMP is not wired in CMake by default; do not assume multi-threaded OpenMP without a custom build.

## Linear algebra and Ceres backends

Bundle adjustment and many solvers use **Ceres Solver**. Defaults favor portable CPU backends (`EIGEN` dense, `EIGEN_SPARSE` sparse).

- **Pre-built wheels** link a **CPU** Ceres build. For GPU linear algebra, build Ceres locally with CUDA (and optionally cuDSS for sparse CUDA), then build pyTheia against that Ceres. See [Building](building.md) and [Bundle adjustment](bundle_adjustment.md).
- **SuiteSparse / CHOLMOD:** this fork removed GPL-dependent SuiteSparse code from `sparse_cholesky_llt.cc` and uses **Eigen::SimplicialLDLT** instead. That improves portability but can be slower or slightly less stable on very large sparse problems than a SuiteSparse-enabled Ceres build.

## Python / pybind11 boundary

Most bound functions **hold the Python GIL** for the full C++ call. Long-running work (full reconstruction, bundle adjustment, single-pair two-view estimation) blocks other Python threads.

**Exception:** `BulkEstimateTwoViewInfo` releases the GIL while processing many view pairs in C++. Prefer batch APIs where available when driving pyTheia from multi-threaded Python or async code.

Passing large correspondences or descriptors through Python lists incurs conversion overhead. For hot loops, keep data in NumPy arrays and batch work on the C++ side when bindings support it.

## Local build tuning

| Setting | Effect |
|---------|--------|
| `BUILD_MARCH_NATIVE=1` (env) / `-DBUILD_WITH_MARCH_NATIVE=ON` | Enables `-march=native -mtune=native` for local max performance; **off** for portable wheels |
| CUDA-enabled Ceres | Speeds large BA when `BundleAdjustmentOptions` selects GPU dense/sparse backends |
| Higher `num_threads` on estimators | Better CPU utilization for reconstruction pipelines |

## Two-view relative pose estimation

`EstimateTwoViewInfo` / `EstimateRelativePose` were optimized along independent, composable axes (see [RANSAC and robust estimation](ransac.md) and [Pose — relative pose](pose.md) for the API):

1. **RANSAC-loop optimizations** (always on): Sampson-only scoring with a single final essential-matrix decomposition instead of one per candidate model, vectorized Sampson residuals (`SquaredSampsonDistances`), hoisted per-iteration allocations, count-only inlier scoring, and `PartialPivLU` in the 10×10 elimination.
2. **`use_sturm_5pt`** (default `true`): swaps in a Sturm-sequence-based 5-point minimal solver ([`FivePointRelativePoseSturm`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/five_point_relative_pose_sturm.h), adapted from [PoseLib](bibliography.md#LarssonPoseLib)) that avoids the eigendecomposition theia's original Stewénius-style solver performs per RANSAC iteration.
3. **`use_monodepth`** (default `false`, requires per-feature `depth_prior`): drops the minimal sample size from 5 (or 8, uncalibrated) to 3, which sharply reduces the number of RANSAC iterations needed for a given inlier ratio/confidence — the effect compounds with the outlier ratio.
4. **Dense LM local optimization** when **`use_lo=true`**: `RefineModel` for calibrated relative pose, monodepth relative pose, and calibrated absolute pose uses a small analytic LM (`theia/math/lmlsq`) instead of constructing a Ceres problem (or a one-view `Reconstruction`) on every LO call. See [RANSAC — local optimization](ransac.md#ransac-local-optimization).

Representative numbers from `dev/benchmark_two_view_estimation.py` (2000 correspondences, synthetic scene, `use_lo=false`; classic = `use_sturm_5pt=False`):

| outlier ratio | classic 5pt | Sturm 5pt | monodepth 3pt |
|---|---|---|---|
| 10% | 1.51 ms | 0.96 ms | 0.56 ms |
| 30% | 3.26 ms | 2.86 ms | 0.68 ms |
| 50% | 15.69 ms | 13.46 ms | 1.05 ms |

Absolute numbers depend heavily on machine, correspondence count, and outlier ratio — re-run the script on your own hardware/dataset before relying on these for capacity planning. With **`use_lo=true`**, wall time rises with the number of LO calls (`RansacSummary.num_lo_iterations`); the dense LM path keeps that overhead much smaller than the previous Ceres-backed LO.

## Measuring performance

For two-view relative pose estimation, run `dev/benchmark_two_view_estimation.py` (see above). For other paths, there is no general in-repo microbenchmark suite; see wall-clock patterns in `pyexamples/` and `examples/vismatch_sfm/`. When comparing changes, use the same Ceres build, thread count, and dataset.
