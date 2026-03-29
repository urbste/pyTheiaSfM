# Bundle adjustment {#documentation-bundle-adjustment}

**Bundle adjustment (BA)** jointly refines **camera poses (and optionally intrinsics)** and **3D points** by minimizing **reprojection error** (and optional priors), usually with **[Ceres Solver](http://ceres-solver.org/)**. In Theia this is centered on **`BundleAdjustmentOptions`**, **`BundleAdjustmentSummary`**, and high-level functions that take a **`Reconstruction`**.

C++: [`src/theia/sfm/bundle_adjustment/bundle_adjustment.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/bundle_adjustment/bundle_adjustment.h), incremental builder [`bundle_adjuster.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/bundle_adjustment/bundle_adjuster.h).

Python: **`pytheia.sfm`** — options, summaries, Ceres enums, and `BundleAdjust*` wrappers; see [Python API overview](python_wrapper.md).

## What is optimized {#bundle-adjustment-what}

- **Extrinsics**: camera orientation and position (per **`View`**) unless marked constant in options.
- **Structure**: **`Track`** 3D positions (homogeneous or inverse-depth depending on options).
- **Intrinsics**: only parameters selected by **`OptimizeIntrinsicsType`** bitmask (`FOCAL_LENGTH`, `PRINCIPAL_POINTS`, distortion flags, `NONE`, `ALL`, etc.). Defaults are conservative (often **no** intrinsics optimization).

Optional **priors** in **`BundleAdjustmentOptions`**: position, orientation, gravity, depth—enable only when you have reliable metadata.

## High-level entry points {#bundle-adjustment-entry-points}

| Python (`pt.sfm`) | Effect |
|-------------------|--------|
| `BundleAdjustReconstruction` | Full BA: all **estimated** views and tracks in the reconstruction. |
| `BundleAdjustPartialReconstruction` | Only the given **view** and **track** ID sets (plus their incident residuals). |
| `BundleAdjustPartialViewsConstant` | Some views **variable**, others **held fixed** (see C++ signature). |
| `BundleAdjustView` / `BundleAdjustViews` | Local BA around one or more views. |
| `BundleAdjustTrack` / `BundleAdjustTracks` | Refine selected tracks (and connected cameras). |
| `BundleAdjustTwoViewsAngular` | Specialized **two-view** angular / relative adjustment. |

Variants suffixed with **`WithCov`** return **empirical covariance** blocks for tracks or views (C++ overloads wrapped in Python).

## `BundleAdjustmentOptions` {#bundle-adjustment-options}

Defaults below are from [`bundle_adjustment.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/bundle_adjustment/bundle_adjustment.h) (C++). In **pyTheia**, all listed fields are exposed on `pt.sfm.BundleAdjustmentOptions` except where noted. Ceres enumerators are re-exported under `pt.sfm` (e.g. `pt.sfm.LinearSolverType.SPARSE_SCHUR`).

### Ceres linear solver (`linear_solver_type`) {#ba-linear-solver-type}

Maps to `ceres::SolverOptions::linear_solver_type`. Exposed as **`LinearSolverType`** in Python.

| Value | Role (short) |
|-------|----------------|
| `DENSE_QR` | Dense normal equations via QR. Small problems only. |
| `DENSE_NORMAL_CHOLESKY` | Dense normal equations + Cholesky. Small / dense structure. |
| `DENSE_SCHUR` | Dense **Schur complement** elimination. Medium problems; uses **dense** linear algebra backend. |
| `SPARSE_NORMAL_CHOLESKY` | Sparse Cholesky on the normal equations. |
| `SPARSE_SCHUR` | Sparse Schur complement (**default** in `BundleAdjustmentOptions`). Good default for many SfM-sized problems. |
| `ITERATIVE_SCHUR` | Iterative Schur + preconditioner; preferred for **very large** camera counts when memory or factorization cost bites. |
| `CGNR` | CGNR on the reduced camera system. |

**Practical note:** Header comments suggest considering **`ITERATIVE_SCHUR`** when you have on the order of **\(> 1000\)** cameras. Pair it with a Schur-appropriate **`preconditioner_type`** (often `SCHUR_JACOBI` or a **cluster** preconditioner for iterative Schur). See also the [Ceres documentation on solvers](http://ceres-solver.org/nnls_solving.html).

### Preconditioner (`preconditioner_type`) {#ba-preconditioner-type}

Exposed as **`PreconditionerType`**.

| Value | Typical use |
|-------|-------------|
| `IDENTITY` | No preconditioning (rarely best for large BA). |
| `JACOBI` | Jacobi preconditioner. |
| `SCHUR_JACOBI` | **Default.** Schur–Jacobi; works well with Schur-based solvers. |
| `CLUSTER_JACOBI` | Clustering-based Jacobi; used with **`ITERATIVE_SCHUR`** and visibility clustering. |
| `CLUSTER_TRIDIAGONAL` | Cluster tridiagonal preconditioner; likewise for iterative Schur + clusters. |

### Visibility clustering (`visibility_clustering_type`) {#ba-visibility-clustering-type}

Used when the preconditioner uses **clusters** (`CLUSTER_*`). Exposed as **`VisibilityClusteringType`**.

| Value | Meaning |
|-------|---------|
| `CANONICAL_VIEWS` | **Default.** Ceres “canonical views” clustering. |
| `SINGLE_LINKAGE` | Single-linkage style clustering. |

### Dense and sparse linear algebra libraries {#ba-linear-algebra-backends}

These select **which library implements** dense/sparse factorization and mat-vec work inside Ceres. Exposed as **`DenseLinearAlgebraLibraryType`** and **`SparseLinearAlgebraLibraryType`**.

**Dense (`dense_linear_algebra_library_type`)** — default **`EIGEN`**

| Value | Notes |
|-------|--------|
| `EIGEN` | Eigen dense linear algebra (**default**). |
| `LAPACK` | LAPACK-backed dense solves (if Ceres built with it). |
| `CUDA` | GPU dense linear algebra — only if Ceres was **compiled with CUDA** **and** pyTheia was built with `THEIA_CERES_HAS_CUDA_DENSE`. Otherwise unavailable in Python. |

**Sparse (`sparse_linear_algebra_library_type`)** — default **`EIGEN_SPARSE`**

| Value | Notes |
|-------|--------|
| `SUITE_SPARSE` | SuiteSparse (CHOLMOD etc.) when Ceres links it. |
| `EIGEN_SPARSE` | Eigen sparse (**default**). |
| `ACCELERATE_SPARSE` | Apple Accelerate sparse (platform-specific). |
| `CUDA_SPARSE` | GPU sparse — only if Ceres has CUDA sparse **and** pyTheia defines `THEIA_CERES_HAS_CUDA_SPARSE`. |

#### CUDA / GPU usage {#ba-cuda}

GPU backends are **optional** and depend entirely on your **Ceres build** and Theia’s CMake detecting CUDA support:

- For **dense** Schur-style solvers (e.g. `DENSE_SCHUR`), set **`dense_linear_algebra_library_type = CUDA`** when available.
- For **sparse** solvers (e.g. `SPARSE_SCHUR`, `ITERATIVE_SCHUR`), set **`sparse_linear_algebra_library_type = CUDA_SPARSE`** when available.

If CUDA was not enabled in Ceres, these settings either do not exist in the Python module or will not accelerate solves. **`num_threads`** still controls Ceres’ **CPU** parallelism.

### Robust loss (`loss_function_type`, widths) {#ba-robust-loss}

**`loss_function_type`** — exposed as **`LossFunctionType`** (`TRIVIAL`, `HUBER`, `SOFTLONE`, `CAUCHY`, `ARCTAN`, `TUKEY`). Implemented in [`create_loss_function.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/bundle_adjustment/create_loss_function.cc).

| Value | Effect |
|-------|--------|
| `TRIVIAL` | Pure \(L_2\) on reprojection residuals (**default** in `BundleAdjustmentOptions`). |
| `HUBER` | Huber; transitions at **`robust_loss_width`**. |
| `SOFTLONE` | `SoftLOneLoss` with scale **`robust_loss_width`**. |
| `CAUCHY` | Cauchy robust loss. |
| `ARCTAN` | Arctan robust loss. |
| `TUKEY` | Tukey; errors beyond the width are effectively capped. |

**Widths**

- **`robust_loss_width`** — default `2.0` in `BundleAdjustmentOptions`; pipeline defaults from `ReconstructionEstimatorOptions` often use `10.0` for **`bundle_adjustment_robust_loss_width`** (see below).
- **`robust_loss_width_depth_prior`** — default `0.01`; scale for robust treatment of **depth priors** when enabled.

C++ also defines **`LossFunctionType::TRUNCATED`** (`TruncatedLoss`); it is **not** currently exposed in pybind.

### Intrinsics bitmask (`intrinsics_to_optimize`) {#ba-intrinsics}

**`OptimizeIntrinsicsType`** is a **bitmask** (combine with `|` in C++; in Python use `|` on enum values).

| Flag | Bit |
|------|-----|
| `NONE` | Optimize no intrinsics (**default** on raw `BundleAdjustmentOptions`). |
| `FOCAL_LENGTH` | |
| `ASPECT_RATIO` | |
| `SKEW` | |
| `PRINCIPAL_POINTS` | |
| `RADIAL_DISTORTION` | |
| `TANGENTIAL_DISTORTION` | |
| `DISTORTION` | Alias: radial \| tangential. |
| `FOCAL_LENGTH_DISTORTION` | Focal + both distortions. |
| `FOCAL_LENGTH_RADIAL_DISTORTION` | Focal + radial. |
| `ALL` | All of the above. |

**Pipeline default** (`ReconstructionEstimatorOptions`): `FOCAL_LENGTH | RADIAL_DISTORTION` — stricter than the bare `BundleAdjustmentOptions` default.

### 3D point parametrization {#ba-point-parametrization}

| Field | Default | Meaning |
|-------|---------|---------|
| `use_homogeneous_point_parametrization` | `true` | Homogeneous point with **local tangent** (manifold) updates; 4D state with 3 DOF increments. |
| `use_inverse_depth_parametrization` | `false` | If **`true`**, inverse-depth parametrization is used and **`use_homogeneous_point_parametrization` is ignored** (per header). |

When you run **incremental / global** reconstruction, the pipeline sets these from **`TrackParametrizationType`** (`XYZW`, `XYZW_MANIFOLD`, `INVERSE_DEPTH`) via `SetBundleAdjustmentOptions` — see [SfM](sfm.md).

### Camera constants, trajectory mode, orthographic {#ba-camera-flags}

| Field | Default | Meaning |
|-------|---------|---------|
| `constant_camera_orientation` | `false` | Fix all camera rotations. |
| `constant_camera_position` | `false` | Fix all camera positions. |
| `optimize_for_forward_facing_trajectory` | `false` | Merge intrinsics + extrinsics into one parameter block for forward-facing trajectories. |
| `orthographic_camera` | `false` | Use orthographic camera residuals where applicable. |

### Priors {#ba-priors}

| Field | Default | Meaning |
|-------|---------|---------|
| `use_position_priors` | `false` | Add position prior residuals (metadata must be present on views). |
| `use_orientation_priors` | `false` | Orientation priors. |
| `use_depth_priors` | `false` | Depth priors (uses **`robust_loss_width_depth_prior`** when robust). |
| `use_gravity_priors` | `false` | Gravity-aligned orientation priors. |

### Iterations, stopping, trust region, refinement {#ba-iterations-stopping}

| Field | Default | Meaning |
|-------|---------|---------|
| `verbose` | `false` | Ceres logging (`PER_MINIMIZER_ITERATION` vs silent). |
| `num_threads` | `hardware_concurrency()` | Ceres thread count (overridden when BA is invoked from **`ReconstructionEstimatorOptions.num_threads`**). |
| `max_num_iterations` | `100` | Max optimizer iterations (pipeline default is often **50** via reconstruction options). |
| `max_solver_time_in_seconds` | `3600` | Wall-clock cap for the solver. |
| `use_inner_iterations` | `true` | Ceres inner iterations (can improve convergence). |
| `function_tolerance` | `1e-6` | Stop when relative cost change is small. |
| `gradient_tolerance` | `1e-10` | Gradient norm tolerance. |
| `parameter_tolerance` | `1e-8` | Step tolerance. |
| `max_trust_region_radius` | `1e12` | Trust-region radius cap. |
| `use_mixed_precision_solves` | `false` | Ceres mixed-precision iterative refinement. |
| `max_num_refinement_iterations` | `2` | Refinement iterations when mixed precision is on. |

### Full field list (quick reference) {#ba-options-quick-ref}

| Field | Default (C++ `BundleAdjustmentOptions`) |
|-------|----------------------------------------|
| `loss_function_type` | `TRIVIAL` |
| `robust_loss_width` | `2.0` |
| `robust_loss_width_depth_prior` | `0.01` |
| `linear_solver_type` | `SPARSE_SCHUR` |
| `preconditioner_type` | `SCHUR_JACOBI` |
| `visibility_clustering_type` | `CANONICAL_VIEWS` |
| `dense_linear_algebra_library_type` | `EIGEN` |
| `sparse_linear_algebra_library_type` | `EIGEN_SPARSE` |
| `optimize_for_forward_facing_trajectory` | `false` |
| `use_mixed_precision_solves` | `false` |
| `max_num_refinement_iterations` | `2` |
| `verbose` | `false` |
| `constant_camera_orientation` | `false` |
| `constant_camera_position` | `false` |
| `use_homogeneous_point_parametrization` | `true` |
| `use_inverse_depth_parametrization` | `false` |
| `intrinsics_to_optimize` | `NONE` |
| `num_threads` | hardware concurrency |
| `max_num_iterations` | `100` |
| `max_solver_time_in_seconds` | `3600` |
| `use_inner_iterations` | `true` |
| `function_tolerance` | `1e-6` |
| `gradient_tolerance` | `1e-10` |
| `parameter_tolerance` | `1e-8` |
| `max_trust_region_radius` | `1e12` |
| `use_position_priors` | `false` |
| `use_orientation_priors` | `false` |
| `use_depth_priors` | `false` |
| `orthographic_camera` | `false` |
| `use_gravity_priors` | `false` |

## `TwoViewBundleAdjustmentOptions` {#two-view-ba-options}

Used by **`BundleAdjustTwoViews`** (C++); Python exposes **`pt.sfm.TwoViewBundleAdjustmentOptions`** with:

- **`ba_options`** — a nested **`BundleAdjustmentOptions`**.
- **`constant_camera1_intrinsics`** / **`constant_camera2_intrinsics`** — default **`true`** (hold intrinsics fixed per camera).

**`BundleAdjustTwoViewsAngular`** takes a plain **`BundleAdjustmentOptions`** (angular / epipolar formulation; see C++ header).

## Pipeline-level BA settings (`ReconstructionEstimatorOptions`) {#reconstruction-estimator-ba}

When you use **incremental / global / hybrid** reconstruction, bundle adjustment is configured from **`ReconstructionEstimatorOptions`**. These fields are copied into **`BundleAdjustmentOptions`** by **`SetBundleAdjustmentOptions`** (see [`reconstruction_estimator_utils.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/reconstruction_estimator_utils.cc)):

| Field | Role |
|-------|------|
| `bundle_adjustment_loss_function_type` | Maps to `loss_function_type`. |
| `bundle_adjustment_robust_loss_width` | Maps to `robust_loss_width` (default **10.0** here vs **2.0** on raw BA options). |
| `intrinsics_to_optimize` | Same bitmask as BA. |
| `track_parametrization_type` | Sets `use_homogeneous_point_parametrization` / `use_inverse_depth_parametrization`. |
| `dense_linear_algebra_library_type` | Same as BA. |
| `sparse_linear_algebra_library_type` | Same as BA. |
| `linear_solver_type` | Same as BA. |
| `preconditioner_type` | Same as BA. |
| `visibility_clustering_type` | Same as BA. |
| `max_num_iterations` | BA iteration cap (default **50** in reconstruction options). |
| `use_inner_iterations` | Passed through (stored as `int` in the struct). |
| `optimize_for_forward_facing_trajectory` | Passed through. |
| `num_threads` | Sets BA **`num_threads`**. |

**Scheduling / quality trade-offs** (do not map 1:1 to a single `BundleAdjustmentOptions` field):

- **`full_bundle_adjustment_growth_percent`** — how much the model grows before a **full** BA (incremental/hybrid).
- **`partial_bundle_adjustment_num_views`** — window size for **partial** BA.
- **`subsample_tracks_for_bundle_adjustment`** and related **track selection** thresholds — reduce the number of 3D points in BA.
- **`bundle_adjust_tracks`** — BA immediately after triangulating a track (triangulation path).
- **`num_retriangulation_iterations`** — global pipeline: retriangulation + **full** BA rounds.

**`min_cameras_for_iterative_solver`** is documented on `ReconstructionEstimatorOptions` (default **1000**) as a threshold for switching to iterative Schur, but **`SetBundleAdjustmentOptions` does not apply it automatically** — you still choose **`linear_solver_type`** explicitly if you want **`ITERATIVE_SCHUR`**.

## `BundleAdjuster` (incremental / custom subsets) {#bundle-adjuster-class}

**`BundleAdjuster(options, reconstruction)`** builds a Ceres problem explicitly:

- **`AddView(view_id)`** — include this view’s parameters and reprojection residuals for observed tracks.
- **`AddTrack(track_id)`** — include this point (must be consistent with added views).
- **`Optimize()`** — run the solver.

Use this when the convenience `BundleAdjust*` functions do not match your scheduling (e.g. sliding-window BA).

## `BundleAdjustmentSummary` {#bundle-adjustment-summary}

- `success` — optimization ran without Ceres **failure** (does **not** guarantee geometric quality).
- `initial_cost` / `final_cost`
- `setup_time_in_seconds` / `solve_time_in_seconds`

Always inspect **reprojection errors** and **inlier counts** in the reconstruction after BA.

## Related helpers {#bundle-adjustment-related}

- **`SelectGoodTracksForBundleAdjustment`** — subsample long tracks / balance coverage before a large BA.
- **`SetOutlierTracksToUnestimated`** — drop poorly supported points after BA or filtering.

## See also {#bundle-adjustment-see-also}

- [SfM](sfm.md) — `Reconstruction`, `View`, `Track`, pipelines  
- [Triangulation](triangulation.md) — initial structure before refinement  
- [Transformations](transformations.md) — align or apply a global Sim3 before/after BA  
- [Cameras](cameras.md) — intrinsics models and projection  
