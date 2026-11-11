# Global pose estimation {#documentation-global-pose-estimation}

Code under [`src/theia/sfm/global_pose_estimation`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/sfm/global_pose_estimation) implements **rotation averaging** and **global camera position** solvers used by the **global SfM** pipeline. They fuse **pairwise** geometric constraints from a view graph into a **consistent set of absolute camera orientations and positions**.

This is **not** two-view minimal estimation (see [Estimators](estimators.md)); inputs are already aggregated **`TwoViewInfo`** edges between views (relative rotation and translation **direction**, etc.). The high-level **global SfM** flow is described in [SfM — Global SfM Pipeline](sfm.md#global-sfm-pipeline).

---

## Problem shape {#global-pose-problem-shape}

1. **Global rotations:** From relative rotations \(R_{ij}\) on edges \((i,j)\), estimate an angle-axis (or equivalent) **global orientation** per view so that constraints like \(R_{ij} \approx R_j R_i^\top\) are satisfied in a robust way.

2. **Global positions:** Given **fixed global rotations** and **pairwise translation directions** (from `TwoViewInfo`), estimate camera **centers** up to the usual **gauge** (translation + scale; global SfM fixes this in later bundle adjustment).

Several position methods also use **view triplets** and/or **tracks** from a `Reconstruction` (linear triplet / LiGT / nonlinear with point priors).

---

## Base interfaces (C++) {#global-pose-base-interfaces}

### `RotationEstimator` {#rotation-estimator}

**Header:** [`rotation_estimator.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/rotation_estimator.h)

```cpp
virtual bool EstimateRotations(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations) = 0;
```

**pyTheia:** abstract base `pt.sfm.RotationEstimator`; concrete classes expose `EstimateRotations` / `EstimateRotationsWrapper` (see below). Most solvers need a **reasonable initial guess** for global orientations (e.g. spanning tree).

---

### `PositionEstimator` {#position-estimator}

**Header:** [`position_estimator.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/position_estimator.h)

```cpp
virtual bool EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions) = 0;
```

**pyTheia:** abstract base `pt.sfm.PositionEstimator`; subclasses take a `Reconstruction` in the constructor when they need tracks / triplets.

---

## Global rotation averaging {#global-rotation-estimators}

Implementations derive from `RotationEstimator`. In **`ReconstructionEstimatorOptions`**, the enum **`GlobalRotationEstimatorType`** (`pt.sfm.GlobalRotationEstimatorType`) selects which one the global pipeline uses.

| Python / C++ class | `GlobalRotationEstimatorType` | Summary |
|--------------------|------------------------------|---------|
| [`RobustRotationEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/robust_rotation_estimator.h) | `ROBUST_L1L2` (default) | L1 then IRLS; *Efficient and Large Scale Rotation Averaging*, Chatterjee & Govindu (ICCV 2013). |
| [`NonlinearRotationEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/nonlinear_rotation_estimator.h) | `NONLINEAR` | Ceres + robust loss on rotation manifold. |
| [`LinearRotationEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/linear_rotation_estimator.h) | `LINEAR` | Linearized system + projection to SO(3); Martinec & Pajdla (CVPR 2007). Optional `AddRelativeRotationConstraint` for multiple constraints per pair (Zhu et al., arXiv 2017). |
| [`LagrangeDualRotationEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/lagrange_dual_rotation_estimator.h) | `LAGRANGE_DUAL` | SDP / Lagrange dual rotation averaging (Eriksson et al., CVPR 2018 / PAMI 2019). |
| [`HybridRotationEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/hybrid_rotation_estimator.h) | `HYBRID` | Combines Lagrange-dual estimation with **IRLS** local refinement (`irls_rotation_local_refiner.h`). |

**`RobustRotationEstimator`** (pyTheia) also exposes `AddRelativeRotationConstraint` and `SetFixedGlobalRotations` for specialized setups.

### Rotation refinement: MORE {#rotation-refinement-more}

After rotation filtering, the global pipeline can refine the orientations with the rotation-only reprojection error of MORE (Li, Cai, Wu, arXiv:2611.11770, Sec. 5.2, Eq. 21-23 and Alg. 2). The camera translations are eliminated analytically: each triplet's baselines are the ART closed-form solution (Theorem 1) evaluated inside the residual, so only the rotations are optimized. Two-view tuples use the same residual with the two-view translation (TRRM, Eq. 11).

- Enable it with `ReconstructionEstimatorOptions.refine_rotations_with_rotation_only_error = True` (default `False`). It runs for every global position estimator. On failure the unrefined rotations are kept and a warning is logged.
- Parameters are in `ReconstructionEstimatorOptions.rotation_only_refiner_options` (`RotationOnlyRefinerOptions`): `num_outer_iterations`, `max_num_iterations`, `loss_scale_px`, `max_reference_candidates`, `tuple_gate_px`, `max_tracks_per_triplet` (default 50; ART's own option keeps 200, since every track adds to each Ceres residual evaluation of its triplet), `max_observations_per_view`, `use_two_view_tuples`, `require_view_graph_edges`, `num_threads` (parallelizes the tuple build and the Ceres evaluation; the result does not depend on it).
- Before the refinement the observations are verified against the initial rotations (`verify_observations_px`, default 4 px, 0 disables it): per view pair a robust (least median) two-view translation, and an observation is dropped if its epipolar distance exceeds `max(verify_observations_px, 5 * median distance of the pair)` for most of the other views of its track. Matches that passed two-view RANSAC rarely fail it; unverified mismatches would otherwise enter the Theorem 1 solves inside the residuals and make the refinement worse (synthetic test with 10% random-pixel outliers: 1.00° → 1.23° without, 1.00° → 0.13° with the verification).
- The standalone function is `pt.sfm.RefineRotationsWithRotationOnlyError(options, reconstruction, view_pairs, orientations)`; it returns `(success, refined_orientations)`.
- The optimizer schedule (outer iterations, Cauchy IRLS weights from `loss_scale_px`, tuple selection with the residual gate, and the observation filter) is a pyTheia choice. The paper does not specify these parameters.

---

## Global camera positions {#global-position-estimators}

Implementations derive from `PositionEstimator`. **`GlobalPositionEstimatorType`** (`pt.sfm.GlobalPositionEstimatorType`) selects the solver in the global pipeline.

| Python / C++ class | `GlobalPositionEstimatorType` | Summary |
|--------------------|--------------------------------|---------|
| [`NonlinearPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/nonlinear_position_estimator.h) | `NONLINEAR` | Robust nonlinear alignment to pairwise translation directions; *Robust Global Translations with 1DSfM*, Wilson & Snavely (ECCV 2014). Uses `Reconstruction` for optional point–camera constraints. |
| [`LinearPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/linear_position_estimator.h) | `LINEAR_TRIPLET` | Linear system from **view triplets** and baseline ratios; Jiang et al. (ICCV 2013). |
| [`LeastUnsquaredDeviationPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h) | `LEAST_UNSQUARED_DEVIATION` (default) | L1-style via IRLS / ADMM; Ozyesil & Singer (CVPR 2015). Optional fixed metric edges via `Options::use_scale_estimates` and `TwoViewInfo::scale_estimate`. |
| [`LiGTPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/LiGT_position_estimator.h) | `LIGT` | Linear formulation using **pose-only** / triplet constraints (Cai et al., IEEE TPAMI 2023, arXiv:2103.01530). Requires `Reconstruction`. Robustified with a scale constraint and Huber IRLS (see [Releases](releases.md)). |
| [`ARTPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/art_position_estimator.h) | `ART` | Analytical Rotation-guided Translation: closed-form LiGT-based triplet translations (Theorem 1) registered by one sparse weighted least-squares solve with per-triplet scales and Huber IRLS (Li, Cai, Wu, arXiv:2611.11770). Requires `Reconstruction`. Screening/IRLS parameters are pyTheia choices (the paper defers them to an unpublished supplement). See [ART view coverage](#art-coverage). |
| [`GlomapPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/glomap_position_estimator.h) | `GLOMAP` | Track-based global positioning following GLOMAP (Pan et al., ECCV 2024). Requires `Reconstruction` with tracks and fixed global rotations. |

### ART view coverage {#art-coverage}

ART solves for one scale per triplet, so a triplet only fixes positions relative to others if they share at least two views. The paper keeps the triplets that are edge-connected to the reference. pyTheia keeps more views, with these steps (all pyTheia additions):

- **Rigid components.** Triplets that share an edge are linked, and two linked groups that share at least two views are merged: the two common centers fix their relative scale and translation. After selection, valid triplets that attach a new view rigidly to the main component are added back, also ones the confidence test rejected.
- **Bridging triplets** (`num_coverage_rounds`, default 3): views outside the main component get candidate triplets with an edge of it, sharing at least `min_tracks_per_triplet_fallback` tracks.
- **Track constraints** (`min_bridge_tracks`, default 10, 0 disables it): views that triplets still cannot attach, e.g. a group of views that shares a single view with the main component, are attached with the LiGT constraints of the tracks they share with it. These are linear in the centers without a scale unknown and get their own Huber scale in the IRLS. On ETH3D `office` 6 of 26 views were lost without this step.
- **Robust triplet start** (`num_robust_samples`, default 20): the observation filter of each triplet starts from the least-median-of-squares solution over random 8-track subsets if its median residual is less than half of the least-squares one (gross outliers); otherwise from the least-squares solution as before.

LiGT drops views that are seen in fewer than `LiGTPositionEstimatorOptions.min_tracks_per_view` (default 5) tracks of length three or more. Such a view has a nearly singular block in the LiGT system, and the scale-constrained solution then puts almost all of its norm on that view (ETH3D `kicker`: one view with 2 tracks at distance 1, all others within 1e-4).

---

## Triplet baseline helper {#compute-triplet-baseline-ratios}

### `ComputeTripletBaselineRatios`

**Header:** [`compute_triplet_baseline_ratios.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/compute_triplet_baseline_ratios.h)

**pyTheia:** `pt.sfm.ComputeTripletBaselineRatios(triplet, features1, features2, features3)` → `(success, baselines)` where `baselines` is a 3-vector: scale of baseline between views **1–2**, **1–3**, and **2–3**, from triangulated depths of **aligned** feature triplets.

Used when turning triplet geometry into constraints for linear / LiGT-style position solvers.

---

## Supporting and C++-only sources {#global-pose-supporting-files}

These files support the estimators above (Ceres costs, internal IRLS / L1 blocks, tests). They are not separate top-level pyTheia APIs:

- **Pairwise residuals:** `pairwise_rotation_error.{h,cc}`, `pairwise_translation_error.{h,cc}`, `pairwise_translation_and_scale_error.{h,cc}`
- **Rotation building blocks:** `l1_rotation_global_estimator.*`, `irls_rotation_local_refiner.*`, `rotation_estimator_util.h`
- **C++-only position variant:** [`bata_position_estimator.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/bata_position_estimator.h) (revised least-unsquared / BATA-style formulation; not exposed in pybind as of this writing)

**Python/C++ bridge** for triplet baselines: [`global_pose_estimation_wrapper.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/global_pose_estimation_wrapper.cc).

---

## pyTheia usage notes {#global-pose-python-notes}

- Configure the global pipeline via `ReconstructionEstimatorOptions.global_rotation_estimator_type` and `global_position_estimator_type` (see [SfM](sfm.md)).
- To call estimators **directly**, construct the concrete `Options` struct (`RobustRotationEstimatorOptions`, `LinearPositionEstimatorOptions`, …), build the `view_pairs` map and orientation / position maps, then call `EstimateRotationsWrapper` / `EstimatePositionsWrapper` where provided (signatures match the C++ `EstimateRotations` / `EstimatePositions` but return or mutate maps in a pybind-friendly way).

---

## See also {#global-pose-see-also}

- [SfM](sfm.md) — `GlobalReconstructionEstimator`, view-graph filtering, full global pipeline  
- [Estimators](estimators.md) — robust **two-view** models feeding `TwoViewInfo`  
- [Ransac](ransac.md) — sample consensus used upstream of global pose  
