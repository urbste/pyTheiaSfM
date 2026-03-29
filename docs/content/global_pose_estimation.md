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

---

## Global camera positions {#global-position-estimators}

Implementations derive from `PositionEstimator`. **`GlobalPositionEstimatorType`** (`pt.sfm.GlobalPositionEstimatorType`) selects the solver in the global pipeline.

| Python / C++ class | `GlobalPositionEstimatorType` | Summary |
|--------------------|--------------------------------|---------|
| [`NonlinearPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/nonlinear_position_estimator.h) | `NONLINEAR` (default) | Robust nonlinear alignment to pairwise translation directions; *Robust Global Translations with 1DSfM*, Wilson & Snavely (ECCV 2014). Uses `Reconstruction` for optional point–camera constraints. |
| [`LinearPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/linear_position_estimator.h) | `LINEAR_TRIPLET` | Linear system from **view triplets** and baseline ratios; Jiang et al. (ICCV 2013). |
| [`LeastUnsquaredDeviationPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h) | `LEAST_UNSQUARED_DEVIATION` | L1-style via IRLS / ADMM; Ozyesil & Singer (CVPR 2015). |
| [`LiGTPositionEstimator`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/global_pose_estimation/LiGT_position_estimator.h) | `LIGT` | Linear formulation using **pose-only** / triplet constraints (Cai et al., arXiv:2103.01530). Requires `Reconstruction`. |

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
