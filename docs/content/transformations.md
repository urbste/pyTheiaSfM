# Transformations, alignment, and Sim3 {#documentation-transformations}

This chapter covers **similarity / rigid transforms** for **point clouds**, **full reconstructions**, and **ray–point (gDLS)** problems. The C++ code lives under [`src/theia/sfm/transformation/`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/sfm/transformation). Most entry points are on **`pytheia.sfm`** (`import pytheia as pt` → `pt.sfm`).

For **Python** layout, see [Python API overview](python_wrapper.md).

## Similarity convention {#transformations-convention}

**Point map:** a similarity acts on Euclidean 3D points as  

\[
\mathbf{p}' = s \, \mathbf{R} \, \mathbf{p} + \mathbf{t}.
\]

**Umeyama alignment** maps the **left** set **A** to the **right** set **B**:  

\[
\mathbf{B} \approx s \, \mathbf{R} \, \mathbf{A} + \mathbf{t}.
\]

(`AlignPointCloudsUmeyama(left, right)` in Theia follows that naming.)

**Whole `Reconstruction`:** `TransformReconstruction` applies the same \((\mathbf{R}, \mathbf{t}, s)\) to every **estimated** camera and track: 3D points use the formula above; camera orientations and positions are updated consistently (see [`transform_reconstruction.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/transformation/transform_reconstruction.cc)).

## Point cloud alignment (Umeyama) {#transformations-umeyama}

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `AlignPointCloudsUmeyama(left, right)` | Closed-form **least-squares similarity** between matched 3D points (Umeyama). Returns **`(R, t, scale)`** as `numpy` arrays / Eigen types. |
| `AlignPointCloudsUmeyamaWithWeights(left, right, weights)` | Same with a **positive weight** per correspondence. |

Correspondences must be **aligned by index**; `left[i]` matches `right[i]`.

## gDLS: generalized pose and scale from rays {#transformations-gdls}

**`GdlsSimilarityTransform`** implements *gDLS* (Sweeney et al., ECCV 2014): given **camera centers** `ray_origin`, **unit** view directions `ray_direction`, and **world** points `world_point` (same count, ≥ 4), it returns **multiple** candidate similarities that align rays to points.

Python returns a tuple **`(quats_wxyz, translations, scales)`** — per-solution quaternion as length-4 \([w,x,y,z]\), translation `Vector3`, and scalar scale (see [`transformation_wrapper.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/transformation/transformation_wrapper.cc)).

Use this for **loop closure**, **localization**, or other **PnP-with-scale** settings where a single similarity explains many ray–point pairs.

## Aligning two reconstructions {#transformations-align-reconstructions}

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `AlignReconstructions(fixed_recon, variable_recon)` | Estimates a similarity that best aligns **`variable_recon`** to **`fixed_recon`** using **shared** (same view name) **estimated** cameras; **updates** `variable_recon` in place. Returns **`(R, t, scale)`**. |
| `AlignReconstructionsRobust(threshold, fixed_recon, variable_recon)` | **RANSAC** over camera centers, then refit on inliers; `threshold` is allowed **position** error for inliers. |

These wrap [`align_reconstructions.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/transformation/align_reconstructions.h).

## Cross-reconstruction Sim(3) pose graph (Python) {#transformations-pose-graph}

**Pose-graph alignment** fuses a **fixed** reference reconstruction (segment / map) with a **variable** reconstruction (run) using sparse **keyframes** and measured **Sim(3)** constraints. Optimization is a standalone **Ceres** problem; it does **not** call `BundleAdjustReconstruction` or change global SfM estimators.

Typical use: align a localized or partial run to a segment map when you have relative odometry between run keyframes and absolute PnP poses of run cameras in the segment frame.

### One-shot API

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `AlignReconstructionsWithPoseGraph(fixed_recon, variable_recon, constraints, options=..., apply_to_variable_reconstruction=True)` | Build optimizer from `constraints`, solve, optionally **update** `variable_recon` (views + tracks). Returns **`(ok, summary)`**. |

### Constraint types

| Type | Python struct | Role |
|------|---------------|------|
| Sequential | `SequentialSim3Edge` | Relative Sim(3) \(S_{ji}\) between consecutive **variable** keyframes (`view_id_i`, `view_id_j`, `measured_S_ji_log` as 7-vector). |
| Cross-view anchor | `CrossViewAnchorEdge` | Absolute pose of a run keyframe in the **segment** world (`variable_view_id`, `measured_S_run_in_seg_log`, `weight`). |
| Scale smoothness | (automatic) | When `CrossReconstructionPoseGraphOptions.auto_scale_smoothness` is `True` (default), penalizes scale jumps along consecutive keyframes in keyframe order. |

Bundle edges in **`CrossReconstructionConstraints`**: `variable_keyframe_view_ids`, `fixed_anchor_view_ids`, `sequential_edges`, `cross_view_edges`.

### Options and summary

**`CrossReconstructionPoseGraphOptions`** (defaults in parentheses):

| Field | Default | Meaning |
|-------|---------|---------|
| `sequential_weight` | `1.0` | Weight on sequential Sim(3) edges. |
| `anchor_weight` | `1.0` | Weight on cross-view anchor edges. |
| `scale_smooth_weight` | `0.1` | Weight for auto scale-smoothness edges. |
| `huber_delta_anchor` | `1.0` | Huber loss on anchors (`0` disables robust loss). |
| `auto_scale_smoothness` | `True` | Add scale-smoothness along consecutive variable keyframes. |
| `max_num_iterations` | `50` | Ceres iteration limit. |
| `verbose` | `False` | Ceres logging. |
| `debug_cost_breakdown` | `False` | Log per-block costs and finiteness checks. |

**`CrossReconstructionPoseGraphSummary`**: `success`, `initial_cost`, `final_cost`, `num_iterations`, `poses_finite_before` / `poses_finite_after`, and squared residual costs per edge family (`sequential_residual_cost`, `anchor_residual_cost`, `scale_smooth_residual_cost`).

### Low-level optimizer

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `CrossReconstructionSim3PoseGraphOptimizer(options=...)` | Incremental builder. |
| `set_fixed_reconstruction(fixed_recon, anchor_view_ids)` | Constant segment poses for anchor views. |
| `set_variable_reconstruction(variable_recon, keyframe_view_ids)` | Optimized run keyframe poses (order defines auto scale-smoothness). |
| `set_constraints(constraints)` | Load edge lists from a `CrossReconstructionConstraints` object. |
| `add_sequential_edge` / `add_cross_view_edge` / `add_scale_smoothness_edge` | Add edges individually. |
| `optimize()` | Returns **`(ok, summary)`**. |
| `apply_to_variable_reconstruction(variable_recon, transform_tracks=True)` | Write optimized Sim(3) poses back into the reconstruction. |
| `variable_poses` | Map `view_id →` 7-vector lie logs after optimization. |

### Sim(3) helpers for building edges

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `GetSim3LieFromView(view)` | Camera pose as Sim(3) lie log (Sophus `Sim3d::exp` parameterization). |
| `RelativeSim3BetweenViews(view_i, view_j)` | Relative Sim(3) \(S_i^{-1} S_j\) as 7-vector log. |

### Example (sketch)

```python
import pytheia as pt

constraints = pt.sfm.CrossReconstructionConstraints()
constraints.variable_keyframe_view_ids = run_keyframe_ids
constraints.fixed_anchor_view_ids = segment_anchor_ids

seq = pt.sfm.SequentialSim3Edge()
seq.view_id_i = run_keyframe_ids[0]
seq.view_id_j = run_keyframe_ids[1]
seq.measured_S_ji_log = pt.sfm.RelativeSim3BetweenViews(
    variable_recon.View(seq.view_id_i),
    variable_recon.View(seq.view_id_j),
)
constraints.sequential_edges.append(seq)

anchor = pt.sfm.CrossViewAnchorEdge()
anchor.variable_view_id = run_keyframe_ids[0]
anchor.measured_S_run_in_seg_log = pnp_sim3_log_in_segment_frame  # from localization
anchor.weight = 1.0
constraints.cross_view_edges.append(anchor)

options = pt.sfm.CrossReconstructionPoseGraphOptions()
options.verbose = True

ok, summary = pt.sfm.AlignReconstructionsWithPoseGraph(
    segment_recon, run_recon, constraints, options
)
```

C++ reference: [`cross_reconstruction_sim3_pose_graph_optimizer.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/transformation/cross_reconstruction_sim3_pose_graph_optimizer.h).

!!! note "Not yet exposed in Python"

    [`AlignOverlapReconstructionsWithPointsAndPosesRobust`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/transformation/align_reconstructions.h) remains C++ only.

## Applying a known similarity to a reconstruction {#transformations-transform-reconstruction}

| Python (`pt.sfm`) | Meaning |
|-------------------|---------|
| `TransformReconstruction(reconstruction, R, t, scale)` | Apply \((\mathbf{R}, \mathbf{t}, s)\) to all **estimated** views and tracks **in place**. |
| `TransformReconstruction4(reconstruction, T4)` | Same using a **`4×4`** matrix: **`T4[:3,:3]`** = \(\mathbf{R}\), **`T4[:3,3]`** = \(\mathbf{t}\), **`T4[3,3]`** = **scale** (Theia packs Sim3 this way, not a standard \([R|t; 0\,0\,0\,1]\) rigid block alone). |

## Ceres-based Sim3 point cloud alignment {#transformations-optimize-sim3}

**`OptimizeAlignmentSim3(source_points, target_points, options)`** aligns two point clouds with **Sim(3)**: unless you pass an initial guess via **`Sim3AlignmentOptions.set_initial_sim3_params`**, it **initializes with weighted Umeyama** and then runs **Ceres** refinement (point-to-point, robust, or point-to-plane per **`alignment_type`**).

- **`Sim3AlignmentType`:** `POINT_TO_POINT`, `ROBUST_POINT_TO_POINT`, `POINT_TO_PLANE`
- **`Sim3AlignmentOptions`:** alignment type, Huber / outlier thresholds, `max_iterations`, per-point weights (`set_point_weights`), optional target normals for point-to-plane (`set_target_normals`), optional initial Sim3 (`set_initial_sim3_params`), Ceres-related fields on the C++ struct
- **`Sim3AlignmentSummary`:** `success`, `sim3_params` (7-vector), `alignment_error`, `num_iterations`, `final_cost`

**Parameterization helpers** ([`Sophus::Sim3`](https://github.com/strasdat/Sophus)):

- **`Sim3FromRotationTranslationScale(R, t, scale)`** → `Vector7d` (`sim3.log()`)
- **`Sim3ToRotationTranslationScale(sim3_params)`** → `(R, t, scale)`
- **`Sim3ToHomogeneousMatrix(sim3_params)`** → `4×4` in the same layout as `TransformReconstruction4`

## Small structs {#transformations-structs}

- **`SimilarityTransformation`** — `rotation` (`3×3`), `translation` (`3`), `scale` (`float`)
- **`RigidTransformation`** — `rotation`, `translation` (scale fixed to 1 in semantics elsewhere)

## Rotating a set of global orientations to reference {#transformations-align-rotations}

To align a **list of angle-axis rotations** \(\omega_i\) (Ceres convention) to **ground-truth** rotations of the **same length**, use **`pytheia.math.AlignRotations(gt_rotations, rotations)`**: it solves for a **global** rotation correction and updates **`rotations` in place** so that `rotations * R ≈ gt_rotations` in the aggregate least-squares sense (see [`rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/math/rotation.h)).

**`pytheia.math.AlignOrientations`** is a thin wrapper when data are stored as **`dict[view_id → angle_axis]`**.

Prefer these **`math`** helpers over any duplicate **`sfm`** binding for rotation alignment unless you have verified the **`sfm`** signature matches your use case.

## See also {#transformations-see-also}

- [SfM](sfm.md) — `Reconstruction`, `View`, `Track`  
- [Bundle adjustment](bundle_adjustment.md) — refine after applying a global transform  
- [Pose](pose.md) — geometry that produces rays for gDLS  
- [Math](math.md) — `AlignRotations`, `AlignOrientations`, `MultiplyRotations`, …  
