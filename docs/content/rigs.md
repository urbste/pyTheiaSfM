# Camera rigs (stereo / multi-camera) {#documentation-rigs}

**Since pyTheia 1.2.0.** Calibrated multi-camera / stereo rigs as an additive layer on the usual `View` / `Track` / `ViewGraph` model.

## Concepts

| Type | Meaning |
|------|---------|
| **`CameraRig`** | Abstract **body** frame + per-sensor extrinsics (`RigSensor`) in that frame |
| **`RigCapture`** | One synchronized sample: **one timestamp = one body pose** |
| **`View`** | Still one image; features and matches stay on Views |
| Membership | Each View may belong to `(rig_id, rig_camera_id, capture_id)` |

Pose composition (Camera convention):

\[
R_{w\leftarrow c} = R_{\text{rig}\leftarrow c}\, R_{w\leftarrow\text{rig}},\quad
c_c = c_{\text{rig}} + R_{w\leftarrow\text{rig}}^{\top} c_{\text{sensor}}
\]

`PropagateCameraPosesForCapture` writes composed extrinsics into each member `View.Camera`.

**Fixed extrinsics:** the reconstructors do **not** optimize inter-camera extrinsics (no live rig calibration). Set sensor poses (e.g. stereo baseline) when defining the `CameraRig`. Since 1.4.1 these fixed extrinsics are used *inside* the optimizers: they supply the metric scale in the global position estimators and are kept exactly constant in bundle adjustment (see below).

## Metric relative rig pose (5+1)

For capture–capture edges with a known baseline, prefer metric relative pose instead of unit-scale monocular essentials:

- **`FivePointOnePointGeneralizedRelativePose`** — central 5-pt on same-sensor bearings + 1 possibly cross-sensor ray to fix scale (PoseLib `gen_relpose_5p1pt` algorithm, implemented in-tree).
- **`FourPointUprightGeneralizedRelativePose`** — upright (gravity-axis) generalized 4-pt; wraps Theia’s Sweeney `FourPointRelativePosePartialRotation`.
- **`EstimateRelativeRigInfo`** / **`EstimateRelativeRigInfoUpright`** — RANSAC over `GeneralizedRayCorrespondence` (ray origins + bearings in each rig frame). Output `RelativeRigInfo` has metric `translation` / `position`; call `ToTwoViewInfo()` for capture-graph edges.

Lift pixels with known `RigSensor` extrinsics into the abstract body frame, then estimate.

**Degeneracy:** 5+1 cannot fix scale when relative translation is parallel to the offset used by the scale ray (typical failure: side-by-side stereo baseline along \(X\) with pure \(X\) motion). Forward motion (\(Z\)) with a lateral baseline is well-conditioned.

**`BuildCaptureViewGraph`** (standalone default `use_metric_relative_rig_pose = True`): when tracks span two captures, lifts observations into the rig frame and runs **`EstimateRelativeRigInfo`** so capture edges are **metric**. Falls back to stripping View–View essentials only for pairs that cannot be estimated metrically.

!!! note "Strip edges (1.4.1)"
    A two-view translation is a unit direction, so a metric sensor offset can only be removed consistently when both views use the **same sensor** (the offsets then nearly cancel). The strip path therefore prefers same-sensor view edges (the one with most verified matches per capture pair) and uses cross-sensor edges only for capture pairs without a same-sensor edge.

!!! warning "The metric 5+1 edges are fragile on forward motion"
    On KITTI odometry (forward motion, 0.54 m baseline, distant points) the RANSAC 5+1 capture edges had median rotation errors of 12–47° and took about 340 s for 270 captures (every candidate pair scans all tracks). `GlobalRigReconstructor` therefore does **not** build them when `use_rig_constraints_in_position_estimation` is on (the default): the metric scale then comes from the sensor offsets inside the position estimators, and the capture graph is built from the verified view-graph edges.

```python
opts = pt.sfm.BuildCaptureViewGraphOptions()
opts.use_metric_relative_rig_pose = True
opts.fallback_to_twoview_strip = True
pt.sfm.BuildCaptureViewGraph(recon, view_graph, capture_graph, opts)

gro = pt.sfm.GlobalRigReconstructorOptions()
gro.capture_graph_options = opts
gro.rescale_positions_to_metric_edges = True
```

## Reconstructors

### `IncrementalRigReconstructor`

Seeds the first capture at identity, triangulates (intra-rig matches are metric), then localizes later captures (generalized 2D–3D / single-view fallback).

### `GlobalRigReconstructor` (MGSfM-inspired)

1. `BuildCaptureViewGraph` — from the verified view-graph edges (same-sensor preferred); with `use_rig_constraints_in_position_estimation = False` the metric 5+1 edges are built first  
2. **Rotation averaging** on captures — any `GlobalRotationEstimatorType` (`ROBUST_L1L2`, `NONLINEAR`, `LINEAR`, `LAGRANGE_DUAL`, `HYBRID`)  
3. **Position averaging** with **rig offsets** (below) — `LEAST_UNSQUARED_DEVIATION`, `NONLINEAR`, `LIGT`, `GLOMAP` via the View graph after propagating orientations (`LINEAR_TRIPLET` ignores the rig offsets); without rig constraints a median rescale to the metric edge lengths is applied  
4. Propagate → triangulate → **rig-constrained BA** (`use_rig_constrained_ba`)

### Rig offsets in the position estimators (1.4.1)

With known capture rotations, the centre of a camera is a *linear* function of its capture centre, `c_view = C_capture + R_wr^T o_sensor`, so two views of one capture have a fully known metric baseline. `GlobalRigReconstructor` hands these offsets to the position estimator as `RigOffsetConstraint`s (`anchor` view and world-frame `delta`, via `PositionEstimator::SetRigConstraints`). The problem becomes metric without any post-hoc rescale. `GLOMAP` and `NONLINEAR` add `rig_constraint_weight`-weighted residuals (default `1e3`) and initialize from the anchor; `LEAST_UNSQUARED_DEVIATION` and `LIGT` substitute the constrained views by anchor + offset (LiGT then solves an inhomogeneous, regularized system). With `use_rig_edge_scales` (default) the metric distance of each same-sensor view-graph edge is also estimated from the stereo geometry (`EstimateMetricEdgeScales`) and passed as `scale_estimate`, which fixes the spacing of the captures on near-collinear motion; `GLOMAP` additionally initializes its positions by chaining these scales and offsets, and `LUD` is run on the view graph (not the capture graph) when rig constraints are on. Without constraints the estimators behave exactly as before. If all sensor offsets are (nearly) zero the scale is unobservable from the rig.

### Rig-constrained bundle adjustment (1.4.1)

`BundleAdjustRigReconstruction(options, reconstruction)` optimizes **one 6-DoF pose per capture** instead of one per camera. The sensor extrinsics are constants in the residual (`RigReprojectionError` composes capture pose and sensor extrinsic, analytic or autodiff Jacobians chained through the composition), so left/right stay exactly one baseline apart. Intrinsics, point priors, position / orientation / gravity / depth priors on the member views, robust loss and constant intrinsics work as in the monocular BA; priors act on the capture. Views outside a capture are optimized as ordinary cameras. After the solve the capture poses are propagated to the views without changing their `IsEstimated` flags. Both rig reconstructors use it by default (`use_rig_constrained_ba`); the old per-view BA followed by re-snapping is used with `use_rig_constrained_ba = False`.

```python
opts = pt.sfm.GlobalRigReconstructorOptions()
opts.sfm_options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2
opts.sfm_options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
opts.use_rig_constraints_in_position_estimation = True   # default
opts.use_rig_constrained_ba = True                       # default
summary = pt.sfm.GlobalRigReconstructor(opts).Estimate(view_graph, reconstruction)
```

## KITTI odometry, sequence 04

Colour stereo pair (baseline 0.538 m), 271 frames, 542 images, the same SIFT matches for every tool (pairs within ±5 frames). Left-camera trajectory against the ground truth; SE(3) ATE does not correct the scale and therefore measures the metric scale.

| Method | Setting | Reg. | Sim(3) ATE (m) | SE(3) ATE (m) | Scale | Baseline ratio | RPE@10 (m) | Time (s) |
|---|---|---|---|---|---|---|---|---|
| COLMAP mapper | no rig | 271/271 | 0.231 | 109.50 | 31.37 | 0.03 | 14.145 | 405 |
| COLMAP mapper | rig | 271/271 | 0.157 | 2.03 | 0.98 | 1.00 | 0.262 | 236 |
| COLMAP global_mapper | no rig | 271/271 | 0.296 | 109.50 | 31.37 | 0.03 | 14.145 | 303 |
| COLMAP global_mapper | rig | 271/271 | 0.155 | 2.10 | 0.98 | 1.00 | 0.272 | 165 |
| pyTheia incremental | no rig | 271/271 | 0.341 | 74.94 | 0.60 | 1.66 | 9.678 | 397 |
| pyTheia incremental | rig, rig BA | 271/271 | 14.405 | 14.94 | 1.04 | 1.00 | 0.470 | 1011 |
| pyTheia global LUD | no rig | 271/271 | 0.191 | 23.33 | 1.26 | 0.79 | 3.022 | 35 |
| pyTheia global LUD | rig, offsets + scales + rig BA | 271/271 | 0.129 | 2.09 | 0.98 | 1.00 | 0.270 | 54 |
| pyTheia global LiGT | no rig | 271/271 | 0.381 | 113.08 | 4995.72 | 0.00 | 14.607 | 52 |
| pyTheia global LiGT | rig, no constraints | 271/271 | 90.130 | 112.81 | 141.66 | 1.00 | 14.307 | 379 |
| pyTheia global LiGT | rig, offsets + scales | 271/271 | 0.384 | 7.44 | 1.07 | 1.00 | 0.968 | 40 |
| pyTheia global LiGT | rig, rig BA only | 271/271 | 81.582 | 112.67 | 124.17 | 1.00 | 14.296 | 380 |
| pyTheia global LiGT | rig, offsets + scales + rig BA | 271/271 | 0.127 | 2.08 | 0.98 | 1.00 | 0.269 | 75 |
| pyTheia global GLOMAP | no rig | 271/271 | 21.546 | 112.40 | 153.71 | 0.01 | 14.508 | 134 |
| pyTheia global GLOMAP | rig, offsets + scales + rig BA | 271/271 | 0.128 | 2.09 | 0.98 | 1.00 | 0.271 | 98 |
| pyTheia global NONLINEAR | no rig | 271/271 | 0.235 | 1768.04 | 0.06 | 16.64 | 228.364 | 43 |
| pyTheia global NONLINEAR | rig, offsets + scales | 271/271 | 0.347 | 0.64 | 1.00 | 1.00 | 0.111 | 28 |
| pyTheia global NONLINEAR | rig, offsets + scales + rig BA | 271/271 | 0.127 | 2.08 | 0.98 | 1.00 | 0.269 | 36 |

Rig incremental SfM (`IncrementalRigReconstructor`) registers all frames but is slow (17 min at 4 threads) and drifts (Sim(3) ATE 14 m); its seeding and per-capture bundle adjustment are the remaining weak point.

Rig information turns a scale-free reconstruction (scale factor 31 for COLMAP, 5000 for LiGT without rig) into a metric one. With rig offsets, metric edge scales and the rig-constrained BA, all four global position estimators (LUD, LiGT, GLOMAP, NONLINEAR) reach a Sim(3) ATE of 0.13 m and a metric scale (0.98), on par with COLMAP with a rig (0.16 m), in 36-98 s. The offsets alone fix the scale but not the spacing: on this almost collinear forward motion the translation *directions* between consecutive captures are nearly parallel and barely constrain the spacing of the captures along the road. Without the metric edge scales the median ratio of estimated to true distance between consecutive captures was LiGT 0.94, LUD 0.24, GLOMAP 0.33 and NONLINEAR 2.1 (with a large spread), and LUD, GLOMAP and NONLINEAR failed with 8, 112 and 107 m. `EstimateMetricEdgeScales` supplies the missing information: it triangulates the stereo pair of the first capture with the known baseline and solves a robust one-parameter problem for the distance to the second capture, which the estimators consume as `TwoViewInfo.scale_estimate`. Scripts: `experiments/kitti/` of the paper repository (`kitti_prepare.py`, `kitti_colmap.py`, `kitti_rig_pytheia.py`, `make_kitti_table.py`).

## Example

See [`pyexamples/stereo/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/stereo_rig_reconstruction.py): set focal / principal point / baseline, match with **vismatch** (`edm` by default), run global or incremental rig SfM.

**ZED SVO:** [`pyexamples/preprocess/zed_svo_extract_stereo.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/preprocess/zed_svo_extract_stereo.py) opens a Stereolabs `.svo` / `.svo2` (needs `pyzed`), writes rectified `left/` / `right/` frames plus `rig_calibration.json` (intrinsics, baseline, mid-point body-frame sensor poses) for the reconstruction example above.

**Visualization:** `pt.io.WriteRigPlyFile(...)` exports tracks plus capture trajectory and intra-capture sensor baselines (densely sampled polylines) for MeshLab / Open3D.

## See also

- [SfM](sfm.md) — `Reconstruction`, Views, Tracks  
- [View graph](view_graph.md) — pairwise matches  
- [Global pose estimation](global_pose_estimation.md) — averaging backends  
- [Matching](matching.md) / [vismatch + SfM](examples_vismatch_sfm.md)
