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

**Calibrated by default:** averaging and the v1 reconstructors do **not** optimize inter-camera extrinsics. Set sensor poses (e.g. stereo baseline) when defining the `CameraRig`.

## Metric relative rig pose (5+1)

For capture–capture edges with a known baseline, prefer metric relative pose instead of unit-scale monocular essentials:

- **`FivePointOnePointGeneralizedRelativePose`** — central 5-pt on same-sensor bearings + 1 possibly cross-sensor ray to fix scale (PoseLib `gen_relpose_5p1pt` algorithm, implemented in-tree).
- **`FourPointUprightGeneralizedRelativePose`** — upright (gravity-axis) generalized 4-pt; wraps Theia’s Sweeney `FourPointRelativePosePartialRotation`.
- **`EstimateRelativeRigInfo`** / **`EstimateRelativeRigInfoUpright`** — RANSAC over `GeneralizedRayCorrespondence` (ray origins + bearings in each rig frame). Output `RelativeRigInfo` has metric `translation` / `position`; call `ToTwoViewInfo()` for capture-graph edges.

Lift pixels with known `RigSensor` extrinsics into the abstract body frame, then estimate.

**Degeneracy:** 5+1 cannot fix scale when relative translation is parallel to the offset used by the scale ray (typical failure: side-by-side stereo baseline along \(X\) with pure \(X\) motion). Forward motion (\(Z\)) with a lateral baseline is well-conditioned.

**`BuildCaptureViewGraph`** (default): when tracks span two captures, lifts observations into the rig frame and runs **`EstimateRelativeRigInfo`** so capture edges are **metric**. Falls back to stripping View–View essentials only for pairs that cannot be estimated metrically. `GlobalRigReconstructor` then optionally **rescales** averaged positions to match those metric edge lengths (`rescale_positions_to_metric_edges`).

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

1. `BuildCaptureViewGraph` — metric 5+1 from tracks when possible; optional strip of View–View essentials for missing pairs  
2. **Rotation averaging** on captures — any `GlobalRotationEstimatorType` (`ROBUST_L1L2`, `NONLINEAR`, `LINEAR`, `LAGRANGE_DUAL`, `HYBRID`)  
3. **Position averaging** — `LEAST_UNSQUARED_DEVIATION` on the capture graph, or other `GlobalPositionEstimatorType`s (`NONLINEAR`, `LINEAR_TRIPLET`, `LIGT`, `GLOMAP`) via the View graph after propagating orientations; optional median rescale to metric edge lengths  
4. Propagate → triangulate → BA (then re-snap Views to the calibrated rig)

```python
opts = pt.sfm.GlobalRigReconstructorOptions()
opts.sfm_options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2
opts.sfm_options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
summary = pt.sfm.GlobalRigReconstructor(opts).Estimate(view_graph, reconstruction)
```

## Example

See [`pyexamples/stereo/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/stereo_rig_reconstruction.py): set focal / principal point / baseline, match with **vismatch** (`edm` by default), run global or incremental rig SfM.

**ZED SVO:** [`pyexamples/preprocess/zed_svo_extract_stereo.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/preprocess/zed_svo_extract_stereo.py) opens a Stereolabs `.svo` / `.svo2` (needs `pyzed`), writes rectified `left/` / `right/` frames plus `rig_calibration.json` (intrinsics, baseline, mid-point body-frame sensor poses) for the reconstruction example above.

**Visualization:** `pt.io.WriteRigPlyFile(...)` exports tracks plus capture trajectory and intra-capture sensor baselines (densely sampled polylines) for MeshLab / Open3D.

## See also

- [SfM](sfm.md) — `Reconstruction`, Views, Tracks  
- [View graph](view_graph.md) — pairwise matches  
- [Global pose estimation](global_pose_estimation.md) — averaging backends  
- [Matching](matching.md) / [vismatch + SfM](examples_vismatch_sfm.md)
