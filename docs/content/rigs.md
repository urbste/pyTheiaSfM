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

## Metric capture–capture pose

For capture–capture edges with a known stereo baseline, prefer metric relative pose instead of unit-scale monocular essentials.

**`BuildCaptureViewGraph`** (default): for each **ViewGraph** capture pair,

1. Strip the View–View essential into the rig frame (rotation + unit translation).
2. Recover **metric scale** from tracks that see both captures: stereo midpoint triangulation in each capture (3D–3D), then stereo 3D + a bearing in the other capture, then generalized-ray \(\gamma\).
3. **`EstimateRelativeRigInfo` (5+1)** only if those scale recoveries fail.

5+1 is also skipped when the stripped motion is nearly parallel to a sensor baseline (degenerate scale ray). Pairs that cannot be estimated metrically optionally fall back to the unit-scale stripped essential (`scale_estimate < 0`, treated as a free-scale direction). Intra-capture stereo edges are never motion edges.

The 5+1 solver itself remains available:

- **`FivePointOnePointGeneralizedRelativePose`** — central 5-pt on same-sensor bearings + 1 possibly cross-sensor ray to fix scale (PoseLib `gen_relpose_5p1pt` algorithm, implemented in-tree).
- **`FourPointUprightGeneralizedRelativePose`** — upright (gravity-axis) generalized 4-pt; wraps Theia’s Sweeney `FourPointRelativePosePartialRotation`.
- **`EstimateRelativeRigInfo`** / **`EstimateRelativeRigInfoUpright`** — RANSAC over `GeneralizedRayCorrespondence`. Output `RelativeRigInfo` has metric `translation` / `position`; call `ToTwoViewInfo()` for capture-graph edges.

**Degeneracy:** 5+1 cannot fix scale when relative translation is parallel to the offset used by the scale ray (typical failure: side-by-side stereo baseline along \(X\) with pure \(X\) motion). Forward motion (\(Z\)) with a lateral baseline is well-conditioned. Stereo 3D–3D scale recovery does not have that degeneracy.

`GlobalRigReconstructor` then:

1. LUD position averaging with `use_scale_estimates` on metric (`scale_estimate > 0`) capture edges  
2. Optional median rescale using those metric lengths  
3. Rig-constrained BA (`use_rig_constraints`): one 6-DoF per `RigCapture`, calibrated sensor extrinsics held constant

```python
opts = pt.sfm.BuildCaptureViewGraphOptions()
opts.use_metric_relative_rig_pose = True
opts.fallback_to_twoview_strip = True
opts.metric_only_for_viewgraph_pairs = True
opts.skip_metric_if_baseline_degenerate = True
pt.sfm.BuildCaptureViewGraph(recon, view_graph, capture_graph, opts)

gro = pt.sfm.GlobalRigReconstructorOptions()
gro.capture_graph_options = opts
gro.rescale_positions_to_metric_edges = True
```

## Reconstructors

### `IncrementalRigReconstructor`

Seeds the first capture at identity, triangulates (intra-rig stereo tracks are metric from known extrinsics), then localizes later captures (generalized 2D–3D / single-view fallback). Bundle adjustment uses `use_rig_constraints` (shared body pose). Partial BA runs on the most recent captures; a full BA runs every `bundle_adjust_every_n_captures` localizations and once at the end.

### `GlobalRigReconstructor` (MGSfM-inspired, calibrated extrinsics)

1. `BuildCaptureViewGraph` — stripped essential + stereo metric scale on ViewGraph capture pairs; 5+1 then unit-scale strip as fallbacks (`scale_estimate < 0`)  
2. **Rotation averaging** on captures — any `GlobalRotationEstimatorType` (`ROBUST_L1L2`, `NONLINEAR`, `LINEAR`, `LAGRANGE_DUAL`, `HYBRID`)  
3. **Position averaging** — LUD on the capture graph with metric scale estimates, optional median rescale to metric edge lengths  
4. Propagate → triangulate → **rig BA** (one 6-DoF per capture; sensor extrinsics stay calibrated)

Unknown-extrinsic MGSfM (decoupled auto-calibration RA/TA) is **not** implemented; set `RigSensor` poses when defining the `CameraRig`.

```python
opts = pt.sfm.GlobalRigReconstructorOptions()
opts.sfm_options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2
opts.sfm_options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
summary = pt.sfm.GlobalRigReconstructor(opts).Estimate(view_graph, reconstruction)
```

Stereo verification helpers (known extrinsics):

```python
E = pt.sfm.EssentialMatrixFromRigSensors(left_sensor, right_sensor)
inliers = pt.sfm.FilterCorrespondencesWithEssential(E, cam_left, cam_right, cors, 2.0)
```

## Example

See [`pyexamples/stereo/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/stereo_rig_reconstruction.py): set focal / principal point / baseline, match with **vismatch** (`disk-lightglue` by default: detect once per image, LightGlue matching, features cached under `.pytheia_features/`). Default matching is **left-cascade** (frame \(i\) vs later left frames). Same-timestamp stereo is filtered with the known essential and attached **only** to tracks that already span several rig poses. Right–right temporal matching is off unless `--match_right_temporal`. Pass `--left_match_mode window` for the older ±N + `--loop_stride` schedule. Pass `--trajectory_has_loops` to precompute CosPlace (ResNet18, 128-D) on **left** images only (one `.npz` cache per dataset, row = frame id) and match `GraphMatch` loop candidates (needed on long revisiting trajectories; leave off for forward odometry). Pass `--visualize` to open an **Open3D** window (tracks, capture trajectory, stereo baselines) if `open3d` is installed.

**KITTI Odometry:** [`pyexamples/stereo/kitti_rig_benchmark.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/kitti_rig_benchmark.py) runs the same pipeline on `sequences/XX` (`calib.txt`, `image_0`/`image_1`), then reports Sim(3) and SE(3) ATE plus RPE against `poses/XX.txt` when present. SE(3) ATE (scale fixed) is the metric check: the calibrated baseline should keep Umeyama scale near 1.

```bash
python pyexamples/stereo/kitti_rig_benchmark.py \
  --kitti_root /data/kitti/odometry --sequences 00 --max_frames 200 \
  --trajectory_has_loops --visualize
```

**ZED SVO:** [`pyexamples/preprocess/zed_svo_extract_stereo.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/preprocess/zed_svo_extract_stereo.py) opens a Stereolabs `.svo` / `.svo2` (needs `pyzed`), writes rectified `left/` / `right/` frames plus `rig_calibration.json` (intrinsics, baseline, mid-point body-frame sensor poses) for the reconstruction example above.

**Visualization:** `pt.io.WriteRigPlyFile(...)` exports tracks plus capture trajectory and intra-capture sensor baselines (densely sampled polylines) for MeshLab / Open3D.

## See also

- [SfM](sfm.md) — `Reconstruction`, Views, Tracks  
- [View graph](view_graph.md) — pairwise matches  
- [Global pose estimation](global_pose_estimation.md) — averaging backends  
- [Matching](matching.md) / [vismatch + SfM](examples_vismatch_sfm.md)
