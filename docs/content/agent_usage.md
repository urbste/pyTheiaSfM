# Agent usage guide (C++ and Python) {#agent-usage}

**Audience:** coding agents and humans who need a compact, actionable map of how to *use* pyTheia / Theia (not how to contribute to the repo).

**Goal:** enough context to write a correct monocular or calibrated-rig SfM pipeline without inventing APIs.

| Read this if you need… | Skip this if you need… |
|------------------------|------------------------|
| Mental model + recipes for Views / Tracks / ViewGraph / estimators | Deep algorithm theory → [SfM](sfm.md), papers in [Bibliography](bibliography.md) |
| Python ↔ C++ name mapping | Binding implementation details → `src/pytheia/` |
| Do / don't for features & matching | Repo build / CI → root [`AGENTS.md`](https://github.com/urbste/pyTheiaSfM/blob/master/AGENTS.md), [Building](building.md) |

---

## 1. Facts agents must not invent

1. **Public Python import:** `import pytheia as pt` (never require `import pytheia.pytheia` in app code).
2. **C++ types live in namespace `theia`**, usually under includes like `"theia/sfm/reconstruction.h"`. Prefer `theia::Reconstruction`, not `theia::sfm::Reconstruction`.
3. **pyTheia is not end-to-end SfM.** Feature detection, descriptors, putative matching, and image I/O are **application-owned** (OpenCV, vismatch, COLMAP exports, etc.). The library owns geometric verification, reconstruction data structures, pose averaging, triangulation, BA, and I/O of reconstructions.
4. **Interfaces evolve.** Prefer additive changes; check stubs under `src/pytheia/pytheia/` and examples under `pyexamples/` before assuming a symbol exists.
5. **Calibrated rigs (1.2.0+)** add `CameraRig` / `RigCapture` on top of Views; features and matches still live on **Views** / **ViewGraph**.

---

## 2. Mental model

```text
Images (your code)
    → features + putative matches (your code)
        → geometric verification → TwoViewInfo edges (pyTheia)
            → ViewGraph + TrackBuilder → Reconstruction
                → Incremental / Global / Hybrid estimator (or Rig reconstructor)
                    → triangulate + BA (inside estimator / follow-up)
                        → WriteReconstruction / WritePlyFile
```

| Concept | Role | Owned by |
|---------|------|----------|
| `View` | One image: name, camera pose, intrinsics prior, observed features | `Reconstruction` |
| `Camera` | Pose + intrinsics on a `View` | `View` |
| `Track` | Multi-view feature → 3D point | `Reconstruction` |
| `ViewGraph` | Undirected graph of verified relative poses (`TwoViewInfo`) | Caller (passed into estimator) |
| `TwoViewInfo` | Relative rotation/translation (and related metadata) between two views | Edge payload |
| `TrackBuilder` | Aggregates pairwise feature correspondences into tracks | Caller |
| `Reconstruction` | Views + Tracks (+ optional rigs/captures) | Caller / estimator mutates in place |
| `CameraRig` / `RigCapture` | Calibrated multi-camera body + synchronized samples | `Reconstruction` (additive) |

**Identity rules**

- Views are uniquely named (typically the image filename).
- `AddView` returns a `ViewId`; use `ViewIdFromName` / `View(view_id)` afterward.
- Prefer mutating through `Reconstruction::MutableView` / `MutableTrack` so visibility stays consistent.
- Shared intrinsics: pass the same `CameraIntrinsicsGroupId` when adding views (or set priors identically and call `SetCameraIntrinsicsFromPriors`).

---

## 3. Python ↔ C++ map

| Python | C++ | Notes |
|--------|-----|--------|
| `import pytheia as pt` | `#include "theia/..."` | Submodules: `pt.sfm`, `pt.io`, `pt.matching`, `pt.solvers`, `pt.math`, `pt.mvs` |
| `pt.sfm.Reconstruction()` | `theia::Reconstruction` | Header: `theia/sfm/reconstruction.h` |
| `pt.sfm.ViewGraph()` | `theia::ViewGraph` | `theia/sfm/view_graph/view_graph.h` |
| `pt.sfm.TrackBuilder(min_track_length, max_track_length)` | `theia::TrackBuilder` | `theia/sfm/track_builder.h` |
| `pt.sfm.Feature(xy)` | `theia::Feature` | 2D observation |
| `pt.matching.FeatureCorrespondence(f1, f2)` | `theia::FeatureCorrespondence` | Pair for verification / tracks |
| `pt.sfm.CameraIntrinsicsPrior()` | `theia::CameraIntrinsicsPrior` | Focal, PP, model type string, size |
| `pt.sfm.EstimateTwoViewInfo(opts, prior1, prior2, corrs)` | `theia::EstimateTwoViewInfo(...)` | Returns success, `TwoViewInfo`, inliers (Python wrapper packs returns) |
| `pt.sfm.SetCameraIntrinsicsFromPriors(recon)` | `theia::SetCameraIntrinsicsFromPriors` | After views have priors |
| `pt.sfm.ReconstructionEstimatorOptions()` | `theia::ReconstructionEstimatorOptions` | Shared by global / incremental / hybrid |
| `pt.sfm.GlobalReconstructionEstimator(opts)` | `theia::GlobalReconstructionEstimator` | `.Estimate(view_graph, reconstruction)` |
| `pt.sfm.IncrementalReconstructionEstimator(opts)` | `theia::IncrementalReconstructionEstimator` | Same |
| `pt.sfm.HybridReconstructionEstimator(opts)` | `theia::HybridReconstructionEstimator` | Same |
| `pt.sfm.GlobalRigReconstructor(opts)` | `theia::GlobalRigReconstructor` | Calibrated rigs; see [Rigs](rigs.md) |
| `pt.sfm.IncrementalRigReconstructor(opts)` | `theia::IncrementalRigReconstructor` | Calibrated rigs |
| `pt.io.WriteReconstruction(recon, path)` | `theia::WriteReconstruction` | Binary / library format |
| `pt.io.ReadReconstruction(path)` | `theia::ReadReconstruction` | |
| `pt.io.WritePlyFile(path, recon, color, ...)` | `theia::WritePlyFile` | Point cloud export |

**Exact Python signatures:** use `src/pytheia/pytheia/**/*.pyi` or `help(pt.sfm.X)`.  
**Exact C++ signatures:** headers under `src/theia/`.

---

## 4. Canonical monocular SfM recipe

Minimal happy path used by [`pyexamples/sfm/sfm_pipeline_fountain.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/sfm/sfm_pipeline_fountain.py).

### 4.1 Create containers and intrinsics

=== "Python"

    ```python
    import pytheia as pt

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(4, 30)  # min / max track length

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [2759.48]
    prior.aspect_ratio.value = [2764.16 / 2759.48]
    prior.principal_point.value = [1520.69, 1006.81]
    prior.image_width = 3072
    prior.image_height = 2048
    prior.skew.value = [0]
    prior.camera_intrinsics_model_type = "PINHOLE"
    ```

=== "C++"

    ```cpp
    #include "theia/sfm/camera_intrinsics_prior.h"
    #include "theia/sfm/reconstruction.h"
    #include "theia/sfm/track_builder.h"
    #include "theia/sfm/view_graph/view_graph.h"

    theia::ViewGraph view_graph;
    theia::Reconstruction reconstruction;
    theia::TrackBuilder track_builder(/*min*/ 4, /*max*/ 30);

    theia::CameraIntrinsicsPrior prior;
    prior.focal_length.is_set = true;
    prior.focal_length.value[0] = 2759.48;
    prior.aspect_ratio.is_set = true;
    prior.aspect_ratio.value[0] = 2764.16 / 2759.48;
    prior.principal_point.is_set = true;
    prior.principal_point.value[0] = 1520.69;
    prior.principal_point.value[1] = 1006.81;
    prior.image_width = 3072;
    prior.image_height = 2048;
    prior.skew.is_set = true;
    prior.skew.value[0] = 0.0;
    prior.camera_intrinsics_model_type = "PINHOLE";
    ```

### 4.2 Add views and apply priors

=== "Python"

    ```python
    for idx, name in enumerate(image_names):
        vid = recon.AddView(name, 0, idx)  # name, intrinsics_group_id, timestamp-ish index
        recon.MutableView(vid).SetCameraIntrinsicsPrior(prior)

    pt.sfm.SetCameraIntrinsicsFromPriors(recon)
    ```

=== "C++"

    ```cpp
    for (int idx = 0; idx < image_names.size(); ++idx) {
      const theia::ViewId vid =
          reconstruction.AddView(image_names[idx], /*group*/ 0, idx);
      reconstruction.MutableView(vid)->SetCameraIntrinsicsPrior(prior);
    }
    theia::SetCameraIntrinsicsFromPriors(&reconstruction);
    ```

### 4.3 Match outside pyTheia, verify inside

Putative matches (OpenCV BF/FLANN, LightGlue, vismatch, …) → `FeatureCorrespondence` list → `EstimateTwoViewInfo` → inliers → `ViewGraph::AddEdge` + `TrackBuilder::AddFeatureCorrespondence`.

=== "Python"

    ```python
    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.max_sampson_error_pixels = 1.0
    options.use_lo = True
    options.use_mle = True

    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
        options, prior, prior, correspondences)
    if not success or len(inlier_indices) < min_inliers:
        return

    # Keep only inlier correspondences, then:
    view_graph.AddEdge(view_id1, view_id2, twoview_info)
    for corr in inlier_correspondences:
        track_builder.AddFeatureCorrespondence(
            view_id1, corr.feature1, view_id2, corr.feature2)
    ```

=== "C++"

    ```cpp
    theia::EstimateTwoViewInfoOptions options;
    options.max_sampson_error_pixels = 1.0;

    theia::TwoViewInfo twoview_info;
    std::vector<int> inlier_indices;
    const bool ok = theia::EstimateTwoViewInfo(
        options, prior, prior, correspondences, &twoview_info, &inlier_indices);
    if (!ok || inlier_indices.size() < min_inliers) {
      return;
    }

    view_graph.AddEdge(view_id1, view_id2, twoview_info);
    // Add inlier FeatureCorrespondences to TrackBuilder similarly.
    ```

### 4.4 Build tracks and run an estimator

=== "Python"

    ```python
    track_builder.BuildTracks(recon)

    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = 4
    options.filter_relative_translations_with_1dsfm = True
    # Global example:
    options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
    options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT

    estimator = pt.sfm.GlobalReconstructionEstimator(options)
    # or: IncrementalReconstructionEstimator / HybridReconstructionEstimator
    summary = estimator.Estimate(view_graph, recon)
    print(summary.message)
    ```

=== "C++"

    ```cpp
    track_builder.BuildTracks(&reconstruction);

    theia::ReconstructionEstimatorOptions options;
    options.num_threads = 4;
    options.filter_relative_translations_with_1dsfm = true;
    options.global_rotation_estimator_type =
        theia::GlobalRotationEstimatorType::HYBRID;
    options.global_position_estimator_type =
        theia::GlobalPositionEstimatorType::LIGT;

    theia::GlobalReconstructionEstimator estimator(options);
    const theia::ReconstructionEstimatorSummary summary =
        estimator.Estimate(&view_graph, &reconstruction);
    ```

### 4.5 Export

=== "Python"

    ```python
    pt.io.WriteReconstruction(recon, "/tmp/scene.recon")
    pt.io.WritePlyFile("/tmp/scene.ply", recon, (255, 0, 0), 2)
    ```

=== "C++"

    ```cpp
    #include <Eigen/Core>
    #include "theia/io/reconstruction_writer.h"
    #include "theia/io/write_ply_file.h"

    theia::WriteReconstruction(reconstruction, "/tmp/scene.recon");
    theia::WritePlyFile("/tmp/scene.ply",
                        reconstruction,
                        Eigen::Vector3i(255, 0, 0),
                        /*min_num_observations_per_point=*/2);
    ```

---

## 5. Calibrated stereo / multi-camera rigs (1.2.0+)

Additive layer; do **not** replace Views.

| Step | Action |
|------|--------|
| 1 | `AddCameraRig` with per-sensor extrinsics in the body frame (calibrated) |
| 2 | For each timestamp, `AddRigCapture` and attach member Views via membership APIs |
| 3 | Features / matches still on Views; build `ViewGraph` as usual (including intra-rig edges if available) |
| 4 | Run `IncrementalRigReconstructor` or `GlobalRigReconstructor` |

=== "Python"

    ```python
    opts = pt.sfm.GlobalRigReconstructorOptions()
    opts.sfm_options.global_rotation_estimator_type = (
        pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2)
    opts.sfm_options.global_position_estimator_type = (
        pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION)
    summary = pt.sfm.GlobalRigReconstructor(opts).Estimate(view_graph, reconstruction)
    ```

=== "C++"

    ```cpp
    #include "theia/sfm/global_rig_reconstructor.h"

    theia::GlobalRigReconstructorOptions opts;
    opts.sfm_options.global_rotation_estimator_type =
        theia::GlobalRotationEstimatorType::ROBUST_L1L2;
    opts.sfm_options.global_position_estimator_type =
        theia::GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION;
    theia::GlobalRigReconstructor reconstructor(opts);
    const auto summary = reconstructor.Estimate(&view_graph, &reconstruction);
    ```

Full concepts and pose composition: [Rigs](rigs.md). Runnable script: [`pyexamples/stereo/stereo_rig_reconstruction.py`](https://github.com/urbste/pyTheiaSfM/blob/master/pyexamples/stereo/stereo_rig_reconstruction.py).

**Agent constraints for rigs**

- Averaging does **not** optimize inter-camera extrinsics (calibrated-only for now).
- One capture = one timestamp = one body pose.
- After BA, View poses are expected to stay consistent with the calibrated rig (propagated / snapped).

---

## 6. Choosing an estimator

| Need | Prefer |
|------|--------|
| Robust sequential growth, smaller sets | `IncrementalReconstructionEstimator` |
| Speed / whole-graph posing | `GlobalReconstructionEstimator` (+ rotation/position estimator enums) |
| Mix of both | `HybridReconstructionEstimator` |
| Calibrated multi-camera body trajectory | `IncrementalRigReconstructor` / `GlobalRigReconstructor` |
| Fuse two existing reconstructions | [Cross-run alignment](cross_run_alignment.md) (Sim(3) / relative pose BA) |

Tune via `ReconstructionEstimatorOptions` (threads, BA loss, rotation filtering, 1DSfM translation filtering, global estimator enums). See [Global pose estimation](global_pose_estimation.md) and [Bundle adjustment](bundle_adjustment.md).

---

## 7. Do / don't checklist

**Do**

- Detect and match features in Python (or your C++ app), then call Theia for geometry.
- Set intrinsics priors (or EXIF-derived priors) before `SetCameraIntrinsicsFromPriors`.
- Threshold on **verified** inliers after `EstimateTwoViewInfo`, not only putative matches.
- Call `TrackBuilder.BuildTracks` before `Estimate`.
- Pass the **same** `ViewGraph` + `Reconstruction` pair into the estimator; both are updated / consumed as designed by that estimator.
- Prefer dual-language examples in docs with MkDocs tabs (`=== "Python"` / `=== "C++"`) when adding usage docs ([Contributing](contributions.md)).

**Don't**

- Don't assume OpenCV/OpenImageIO image loaders inside the C++ core (removed in pyTheia).
- Don't invent `theia::sfm::Reconstruction` — namespace is `theia::`.
- Don't put features only on the rig object — Views still own observations.
- Don't expect SuiteSparse/Cholmod-era sparse behavior; pyTheia uses Eigen-backed paths where GPL code was removed (see project README).
- Don't treat `pyexamples/` dataset paths or focal lengths as universal defaults.

---

## 8. Where to look (source of truth)

| Need | Location |
|------|----------|
| Python symbols / overloads | `src/pytheia/pytheia/` stubs, bindings in `src/pytheia/*/…` |
| C++ API | `src/theia/sfm/*.h`, `src/theia/io/*.h` |
| Monocular pipeline example | `pyexamples/sfm/sfm_pipeline_fountain.py` |
| Matcher showcase | `examples/vismatch_sfm/`, [vismatch + SfM](examples_vismatch_sfm.md) |
| Rig example / tests | `pyexamples/stereo/stereo_rig_reconstruction.py`, `pytests/test_camera_rig.py` |
| ZED SVO → stereo frames | `pyexamples/preprocess/zed_svo_extract_stereo.py` (needs `pyzed`) |
| Repo contributor map (build, CI) | Root `AGENTS.md` |
| Human narrative API chapters | [API Reference](api.md) |

---

## 9. Minimal smoke test (Python)

After install (`pip install pytheia` or editable build):

```python
import pytheia as pt

r = pt.sfm.Reconstruction()
vid = r.AddView("frame0.png", 0)
assert vid != pt.sfm.kInvalidViewId
assert r.NumViews() == 1
print("ok", pt.sfm.Reconstruction)
```

If import fails with `GLIBCXX` on Linux/Conda, see README troubleshooting — do not “fix” by rewriting the API.
