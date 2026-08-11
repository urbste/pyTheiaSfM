# GlobalRigReconstructor + Stereo Example Implementation Plan

> **For agentic workers:** Execute task-by-task. Calibrated rigs only (no extrinsic estimation in averaging).

**Goal:** Add MGSfM-inspired `GlobalRigReconstructor` using Theia’s rotation/position averaging stack on capture nodes, plus docs, bindings, and a stereo example with vismatch EDM.

**Architecture:** Derive a capture–capture `ViewGraph` (vertices = `CaptureId`) from the View–View match graph using known `RigSensor` extrinsics. Run selectable `GlobalRotationEstimatorType` / `GlobalPositionEstimatorType` on that graph. Write poses to `RigCapture`, propagate to Views, triangulate, BA.

**Tech Stack:** Existing Theia global estimators, `ViewGraph`/`TwoViewInfo`, vismatch `edm` matcher in Python example.

## Global Constraints

- Calibrated rigs only — do not optimize `RigSensor` extrinsics during averaging or BA in this PR.
- Reuse Theia averaging (`ROBUST_L1L2`, `NONLINEAR`, `LINEAR`, `LAGRANGE_DUAL`, `HYBRID` rotations; `NONLINEAR`, `LINEAR_TRIPLET`, `LEAST_UNSQUARED_DEVIATION`, `LIGT`, `GLOMAP` positions).
- Monocular APIs unchanged.
- Example may soft-depend on `vismatch` (optional extras).

---

### Task 1: Capture-relative pose helpers

**Files:**
- Create: `src/theia/sfm/rig/capture_view_graph.h`, `.cc`
- Modify: `src/theia/CMakeLists.txt`
- Test: extend `pytests/test_camera_rig.py` or add `pytests/test_global_rig_reconstructor.py`

**Produces:**
```cpp
bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph);
// Vertices of *capture_view_graph are CaptureIds (uint32).
// TwoViewInfo is relative pose of capture2 w.r.t. capture1 at identity,
// after stripping known RigSensor extrinsics.
```

Compose per MGSfM Eq. (8) style with our Camera convention. When multiple View edges map to the same Capture pair, keep the edge with most `num_verified_matches`.

### Task 2: GlobalRigReconstructor

**Files:**
- Create: `src/theia/sfm/global_rig_reconstructor.h`, `.cc`
- Modify: `src/theia/CMakeLists.txt`

**Produces:**
```cpp
struct GlobalRigReconstructorOptions {
  GlobalRotationEstimatorType global_rotation_estimator_type =
      GlobalRotationEstimatorType::ROBUST_L1L2;
  GlobalPositionEstimatorType global_position_estimator_type =
      GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION;
  // Same filtering / BA knobs as global SfM where applicable.
  int min_num_two_view_inliers = 30;
  double max_reprojection_error_in_pixels = 4.0;
  ...
};

class GlobalRigReconstructor {
  ReconstructionEstimatorSummary Estimate(ViewGraph* view_graph,
                                          Reconstruction* reconstruction);
};
```

Flow: filter → build capture graph → estimate rotations → filter → optimize/filter translations → estimate positions → set RigCapture poses + propagate → triangulate → BA (views; extrinsics stay snapped via re-propagate).

### Task 3: Bindings + docs

**Files:**
- Modify: `src/pytheia/sfm/sfm.cc`
- Create: `docs/content/rigs.md`
- Modify: `docs/mkdocs.yml`, `docs/content/api.md`
- Update: design/spec notes (calibrated-only global path)

### Task 4: Stereo example

**Files:**
- Create: `pyexamples/stereo_rig_reconstruction.py`
- Optional: `docs/content/examples` cross-link / `examples_showcase.md` bullet

Args: image dirs or left/right sequences, `--baseline`, `--focal`, `--cx/--cy`, `--matcher edm`, `--rotation_estimator`, `--position_estimator`, `--method global|incremental`.

Use `vismatch.get_matcher("edm")` when available; clear error if missing.
