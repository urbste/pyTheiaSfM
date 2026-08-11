# Multi-camera Rig Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** Add `CameraRig` / `RigCapture` to `Reconstruction` and an `IncrementalRigReconstructor` that seeds from intra-rig triangulation and localizes capture-by-capture.

**Architecture:** Additive maps on `Reconstruction` (like intrinsics groups). Abstract body frame at identity; sensors store poses in that rig CS. Views keep features; reconstructor propagates camera extrinsics from each capture pose. Triangulation reuses `TrackEstimator`.

**Tech Stack:** C++17 Theia, Eigen, existing estimators (`EstimateRigidTransformation2D3D`, `LocalizeViewToReconstruction`, `TrackEstimator`), pybind11.

## Global Constraints

- No breaking monocular `AddView` / estimator APIs.
- Default rig CS = abstract body (identity); sensors carry offsets.
- One capture = one timestamp = one rig pose.
- Features/matches remain on Views / ViewGraph.

---

### Task 1: Types + CameraRig + RigCapture

**Files:**
- Modify: `src/theia/sfm/types.h`
- Create: `src/theia/sfm/rig/camera_rig.h`, `camera_rig.cc`, `rig_capture.h`, `rig_capture.cc`, `rig_utils.h`, `rig_utils.cc`
- Modify: `src/theia/CMakeLists.txt` (add sources)

- [ ] Add `RigId`, `RigCameraId`, `CaptureId` + invalid sentinels
- [ ] Implement `RigSensor` / `CameraRig` / `RigCapture`
- [ ] Implement `ComposeCameraPoseFromRig`, `PropagateCameraPosesForCapture`

### Task 2: Reconstruction membership API

**Files:**
- Modify: `src/theia/sfm/reconstruction.h`, `reconstruction.cc`

- [ ] Maps for rigs, captures, view membership; cereal v1
- [ ] `AddCameraRig`, `AddRigCapture`, accessors; allow shared View timestamps within a capture

### Task 3: IncrementalRigReconstructor

**Files:**
- Create: `src/theia/sfm/incremental_rig_reconstructor.h`, `.cc`
- Modify: `src/theia/CMakeLists.txt`

- [ ] Seed identity capture + triangulate
- [ ] Localize next captures (rigid 2D–3D, fallback single-view)
- [ ] TrackEstimator + optional BA + re-propagate

### Task 4: Python bindings + pytest

**Files:**
- Modify: `src/pytheia/sfm/sfm.cc`, stubs if practical
- Create: `pytests/test_camera_rig.py`

- [ ] Bind new types and reconstructor
- [ ] Synthetic stereo trajectory smoke test
