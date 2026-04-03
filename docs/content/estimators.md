# Geometric estimators (RANSAC) {#documentation-estimators}

The headers under [`src/theia/sfm/estimators`](https://github.com/urbste/pyTheiaSfM/tree/master/src/theia/sfm/estimators) implement **high-level estimators**: they wrap minimal solvers (see [Pose](pose.md)) inside **sample-consensus** loops using `RansacParameters`, `RansacType`, and related types from [Ransac](ransac.md).

**pyTheia:** all bound estimators live on `pytheia.sfm` and return a tuple of the form **`(success, result, ransac_summary)`** (exact result type varies). Build RANSAC settings with `pytheia.solvers.RansacParameters()` and pass `pytheia.sfm.RansacType` (e.g. `RANSAC`, `PROSAC`, `LMED`, `EXHAUSTIVE`).

**Correspondence types**

- **2D–3D:** `pytheia.sfm.FeatureCorrespondence2D3D` (`feature`, `world_point`) — use **normalized** image coordinates when the header says so.
- **2D–2D:** `pytheia.matching.FeatureCorrespondence` (`feature1`, `feature2`) — **normalized** for essential / relative pose; **centered** (principal point at origin) for uncalibrated relative pose as noted below.
- **Multi-camera 2D–3D:** `pytheia.sfm.CameraAndFeatureCorrespondence2D3D` (`camera`, `observation`, `point3d`) for rigid alignment across rigs.

---

## How this relates to absolute vs relative pose {#estimators-problem-types}

The same distinction as on the [Pose](pose.md#pose-problem-types) page applies:

| Kind | Estimators | Typical inputs | Typical outputs |
|------|------------|----------------|-----------------|
| **Absolute** | Calibrated / uncalibrated PnP, known orientation, rigid or similarity 2D–3D alignment | 2D–3D matches (+ cameras for multi-view rig case) | Camera pose, focal length, or world alignment transform |
| **Relative / two-view** | Essential, fundamental, homography, relative pose, uncalibrated relative, known orientation | 2D–2D matches | \(E\), \(F\), \(H\), `RelativePose`, `UncalibratedRelativePose`, etc. |
| **Other** | Triangulation, dominant plane | Multi-view rays or 3D points | 3D point, plane |

Minimal closed-form solvers (five-point, eight-point, DLS PnP, …) are documented under [Pose](pose.md); **this page** is about the **RANSAC-fronted** APIs.

---

## Absolute pose and 2D–3D alignment {#estimators-absolute}

### `EstimateCalibratedAbsolutePose` {#estimate-calibrated-absolute-pose}

**Header:** [`estimate_calibrated_absolute_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_calibrated_absolute_pose.h)

Calibrated perspective-*n*-point with RANSAC. Correspondences must be **normalized** by intrinsics. Choose backend via `PnPType`: `KNEIP`, `DLS`, or `SQPnP`.

**Returns:** `(success, CalibratedAbsolutePose, RansacSummary)` — pose has `rotation` and `position`.

---

### `EstimateUncalibratedAbsolutePose` {#estimate-uncalibrated-absolute-pose}

**Header:** [`estimate_uncalibrated_absolute_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h)

Single focal length (shared \(f\) for \(K \sim \mathrm{diag}(f,f,1)\)) plus rotation and translation. Input correspondences **normalized** as for calibrated PnP.

**Returns:** `(success, UncalibratedAbsolutePose, RansacSummary)` — includes `focal_length`.

---

### `EstimateRadialDistUncalibratedAbsolutePose` {#estimate-radial-dist-uncalibrated-absolute-pose}

**Header:** [`estimate_radial_dist_uncalibrated_absolute_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h)

**Uncalibrated absolute pose with one radial distortion parameter** (division model), estimated with RANSAC / PROSAC / LMED / exhaustive sampling. Minimal sample size is **four** normalized 2D–3D correspondences (principal point at origin), same normalization as other PnP-style estimators.

Pass bounds and filtering limits via **`RadialDistUncalibratedAbsolutePoseMetaData`**: `min_focal_length`, `max_focal_length`, `min_radial_distortion`, `max_radial_distortion` (defaults in the header are often tightened in application code).

**Returns:** `(success, RadialDistUncalibratedAbsolutePose, RansacSummary)` — pose has `rotation`, `translation`, `focal_length`, and `radial_distortion` (compatible with **division undistortion** interpretation; see the four-point helper in [`pose.md`](pose.md)).

**pyTheia:** `success, pose, summary = pt.sfm.EstimateRadialDistUncalibratedAbsolutePose(ransac_params, ransac_type, correspondences, meta)` with `ransac_type` in `pt.sfm.RansacType` and each correspondence a `pt.sfm.FeatureCorrespondence2D3D`.

---

### `EstimateAbsolutePoseWithKnownOrientation` {#estimate-absolute-pose-known-orientation}

**Header:** [`estimate_absolute_pose_with_known_orientation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_absolute_pose_with_known_orientation.h)

**Position-only** PnP when **rotation** (e.g. from IMU) is known. Normalized 2D–3D correspondences.

**Returns:** `(success, camera_position, RansacSummary)` as `Vector3d`.

---

### `EstimateRigidTransformation2D3D` {#estimate-rigid-2d3d}

**Header:** [`estimate_rigid_transformation_2d_3d.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_rigid_transformation_2d_3d.h)

Two overloads exist in C++; pyTheia exposes them as two functions:

1. **`EstimateRigidTransformation2D3D`** — `CameraAndFeatureCorrespondence2D3D` list: align **multiple cameras** (e.g. SLAM rig) to a **common 3D frame** via a **rigid** transform (same scale), using a generalized PnP-style formulation + RANSAC.
2. **`EstimateRigidTransformation2D3DNormalized`** — `FeatureCorrespondence2D3D` only: **single** normalized camera or non-central / UPnP-style use (see header comments).

**Returns:** `(success, RigidTransformation, RansacSummary)`.

---

### `EstimateSimilarityTransformation2D3D` (C++ only) {#estimate-similarity-2d3d}

**Header:** [`estimate_similarity_transformation_2d_3d.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_similarity_transformation_2d_3d.h)

Like the multi-camera rigid case, but estimates a **7-DOF similarity** (gDLS / scale). **Not** currently exposed in pyTheia; use from C++ when aligning reconstructions at unknown scale. Cite *gDLS* (Sweeney et al., ECCV 2014) per the header.

---

## Relative pose and two-view geometry {#estimators-relative}

### `EstimateRelativePose` {#estimate-relative-pose}

**Header:** [`estimate_relative_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_relative_pose.h)

Calibrated two-view **relative pose** with RANSAC (five-point essential matrix pipeline). Uses **normalized** `FeatureCorrespondence` pairs.

**Returns:** `(success, RelativePose, RansacSummary)` with `essential_matrix`, `rotation`, `position` (translation up to scale).

---

### `EstimateUncalibratedRelativePose` {#estimate-uncalibrated-relative-pose}

**Header:** [`estimate_uncalibrated_relative_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_uncalibrated_relative_pose.h)

Two-view geometry with **unknown focal lengths**. Correspondences must be **centered** so the **principal point is \((0,0)\)**. Pass `min_max_focal_length` as a 2-vector \((f_\min, f_\max)\).

**Returns:** `(success, UncalibratedRelativePose, RansacSummary)` — includes `fundamental_matrix`, `focal_length1`, `focal_length2`, `rotation`, `position`.

---

### `EstimateRelativePoseWithKnownOrientation` {#estimate-relative-pose-known-orientation}

**Header:** [`estimate_relative_pose_with_known_orientation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_relative_pose_with_known_orientation.h)

**Relative translation direction** (between two calibrated views) when rotations are already aligned / known. Uses **normalized, rotated** `FeatureCorrespondence` pairs (see header).

**Returns:** `(success, relative_camera2_position, RansacSummary)`.

---

### `EstimateEssentialMatrix` {#estimate-essential-matrix}

**Header:** [`estimate_essential_matrix.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_essential_matrix.h)

RANSAC **essential matrix** from **normalized** 2D–2D correspondences.

**Returns:** `(success, E (3×3), RansacSummary)`.

---

### `EstimateFundamentalMatrix` {#estimate-fundamental-matrix}

**Header:** [`estimate_fundamental_matrix.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_fundamental_matrix.h)

RANSAC **fundamental matrix** via the **eight-point** pipeline on generic `FeatureCorrespondence` pairs (see tests / error threshold for whether coordinates should be normalized or in pixels).

**Returns:** `(success, F (3×3), RansacSummary)`.

---

### `EstimateHomography` {#estimate-homography}

**Header:** [`estimate_homography.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_homography.h)

RANSAC **planar homography** \(x_2 \sim H x_1\) from `FeatureCorrespondence` pairs.

**Returns:** `(success, H (3×3), RansacSummary)`.

---

### `EstimateRadialHomographyMatrix` {#estimate-radial-homography}

**Header:** [`estimate_radial_distortion_homography.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_radial_distortion_homography.h)

Homography estimation with **division-model radial distortion** on both images. Uses `RadialDistortionFeatureCorrespondence` (features plus distortion bounds).

**Returns:** `(success, RadialHomographyResult, RansacSummary)` (structure mirrors C++; see bindings for fields).

---

## Multi-view point and scene structure {#estimators-structure}

### `EstimateTriangulation` {#estimate-triangulation}

**Header:** [`estimate_triangulation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_triangulation.h)

RANSAC over **2-view subsets** to find a **consistent 3D point** from multiple cameras and observations (not a pose estimator; placed here because it shares the same estimator / RANSAC stack).

**Returns:** `(success, triangulated_point, RansacSummary)` — homogeneous 4-vector.

---

### `EstimateDominantPlaneFromPoints` {#estimate-dominant-plane}

**Header:** [`estimate_dominant_plane_from_points.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimate_dominant_plane_from_points.h)

Fits a **dominant plane** to a cloud of 3D points with RANSAC (`Plane` model).

**Returns:** `(success, Plane, RansacSummary)`.

---

## Python usage sketch {#estimators-python-sketch}

```python
import pytheia as pt

params = pt.solvers.RansacParameters()
params.error_thresh = 1e-2  # tune to your residual units
params.max_iterations = 500

corres = []  # list of pt.sfm.FeatureCorrespondence2D3D()
# ... fill normalized feature + world_point ...

ok, pose, summary = pt.sfm.EstimateCalibratedAbsolutePose(
    params,
    pt.sfm.RansacType.RANSAC,
    pt.sfm.PnPType.DLS,
    corres,
)
# e.g. summary.inliers, summary.num_iterations (pt.solvers.RansacSummary)
```

For 2D–2D relative pose, build `pt.matching.FeatureCorrespondence()` objects with `feature1` / `feature2` **normalized** for `EstimateRelativePose` and `EstimateEssentialMatrix`.

---

## C++-only and related headers {#estimators-cpp-only}

These sources participate in the C++ library but **do not** add a separate top-level `Estimate*` Python entry beyond what is listed above (bindings use the same types):

| Header | Role |
|--------|------|
| [`feature_correspondence_2d_3d.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/feature_correspondence_2d_3d.h) | `FeatureCorrespondence2D3D` struct |
| [`camera_and_feature_correspondence_2d_3d.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/camera_and_feature_correspondence_2d_3d.h) | Multi-camera 2D–3D datum |

Wrappers are implemented in [`estimators_wrapper.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/estimators/estimators_wrapper.cc).

---

## See also {#estimators-see-also}

- [Pose](pose.md) — minimal closed-form solvers (five-point, eight-point, PnP, …)
- [Ransac](ransac.md) — `Estimator`, `RansacParameters`, variants
- [SfM](sfm.md) — reconstruction-level helpers (e.g. `EstimateTwoViewInfo`, which composes two-view estimation for the incremental pipeline)
