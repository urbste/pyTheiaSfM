# Pose and Resectioning {#documentation-pose}

Theia implements **absolute** pose (resectioning / PnP), **relative** two-view geometry (essential and fundamental matrices, homographies), and constrained variants (partial rotation, orthographic models). C++ uses Eigen; pyTheia exposes the same entry points under `pytheia.sfm` with NumPy arrays.

For **Python** vs **C++** usage patterns, see [Python API overview](python_wrapper.md).

## Understanding absolute vs relative pose {#pose-problem-types}

**Absolute pose** means: you have a **3D model or map** in a fixed world (or object) frame and **2D observations** in one image. The goal is to estimate that **camera’s** rigid pose—how the camera sits relative to the 3D structure. In Theia’s usual convention for calibrated pinhole cameras, world points map as \(X_\mathrm{cam} = R X_\mathrm{world} + t\) using **normalized** image coordinates. Some solvers also estimate **unknown focal length** and/or **radial distortion** from the same 2D–3D correspondences.

**Relative pose and two-view geometry** means: you have **matching points in two images** (2D–2D) and want the **epipolar relationship** between the views—typically an **essential matrix** \(E\) (calibrated) or **fundamental matrix** \(F\) (uncalibrated), a **homography** \(H\) when the scene is dominated by a plane, or an explicit **relative** rigid or similarity transform. Translation is usually **recoverable only up to scale** until you fix scale from elsewhere (e.g. PnP or metric scene).

The rest of this page is split into **absolute** solvers, **relative** solvers, and small **matrix utilities** used after estimating \(E\) or \(F\).

For **RANSAC-wrapped** estimators (e.g. calibrated PnP with inliers, two-view models from noisy matches), see [Geometric estimators](estimators.md).

## Absolute pose estimation {#absolute-pose-estimation}

The routines below take **2D–3D correspondences** (and sometimes unknown intrinsics-related parameters). They estimate a **single** camera pose or a specialized absolute model (planar orthographic, etc.).

### MLPnP (maximum likelihood PnP) {#section-mlpnp}

**MLPnP** solves the calibrated perspective-*n*-point problem by maximum-likelihood estimation while accounting for observation noise projected into the tangent space of viewing rays [UrbanMLPnP2016](bibliography.md#UrbanMLPnP2016). It returns a **single** world-to-camera rotation `R` and translation `t` (same \(3 \times 4\) \([R \mid t]\) convention as other absolute-pose helpers: map a world point \(X\) to the camera as \(R X + t\), then divide by \(Z\) for normalized pinhole projection).

**Signature (C++):** [`mlpnp.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/mlpnp.h)

```cpp
bool MLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
           const std::vector<Eigen::Matrix3d>& feature_covariances,
           const std::vector<Eigen::Vector3d>& world_points,
           Eigen::Matrix3d* solution_rotation,
           Eigen::Vector3d* solution_translation);
```

**Inputs**

- **`norm_feature_points`:** `Vector2d` per correspondence — **normalized** coordinates (focal length 1, origin at principal point), i.e. \((x/z,\, y/z)\) in the pinhole camera frame.
- **`world_points`:** `Vector3d` world points; **same length** as `norm_feature_points`.
- **`feature_covariances`:** either **empty** for uniform weighting, or one **`3×3`** matrix per point (symmetric covariance of the homogeneous image / bearing error). When provided, each covariance is mapped to the 2D null space of the corresponding ray inside the solver (see the paper).

Use enough points for stable disambiguation (unit tests typically use on the order of **6–10+** correspondences). Planar 3D configurations are detected and handled in a separate code path.

**pyTheia:** `ok, R, t = pytheia.sfm.MLPnP(norm_features, covariances, world_points)` with `covariances=[]` if you do not use per-point covariances.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    # Ground-truth world-to-camera (R, t): X_cam = R @ X_world + t
    R_true = np.array(
        [[0.996, -0.087, 0.0], [0.087, 0.996, 0.0], [0.0, 0.0, 1.0]]
    )
    t_true = np.array([0.05, -0.02, 1.5])

    world_points = []
    norm_features = []
    for _ in range(10):
        X = np.array(
            [
                np.random.uniform(-1.0, 1.0),
                np.random.uniform(-1.0, 1.0),
                np.random.uniform(2.0, 5.0),
            ]
        )
        world_points.append(X)
        Xc = R_true @ X + t_true
        norm_features.append(np.array([Xc[0] / Xc[2], Xc[1] / Xc[2]]))

    # Uniform noise model: pass empty list for covariances
    ok, R_est, t_est = pt.sfm.MLPnP(norm_features, [], world_points)
    assert ok

    # Optional: one 3x3 covariance per point (e.g. isotropic on homogeneous coords)
    # sigma = 1e-3
    # covs = [np.eye(3) * (sigma ** 2) for _ in norm_features]
    # ok, R_est, t_est = pt.sfm.MLPnP(norm_features, covs, world_points)
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/mlpnp.h"

    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Matrix3d;

    std::vector<Eigen::Vector2d> norm_features;
    std::vector<Eigen::Vector3d> world_points;
    // Fill norm_features with normalized (x/z, y/z) and matching world_points.

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    std::vector<Eigen::Matrix3d> feature_covariances;  // empty => uniform weights
    const bool ok =
        theia::MLPnP(norm_features, feature_covariances, world_points, &R, &t);
    ```

**Reference:** S. Urban, J. Leitloff, S. Hinz, *MLPnP – A Real-Time Maximum Likelihood Solution to the Perspective-n-Point Problem*, ISPRS Annals III-3, 2016 ([DOI](https://doi.org/10.5194/isprs-annals-III-3-131-2016), [arXiv:1607.08112](https://arxiv.org/abs/1607.08112)).

### Perspective Three Point (P3P) {#section-p3p}

**Signature (C++):** [`perspective_three_point.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/perspective_three_point.h)

```cpp
bool PoseFromThreePoints(const std::vector<Eigen::Vector2d>& feature_point,
                         const std::vector<Eigen::Vector3d>& world_point,
                         std::vector<Eigen::Matrix3d>* solution_rotations,
                         std::vector<Eigen::Vector3d>* solution_translations);
```

Computes camera pose using the three point algorithm and returns all possible solutions (up to 4). Follows steps from the paper "A Novel Parameterization of the Perspective-Three-Point Problem for a direct computation of Absolute Camera position and Orientation" by [Kneip](bibliography.md#Kneip). This algorithm has been proven to be up to an order of magnitude faster than other methods. The output rotation and translation define world-to-camera transformation.

`feature_position`: Image points corresponding to model points. These should be calibrated image points as opposed to pixel values.

`world_point`: 3D location of features.

`solution_rotations`: the rotation matrix of the candidate solutions

`solution_translation`: the translation of the candidate solutions

`returns`: Whether the pose was computed successfully, along with the output parameters `rotation` and `translation` filled with the valid poses.

**pyTheia:** `success, rotations, translations = pytheia.sfm.PoseFromThreePoints(features, world_points)` — each of `rotations` and `translations` is a list of candidate poses.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    R_true = np.eye(3)
    t_true = np.array([0.0, 0.0, 2.0])
    world = [
        np.array([1.0, 0.5, 3.0]),
        np.array([-0.5, 1.0, 4.0]),
        np.array([0.2, -0.8, 2.5]),
    ]
    features = []
    for X in world:
        Xc = R_true @ X + t_true
        features.append(np.array([Xc[0] / Xc[2], Xc[1] / Xc[2]]))

    ok, Rs, ts = pt.sfm.PoseFromThreePoints(features, world)
    assert ok and len(Rs) >= 1
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/perspective_three_point.h"

    std::vector<Eigen::Vector2d> features;  // normalized / calibrated 2D
    std::vector<Eigen::Vector3d> world;     // matching 3D points (3+ each)
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    const bool ok = theia::PoseFromThreePoints(features, world, &rotations, &translations);
    ```

### Perspective N-Point {#section-dls_pnp}

**Signature (C++):** [`dls_pnp.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/dls_pnp.h)

```cpp
bool DlsPnp(const std::vector<Eigen::Vector2d>& feature_positions,
            const std::vector<Eigen::Vector3d>& world_point,
            std::vector<Eigen::Quaterniond>* solution_rotation,
            std::vector<Eigen::Vector3d>* solution_translation);
```

Computes the camera pose using the Perspective N-point method from "A Direct Least-Squares (DLS) Method for PnP" by [Hesch](bibliography.md#Hesch) and Stergios Roumeliotis. This method is extremely scalable and highly accurate for the PnP problem. A minimum of 4 points are required, but there is no maximum number of points allowed as this is a least-squared approach. Theoretically, up to 27 solutions may be returned, but in practice only 4 real solutions arise and in almost all cases where \(n \geq 6\) there is only one solution which places the observed points in front of the camera. The returned rotation and translations are world-to-camera transformations.

`feature_position`: Normalized image rays corresponding to model points. Must contain at least 4 points.

`points_3d`: 3D location of features. Must correspond to the image_ray of the same index. Must contain the same number of points as image_ray, and at least 4.

`solution_rotation`: the rotation quaternion of the candidate solutions

`solution_translation`: the translation of the candidate solutions

**pyTheia:** `quats, translations = pytheia.sfm.DlsPnp(features, world_points)` — each quaternion is a \(4 \times 1\) array in **w, x, y, z** order (matching the C++ wrapper).

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    R_true = np.eye(3)
    t_true = np.array([0.0, 0.0, 1.5])
    features, world = [], []
    for _ in range(8):
        X = np.random.uniform(-1.0, 1.0, size=3)
        X[2] += 2.0
        world.append(X)
        Xc = R_true @ X + t_true
        features.append(np.array([Xc[0] / Xc[2], Xc[1] / Xc[2]]))

    quats, ts = pt.sfm.DlsPnp(features, world)
    assert len(quats) >= 1
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/dls_pnp.h"

    std::vector<Eigen::Vector2d> features;
    std::vector<Eigen::Vector3d> world;
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    const bool ok = theia::DlsPnp(features, world, &rotations, &translations);
    ```

### SQPnP (Globally Optimal PnP) {#section-sqpnp}

**Signature (C++):** [`sqpnp.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/sqpnp.h)

```cpp
bool SQPnP(const std::vector<Eigen::Vector2d>& feature_positions,
           const std::vector<Eigen::Vector3d>& world_points,
           std::vector<Eigen::Quaterniond>* solution_rotation,
           std::vector<Eigen::Vector3d>* solution_translation);
```

Consistently fast and globally optimal PnP solver [TerzakisECCV2020](bibliography.md#TerzakisECCV2020). Requires at least three correspondences.

**pyTheia:** `quats, translations = pytheia.sfm.SQPnP(features, world_points)` — quaternions as \(4 \times 1\) in **w, x, y, z** order.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    R_true = np.eye(3)
    t_true = np.array([0.0, 0.0, 1.2])
    features, world = [], []
    for _ in range(6):
        X = np.random.uniform(-0.5, 0.5, 3) + np.array([0.0, 0.0, 2.0])
        world.append(X)
        Xc = R_true @ X + t_true
        features.append(np.array([Xc[0] / Xc[2], Xc[1] / Xc[2]]))

    quats, ts = pt.sfm.SQPnP(features, world)
    assert len(quats) >= 1
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/sqpnp.h"

    std::vector<Eigen::Vector2d> features;
    std::vector<Eigen::Vector3d> world;
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    const bool ok = theia::SQPnP(features, world, &rotations, &translations);
    ```

### Four Point Focal Length {#section-four_point_focal_length}

**Signature (C++):** [`four_point_focal_length.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/four_point_focal_length.h)

```cpp
int FourPointPoseAndFocalLength(
    const std::vector<Eigen::Vector2d>& feature_vectors,
    const std::vector<Eigen::Vector3d>& world_points,
    std::vector<Eigen::Matrix<double, 3, 4>>* projection_matrices);
```

Computes the camera pose and unknown focal length of an image given four 2D-3D correspondences, following the method of [Bujnak](bibliography.md#Bujnak). This method involves computing a grobner basis from a modified constraint of the focal length and pose projection.

`feature_position`: Normalized image rays corresponding to model points. Must contain at least 4 points.

`points_3d`: 3D location of features. Must correspond to the image_ray of the same index. Must contain the same number of points as image_ray, and at least 4.

`projection_matrices`: The solution world-to-camera projection matrices, inclusive of the unknown focal length. For a focal length f and a camera calibration matrix $K=diag(f, f, 1)$, the projection matrices returned are of the form $P = K * [R | t]$.

**pyTheia:** `num_solutions, Ps = pytheia.sfm.FourPointPoseAndFocalLength(features, world_points)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    # Synthetic normalized 2D + 3D (four pairs); real data should come from calibration.
    f = 600.0
    R = np.eye(3)
    t = np.array([0.01, -0.02, 1.2])
    world = [
        np.array([0.1, 0.2, 2.5]),
        np.array([-0.2, 0.15, 3.0]),
        np.array([0.05, -0.25, 2.8]),
        np.array([-0.1, -0.1, 3.2]),
    ]
    features = []
    for X in world:
        Xc = R @ X + t
        features.append(np.array([f * Xc[0] / Xc[2], f * Xc[1] / Xc[2]]) / f)

    n, Ps = pt.sfm.FourPointPoseAndFocalLength(features, world)
    assert n > 0 and len(Ps) > 0
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/four_point_focal_length.h"

    std::vector<Eigen::Vector2d> features;
    std::vector<Eigen::Vector3d> world;
    std::vector<Eigen::Matrix<double, 3, 4>> projection_matrices;
    const int n = theia::FourPointPoseAndFocalLength(features, world, &projection_matrices);
    ```

### Five Point Focal Length and Radial Distortion {#section-five_point_focal_length_radial_distortion}

**Signature (C++):** [`five_point_focal_length_radial_distortion.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/five_point_focal_length_radial_distortion.h)

```cpp
bool FivePointFocalLengthRadialDistortion(
    const std::vector<Eigen::Vector2d>& feature_positions,
    const std::vector<Eigen::Vector3d>& world_points,
    const int num_radial_distortion_params,
    std::vector<Eigen::Matrix<double, 3, 4>>* projection_matrices,
    std::vector<std::vector<double>>* radial_distortions);
```

Compute the absolute pose, focal length, and radial distortion of a camera using five 3D-to-2D correspondences [Kukelova](bibliography.md#Kukelova). The method solves for the projection matrix (up to scale) by using a cross product constraint on the standard projection equation. This allows for simple solution to the first two rows of the projection matrix, and the third row (which contains the focal length and distortion parameters) can then be solved with SVD on the remaining constraint equations from the first row of the projection matrix. See the paper for more details.

`feature_positions`: the 2D location of image features. Exactly five features must be passed in.

`world_points`: 3D world points corresponding to the features observed. Exactly five points must be passed in.

`num_radial_distortion_params`: The number of radial distortion paramters to solve for. Must be 1, 2, or 3.

`projection_matrices`: Camera projection matrices (that encapsulate focal length). These solutions are only valid up to scale.

`radial_distortions`: Each entry of this vector contains a vector with the radial distortion parameters (up to 3, but however many were specified in `num_radial_distortion_params`).

`return`: true if successful, false if not.

**pyTheia:** `success, projection_matrices, radial_distortions = pytheia.sfm.FivePointFocalLengthRadialDistortion(features, world_points, num_radial_distortion_params)` with `num_radial_distortion_params` in \(\{1,2,3\}\).

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    f = 400.0
    R = np.eye(3)
    t = np.array([[0.0], [0.0], [1.0]])
    K = np.diag([1.0, 1.0, 1.0 / f])
    P = K @ np.hstack([R, t])
    world = [np.random.uniform(-0.5, 0.5, 3) + np.array([0.0, 0.0, 2.0]) for _ in range(5)]
    features = []
    for X in world:
        x = P @ np.append(X, 1.0)
        features.append(np.array([x[0] / x[2], x[1] / x[2]]))

    ok, Ps, rads = pt.sfm.FivePointFocalLengthRadialDistortion(features, world, 1)
    assert ok
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/five_point_focal_length_radial_distortion.h"

    std::vector<Eigen::Vector2d> features(5);
    std::vector<Eigen::Vector3d> world(5);
    const int num_k = 1;
    std::vector<Eigen::Matrix<double, 3, 4>> projection_matrices;
    std::vector<std::vector<double>> radial_distortions;
    const bool ok = theia::FivePointFocalLengthRadialDistortion(
        features, world, num_k, &projection_matrices, &radial_distortions);
    ```

### Four point focal length and radial distortion {#section-four_point_focal_radial}

**Signature (C++):** [`four_point_focal_length_radial_distortion.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/four_point_focal_length_radial_distortion.h)

```cpp
bool FourPointsPoseFocalLengthRadialDistortion(
    const std::vector<Eigen::Vector2d> feature_vectors,
    const std::vector<Eigen::Vector3d> world_points,
    double max_focal_length,
    double min_focal_length,
    double max_distortion,
    double min_distortion,
    std::vector<Eigen::Matrix3d>* rotations,
    std::vector<Eigen::Vector3d>* translations,
    std::vector<double>* radial_distortions,
    std::vector<double>* focal_lengths);
```

Solves for pose, focal length, and radial distortion from **four** correspondences, with focal length and distortion bounded by the min/max parameters (Python wraps these in `RadialDistUncalibratedAbsolutePoseMetaData`).

**pyTheia (minimal solver):** `success, rotations, translations, radial_distortions, focal_lengths = pytheia.sfm.FourPointsPoseFocalLengthRadialDistortion(features, world_points, meta)` where `meta` is `pytheia.sfm.RadialDistUncalibratedAbsolutePoseMetaData` (`min_focal_length`, `max_focal_length`, `min_radial_distortion`, `max_radial_distortion`).

**pyTheia (RANSAC wrapper):** for **many** normalized `FeatureCorrespondence2D3D` and the same `meta`, use **`pytheia.sfm.EstimateRadialDistUncalibratedAbsolutePose(ransac_params, ransac_type, correspondences, meta)`** — returns `(success, RadialDistUncalibratedAbsolutePose, RansacSummary)` with a single best model (`rotation`, `translation`, `focal_length`, `radial_distortion`). See [Geometric estimators — `EstimateRadialDistUncalibratedAbsolutePose`](estimators.md#estimate-radial-dist-uncalibrated-absolute-pose).

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    meta = pt.sfm.RadialDistUncalibratedAbsolutePoseMetaData()
    meta.min_focal_length = 200.0
    meta.max_focal_length = 800.0
    meta.min_radial_distortion = -0.5
    meta.max_radial_distortion = 0.5

    f = 400.0
    R = np.eye(3)
    t = np.array([[0.0], [0.0], [1.0]])
    K = np.diag([1.0, 1.0, 1.0 / f])
    P = K @ np.hstack([R, t])
    world = [np.random.uniform(-0.3, 0.3, 3) + np.array([0.0, 0.0, 2.0]) for _ in range(4)]
    features = []
    for X in world:
        x = P @ np.append(X, 1.0)
        features.append(np.array([x[0] / x[2], x[1] / x[2]]))

    ok, Rs, ts, kappa, fs = pt.sfm.FourPointsPoseFocalLengthRadialDistortion(
        features, world, meta
    )
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/four_point_focal_length_radial_distortion.h"

    std::vector<Eigen::Vector2d> features(4);
    std::vector<Eigen::Vector3d> world(4);
    const double max_f = 800.0, min_f = 200.0;
    const double max_k = 0.5, min_k = -0.5;
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    std::vector<double> radial, focal;
    const bool ok = theia::FourPointsPoseFocalLengthRadialDistortion(
        features, world, max_f, min_f, max_k, min_k,
        &rotations, &translations, &radial, &focal);
    ```

### Two Point Absolute Pose with a Partially Known Rotation {#section-two_point_absolute_partial_rotation}

**Signature (C++):** [`two_point_pose_partial_rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/two_point_pose_partial_rotation.h)

```cpp
int TwoPointPosePartialRotation(
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& model_point_1,
    const Eigen::Vector3d& model_point_2,
    const Eigen::Vector3d& image_ray_1,
    const Eigen::Vector3d& image_ray_2,
    Eigen::Quaterniond soln_rotations[2],
    Eigen::Vector3d soln_translations[2]);
```

Solves for the limited pose of a camera from two 3D points to image ray correspondences. The pose is limited in that while it solves for the three translation components, it only solves for a single rotation around a passed axis.

This is intended for use with camera phones that have accelerometers, so that the 'up' vector is known, meaning the other two rotations are known. The effect of the other rotations should be removed before using this function.

This implementation is intended to form the core of a RANSAC routine, and as such has an optimized interface for this use case.

Computes the limited pose between the 3D model points and the (unit-norm) image rays. Places the rotation and translation solutions in soln_rotations and soln_translations. There are at most 2 solutions, and the number of solutions is returned.

The rotations and translation are defined such that model points are transformed according to $image_point = Q * model_point + t$

This function computes the rotation and translation such that the model points, after transformation, lie along the corresponding image_rays. The axis referred to is the axis of rotation between the camera coordinate system and world (3D point) coordinate system. For most users, this axis will be (0, 1, 0) i.e., the up direction. This requires that the input image rays have been rotated such that the up direction of the camera coordinate system is indeed equal to (0, 1, 0).

When using this algorithm please cite the paper [SweeneyISMAR2015](bibliography.md#SweeneyISMAR2015).

**pyTheia:** `num_solutions, quats, translations = pytheia.sfm.TwoPointPosePartialRotation(axis, model1, model2, ray1, ray2)` with **unit** image rays.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    axis = np.array([0.0, 1.0, 0.0])
    m1 = np.array([1.0, 0.0, 3.0])
    m2 = np.array([-0.5, 0.8, 2.5])
    r1 = m1 / np.linalg.norm(m1)
    r2 = m2 / np.linalg.norm(m2)
    n, quats, ts = pt.sfm.TwoPointPosePartialRotation(axis, m1, m2, r1, r2)
    ```

=== "C++"

    ```cpp
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/two_point_pose_partial_rotation.h"

    const Eigen::Vector3d axis(0, 1, 0);
    const Eigen::Vector3d model1(1, 0, 3), model2(-0.5, 0.8, 2.5);
    const Eigen::Vector3d ray1 = model1.normalized(), ray2 = model2.normalized();
    Eigen::Quaterniond soln_rot[2];
    Eigen::Vector3d soln_t[2];
    const int n = theia::TwoPointPosePartialRotation(
        axis, model1, model2, ray1, ray2, soln_rot, soln_t);
    ```

### Planar Uncalibrated Orthographic Pose {#section-orthographic_planar_pose}

**Signature (C++):** [`orthographic_four_point.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/orthographic_four_point.h)

```cpp
bool PlanarUncalibratedOrthographicPose(
    const std::vector<theia::FeatureCorrespondence2D3D>& correspondences,
    const Eigen::Vector2d& principal_point,
    std::vector<Eigen::Matrix3d>* solution_rotations,
    std::vector<Eigen::Vector3d>* solution_translations,
    double* magnification);

bool PlanarUncalibratedOrthographicPose(
    const std::vector<Eigen::Vector2d>& feature_point,
    const std::vector<Eigen::Vector3d>& world_point,
    const Eigen::Vector2d& principal_point,
    std::vector<Eigen::Matrix3d>* solution_rotations,
    std::vector<Eigen::Vector3d>* solution_translations,
    double* magnification);
```

**pyTheia:** `success, rotations, translations, magnification = pytheia.sfm.PlanarUncalibratedOrthographicPose(correspondences, principal_point)` where `correspondences` is a list of `FeatureCorrespondence2D3D` (`feature`, `world_point`).

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    principal = np.array([500.0, 500.0])
    correspondences = []
    for _ in range(4):
        c = pt.sfm.FeatureCorrespondence2D3D()
        c.world_point = np.array(
            [np.random.uniform(-0.02, 0.02), np.random.uniform(-0.02, 0.02), 0.0]
        )
        c.feature = np.array([200.0 + np.random.randn(), 300.0 + np.random.randn()])
        correspondences.append(c)

    ok, Rs, ts, mag = pt.sfm.PlanarUncalibratedOrthographicPose(
        correspondences, principal
    )
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/orthographic_four_point.h"
    #include "theia/sfm/estimators/feature_correspondence_2d_3d.h"

    std::vector<theia::FeatureCorrespondence2D3D> corres;
    const Eigen::Vector2d principal(500, 500);
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    double magnification = 0;
    const bool ok = theia::PlanarUncalibratedOrthographicPose(
        corres, principal, &rotations, &translations, &magnification);
    ```

### Position from two rays {#section-position_from_two_rays}

**Signature (C++):** [`position_from_two_rays.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/position_from_two_rays.h)

```cpp
bool PositionFromTwoRays(const Eigen::Vector2d& rotated_feature1,
                         const Eigen::Vector3d& point1,
                         const Eigen::Vector2d& rotated_feature2,
                         const Eigen::Vector3d& point2,
                         Eigen::Vector3d* position);
```

Intersects two bearing–point constraints when the **world-to-camera rotation** is known (see the header for how `rotated_feature*` relate to \(R^\top [u,v,1]^\top\)).

**pyTheia:** `success, position = pytheia.sfm.PositionFromTwoRays(f1, p1, f2, p2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    # Toy values; real rotated_feature_* come from R^T * K^{-1} * [u, v, 1].
    f1 = np.array([0.0, 0.0])
    p1 = np.array([0.0, 0.0, 5.0])
    f2 = np.array([0.1, 0.0])
    p2 = np.array([1.0, 0.0, 5.0])
    ok, c = pt.sfm.PositionFromTwoRays(f1, p1, f2, p2)
    ```

=== "C++"

    ```cpp
    #include <Eigen/Core>
    #include "theia/sfm/pose/position_from_two_rays.h"

    const Eigen::Vector2d rotated_feature1(0, 0);
    const Eigen::Vector3d point1(0, 0, 5);
    const Eigen::Vector2d rotated_feature2(0.1, 0);
    const Eigen::Vector3d point2(1, 0, 5);
    Eigen::Vector3d position;
    const bool ok = theia::PositionFromTwoRays(
        rotated_feature1, point1, rotated_feature2, point2, &position);
    ```


## Relative pose and two-view geometry {#relative-pose-estimation}

The routines below take **2D–2D correspondences** (and optional structure such as a known gravity axis or generalized-camera rays). They estimate **epipolar geometry**, **homographies**, or **relative** transforms between two views or rigs.

### Five Point Relative Pose {#section-five_point_essential_matrix}

**Signature (C++):** [`five_point_relative_pose.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/five_point_relative_pose.h)

```cpp
bool FivePointRelativePose(
    const std::vector<Eigen::Vector2d>& image1_points,
    const std::vector<Eigen::Vector2d>& image2_points,
    std::vector<Eigen::Matrix3d>* essential_matrices);
```

Computes candidate **essential matrices** \(E\) from five or more normalized correspondences such that \(y^\top E x = 0\) for points \(x\) in image 1 and \(y\) in image 2, with \(E = [t]_\times R\). Algorithm from "Recent Developments on Direct Relative Orientation" [Stewenius5pt](bibliography.md#Stewenius5pt). Recover rotation and translation with `DecomposeEssentialMatrix` (and disambiguate with `GetBestPoseFromEssentialMatrix` when you have extra matches).

`image1_points` / `image2_points`: Normalized image coordinates (same count, at least five).

**pyTheia:** `success, essential_matrices = pytheia.sfm.FivePointRelativePose(pts1, pts2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    np.random.seed(0)
    R = np.array([[0.996, -0.087, 0.0], [0.087, 0.996, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([0.3, 0.1, 0.05])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R

    pts1, pts2 = [], []
    for _ in range(8):
        u, v = np.random.randn(2)
        xh = np.array([u, v, 1.0])
        l = E @ xh  # epipolar line in image 2: l^T y_h = 0
        if abs(l[1]) > 1e-8:
            y_u = np.random.randn()
            y_v = -(l[0] * y_u + l[2]) / l[1]
        else:
            y_v = np.random.randn()
            y_u = -(l[1] * y_v + l[2]) / (l[0] + 1e-10)
        pts1.append(np.array([u, v]))
        pts2.append(np.array([y_u, y_v]))

    ok, Es = pt.sfm.FivePointRelativePose(pts1, pts2)
    assert ok and len(Es) >= 1
    R1, R2, t12 = pt.sfm.DecomposeEssentialMatrix(Es[0])
    # Use GetBestPoseFromEssentialMatrix with normalized FeatureCorrespondence
    # from pytheia.matching when you need a single (R, t).
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/five_point_relative_pose.h"
    #include "theia/sfm/pose/essential_matrix_utils.h"

    std::vector<Eigen::Vector2d> image1, image2;  // at least 5 pairs
    std::vector<Eigen::Matrix3d> essential_matrices;
    const bool ok =
        theia::FivePointRelativePose(image1, image2, &essential_matrices);
    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t;
    theia::DecomposeEssentialMatrix(essential_matrices[0], &R1, &R2, &t);
    ```

### Four Point Algorithm for Homography {#section-four_point_homography}

**Signature (C++):** [`four_point_homography.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/four_point_homography.h)

```cpp
bool FourPointHomography(const std::vector<Eigen::Vector2d>& image_1_points,
                         const std::vector<Eigen::Vector2d>& image_2_points,
                         Eigen::Matrix3d* homography);
```

Computes the 2D [homography](http://en.wikipedia.org/wiki/Homography_(computer_vision)) mapping points in image 1 to image 2 such that: $x' = Hx$ where $x$ is a point in image 1 and $x'$ is a point in image 2. The algorithm implemented is the DLT algorithm based on algorithm 4.2 in [HartleyZisserman](bibliography.md#HartleyZisserman).

`image_1_points`: Image points from image 1. At least 4 points must be passed in.

`image_2_points`: Image points from image 2. At least 4 points must be passed in.

`homography`: The computed 3x3 homography matrix.

**pyTheia:** `success, H = pytheia.sfm.FourPointHomography(pts1, pts2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    H_true = np.array([[1.02, 0.01, -5.0], [0.02, 0.98, 3.0], [0.0001, 0.0002, 1.0]])
    pts1, pts2 = [], []
    for _ in range(6):
        x = np.array([np.random.uniform(-200, 200), np.random.uniform(-200, 200), 1.0])
        xp = H_true @ x
        pts1.append(x[:2] / x[2])
        pts2.append(xp[:2] / xp[2])

    ok, H = pt.sfm.FourPointHomography(pts1, pts2)
    assert ok
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/four_point_homography.h"

    std::vector<Eigen::Vector2d> image_1, image_2;  // >= 4 pairs
    Eigen::Matrix3d H;
    const bool ok = theia::FourPointHomography(image_1, image_2, &H);
    ```

### Normalized eight-point fundamental matrix {#section-eight_point}

**Signature (C++):** [`eight_point_fundamental_matrix.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/eight_point_fundamental_matrix.h)

```cpp
bool NormalizedEightPointFundamentalMatrix(
    const std::vector<Eigen::Vector2d>& image_1_points,
    const std::vector<Eigen::Vector2d>& image_2_points,
    Eigen::Matrix3d* fundamental_matrix);
```

Computes the [fundamental matrix](http://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)) \(F\) with \(x_2^\top F x_1 = 0\) for normalized correspondences (Alg. 11.1 in [HartleyZisserman](bibliography.md#HartleyZisserman)).

**pyTheia:** `success, F = pytheia.sfm.NormalizedEightPointFundamentalMatrix(pts1, pts2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    # For normalized cameras, F matches the essential matrix E = [t]_x R.
    np.random.seed(1)
    R = np.array([[0.996, -0.087, 0.0], [0.087, 0.996, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([0.3, 0.1, 0.05])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R
    pts1, pts2 = [], []
    for _ in range(10):
        u, v = np.random.randn(2)
        xh = np.array([u, v, 1.0])
        l = E @ xh
        if abs(l[1]) > 1e-8:
            y_u = np.random.randn()
            y_v = -(l[0] * y_u + l[2]) / l[1]
        else:
            y_v = np.random.randn()
            y_u = -(l[1] * y_v + l[2]) / (l[0] + 1e-10)
        pts1.append(np.array([u, v]))
        pts2.append(np.array([y_u, y_v]))

    ok, F = pt.sfm.NormalizedEightPointFundamentalMatrix(pts1, pts2)
    assert ok
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/eight_point_fundamental_matrix.h"

    std::vector<Eigen::Vector2d> image_1, image_2;  // >= 8 pairs, normalized
    Eigen::Matrix3d F;
    const bool ok =
        theia::NormalizedEightPointFundamentalMatrix(image_1, image_2, &F);
    ```

### Seven-point fundamental matrix {#section-seven_point_fundamental}

**Signature (C++):** [`seven_point_fundamental_matrix.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/seven_point_fundamental_matrix.h)

```cpp
bool SevenPointFundamentalMatrix(
    const std::vector<Eigen::Vector2d>& image1_points,
    const std::vector<Eigen::Vector2d>& image2_points,
    std::vector<Eigen::Matrix3d>* fundamental_matrices);
```

Returns up to three candidate fundamental matrices from **exactly seven** correspondences (complex solutions discarded).

**pyTheia:** `success, Fs = pytheia.sfm.SevenPointFundamentalMatrix(pts1, pts2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    np.random.seed(2)
    R = np.eye(3)
    t = np.array([0.2, 0.0, 0.0])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R
    pts1, pts2 = [], []
    for _ in range(7):
        u, v = np.random.randn(2)
        xh = np.array([u, v, 1.0])
        l = E @ xh
        if abs(l[1]) > 1e-8:
            y_u = np.random.randn()
            y_v = -(l[0] * y_u + l[2]) / l[1]
        else:
            y_v = np.random.randn()
            y_u = -(l[1] * y_v + l[2]) / (l[0] + 1e-10)
        pts1.append(np.array([u, v]))
        pts2.append(np.array([y_u, y_v]))

    ok, Fs = pt.sfm.SevenPointFundamentalMatrix(pts1, pts2)
    assert ok and len(Fs) >= 1
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include "theia/sfm/pose/seven_point_fundamental_matrix.h"

    std::vector<Eigen::Vector2d> image1, image2;  // exactly 7 pairs
    std::vector<Eigen::Matrix3d> F_candidates;
    const bool ok =
        theia::SevenPointFundamentalMatrix(image1, image2, &F_candidates);
    ```

### Three Point Relative Pose with a Partially Known Rotation {#section-three_point_relative_partial_rotation}

**Signature (C++):** [`three_point_relative_pose_partial_rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/three_point_relative_pose_partial_rotation.h)

```cpp
void ThreePointRelativePosePartialRotation(
    const Eigen::Vector3d& rotation_axis,
    const Eigen::Vector3d image_1_rays[3],
    const Eigen::Vector3d image_2_rays[3],
    std::vector<Eigen::Quaterniond>* soln_rotations,
    std::vector<Eigen::Vector3d>* soln_translations);
```

Computes the relative pose between two cameras using 3 correspondences and a known vertical direction as a Quadratic Eigenvalue Problem [SweeneyQEP](bibliography.md#SweeneyQEP). Up to 6 solutions are returned such that $x_2 = R * x_1 + t$ for rays $x_1$ in image 1 and rays $x_2$ in image 2. The `axis` that is passed in as a known axis of rotation (when considering rotations as an angle axis). This is equivalent to aligning the two cameras to a common direction such as the vertical direction, which can be done using IMU data.

**pyTheia:** `quats, translations = pytheia.sfm.ThreePointRelativePosePartialRotation(axis, rays1, rays2)` with three **unit-length** ray directions per view.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    axis = np.array([0.0, 1.0, 0.0])
    rays1 = [np.array([0.0, 0.0, 1.0]), np.array([0.5, 0.0, 1.0]), np.array([0.0, 0.3, 1.0])]
    rays1 = [r / np.linalg.norm(r) for r in rays1]
    rays2 = [r.copy() for r in rays1]

    quats, ts = pt.sfm.ThreePointRelativePosePartialRotation(axis, rays1, rays2)
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/three_point_relative_pose_partial_rotation.h"

    const Eigen::Vector3d axis(0, 1, 0);
    const Eigen::Vector3d rays1[3] = {
        Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.5, 0, 1), Eigen::Vector3d(0, 0.3, 1)};
    const Eigen::Vector3d rays2[3] = {rays1[0], rays1[1], rays1[2]};
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    theia::ThreePointRelativePosePartialRotation(
        axis, rays1, rays2, &rotations, &translations);
    ```

### Four Point Relative Pose with a Partially Known Rotation {#section-four_point_relative_partial_rotation}

**Signature (C++):** [`four_point_relative_pose_partial_rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/four_point_relative_pose_partial_rotation.h)

```cpp
void FourPointRelativePosePartialRotation(
    const Eigen::Vector3d& rotation_axis,
    const Eigen::Vector3d image_one_ray_directions[4],
    const Eigen::Vector3d image_one_ray_origins[4],
    const Eigen::Vector3d image_two_ray_directions[4],
    const Eigen::Vector3d image_two_ray_origins[4],
    std::vector<Eigen::Quaterniond>* soln_rotations,
    std::vector<Eigen::Vector3d>* soln_translations);
```

Computes the relative pose between two generalized cameras using 4 correspondences and a known vertical direction as a Quadratic Eigenvalue Problem [SweeneyQEP](bibliography.md#SweeneyQEP). A generalized camera is a camera setup with multiple cameras such that the cameras do not have the same center of projection (e.g., a multi-camera rig mounted on a car). Up to 8 solutions are returned such that $x_2 = R * x_1 + t$ for rays $x_1$ in image 1 and rays $x_2$ in image 2. The axis that is passed in as a known axis of rotation (when considering rotations as an angle axis). This is equivalent to aligning the two cameras to a common direction such as the vertical direction, which can be done using IMU data.

**pyTheia:** `quats, translations = pytheia.sfm.FourPointRelativePosePartialRotation(axis, dirs1, origins1, dirs2, origins2)` — four ray **directions** and **origins** per generalized view.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    axis = np.array([0.0, 1.0, 0.0])
    o = np.zeros(3)
    dirs1 = [np.array([0.0, 0.0, 1.0]) for _ in range(4)]
    origins1 = [o.copy() for _ in range(4)]
    dirs2 = [d / np.linalg.norm(d) for d in dirs1]
    origins2 = [np.array([0.1, 0.0, 0.0]) for _ in range(4)]

    quats, ts = pt.sfm.FourPointRelativePosePartialRotation(
        axis, dirs1, origins1, dirs2, origins2
    )
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/four_point_relative_pose_partial_rotation.h"

    const Eigen::Vector3d axis(0, 1, 0);
    const Eigen::Vector3d d1[4] = {
        Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)};
    const Eigen::Vector3d o1[4] = {
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    const Eigen::Vector3d d2[4] = {d1[0], d1[1], d1[2], d1[3]};
    const Eigen::Vector3d o2[4] = {
        Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.1, 0, 0),
        Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.1, 0, 0)};
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    theia::FourPointRelativePosePartialRotation(
        axis, d1, o1, d2, o2, &rotations, &translations);
    ```

### Relative pose from two points with known rotation {#section-relative_pose_two_points_known_rotation}

**Signature (C++):** [`relative_pose_from_two_points_with_known_rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/relative_pose_from_two_points_with_known_rotation.h)

```cpp
bool RelativePoseFromTwoPointsWithKnownRotation(
    const Eigen::Vector2d rotated_features1[2],
    const Eigen::Vector2d rotated_features2[2],
    Eigen::Vector3d* relative_position2);
```

Expects **two** preprocessed 2D coordinates per camera (see the header for rotated / homogeneous normalization).

**pyTheia:** `success, t = pytheia.sfm.RelativePoseFromTwoPointsWithKnownRotation(f1, f2)` with `f1`, `f2` each a list of two `Vector2d`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    f1 = [np.array([0.0, 0.0]), np.array([0.2, 0.0])]
    f2 = [np.array([0.05, 0.0]), np.array([0.25, 0.0])]
    ok, t2 = pt.sfm.RelativePoseFromTwoPointsWithKnownRotation(f1, f2)
    ```

=== "C++"

    ```cpp
    #include <Eigen/Core>
    #include "theia/sfm/pose/relative_pose_from_two_points_with_known_rotation.h"

    Eigen::Vector2d rf1[2] = {Eigen::Vector2d(0, 0), Eigen::Vector2d(0.2, 0)};
    Eigen::Vector2d rf2[2] = {Eigen::Vector2d(0.05, 0), Eigen::Vector2d(0.25, 0)};
    Eigen::Vector3d relative_position2;
    const bool ok = theia::RelativePoseFromTwoPointsWithKnownRotation(
        rf1, rf2, &relative_position2);
    ```

### Similarity transform with partial rotation {#section-sim_transform_partial_rotation}

**Signature (C++):** [`sim_transform_partial_rotation.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/pose/sim_transform_partial_rotation.h)

```cpp
void SimTransformPartialRotation(
    const Eigen::Vector3d& rotation_axis,
    const Eigen::Vector3d image_one_ray_directions[5],
    const Eigen::Vector3d image_one_ray_origins[5],
    const Eigen::Vector3d image_two_ray_directions[5],
    const Eigen::Vector3d image_two_ray_origins[5],
    std::vector<Eigen::Quaterniond>* soln_rotations,
    std::vector<Eigen::Vector3d>* soln_translations,
    std::vector<double>* soln_scales);
```

Similarity between two generalized views (**five** ray pairs per side) with rotation constrained about a known axis.

**pyTheia:** `quats, translations, scales = pytheia.sfm.SimTransformPartialRotation(axis, dirs1, origins1, dirs2, origins2)`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    axis = np.array([0.0, 1.0, 0.0])
    z = np.array([0.0, 0.0, 1.0])
    dirs1 = [z.copy() for _ in range(5)]
    origins1 = [np.zeros(3) for _ in range(5)]
    dirs2 = [z.copy() for _ in range(5)]
    origins2 = [np.array([0.05, 0.0, 0.0]) for _ in range(5)]

    quats, ts, s = pt.sfm.SimTransformPartialRotation(
        axis, dirs1, origins1, dirs2, origins2
    )
    ```

=== "C++"

    ```cpp
    #include <vector>
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include "theia/sfm/pose/sim_transform_partial_rotation.h"

    const Eigen::Vector3d axis(0, 1, 0);
    Eigen::Vector3d d1[5], o1[5], d2[5], o2[5];
    for (int i = 0; i < 5; ++i) {
      d1[i] = Eigen::Vector3d(0, 0, 1);
      o1[i].setZero();
      d2[i] = Eigen::Vector3d(0, 0, 1);
      o2[i] = Eigen::Vector3d(0.05, 0, 0);
    }
    std::vector<Eigen::Quaterniond> rotations;
    std::vector<Eigen::Vector3d> translations;
    std::vector<double> scales;
    theia::SimTransformPartialRotation(
        axis, d1, o1, d2, o2, &rotations, &translations, &scales);
    ```


## Essential and fundamental matrix utilities {#section-essential_fundamental_utils}

These helpers on `pytheia.sfm` **decompose** or **convert** between two-view matrices—typically after a minimal solver returns an essential or fundamental matrix: `DecomposeEssentialMatrix`, `GetBestPoseFromEssentialMatrix`, `EssentialMatrixFromTwoProjectionMatrices`, `FundamentalMatrixFromProjectionMatrices`, `EssentialMatrixFromFundamentalMatrix`, `ComposeFundamentalMatrix`, `FocalLengthsFromFundamentalMatrix`, `SharedFocalLengthsFromFundamentalMatrix`, `ProjectionMatricesFromFundamentalMatrix`.

=== "Python"

    ```python
    import numpy as np
    import pytheia as pt

    R = np.array([[0.996, -0.087, 0.0], [0.087, 0.996, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([0.3, 0.1, 0.05])
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R
    R1, R2, t12 = pt.sfm.DecomposeEssentialMatrix(E)

    F = np.eye(3)
    P1, P2 = pt.sfm.ProjectionMatricesFromFundamentalMatrix(F)
    F2 = pt.sfm.FundamentalMatrixFromProjectionMatrices(P1, P2)
    ```

=== "C++"

    ```cpp
    #include <Eigen/Core>
    #include "theia/sfm/pose/essential_matrix_utils.h"

    Eigen::Matrix3d E;  // set to a valid essential matrix
    Eigen::Matrix3d R1, R2;
    Eigen::Vector3d t;
    theia::DecomposeEssentialMatrix(E, &R1, &R2, &t);
    ```
