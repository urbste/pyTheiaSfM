# Camera models {#chapter-cameras}

At the base of structure-from-motion is the imaging device. Theia represents each view with a `Camera` that holds **extrinsics** (pose) and a polymorphic **intrinsics** object (`CameraIntrinsicsModel`) chosen at runtime. Any model is usable in the SfM stack as long as forward projection and ray back-projection are implemented consistently.

## Coordinate conventions

Theia uses three right-handed frames:

- **World:** global 3D coordinates for points and camera centers.
- **Camera:** origin at the camera center; **+Z is the optical axis** (a point straight ahead projects near the principal point). In this frame the optical axis direction is \((0,0,1)^\).
- **Image:** pixel coordinates; origin at the **top-left**, **+X** right, **+Y** down.

For projection, a homogeneous world point is first mapped to camera coordinates using the stored pose, then intrinsics map the camera ray to pixels.

## Implemented intrinsics models (overview)

These are the values of `CameraIntrinsicsModelType` in [`camera_intrinsics_model_type.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/sfm/camera/camera_intrinsics_model_type.h) and their C++ classes under `src/theia/sfm/camera/`.

| Enum / string | C++ class | `#` intrinsics | Skew in \(K\)? | In pyTheia `pt.sfm` |
|---------------|-----------|----------------|----------------|----------------------|
| `PINHOLE` | `PinholeCameraModel` | 7 | yes | yes |
| `PINHOLE_RADIAL_TANGENTIAL` | `PinholeRadialTangentialCameraModel` | 10 | yes | yes |
| `FISHEYE` | `FisheyeCameraModel` | 9 | yes | yes |
| `FOV` | `FOVCameraModel` | 5 | no (diagonal focal lengths only) | yes |
| `DIVISION_UNDISTORTION` | `DivisionUndistortionCameraModel` | 5 | no | yes |
| `DOUBLE_SPHERE` | `DoubleSphereCameraModel` | 7 | yes | **C++ only** (not pybind11 yet) |
| `EXTENDED_UNIFIED` | `ExtendedUnifiedCameraModel` | 7 | yes | **C++ only** (not pybind11 yet) |
| `ORTHOGRAPHIC` | `OrthographicCameraModel` | 7 | yes | yes |

Strings accepted by I/O and `StringToCameraIntrinsicsModelType` match the enum names above (e.g. `"PINHOLE_RADIAL_TANGENTIAL"`, `"DOUBLE_SPHERE"`).

Parameter **order** below matches the internal `InternalParametersIndex` enums in each header (also the order stored in `CameraIntrinsicsModel::parameters_`).

---

## `Camera`

`Camera` stores pose as a rotation (angle-axis internally, with helpers for rotation matrices) and **position** (not translation). Intrinsics are held as `std::shared_ptr<CameraIntrinsicsModel>` and can be switched with `SetCameraIntrinsicsModelType` / `CameraIntrinsics`.

Typical use: project a homogeneous 3D point to a pixel and read depth:

```cpp
Camera camera;
camera.SetOrientationFromRotationMatrix(R);
camera.SetPosition(C);
Eigen::Vector4d X;  // homogeneous world point
Eigen::Vector2d pixel;
const double depth = camera.ProjectPoint(X, &pixel);
```

`GetProjectionMatrix` / `GetCalibrationMatrix` follow the **pinhole** \(K [R \mid -RC]\) style where applicable; they **do not encode non-pinhole distortion** in the \(3 \times 4\) matrix—distortion is always applied inside the intrinsics model.

Useful APIs include `SetFromCameraIntrinsicsPriors`, `InitializeFromProjectionMatrix` (RQ decomposition; distortion must be set separately), `PixelToUnitDepthRay`, and `PixelToNormalizedCoordinates`.

---

## `CameraIntrinsicsModel`

Abstract base class. Implementations must provide:

- `Type()`, `NumParameters()`, `PrintIntrinsics()`
- `SetFromCameraIntrinsicsPriors` / `CameraIntrinsicsPriorFromIntrinsics`
- `GetCalibrationMatrix` (linear \(3 \times 3\) part, same structural form as pinhole \(K\) where applicable)
- `GetSubsetFromOptimizeIntrinsicsType` (bundle adjustment parameter masking)
- Static `CameraToPixelCoordinates` / `PixelToCameraCoordinates` / `DistortPoint` / `UndistortPoint` on the parameter vector

Unless noted, models first reduce a camera-space point to a **direction** (often normalized plane \(z=1\)) and then apply model-specific distortion before multiplying by \(K\).

---

## `PinholeCameraModel` (`PINHOLE`)

**7** parameters: focal length \(f\), aspect ratio \(a\), skew \(s\), principal point \((p_x,p_y)\), two radial coefficients \(k_1, k_2\).

\[
K = \begin{bmatrix}
f & s & p_x \\
0 & f \cdot a & p_y \\
0 & 0 & 1
\end{bmatrix}
\]

After perspective division to \((x/z,\, y/z)\), radial distortion uses

\[
d = 1 + k_1 r^2 + k_2 r^4,\quad r^2 = (x/z)^2 + (y/z)^2,
\]

then \((x',y') = d \cdot (x/z,\, y/z)\) and \((u,v)^\top = K (x',y',1)^\top\).

---

## `PinholeRadialTangentialCameraModel` (`PINHOLE_RADIAL_TANGENTIAL`)

**10** parameters: same \(f, a, s, p_x, p_y\) as pinhole, plus **three** radial terms \(k_1, k_2, k_3\) and **two** tangential terms \(t_1, t_2\) (Brown–Conrady style, consistent with OpenCV ordering of tangential terms).

With \(r^2 = x^2 + y^2\) on the normalized plane,

\[
d = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6,
\]

\[
\delta_x = 2 t_1 x y + t_2 (r^2 + 2 x^2), \quad
\delta_y = t_1 (r^2 + 2 y^2) + 2 t_2 x y,
\]

\[
x_d = d\, x + \delta_x,\quad y_d = d\, y + \delta_y,
\]

then multiply by \(K\) as for the pinhole model.

---

## `FisheyeCameraModel` (`FISHEYE`)

**9** parameters: \(f, a, s, p_x, p_y\) plus four angular distortion coefficients \(k_{1..4}\).

Based on OpenCV’s fisheye formulation (see [OpenCV fisheye calibration](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)). The implementation uses the **full 3D** camera point: it forms

\[
\theta = \operatorname{atan2}\bigl(\sqrt{x^2+y^2},\, |z|\bigr),
\]

\[
\theta_d = \theta \left(1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8\right),
\]

then maps back to a normalized-plane direction (with a sign flip for points behind the camera, \(z<0\)) before applying \(K\). This stays more stable for very wide fields of view than dividing by \(z\) first.

---

## `FOVCameraModel` (`FOV`)

**5** parameters: \(f\), aspect ratio \(a\), \(p_x\), \(p_y\), and a single **field-of-view** parameter \(\omega\) stored as `RADIAL_DISTORTION_1` (default initialization uses \(\omega \approx 0.75\); all-zero is a poor starting value for optimization).

There is **no skew**; pixels are

\[
u = f\, x_d + p_x,\quad v = (f a)\, y_d + p_y
\]

after FOV distortion on the normalized plane. The radial scale \(r_d\) follows Devernay–Faugeras ([Devernay](bibliography.md#Devernay)); for \(\omega\) and \(r_u = \sqrt{x^2+y^2}\) away from singular cases,

\[
r_d = \frac{\arctan\bigl(2 r_u \tan(\omega/2)\bigr)}{r_u\, \omega},
\]

and \((x_d, y_d) = r_d (x,y)\). Small-\(\omega\) and near-center branches use Taylor expansions (see `fov_camera_model.h`, aligned with COLMAP’s treatment).

---

## `DivisionUndistortionCameraModel` (`DIVISION_UNDISTORTION`)

**5** parameters: \(f\), \(a\), \(p_x\), \(p_y\), and one division coefficient \(k\) (`RADIAL_DISTORTION_1`).

Fitzgibbon’s division model ([CVPR 2001](https://www.robots.ox.ac.uk/~vgg/publications/papers/fitzgibbon01b.pdf)): after scaling normalized coordinates by focal length (and aspect) but **before** adding the principal point, distorted coordinates satisfy

\[
x_u = \frac{x_d}{1 + k\, r_d^2}, \quad y_u = \frac{y_d}{1 + k\, r_d^2},
\quad r_d^2 = x_d^2 + y_d^2.
\]

Forward projection inverts this relationship (see closed form in `division_undistortion_camera_model.h`). The class also exposes `CameraToUndistortedPixelCoordinates` and `DistortedPixelToUndistortedPixel` for special cases.

---

## `DoubleSphereCameraModel` (`DOUBLE_SPHERE`)

**7** parameters: \(f, a, s, p_x, p_y\), then **`XI`** (\(\xi\), typically in \((-1,1)\)) and **`ALPHA`** (\(\alpha\), in \([0,1]\)).

Implements the double-sphere model from [Usenko et al., *The Double Sphere Camera Model*](https://arxiv.org/pdf/1807.08957.pdf). Projection maps a 3D point through two sphere centers separated by \(\alpha\), with \(\xi\) controlling the second sphere; see `DistortPoint` in `double_sphere_camera_model.h` for the exact steps and validity checks (projection can fail for points outside the valid region).

---

## `ExtendedUnifiedCameraModel` (`EXTENDED_UNIFIED`)

**7** parameters: \(f, a, s, p_x, p_y\), then **`ALPHA`** (\(\alpha \in [0,1]\)) and **`BETA`** (\(\beta > 0\)).

Extended unified / EUCM variant from the same paper as double sphere ([arXiv:1807.08957](https://arxiv.org/pdf/1807.08957.pdf)). With \(\rho = \sqrt{\beta (x^2+y^2) + z^2}\),

\[
\text{norm} = \alpha \rho + (1-\alpha)\, z,
\quad
x' = x/\text{norm},\; y' = y/\text{norm}
\]

(with guards for small norm and hemisphere checks when \(\alpha > 1/2\)). See `extended_unified_camera_model.h` for full detail.

---

## `OrthographicCameraModel` (`ORTHOGRAPHIC`)

**7** parameters: stored under the same index names as pinhole—`FOCAL_LENGTH` acts as **magnification** \(m\), `ASPECT_RATIO` relates effective focal lengths in \(x\) and \(y\), `SKEW` is supported, `PRINCIPAL_POINT_*` are the principal point, and two radial factors \(k_1,k_2\) apply in the **plane** after scaling \((x_c,y_c)\) from camera coordinates by magnification (depth does not affect \((x,y)\) in the orthographic ideal model; the implementation still produces a direction with \(z=1\) after back-projection for consistency with other models).

Distortion on the magnified plane matches the pinhole-style polynomial \(d = 1 + k_1 r^2 + k_2 r^4\) before applying the affine pixel mapping (see `orthographic_camera_model.h`).

---

## Adding a new camera model

1. Subclass `CameraIntrinsicsModel` and implement all virtual and static projection methods.
2. Add a value to `CameraIntrinsicsModelType` and handle it in `StringToCameraIntrinsicsModelType` / `CameraIntrinsicsModelTypeToString`.
3. Register the type in `CameraIntrinsicsModel::Create` and the `CAMERA_MODEL_SWITCH_STATEMENT` macro in `camera_intrinsics_model.cc`.
4. Add bundle-adjustment support in `create_reprojection_error_cost_function.h` (and any other switch sites).
5. Add unit tests and, if the model should be usable from Python, pybind11 bindings in `src/pytheia/sfm/sfm.cc` mirroring the existing camera classes.
