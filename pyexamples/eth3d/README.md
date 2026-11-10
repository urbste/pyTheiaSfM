# ETH3D examples

Short, runnable pyTheia examples that use the ETH3D undistorted DSLR scenes as
ground truth. Each script prints one or more numbers that are compared against
the ETH3D ground truth (GT). Images are never decoded: everything works on GT
sparse points and GT 2D observations, plus synthetic noise and outliers.

```bash
export ETH3D_ROOT=/media/steffen/Data2/sfm_bench/data/eth3d/extracted  # default
python pyexamples/eth3d/pnp_solvers.py
```

Requires `pytheia` (import as `pt`), `numpy`, and `scipy` (only for
`Rotation` in `eth3d_utils.py`).

## Data

`ETH3D_ROOT/<scene>/dslr_calibration_undistorted/{cameras,images,points3D}.txt`
is a COLMAP text model: `images.txt` stores world-to-camera poses, cameras are
PINHOLE (already undistorted). Thirteen scenes are present (`courtyard`,
`delivery_area`, `electro`, `facade`, `kicker`, `meadow`, `office`, `pipes`,
`playground`, `relief`, `relief_2`, `terrace`, `terrains`).

**No depth maps are used.** Wherever a depth prior is needed, it is the
camera-frame depth of the GT sparse point projected into that view (the GT
point's z in the camera frame). Those priors inherit the GT reconstruction's
accuracy, and they are exact unless noise is added explicitly
(`relative_pose.py --depth_noise`, `bundle_adjustment_priors.py` uses 2 %).

**No IMU or gravity data exists.** Gravity-based examples assume the world -z
axis is gravity (pyTheia's BA default `gravity_world_direction = (0,0,-1)`).
ETH3D does not guarantee this, so treat gravity results as self-consistent with
the GT, not as a physical gravity test.

## Files

| Script | Shows | Output compared against GT |
|---|---|---|
| `eth3d_utils.py` | COLMAP loader, projection, normalization, GT tracks, Umeyama, pytheia camera/prior builders | - |
| `pnp_solvers.py` | `PoseFromThreePoints` (P3P), `DlsPnp`, `MLPnP`, `UPMLPnP`, RANSAC `EstimateCalibratedAbsolutePose` with `PnPType` KNEIP/DLS/SQPnP/MLPnP | rotation (deg), camera centre (m) |
| `radial_pnp.py` | `EstimateRadialDistUncalibratedAbsolutePose` (focal + division-model k + pose) | focal, k, pose |
| `relative_pose.py` | `EstimateRelativePose` (5-pt RANSAC) and `EstimateMonoDepthRelativePose` (3-pt with depth priors) | rotation (deg), translation direction (deg) |
| `relative_pose_partial_rotation.py` | `ThreePointRelativePosePartialRotation` in a small RANSAC loop, known rotation axis | rotation, translation direction |
| `relative_pose_radial.py` | Two-view relative pose under division distortion: ignore vs undistort with known k | rotation, translation direction |
| `triangulation.py` | `Triangulate`, `TriangulateDLT`, `TriangulateMidpoint`, `TriangulateNView(SVD)`, `EstimateTriangulation` | 3D point error (m) |
| `global_sfm.py` | `EstimateTwoViewInfo` -> `ViewGraph` + `TrackBuilder` -> `GlobalReconstructionEstimator` | camera centre and rotation after Sim(3) alignment |
| `bundle_adjustment_priors.py` | `BundleAdjustReconstruction` with position, orientation, gravity, depth and 3D point priors | camera centre, rotation, point error after Sim(3) alignment |
| `init_free_sfm.py` | `INIT_FREE` (experimental, random-start OSE + VarPro, projective refinement, metric upgrade) vs `GlobalReconstructionEstimator` on identical matches | camera centre and rotation after Sim(3) alignment, per-restart diagnostics |

Every script accepts `--help`. Defaults are small, single-threaded, and finish
in seconds (`bundle_adjustment_priors.py` in about 5 s).

## Conventions used

- Pixels go to normalized coordinates with `K^-1` for calibrated solvers.
- Radial (division-model) distortion uses centred pixels, `d = u * (1 - sqrt(1 - 4k r_u^2)) / (2 k r_u^2)`, and undistortion is `d / (1 + k |d|^2)`. This matches `DistortPoint()` in pyTheia.
- `pose.position` from `RelativePose` / `MonoDepthRelativePoseResult` is the second camera centre in the first frame (`-R^T t`), a unit vector. `rotation` is `R` with `X2 = R X1 + t`.
- `CalibratedAbsolutePose.position` is the camera centre in world coordinates.
- `DlsPnp` returns quaternions as `(w, x, y, z)`.
- `Feature(point, depth_prior)` sets the depth prior. `Track.SetPositionPrior(position, sqrt_information)` and the `View` priors take sqrt information `S` (with `S^T S = Sigma^-1`), so `np.eye(3) / sigma` means standard deviation `sigma`.
- `RansacParameters.error_thresh` is the SQUARED error in normalized (`inv(K)`) coordinates for calibrated estimators, so a pixel threshold is `(px / f) ** 2` (e.g. 2 px at f = 3400 is `3.5e-7`). Use `use_lo = True` for PnP and relative pose RANSAC; without it the minimal-sample pose is returned unrefined.

## Known API issues

These are reported, not worked around silently. See the report at
`/media/steffen/Data2/sfm_bench/eth3d_examples_report.md`.

- `pt.sfm.SQPnP` (direct solver) and `pt.sfm.PnPType.SQPnP` (for RANSAC) are separate names; the shadowing bug was fixed in 1.4.1.
- UPnP is not bound in Python.
- `EstimateTriangulation` sometimes returns `success=True` with zero inliers and an uninitialized 4-vector.
- Passing out-of-range radial bounds to the four-point radial solver aborts the whole process (`CHECK` failure) instead of raising. The documented example in `docs/content/pose.md` triggers this.
