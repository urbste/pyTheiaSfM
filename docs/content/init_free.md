# Initialization-free SfM (experimental)

`ReconstructionEstimatorType.INIT_FREE` estimates cameras and points **jointly from random starts**. It does not use rotation averaging, position estimation or incremental registration. It follows the object space error (OSE) line of work:

1. **Problem.** Tracks (and, for `POSE_ROT`, the relative rotations of the view graph) become an `OseProblem`.
2. **OSE stage.** Random cameras are sampled, and an OSE objective is minimized with variable projection (VarPro). The result is a projective reconstruction.
3. **Projective refinement** *(optional)*. Projective bundle adjustment with Ceres (cameras and homogeneous points on unit spheres) removes the bias that the α-weighted affine term of pOSE leaves in the OSE minimum. Optionally with a robust loss (`robust_loss_width`, `loss_function_type`) and an outlier rejection that removes observations above a threshold and solves again (`outlier_rejection_rounds`, `outlier_threshold`); both are needed for real tracks (see *Real matches*). With several restarts, the restarts are ranked by the reprojection RMS after refinement (`restart_selection = REPROJECTION_ERROR`, the default; with outlier rejection, the RMS with every error capped at the threshold) or by the OSE cost.
4. **Metric upgrade with validity checks.** The restarts are upgraded in ranked order, and the first one whose metric upgrade succeeds and passes the checks is used (see *Restarts and validity checks*). The upgrade is self-calibration with the absolute dual quadric: known intrinsics for `CALIBRATED_RAYS`, one focal length per camera for `PIXEL_ISOTROPIC`. A linear estimate, optionally refined nonlinearly. A few cameras that collapsed during the projective refinement are dropped instead of failing the restart.
5. **Metric bundle adjustment.** The standard pyTheia BA, configured by `ReconstructionEstimatorOptions`. Tracks with an observation rejected in step 3 are left out. After the BA, outlier tracks are removed as in the global pipeline, and the BA runs again if any were removed.

The module is a testing ground. Every stage is exposed to Python on its own, so that new formulations, solvers and upgrades can be compared with the same data and diagnostics.

!!! warning "Experimental"
    The interface can still change. All formulations (`POSE`, `POSE_ROT`, `ROSE`, `EXPOSE`, `RPOSE`) and both observation normalizations (`CALIBRATED_RAYS`, `PIXEL_ISOTROPIC`) work end to end, with the linear or the nonlinear metric upgrade. `POSE_ROT` needs `CALIBRATED_RAYS`. `RPOSE` skips projective refinement and always uses the nonlinear upgrade. `DOSE` (depth-augmented OSE, *experimental*) needs calibrated rays and depth priors; it produces Euclidean cameras directly, without projective refinement or metric upgrade (see *Depth-augmented OSE (DOSE)*). Unsupported combinations fail with a clear message instead of falling back silently.

## Usage

```python
import pytheia as pt

opts = pt.sfm.ReconstructionEstimatorOptions()
opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
ifo = opts.init_free_options
ifo.formulation = pt.sfm.OseFormulationType.POSE_ROT
ifo.camera_init = pt.sfm.InitFreeCameraInitType.UNIT_CIRCLE
ifo.num_random_restarts = 3
ifo.projective_refinement.max_num_iterations = 50   # 0 (default) skips the stage

estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
summary = estimator.Estimate(view_graph, reconstruction)
diag = estimator.Diagnostics()   # per-restart costs, upgrade quality, timings
```

The stages can also be run one by one:

```python
ok, problem, err = pt.sfm.BuildOseProblem(reconstruction, view_graph, ifo)
cams0 = pt.sfm.SampleInitialCameras(problem.NumCameras(), ifo.camera_init, seed=0)
ok, sol = pt.sfm.SolveOse(problem, ifo, cams0)         # projective cameras + points (3 x N)
ok, ref = pt.sfm.RefineProjectiveReconstruction(
    problem, sol.cameras, sol.points, ifo.projective_refinement)   # points 4 x N
uo = pt.sfm.MetricUpgradeOptions()
uo.nonlinear_refinement = True
ok, metric, err = pt.sfm.UpgradeToMetric(problem, ref.cameras, ref.points, uo)
```

`RunProjectiveStages(problem, ifo)` runs all restarts of the OSE stage and the refinement and returns `(ok, best, diagnostics, err)`. `ProjectiveRmsError(problem, cameras, points)` gives the RMS reprojection error in observation units (normalized image coordinates for `CALIBRATED_RAYS`). Functions that take points accept 3 × N Cartesian or 4 × N homogeneous arrays.

Two runnable examples show the API:

- `pyexamples/sfm/init_free_synthetic.py` runs the estimator, the stages and a self-calibration variant on a synthetic scene. It needs no data and takes a few seconds.
- `pyexamples/eth3d/init_free_sfm.py` runs the estimator on ETH3D subsets against global SfM (see *Real data (ETH3D)*).

## Options and status

| Option | Values | Status |
|---|---|---|
| `formulation` | `POSE`, `POSE_ROT`, `ROSE`, `EXPOSE` | implemented |
| | `RPOSE` | implemented end to end (radial 2×4 cameras; radial metric upgrade, no projective refinement) |
| | `DOSE` | experimental (Euclidean cameras with depth priors; no projective refinement, no metric upgrade; see *Depth-augmented OSE (DOSE)*) |
| `observation_normalization` | `CALIBRATED_RAYS` | implemented (needed by `POSE_ROT` and the upgrade) |
| | `PIXEL_ISOTROPIC` | implemented (not with `POSE_ROT`); the upgrade estimates the focal lengths |
| `camera_init` | `NORMAL`, `UNIFORM`, `UNIT_CIRCLE` | implemented |
| | `EUCLIDEAN_RANDOM` | experimental (random `[R | t]` for `DOSE`) |
| `varpro.linear_solver_type` | `DENSE_CHOLESKY` | implemented (CPU, Eigen) |
| | `CUDA_DENSE_CHOLESKY` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `POWER_SERIES` | implemented (CPU, matrix-free; `power_series_max_iterations`, `power_series_tolerance`) |
| | `CUDA_POWER_SERIES` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `SPARSE_CHOLESKY` | implemented (CPU, Eigen simplicial LLT; for sequences and sparse view graphs) |
| `varpro.robust_loss_type` | `NONE` | implemented |
| | `CAUCHY` | implemented (IRLS, `robust_loss_scale` in observation units) |
| `projective_refinement.max_num_iterations` | `0` skips, `> 0` runs | implemented (`function_tolerance`, optional robust loss) |
| `projective_refinement.loss_function_type` | `HUBER` (default), `CAUCHY`, and the other `LossFunctionType` values | implemented (used with `robust_loss_width`) |
| `projective_refinement.outlier_rejection_rounds`, `outlier_threshold` | `0` (off) | implemented (threshold in observation units; see *Real matches*) |
| `projective_refinement.deterministic` | `True` | implemented (solves with one thread, so results do not change between runs) |
| `varpro.num_threads` | `0` uses `num_threads` of `InitFreeOptions` | implemented; results do not depend on the thread count |
| `rotation_weighting` | `SCALAR`, `TWO_VIEW_INFORMATION` | implemented (`POSE_ROT` only) |
| `restart_selection` | `REPROJECTION_ERROR`, `OSE_COST` | implemented (sets the order in which the restarts are upgraded) |
| `min_fraction_in_front`, `max_mean_orthogonality_error` | `0.9`, `0.3` | implemented (validity checks of the metric upgrade, see *Restarts and validity checks*) |
| `min_upgrade_conditioning` | `0.003` (`0` disables) | implemented (rejects critical self-calibrations, see *Metric upgrade*) |
| `max_camera_orthogonality_error`, `max_fraction_dropped_cameras` | `1.0`, `0.1` (`0` disables) | implemented (drops single broken cameras, see *Restarts and validity checks*) |
| `max_relative_rotation_error_deg` | `5` (`0` disables) | implemented (`POSE_ROT` only: agreement with the two-view rotations) |
| `max_translation_direction_error_deg` | `0` (disabled) | implemented, measured only (agreement with the two-view translation directions) |
| `upgrade_unrefined_fallback` | `False` | implemented (upgrade the unrefined OSE solutions if no refined restart passes) |
| `projective_refinement.robust_loss_width` | `0` (plain least squares) | implemented (width of `loss_function_type` in observation units; see *Real data (ETH3D)*) |
| `metric_upgrade` | `LINEAR`, `LINEAR_AND_NONLINEAR` | implemented for both normalizations; `RPOSE` always uses the nonlinear refinement |
| `shared_focal_length` | `False`, `True` | implemented (`PIXEL_ISOTROPIC` only) |

### Formulations

With `m` the normalized observation, `P` the 3 × 4 camera, `X̃` the homogeneous point and `α` the `alpha` option:

- **`POSE`**: `(1−α)‖P₁₂X̃ − (p₃ᵀX̃) m‖² + α‖P₁₂X̃ − m‖²` (Hong & Zach).
- **`POSE_ROT`**: `POSE` plus the relative rotation terms `‖A_a A_bᵀ − R_ab‖²` of the view graph (Olsson & Nilsson).
- **`ROSE`**: the OSE term plus `α (p₃ᵀX̃ − 1)²`, a depth regularizer. The OSE minimum is strongly biased (about 11 px RMS on the 1 px test scene), so use it together with projective refinement.
- **`EXPOSE`**: the third residual is `√α · √(exp(−s₀)/2) · (s − 1 − s₀)`, with `s = m̃ᵀPX̃ / |m̃|` and `m̃ = [m; 1]`. The state `s₀` starts at `|m̃|` and, after `expose_warmup_iterations`, follows the current depth, as in InitFreeBA. Cameras with `det(A) < 0` are flipped first. This gives the smallest OSE bias of the formulations here (1.29 px against 1.40 px for `POSE` on the 1 px test scene).
- **`RPOSE`**: the radial version, `(1−α)(m⊥ᵀP₁₂X̃)²/|m|² + α‖P₁₂X̃ − m‖²` with `m⊥ = (−m₂, m₁)`. Only the first two camera rows are determined. The estimator runs the radial metric upgrade (`radial_cameras`, see *Metric upgrade*) with the nonlinear refinement, whatever `metric_upgrade` says: the linear radial estimate is biased by the RPOSE affine term (up to 13° on the test scene), so `LINEAR` is overridden with a log message. It skips projective refinement, which is not defined for radial cameras; the metric bundle adjustment then refines the full cameras. The restart diagnostics report `ose_rms_error = −1`.

With `robust_loss_type = CAUCHY`, every VarPro iteration recomputes the IRLS weights `w = 1/(1 + ‖r‖²/c²)` of the observation residuals (`c = robust_loss_scale`), and the cost is `Σ ½c² log(1 + ‖r‖²/c²)`. The rotation terms are not robustified. With 10 % of the observations displaced by up to 150 px, the inlier median error after the OSE stage falls from 12.3 px to 1.1 px (`c` = 5 px). For robust or state-dependent formulations, the cost history is not monotone, because the weights change between iterations.

- **`DOSE`**: the OSE residuals of `POSE` on Euclidean cameras `P = [R | t]`, plus a relative depth residual per observation with a depth prior. See *Depth-augmented OSE (DOSE)*.

### Rotation weighting

`POSE_ROT` weights each relative rotation term with the scalar `rotation_weight`. With `rotation_weighting = TWO_VIEW_INFORMATION`, each pair instead gets the 9 × 9 weight `W_kl` of Olsson & Nilsson (Eq. 4), computed from the problem's own correspondences of the pair:

- The translation comes from `TwoViewInfo.position_2` or, if that is zero, from the epipolar constraint and cheirality.
- The points are triangulated with the midpoint method.
- The 3 × 3 rotation information `H` is formed with the translation and the points marginalized out.
- In the basis made of the tangent directions `[e_i]× R_ab` and the symmetric normal directions `S R_ab`, `W = diag(H/2, rotation_normal_weight · I₆)`. A tangent step `[ω]× R_ab` then costs `ωᵀHω`, the Gauss-Newton increase of the two-view reprojection error.

Pairs with fewer than `min_rotation_information_points` usable points keep the scalar weight. `pt.sfm.ComputeTwoViewRotationWeight(m_a, m_b, R_ab, t_ab)` exposes the computation.

Compared with the InitFreeBA reference code, `H` here is in observation units (not pixels) and has no extra 1/3 factor. The Jacobian assembly of the reference code also leaves some Jacobian buffers unfilled, so its weights cannot be reproduced exactly. The weighting helps when the relative rotations are estimated and their quality varies. Benchmark: 15 views, 250 points, 45 % visibility, 1 px noise, relative rotations from `EstimateTwoViewInfo` (mean edge error 0.53°, max 3.1°). Relative-rotation error after the OSE stage and the linear upgrade (no refinement), averaged over 4 scenes, best of a `rotation_weight` sweep:

| Weighting | `rotation_weight` | Mean error | Max error |
|---|---|---|---|
| `SCALAR` | 0.1 (best of 0.01 to 1) | 0.26° | 0.72° |
| `TWO_VIEW_INFORMATION` | 3 (best of 0.01 to 30) | 0.19° | 0.42° |

The two-view weights stay better than the best scalar weight for `rotation_weight` from 1 to 30, so 1 to 3 is a good starting range. With exact view-graph rotations, as in the unit tests, and with projective refinement, both give the same final accuracy.

### Metric upgrade

`UpgradeToMetric(problem, cameras, points, options)` estimates the absolute dual quadric `Ω`, with `P_i Ω P_iᵀ ~ K_i K_iᵀ`:

- **`CALIBRATED_RAYS`**: `K_i = I`, five linear equations per camera.
- **`PIXEL_ISOTROPIC`**: `K_i = diag(f_i, f_i, 1)` in the normalized pixel frame. This assumes zero skew, unit aspect ratio and the principal point at the image center. The linear step uses the weighted equations of Pollefeys et al. (2004). They contain soft priors on the aspect ratio, skew and principal point, plus a weak prior `f_i ≈ 1`, and are reweighted by `1/(P₃ΩP₃ᵀ)`. Needs at least 3 cameras.

The linear `Ω` is truncated to rank 3. With `nonlinear_refinement` (`LINEAR_AND_NONLINEAR` in the estimator), `Ω = BBᵀ` with `B` a 4 × 3 matrix, which keeps `Ω` positive semidefinite with rank 3. `B` and the focal lengths are refined with Ceres to minimize `Σ_i ‖3E_i / tr(E_i) − I‖²_F`, with `E_i = K_i⁻¹P_iBBᵀP_iᵀK_i⁻ᵀ`. The refinement drops the priors, and with them their bias. With `shared_focal_length`, one focal length is used for all cameras.

**Radial cameras** (`radial_cameras = True`, used by `RPOSE`) keep only rows 0 and 1 of each camera, and the metric camera has the form `P₁₂H = s[R₁₂ | t₁₂]` with orthonormal rows in `R₁₂`. The linear step uses two equations per camera (equal diagonal and zero off-diagonal entry of `P₁₂ΩP₁₂ᵀ`), so at least five cameras are needed. The optional refinement fits `Ω = BBᵀ` to the same constraint, without priors. Two candidates, `H` and `H·diag(1, 1, −1, 1)`, are built: per camera, the sign of the first two rows is chosen so that the 2D directions agree with the observations, and the forward translation `t₃` (and, for `PIXEL_ISOTROPIC`, the focal length) is fit by least squares to `m(r₃ᵀX + t₃) = f(R₁₂X + t₁₂)`. The candidate with more points in front is kept. Measured with the default `α = 0.05`: on a noise-free 15-view scene, the linear radial upgrade is off by up to 13° (the RPOSE affine term biases the OSE solution) and the refinement by up to 1.3°, with all observations in front; end to end with 1 px noise, the worst relative rotation error after bundle adjustment is 0.25° with `LINEAR_AND_NONLINEAR`, and 0.25° with `LINEAR` too, since the estimator always runs the refinement for radial cameras. For `PIXEL_ISOTROPIC`, the focal error on noise-free data is 16 % at `α = 0.05` and 0.9 % at `α = 0.001`, because the same bias shrinks as `α` does.

`result.focal_lengths` are in observation units. The estimator writes them to the cameras in pixels (`f / s` for the normalization scale `s`), sets the principal point to the image center, and reports them in `Diagnostics().focal_lengths`. Results on a 15-view scene with focal lengths from 650 to 1000 px:

| Noise | Upgrade | Max focal error | Mean rel. rotation error |
|---|---|---|---|
| 0 px | linear | 1.3 % | 0.38° |
| 0 px | + nonlinear | < 1e-5 | 0.000° |
| 0.5 px | linear | 1.7 % | 0.62° |
| 0.5 px | + nonlinear | 0.35 % | 0.09° |

For calibrated rays, the linear estimate is already close, and the refinement changes little.

**Projective frame.** The linear estimate depends on the projective frame. After projective refinement the frame can be badly scaled, and the linear quadric is then dominated by one eigenvalue and not rank-3 positive semidefinite although the reconstruction is accurate (ETH3D relief with SIFT matches). The upgrade therefore computes two candidates, one in the given frame and one in a frame normalized by the points (median centroid at the origin, median distance one), refines each, and keeps the one with the lower orthogonality error. On the synthetic scenes above the final result is the same, and the linear error of the chosen candidate is 5 to 10 times lower than in the given frame.

**Single cameras.** `MetricUpgradeResult.camera_orthogonality_errors` holds the error of every camera (infinity for a camera whose upgraded 3×3 block is singular). `MetricUpgradeOptions.camera_mask` restricts the estimation of the quadric to some cameras; the upgrade is still applied to all of them. The estimator uses both to drop collapsed cameras (see *Restarts and validity checks*).

**Conditioning (critical motions).** Some camera motions do not determine the self-calibration. For varying focal lengths the pure translation is one such motion (all cameras with the same orientation): the upgrade then fits the quadric and the focal lengths, but they are not unique. `MetricUpgradeResult::conditioning` measures this. It is the fifth smallest singular value of the Jacobian of the quadric residuals (with respect to the 12 entries of the normalized `B` and the estimated focal lengths) over the largest one. The four gauge directions of the residuals (`B` times an orthogonal matrix, and the scale of `B`) are excluded. Small values mean an extra ambiguity. Measured on synthetic scenes (noise 0 to 2 px): well-posed scenes 0.018 to 0.56 (calibrated, varying and shared focal lengths, RPOSE). Pure translation with varying focal lengths 1e-8 (noise free) to 6e-4 (2 px), with focal lengths off by 30 to 54 %, while its orthogonality error stays below 0.01, so the orthogonality check does not catch it. Rotation about one axis with the centres in a plane is not critical in this parametrization (conditioning 0.044, focal lengths recovered to 0.4 % at 0.5 px). `min_upgrade_conditioning` (default `0.003`, `0` disables) rejects a restart whose conditioning is below it, with the message `ill-conditioned self-calibration (critical motion?)`. The threshold lies between the two groups: about 5 times above the largest critical value and 6 times below the smallest well-posed one. On real data (ETH3D, calibrated) accepted restarts measured 0.09; the wrong restarts (orthogonality error 0.8 to 2.6) still measured 0.29 to 0.44, so this check flags only the critical ambiguity, not wrong restarts in general.

## Depth-augmented OSE (DOSE)

!!! warning "Experimental"
    DOSE follows the first stage of InitFree-SfM; the paper's joint metric refinement and pair filtering are not implemented (see *Differences to the paper*).

DOSE (arXiv:2611.31516, InitFree-SfM) adds a monocular depth prior to OSE. The OSE term only constrains the direction of each observation's ray. The depth term fixes the position along the ray and therefore the metric scale. The paper uses MoGe-2 depths.

With `m` the normalized observation, `m̃ = [m; 1]`, `y = R X + t` (Euclidean camera `P = [R | t]`) and `d` the depth prior of the observation (camera-frame z), the objective is

```
Σ_ij ‖ y_{1:2} − m y_3 ‖²  +  λ_d Σ_ij ρ( m̃ᵀy / (‖m̃‖² d) − 1 )
```

with `λ_d` the depth weight (`dose_depth_weight`) and `ρ` the Huber loss of width `dose_depth_huber_width`. The first term is the OSE residual. The second compares the depth of the point along the ray with the prior. Its residual is dimensionless: 0.1 means a 10 % depth error.

**Inputs**

- `CALIBRATED_RAYS` observations (`observation_normalization`). `PIXEL_ISOTROPIC` is rejected.
- A view graph. Its two-view rotations enter only the validity check of the restarts (`max_relative_rotation_error_deg`), not the objective.
- Depth priors on the features: `pt.sfm.Feature(pixel, depth)` with the camera-frame z depth in metric units. A depth `<= 0` means no depth for that observation, which then has only the OSE residual. `BuildOseProblem` fails if no observation has a positive depth.
- `camera_init = EUCLIDEAN_RANDOM`: `R` uniform on SO(3) (normalized Gaussian quaternion), `t ~ N(0, 0.5²)` per coordinate, camera 0 fixed to `[I | 0]`. Other initializations are projected to the nearest rotation.

**Solver.** Rotations are updated on SO(3) with a left exponential-map step, translations additively. Camera 0 is held fixed, which removes the gauge freedom, and the depth term fixes the scale. The objective is minimized with the continuation of the paper: three stages with `λ_d` scaled by 5 %, 25 % and 100 % of `dose_depth_weight`, 20 VarPro iterations each, each stage starting from the previous one. The Huber loss is applied from the second stage on (IRLS, `dose_robust_from_stage = 1`).

**Output.** The cameras are Euclidean and metric, so DOSE runs no projective refinement and no metric upgrade. The estimator writes the best restart directly.

### Usage

Estimator:

```python
import pytheia as pt

opts = pt.sfm.ReconstructionEstimatorOptions()
opts.reconstruction_estimator_type = pt.sfm.ReconstructionEstimatorType.INIT_FREE
ifo = opts.init_free_options
ifo.formulation = pt.sfm.OseFormulationType.DOSE
ifo.observation_normalization = pt.sfm.OseObservationNormalization.CALIBRATED_RAYS
ifo.camera_init = pt.sfm.InitFreeCameraInitType.EUCLIDEAN_RANDOM
ifo.num_random_restarts = 3

estimator = pt.sfm.InitFreeReconstructionEstimator(opts)
summary = estimator.Estimate(view_graph, reconstruction)
diag = estimator.Diagnostics()
```

#### Stage-wise use

```python
import pytheia as pt

ok, problem, err = pt.sfm.BuildOseProblem(reconstruction, view_graph, ifo)
cams0 = pt.sfm.SampleInitialCameras(problem.NumCameras(), ifo.camera_init, seed=0)
ok, sol = pt.sfm.SolveOse(problem, ifo, cams0)   # Euclidean cameras (3 x 4) + points (3 x N)
```

For `DOSE`, `SolveOse` runs the staged continuation (`SolveDoseVarPro` in C++), the same as the estimator. `sol.cost_history` concatenates the stages; the cost jumps between stages because `λ_d` changes.

**Accuracy on synthetic data** (10 cameras, `pytests/sfm/test_init_free.py`): with exact depths the OSE stage recovers the poses exactly (rotation error 1e-6°, scale error below 1e-6). With 5 % per-observation depth noise, 20 % missing depths and 0.5 px pixel noise, the depth term dominates: the worst relative rotation error after the OSE stage is 2 to 4° (seeds 0 to 5; 0.05° from the pixel noise alone), and the scale is within 1 %. The metric bundle adjustment of the estimator then brings it to 0.1°.

### Options

All fields are in `InitFreeOptions`.

| Option | Default | Meaning |
|---|---|---|
| `dose_depth_weight` | `30.0` | `λ_d` of the depth term (paper: 30). Multiplied by each entry of the schedule. |
| `dose_depth_huber_width` | `0.1` | Huber width of the relative depth residual (dimensionless). The paper does not state its value; this default is ours. |
| `dose_depth_weight_schedule` | `[0.05, 0.25, 1.0]` | Continuation of `λ_d`, one stage per entry (paper: 5 %, 25 %, 100 %). |
| `dose_iterations_per_stage` | `20` | Maximum VarPro iterations per stage (paper: 20). |
| `dose_robust_from_stage` | `1` | First stage with the Huber loss (IRLS); earlier stages are least squares. |

The per-observation confidence `c_ij` of the paper is not defined there. Here it is 1 for every observation.

### Differences to the paper

The following parts of the paper are not implemented:

- The joint metric refinement with per-image depth scales `α_i`.
- The pairwise relative pose and log-scale constraints.
- The cheirality softplus penalty.
- The depth-consistent pair filtering (depth similarity RANSAC and epipolar cross-check at 15°).

Instead, the result goes directly to the usual metric bundle adjustment. The estimator does not enable the depth priors there: `BundleAdjustmentOptions.use_depth_priors` (default `False`, in `theia/sfm/bundle_adjustment/bundle_adjustment.h`) adds them to a bundle adjustment, but the estimator's bundle adjustment options, built from `ReconstructionEstimatorOptions`, do not set it.

The Huber width and the unit confidence are our choices, not the paper's.

## CUDA backend

The reduced camera system of each VarPro step has the same structure as the Schur complement in bundle adjustment. Both CUDA solvers upload the sparse matrix `Y = F W` (one row block per point, with `FᵀF = V⁻¹`), so that `WᵀV⁻¹W = YᵀY`:

- **`CUDA_DENSE_CHOLESKY`** forms `S = U − YᵀY` on the GPU (cuSPARSE SpMM, one camera column band at a time) and factorizes it with cuSOLVER.
- **`CUDA_POWER_SERIES`** evaluates the power series with cuSPARSE SpMV (`Y`, `Yᵀ`, the pairwise rotation blocks) and batched cuBLAS products for the `D⁻¹` blocks. It reproduces the CPU power series to rounding.

Both use the CUDA host libraries only (cudart, cuBLAS, cuSPARSE, cuSOLVER), so no CUDA compiler is needed:

```bash
CMAKE_ARGS="-DTHEIA_WITH_CUDA_SOLVERS=ON" python setup.py bdist_wheel
python -c "import pytheia as pt; print(pt.sfm.CudaReducedCameraSolverAvailable())"
```

The option is off by default so that the published wheels stay portable.

## Restarts and validity checks

Each random start can converge to a wrong minimum, and the OSE cost and even the reprojection RMS do not always tell. On real data a restart can reach a low RMS and still have a wrong metric upgrade (see *Real data (ETH3D)*). The estimator therefore never trusts the ranking alone:

1. The restarts are ranked by `restart_selection` (ties keep the restart order).
2. The metric upgrade is run on the best restart. If it fails (for example, the dual quadric is not rank-3 positive semidefinite), or if it violates a check, the next restart is tried.
3. The first restart that passes both checks is used and written, and `Diagnostics().best_restart` is its index (the index can differ from the rank). If no restart passes, `Estimate` fails, writes no reconstruction, sets `best_restart` to -1, and its message lists the outcome of every ranked restart.

The checks are:

- `min_fraction_in_front` (default `0.9`): the fraction of observations with positive depth after the metric upgrade. Correct restarts measured 0.998 to 1.0 on ETH3D; wrong ones 0.58 to 0.98.
- `max_mean_orthogonality_error` (default `0.3`): `MetricUpgradeResult::mean_orthogonality_error`, the mean over cameras of `‖MMᵀ/s² − I‖_F` for the upgraded camera. Measured on the upgrade of each restart, before bundle adjustment:

| Data | Correct restarts (rotation error after BA below 0.3°) | Wrong restarts (rotation error above 25°) |
|---|---|---|
| Synthetic, no noise | 1e-12 to 3e-8 | none |
| Synthetic, 1 px noise, 15 views | 0.005 to 0.011 | none |
| ETH3D, 0.5 px, 0 % and 20 % outliers | 0.0005 to 0.17 | 0.63 to 14.5 |

  The threshold 0.3 sits between the largest correct value (0.17) and the smallest wrong one (0.63), about 1.8 times and 2.1 times away. These values were measured with the linear estimate in the given frame only. Since the upgrade also tries a normalized frame (see *Metric upgrade*), wrong restarts can reach lower values: on office 4–15 with 20 % outliers, wrong restarts now measure 0.14 to 0.23. The checks below were added for this reason. Two cases are not separated by it: one correct restart at 0.61 (courtyard 10–29, 20 % outliers) is rejected, which is the safe direction. One accepted restart is inaccurate (office 4–15, 0 % outliers: 0.08, 1.2° rotation error, 2 of 12 views dropped), which this check cannot exclude.

- `min_upgrade_conditioning` (default `0.003`, `0` disables): `MetricUpgradeResult::conditioning` (see *Metric upgrade*). A restart below it is rejected as an ill-conditioned self-calibration (critical motion).
- `max_camera_orthogonality_error` (default `1.0`) and `max_fraction_dropped_cameras` (default `0.1`): a camera with few observations can collapse during the projective refinement while all others are accurate. On ETH3D with SIFT matches (relief, meadow, office) one camera reached 2.5 to 2·10⁵ while all others stayed below 0.11; on office it was also flipped by 180° and biased the quadric of the other cameras by 1 to 5°. Cameras above `max_camera_orthogonality_error` are dropped: the upgrade is estimated again without them (`MetricUpgradeOptions.camera_mask`), the mean orthogonality error and the fraction in front are checked on the remaining cameras, and the dropped views stay unestimated (`Diagnostics().dropped_view_ids`, `upgrade_num_dropped_cameras` per restart). If more than `max_fraction_dropped_cameras` of the cameras would be dropped, the restart is rejected. With fewer than 10 cameras, no camera may be dropped.
- `max_relative_rotation_error_deg` (default `5`, `0` disables, `POSE_ROT` only): the median angle between the relative rotations of the upgraded cameras and the two-view rotations of the view graph. Accepted restarts on ETH3D with SIFT matches measured 0.06 to 0.93°, and on the ETH3D subsets 0.02 to 1.95° (4.5° for an inaccurate office 4–15 result without the Cauchy loss). It turned the wrong office 4–15 result at 20 % outliers with the library defaults into a failure (5.9°), but it also rejects one correct result (courtyard 30–37 at 20 % outliers, library defaults: 5.2°), and it does not catch the wrong result with robust refinement on the same subset (2.9°). The threshold is therefore a coarse check, not a separation.
- `max_translation_direction_error_deg` (default `0`, disabled): the same for the direction of the camera centers. It is recorded for every attempted restart (`upgrade_translation_direction_error_deg`) but not used by default. On the office 4–15 subset the wrong upgrade at 20 % outliers (8.9° median) and the correct one at 0 % (7.0 to 9.2°) have the same distribution of per-edge errors, so no threshold separates them; on the 12 accepted ETH3D scenes with SIFT matches it is 0.13 to 2.7°.

The checks are written so that a NaN fails. The restart diagnostics record every attempt: `upgrade_attempted`, `upgrade_success` (the upgrade succeeded and passed the checks), `upgrade_fraction_in_front`, `upgrade_mean_orthogonality_error`, `upgrade_conditioning`, `upgrade_num_dropped_cameras`, `upgrade_relative_rotation_error_deg`, `upgrade_translation_direction_error_deg` and `upgrade_message` (the upgrade error, `rejected: ...` with the failed check, or `ok`). With `upgrade_unrefined_fallback`, the unrefined OSE solutions are upgraded in the same order if no refined restart passes (`unrefined_upgrade_message`, `Diagnostics().used_unrefined_fallback`). On the ETH3D runs below it was never needed.

```python
ifo.min_fraction_in_front = 0.9
ifo.max_mean_orthogonality_error = 0.3
estimator.Estimate(view_graph, reconstruction)
for k, r in enumerate(estimator.Diagnostics().restarts):
    print(k, r.upgrade_attempted, r.upgrade_success, r.upgrade_message)
```

### Ablation without the Cauchy loss

Without the Cauchy loss in VarPro (`init_free_sfm.py --no_cauchy`, ETH3D), the estimator fails on delivery_area 12–23 (0 % and 20 % outliers) and courtyard 10–29 (20 %). Office 4–15 is accepted with 3.2° at 0 % outliers (1.3° for the global pipeline), and at 20 % outliers it now fails; before the relative rotation check it was accepted with a 26° median rotation error. So the checks reduce silently wrong results but do not exclude them (see the robust refinement on office 4–15 below). Keep the Cauchy loss on for real data.

## Real data (ETH3D)

`pyexamples/eth3d/init_free_sfm.py` runs the estimator on ETH3D subsets, with the observations, outliers and calibrated priors described in the script. Settings: `POSE_ROT`, `CALIBRATED_RAYS`, `LINEAR_AND_NONLINEAR` upgrade, metric BA, 3 restarts (seeds 42 to 44), 0.5 px pixel noise, outlier ratio 0 % and 20 %. Rotation error is the median over the estimated views after a Sim(3) alignment of the centres. Times are 0.4 to 7.1 s per run. The columns are the projective refinement of the library default (plain least squares), the example default (Huber 3 px) and `--robust_refinement` (Cauchy 3 px, outlier rejection at 5 px, see *Real matches*).

| Scene (views) | Outliers | Global SfM | Library default | Example default (Huber 3 px) | Robust refinement |
|---|---|---|---|---|---|
| courtyard 30–37 (8) | 0 % | 0.16° | 0.16° | 0.16° | 0.16° |
| courtyard 30–37 (8) | 20 % | 0.16° | **FAIL** | 0.18° | 0.18° |
| courtyard 10–29 (20) | 0 % | 0.15° | 0.14° | 0.14° | 0.14° |
| courtyard 10–29 (20) | 20 % | 0.13° | **FAIL** | **FAIL** | 0.15° |
| delivery_area 12–23 (12) | 0 % | 0.05° | 0.05° | 0.05° | 0.05° |
| delivery_area 12–23 (12) | 20 % | 0.19° | 0.25° | 0.25° | 0.25° |
| facade 20–31 (12) | 0 % | 0.09° | 0.07° | 0.07° | 0.07° |
| facade 20–31 (12) | 20 % | 0.06° | **FAIL** | 0.05° | 0.05° |
| office 4–15 (12) | 0 % | 1.28°, 10 views | 1.08°, 10 views | 0.62°, 10 views | 0.99°, 10 views |
| office 4–15 (12) | 20 % | 0.17°, 7 views | **FAIL** | **FAIL** | **wrong, 136°**, 8 views |

What this shows:

- The robust refinement is correct in 9 of 10 cells and matches the global pipeline within 0.06° in all but office 4–15. It is the only setting that reconstructs courtyard 10–29 at 20 % outliers.
- Office 4–15 at 20 % outliers with the robust refinement is the one silently wrong result: the accepted restart has 99 % of the observations in front, an orthogonality error of 0.14, and relative rotations within 2.9° of the view graph (median). On this subset of 8 usable views, no check separates it from the correct restarts (see *Restarts and validity checks*).
- With the library default, courtyard 30–37 at 20 % outliers fails because of the relative rotation check (5.2°, a correct restart), and facade 20–31 and courtyard 10–29 at 20 % outliers fail because too many cameras are broken. Office 4–15 at 20 % outliers fails instead of being wrong (86.9° without the relative rotation check).

### Real matches (ETH3D, SIFT)

The estimator was run on real SIFT matches of all 13 ETH3D training scenes, from the shared COLMAP database of the pyTheia paper benchmark: two-view estimation with a 2 px Sampson threshold, at least 30 inliers per pair, tracks of length 3 or more, calibrated intrinsics, one random seed, 8 threads. The global pipeline (robust L1/L2 rotations, LUD positions) on the same view graph is the reference. Errors are medians after a Sim(3) alignment to the ground truth. Settings as above with the robust refinement (`run_pytheia.py --variant init_free` in the paper repository), and, as an ablation, with a Huber loss of 3 px and no outlier rejection (`--variant init_free_huber`).

| Scene (views) | Global LUD | Init-free, robust | Init-free, Huber 3 px | Time robust (LUD) | Rejected observations | Dropped views |
|---|---|---|---|---|---|---|
| courtyard (38) | 38, 6.1 mm, 0.06° | 37, 6.0 mm, 0.06° | fail | 46 s (1.6 s) | 2.6 % | 1 |
| delivery_area (44) | 44, 5.2 mm, 0.04° | 44, 5.7 mm, 0.05° | 44, 5.7 mm, 0.05° | 18 s (0.8 s) | 1.1 % | 0 |
| electro (45) | 43, 3.5 mm, 0.04° | 42, 3.3 mm, 0.04° | 43, 4.9 mm, 0.08° | 18 s (0.7 s) | 0.9 % | 1 |
| facade (76) | 76, 11.9 mm, 0.12° | 76, 12.4 mm, 0.11° | 73, 181 mm, 1.07° | 205 s (6.4 s) | 4.7 % | 0 |
| kicker (31) | 30, 4.1 mm, 0.06° | 30, 2.7 mm, 0.03° | 30, 3.2 mm, 0.03° | 14 s (0.5 s) | | 0 |
| meadow (15) | 13, 66 mm, 1.07° | 14, 40 mm, 0.26° | fail | 1 s (0.1 s) | 4.2 % | 1 |
| office (26) | 26, 2.4 mm, 0.13° | 25, 2.0 mm, 0.13° | 26, 2.7 mm, 0.15° | 3 s (0.2 s) | | 1 |
| pipes (14) | 14, 9.8 mm, 0.48° | 14, 1.7 mm, 0.06° | 14, 1.5 mm, 0.06° | 1 s (0.1 s) | 2.7 % | 0 |
| playground (38) | 38, 7.3 mm, 0.08° | **fail** | fail | 29 s (0.4 s) | | |
| relief (31) | 31, 1.4 mm, 0.05° | 30, 1.3 mm, 0.03° | fail | 39 s (1.0 s) | 1.4 % | 1 |
| relief_2 (31) | 31, 1.4 mm, 0.05° | 31, 1.6 mm, 0.05° | fail | 19 s (0.4 s) | 1.1 % | 0 |
| terrace (23) | 23, 4.2 mm, 0.10° | 23, 3.4 mm, 0.07° | 23, 3.2 mm, 0.07° | 11 s (0.3 s) | | 0 |
| terrains (42) | 42, 3.6 mm, 0.07° | 42, 3.6 mm, 0.06° | fail | 18 s (0.4 s) | 2.3 % | 0 |

With the robust settings the estimator succeeds on 12 of 13 scenes, with errors close to those of the global pipeline (within 0.5 mm and 0.01° on 8 scenes, lower on pipes, kicker, meadow and terrace), but registers one view fewer on five scenes and is 10 to 50 times slower. With the Huber loss instead, it succeeds on 7 scenes and is inaccurate on facade (73 of 76 views, 1.07°). A first run, before the changes described below, succeeded on 6 scenes. The results are deterministic (`projective_refinement.deterministic`).

**Why the first run failed.** Each failing scene was diagnosed against the ground truth: tracks triangulated with the ground-truth cameras, and the projective reconstruction of each restart compared with the ground truth by fundamental matrices of camera pairs, which do not depend on the projective frame.

1. *Outliers in the tracks.* 2 to 12 % of the observations of the real tracks are more than 3 px from the ground-truth projection, mostly from wrongly merged tracks. A projective camera (11 degrees of freedom) can absorb such outliers, and the Huber loss of the projective refinement still gives them a linear influence: started from the ground-truth cameras, the refinement moved far away from them on courtyard, relief_2 and meadow. With the outlier tracks removed (using the ground truth), courtyard, relief_2, facade and electro reconstruct correctly. Without the ground truth, the Cauchy loss with outlier rejection gives the same result.
2. *The projective frame of the linear self-calibration* (relief, see *Metric upgrade*).
3. *A single collapsed camera* (relief, meadow, office; see `max_camera_orthogonality_error`).
4. *The metric BA* included the tracks with outliers and made every scene worse (relief_2: 0.11° after the upgrade, 0.66° after BA). Tracks with a rejected observation are now left out of it. A reprojection filter before the BA would also remove good tracks, because the upgraded cameras are not yet accurate (office: 37 % of the tracks).
5. *Playground* still fails: on every restart 13 to 20 of 38 cameras are broken after the refinement, and the unrefined OSE solutions do not pass the checks either. This is a wrong local minimum, not an outlier problem.

The multithreaded Ceres evaluation in the refinement made the first runs differ between repetitions (office: 0.76 to 1.5°). The differences start at 1e-16 and grow through the outlier rejection, the ranking and the upgrade. `projective_refinement.deterministic` (default) solves with one thread; this doubles the refinement time, (facade: 205 instead of 127 s in total).

### Robust projective refinement

`ProjectiveRefinementOptions.robust_loss_width` is the width of `loss_function_type` (Huber by default) in observation units (normalized coordinates for `CALIBRATED_RAYS`, so about `pixels / focal`). `outlier_rejection_rounds` and `outlier_threshold` add an outlier rejection: after each solve, observations above the threshold are removed and the problem is solved again, until no observation is removed or the rounds are used up. `ProjectiveRefinementResult.observation_inlier` marks the kept observations, and `final_truncated_rms_error` (each error capped at the threshold) ranks the restarts, since the inlier RMS would reward rejecting more. On a synthetic scene with 5 % gross outliers (30 to 150 px), the rejection at 5 px finds more than 95 % of them and removes less than 1 % of the inliers.

The library default stays at plain least squares without rejection, so that the stage remains the plain projective bundle adjustment of the study. For real tracks use

```python
pr = ifo.projective_refinement
pr.max_num_iterations = 50
pr.robust_loss_width = 3.0 / focal_px
pr.loss_function_type = pt.sfm.LossFunctionType.CAUCHY
pr.outlier_rejection_rounds = 3
pr.outlier_threshold = 5.0 / focal_px
```

Measured per restart on the ETH3D SIFT tracks, starting from the same OSE solutions: Huber without rejection leaves the projective reconstruction far from the ground truth on 7 of the 10 scenes measured. Cauchy without rejection fixes most of them but keeps a huge RMS (outliers at their full error), and Huber with rejection removes up to 26 % of the observations on bad restarts. Cauchy with rejection at 5 px is consistently best (rejection at 10 px is similar but worse on terrains).

## Performance

### Sparse Schur complement

`SPARSE_CHOLESKY` stores only the camera blocks that share a point or a rotation term: the lower triangle, one block column per camera. It factorizes with Eigen's simplicial LLT and AMD ordering. The assembly is parallel and deterministic, the symbolic analysis is reused while the pattern stays the same, and the result equals `DENSE_CHOLESKY` to rounding. Synthetic forward-moving sequence, `POSE_ROT`, 20 threads, 30 VarPro iterations:

| Cameras / observations | `DENSE_CHOLESKY` | `SPARSE_CHOLESKY` | `POWER_SERIES` |
|---|---|---|---|
| 150 / 213k | 4.7 s | 1.5 s | 1.6 s (cost not converged) |
| 400 / 591k | 109 s | 6.7 s | 4.9 s (cost not converged) |

For scenes where most cameras see the same points (the ring scenes below), S is dense and `DENSE_CHOLESKY` is the better choice. The 30-iteration budget of this timing run is too small for the 400-camera sequence (see below).

### Convergence on sequences

A study on synthetic forward-moving sequences (100 to 400 cameras, about 90 tracks per camera, 1 px noise, 3 random starts each) found:

- **Iterations:** with `POSE_ROT`, VarPro converges from random starts in 19 (100 cameras), 25 (200) and 55 (400) iterations to the correct solution. The relative-rotation error is about 0.02° between neighbors. The cost decreases steadily rather than stalling, so the default budget of 200 iterations is enough.
- **Initialization matters:** `UNIT_CIRCLE` converged in 6 of 6 runs. `NORMAL` and `UNIFORM` converged in 1 of 6 and 2 of 6, the others ending in wrong basins with cameras flipped by up to 180°. Keep the default `UNIT_CIRCLE`.
- **Rotation terms matter:** `POSE` and `EXPOSE` (no rotation terms) mostly did not converge within 300 iterations on sequences. With `POSE_ROT`, `rotation_weight` 10 gave about 10× lower rotation error than 1 (0.002° vs 0.02°) at the same cost.
- **`alpha`** (0.01 to 0.3) does not change the rotation accuracy. Smaller `alpha` lowers the projective RMS (4.5 px at 0.01, 8 px at 0.3, before refinement).
- **Damping:** `lambda_decrease_factor = lambda_increase_factor = 2` cut the iterations on sequences by about 30%. On the ring scenes it made no difference, or added two iterations for `POSE`, so the default stays at 10.
- **`POWER_SERIES`** reaches the same cost, but needs 2 to 10× more iterations than `SPARSE_CHOLESKY` on sequences, at similar or higher wall time.

### Threads and GPU

The VarPro assembly (point elimination, then the camera blocks through a camera-major index) and the power series are multithreaded. The chunks are combined in a fixed order, so the results are bit-identical for any thread count.

Synthetic `POSE_ROT` scene, 1 px noise, wall time of the VarPro stage on one random start (20-thread CPU, CUDA 13 GPU):

| Solver | Threads | 150 cameras, 180k obs. | 300 cameras, 720k obs. (iterations) |
|---|---|---|---|
| `DENSE_CHOLESKY` | 1 | 6.5 s | 37.7 s (5) |
| `DENSE_CHOLESKY` | 20 | 1.9 s | 8.7 s (5) |
| `POWER_SERIES` | 1 | 6.3 s | 43.9 s (35) |
| `POWER_SERIES` | 20 | 3.2 s | 16.8 s (35) |
| `CUDA_DENSE_CHOLESKY` | 20 | 1.6 s | 8.1 s (5) |
| `CUDA_POWER_SERIES` | 20 | 2.3 s | 15.5 s (35) |

All runs reach the same final cost. At these sizes, most of the time goes into the CPU part of each iteration: evaluating the Jacobians, eliminating the points and uploading `Y`. The GPU therefore gains little so far. Moving the Jacobian evaluation and point elimination onto the GPU is the next step for larger problems.

## Power series solver

`POWER_SERIES` never forms the reduced camera matrix. It writes the damped system as `S + λI = D_λ − M`, with `D_λ` the damped 12 × 12 camera blocks and `M = WᵀV⁻¹W − O`, and sums the Neumann series `Σ_k (D_λ⁻¹M)^k D_λ⁻¹ b` (PoBA / PoVar, Weber et al.). Each term costs one pass over the observations and rotation terms, so memory and time per term are linear in the problem size. For VarPro the series converges for every `λ > 0`: `WᵀV⁻¹W ⪯ D_obs`, and the pairwise rotation blocks satisfy `−D_rot ⪯ O ⪯ D_rot`. Truncating the series gives an inexact LM step. With 20 terms, it reaches the dense-solver cost to 1e-8 relative, in more but cheaper iterations (see *Performance*). Memory stays linear in the number of observations, which matters once the dense camera matrix no longer fits.

## Adding a method

- **Formulation:** implement `OseFormulation` (`ObservationModel`, `CameraJacobian`, and optionally a per-observation state with `HasState` / `UpdatedState`, as `EXPOSE` uses) in `src/theia/sfm/init_free/ose_formulation.cc`, add an enum value and a case in `CreateOseFormulation`. Every solver backend then works with it.
- **Linear solver:** implement `ReducedCameraSolver::Solve` on the block system `ReducedCameraSystem` (dense via `FormDenseSchurComplement`, or matrix-free from the `U`, `W`, `V⁻¹` blocks) and register it in `CreateReducedCameraSolver`.
- **Projective stage:** extend `RunProjectiveStagesRanked` (one pass per restart, ranked) and record the outcome in `InitFreeRestartDiagnostics`.
- **Upgrade:** add a stage in `InitFreeReconstructionEstimator::Estimate` (the restarts are upgraded in ranked order, see *Restarts and validity checks*) and record its outcome in `InitFreeDiagnostics`.

## References

- J. H. Hong, C. Zach. *pOSE: Pseudo Object Space Error for Initialization-Free Bundle Adjustment.* CVPR 2018.
- J. H. Hong, C. Zach, A. Fitzgibbon. *Revisiting the Variable Projection Method for Separable Nonlinear Least Squares Problems.* CVPR 2017.
- S. Weber, J. H. Hong, D. Cremers. *Power Variable Projection for Initialization-Free Large-Scale Bundle Adjustment.* ECCV 2024.
- C. Olsson, A. Nilsson. *Towards Initialization-free Calibrated Bundle Adjustment.* [arXiv:2506.23808](https://arxiv.org/abs/2506.23808).
- S. Weber, M. de Mayo, J. H. Hong, C. Olsson, D. Cremers, R. Clark. *Initialization-Free Bundle Adjustment Revisited: A Controlled Experimental Study.* [arXiv:2608.18028](https://arxiv.org/abs/2608.18028). Code: [InitFreeBA](https://github.com/simonwebertum/InitFreeBA).
- S. Weber, M. de Mayo, J. H. Hong, C. Olsson, D. Cremers, R. Clark. *InitFree-SfM: When Initialization-Free Bundle Adjustment Meets Learned Structure-from-Motion.* [arXiv:2611.31516](https://arxiv.org/abs/2611.31516). DOSE.
