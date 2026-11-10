# Initialization-free SfM (experimental)

`ReconstructionEstimatorType.INIT_FREE` estimates cameras and points **jointly from random starts**. It does not use rotation averaging, position estimation or incremental registration. It follows the object space error (OSE) line of work:

1. **Problem.** Tracks (and, for `POSE_ROT`, the relative rotations of the view graph) become an `OseProblem`.
2. **OSE stage.** Random cameras are sampled, and an OSE objective is minimized with variable projection (VarPro). The result is a projective reconstruction.
3. **Projective refinement** *(optional)*. Projective bundle adjustment with Ceres (cameras and homogeneous points on unit spheres) removes the bias that the α-weighted affine term of pOSE leaves in the OSE minimum. Optionally with a Huber loss (`robust_loss_width`). With several restarts, the restarts are ranked by the reprojection RMS after refinement (`restart_selection = REPROJECTION_ERROR`, the default) or by the OSE cost.
4. **Metric upgrade with validity checks.** The restarts are upgraded in ranked order, and the first one whose metric upgrade succeeds and passes the checks is used (see *Restarts and validity checks*). The upgrade is self-calibration with the absolute dual quadric: known intrinsics for `CALIBRATED_RAYS`, one focal length per camera for `PIXEL_ISOTROPIC`. A linear estimate, optionally refined nonlinearly.
5. **Metric bundle adjustment.** The standard pyTheia BA, configured by `ReconstructionEstimatorOptions`.

The module is a testing ground. Every stage is exposed to Python on its own, so that new formulations, solvers and upgrades can be compared with the same data and diagnostics.

!!! warning "Experimental"
    The interface can still change. The calibrated pipeline (`POSE`, `POSE_ROT`, `ROSE` or `EXPOSE` with `CALIBRATED_RAYS` and the linear upgrade) works end to end, and so does `RPOSE` (radial metric upgrade, no projective refinement). Options marked *not implemented* below fail with a clear message instead of silently falling back.

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

## Options and status

| Option | Values | Status |
|---|---|---|
| `formulation` | `POSE`, `POSE_ROT`, `ROSE`, `EXPOSE` | implemented |
| | `RPOSE` | implemented end to end (radial 2×4 cameras; radial metric upgrade, no projective refinement) |
| `observation_normalization` | `CALIBRATED_RAYS` | implemented (needed by `POSE_ROT` and the upgrade) |
| | `PIXEL_ISOTROPIC` | implemented (not with `POSE_ROT`); the upgrade estimates the focal lengths |
| `camera_init` | `NORMAL`, `UNIFORM`, `UNIT_CIRCLE` | implemented |
| `varpro.linear_solver_type` | `DENSE_CHOLESKY` | implemented (CPU, Eigen) |
| | `CUDA_DENSE_CHOLESKY` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `POWER_SERIES` | implemented (CPU, matrix-free; `power_series_max_iterations`, `power_series_tolerance`) |
| | `CUDA_POWER_SERIES` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `SPARSE_CHOLESKY` | implemented (CPU, Eigen simplicial LLT; for sequences and sparse view graphs) |
| `varpro.robust_loss_type` | `NONE` | implemented |
| | `CAUCHY` | implemented (IRLS, `robust_loss_scale` in observation units) |
| `projective_refinement.max_num_iterations` | `0` skips, `> 0` runs | implemented (`function_tolerance`, optional Huber `robust_loss_width`) |
| `varpro.num_threads` | `0` uses `num_threads` of `InitFreeOptions` | implemented; results do not depend on the thread count |
| `rotation_weighting` | `SCALAR`, `TWO_VIEW_INFORMATION` | implemented (`POSE_ROT` only) |
| `restart_selection` | `REPROJECTION_ERROR`, `OSE_COST` | implemented (sets the order in which the restarts are upgraded) |
| `min_fraction_in_front`, `max_mean_orthogonality_error` | `0.9`, `0.3` | implemented (validity checks of the metric upgrade, see *Restarts and validity checks*) |
| `projective_refinement.robust_loss_width` | `0` (plain least squares) | implemented (Huber loss in observation units; see *Real data (ETH3D)*) |
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

For calibrated rays, the linear estimate is already close, and the refinement changes little. Planar or otherwise critical camera motions are not detected; the result then has no unique focal lengths.

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

  The threshold 0.3 sits between the largest correct value (0.17) and the smallest wrong one (0.63), about 1.8 times and 2.1 times away. Two cases are not separated by it: one correct restart at 0.61 (courtyard 10–29, 20 % outliers) is rejected, which is the safe direction. One accepted restart is inaccurate (office 4–15, 0 % outliers: 0.08, 1.2° rotation error, 2 of 12 views dropped), which this check cannot exclude.

Both comparisons are written so that a NaN fails. The restart diagnostics record every attempt: `upgrade_attempted`, `upgrade_success` (the upgrade succeeded and passed the checks), `upgrade_fraction_in_front`, `upgrade_mean_orthogonality_error` and `upgrade_message` (the upgrade error, `rejected: ...` with the failed check, or `ok`).

```python
ifo.min_fraction_in_front = 0.9
ifo.max_mean_orthogonality_error = 0.3
estimator.Estimate(view_graph, reconstruction)
for k, r in enumerate(estimator.Diagnostics().restarts):
    print(k, r.upgrade_attempted, r.upgrade_success, r.upgrade_message)
```

## Real data (ETH3D)

`pyexamples/eth3d/init_free_sfm.py` runs the estimator on ETH3D subsets, with the observations, outliers and calibrated priors described in the script. Settings: `POSE_ROT`, `CALIBRATED_RAYS`, `LINEAR_AND_NONLINEAR` upgrade, metric BA, 3 restarts (seeds 42 to 44), 0.5 px pixel noise, outlier ratio 0 % and 20 %. Global SfM is correct in all of these. Rotation error is the median over the estimated views after a Sim(3) alignment of the centres. Times are 0.4 to 6.8 s per run.

| Scene (views) | Outliers | Before | Library default (plain least squares) | Example default (Huber 3 px) |
|---|---|---|---|---|
| courtyard 30–37 (8) | 0 % | ok | ok, 0.16°, restart 2 | ok, 0.16°, restart 2 |
| courtyard 30–37 (8) | 20 % | ok | ok, 0.18°, restart 2 | ok, 0.18°, restart 0 |
| courtyard 10–29 (20) | 0 % | ok | ok, 0.14°, restart 2 | ok, 0.14°, restart 2 |
| courtyard 10–29 (20) | 20 % | FAIL | **FAIL**, no restart passed | **FAIL**, no restart passed |
| delivery_area 12–23 (12) | 0 % | ok | ok, 0.05°, restart 2 | ok, 0.05°, restart 1 |
| delivery_area 12–23 (12) | 20 % | ok | ok, 0.25°, restart 1 | ok, 0.25°, restart 2 |
| facade 20–31 (12) | 0 % | ok | ok, 0.07°, restart 1 | ok, 0.07°, restart 2 |
| facade 20–31 (12) | 20 % | wrong, 178° | **FAIL**, no restart passed | ok, 0.05°, restart 1 |
| office 4–15 (12) | 0 % | FAIL | ok, 1.21°, 10 of 12 views, restart 2 | ok, 0.96°, 10 of 12 views, restart 2 |
| office 4–15 (12) | 20 % | wrong, 145° | **FAIL**, no restart passed | **FAIL**, no restart passed |

"Before" is the estimator before this change: the top-ranked restart was used without a metric check. For office 4–15 at 0 % outliers, the top-ranked restart 0 failed the metric upgrade, so that run failed. Now the upgrade falls back to the next ranked restart. The restart index of each success is given.

What this shows:

- No run in this table reports success with a reconstruction that is wrong (rotation error above 1°), and the cases that were wrong before now fail or are correct (facade 20–31 with Huber).
- Of the four former FAIL or WRONG cells, facade 20–31 at 20 % outliers is now correct with Huber 3 px (0.05°). Courtyard 10–29 and office 4–15, both at 20 % outliers, now fail with the per-restart reasons instead of being wrong. Office 4–15 at 0 % outliers now succeeds through the fallback (restarts 0 and 1 fail the upgrade, restart 2 passes), but with 1.2° (0.96° with Huber) rotation error and 2 views dropped. The other correct runs are at 0.05° to 0.25°. The checks accept it, so treat it as a lower-quality result.
- Courtyard 10–29 at 20 % outliers: of its six first restarts, only restart 2 (0.26°) is correct, and its orthogonality error of 0.61 is rejected by the threshold. The others are wrong (0.63 to 0.91). Up to 12 restarts with both settings give no accepted restart.
- Office 4–15 at 20 % outliers: none of the first six restarts is correct (rotation error 109° to 179°), and up to 12 restarts with both settings give no accepted restart.
- Facade 20–31 at 20 % outliers with plain least squares: every one of the first six restarts sits at 1.28 to 1.32 orthogonality error, and up to 12 restarts give no accepted restart.

### Robust projective refinement

`ProjectiveRefinementOptions.robust_loss_width` is a Huber width in observation units (normalized coordinates for `CALIBRATED_RAYS`, so about `pixels / focal`). The example sets it to 3 px by default. The library default stays at plain least squares, because the effect on individual restarts is mixed:

- Facade 20–31, 20 % outliers: all six restarts go from about 1.3 to about 0.0006 orthogonality error, and from 178° to 0.05° after BA. This is the main gain.
- Office 4–15, 0 % outliers: 1.21° to 0.96° after BA.
- Courtyard 30–37 at 20 % outliers and delivery_area: the correct restarts stay correct, and courtyard's orthogonality error drops from 0.17 to 0.01. Delivery_area restart 0 at 0 % outliers gets 0.23° instead of 0.05°, so Huber does not help every restart.
- Courtyard 10–29 at 20 % outliers: the correct restart 2 (plain least squares) fails the upgrade (not positive semidefinite) with Huber, and no other restart passes.
- A 1 px Huber width was measured per restart only. It was not chosen.

## Performance

### Sparse Schur complement

`SPARSE_CHOLESKY` stores only the camera blocks that share a point or a rotation term: the lower triangle, one block column per camera. It factorizes with Eigen's simplicial LLT and AMD ordering. The assembly is parallel and deterministic, the symbolic analysis is reused while the pattern stays the same, and the result equals `DENSE_CHOLESKY` to rounding. Synthetic forward-moving sequence, `POSE_ROT`, 20 threads, 30 VarPro iterations:

| Cameras / observations | `DENSE_CHOLESKY` | `SPARSE_CHOLESKY` | `POWER_SERIES` |
|---|---|---|---|
| 150 / 213k | 4.7 s | 1.5 s | 1.6 s (cost not converged) |
| 400 / 591k | 109 s | 6.7 s | 4.9 s (cost not converged) |

For scenes where most cameras see the same points (the ring scenes below), S is dense and `DENSE_CHOLESKY` is the better choice. Long sequences also converge slowly from random starts, whatever the solver; 30 iterations do not reach the minimum here.

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
