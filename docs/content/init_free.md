# Initialization-free SfM (experimental)

`ReconstructionEstimatorType.INIT_FREE` estimates cameras and points **jointly from random starts**. It does not use rotation averaging, position estimation or incremental registration. It follows the object space error (OSE) line of work:

1. **Problem.** Tracks (and, for `POSE_ROT`, the relative rotations of the view graph) become an `OseProblem`.
2. **OSE stage.** Random cameras are sampled, and an OSE objective is minimized with variable projection (VarPro). The result is a projective reconstruction.
3. **Projective refinement** *(optional)*. Projective bundle adjustment with Ceres (cameras and homogeneous points on unit spheres) removes the bias that the α-weighted affine term of pOSE leaves in the OSE minimum. With several restarts, the winner is chosen by the reprojection RMS after refinement (`restart_selection = REPROJECTION_ERROR`, the default) or by the OSE cost.
4. **Metric upgrade.** A linear absolute-dual-quadric estimate for known intrinsics.
5. **Metric bundle adjustment.** The standard pyTheia BA, configured by `ReconstructionEstimatorOptions`.

The module is a testing ground. Every stage is exposed to Python on its own, so that new formulations, solvers and upgrades can be compared with the same data and diagnostics.

!!! warning "Experimental"
    The interface can still change. The calibrated pipeline (`POSE`, `POSE_ROT`, `ROSE` or `EXPOSE` with `CALIBRATED_RAYS` and the linear upgrade) works end to end. `RPOSE` runs in the stage-wise API only. Options marked *not implemented* below fail with a clear message instead of silently falling back.

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
ok, metric, err = pt.sfm.UpgradeToMetricCalibrated(problem, ref.cameras, ref.points)
```

`RunProjectiveStages(problem, ifo)` runs all restarts of the OSE stage and the refinement and returns `(ok, best, diagnostics, err)`. `ProjectiveRmsError(problem, cameras, points)` gives the RMS reprojection error in observation units (normalized image coordinates for `CALIBRATED_RAYS`). Functions that take points accept 3 × N Cartesian or 4 × N homogeneous arrays.

## Options and status

| Option | Values | Status |
|---|---|---|
| `formulation` | `POSE`, `POSE_ROT`, `ROSE`, `EXPOSE` | implemented |
| | `RPOSE` | OSE stage only (radial 2×4 cameras; no refinement or upgrade) |
| `observation_normalization` | `CALIBRATED_RAYS` | implemented (needed by `POSE_ROT` and the upgrade) |
| | `PIXEL_ISOTROPIC` | problem building only; no uncalibrated upgrade yet |
| `camera_init` | `NORMAL`, `UNIFORM`, `UNIT_CIRCLE` | implemented |
| `varpro.linear_solver_type` | `DENSE_CHOLESKY` | implemented (CPU, Eigen) |
| | `CUDA_DENSE_CHOLESKY` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `POWER_SERIES` | implemented (CPU, matrix-free; `power_series_max_iterations`, `power_series_tolerance`) |
| | `CUDA_POWER_SERIES` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `SPARSE_CHOLESKY` | not implemented |
| `varpro.robust_loss_type` | `NONE` | implemented |
| | `CAUCHY` | implemented (IRLS, `robust_loss_scale` in observation units) |
| `projective_refinement.max_num_iterations` | `0` skips, `> 0` runs | implemented (`function_tolerance`, optional Huber `robust_loss_width`) |
| `varpro.num_threads` | `0` uses `num_threads` of `InitFreeOptions` | implemented; results do not depend on the thread count |
| `rotation_weighting` | `SCALAR`, `TWO_VIEW_INFORMATION` | implemented (`POSE_ROT` only) |
| `restart_selection` | `REPROJECTION_ERROR`, `OSE_COST` | implemented |
| `metric_upgrade` | `LINEAR` | implemented for calibrated rays |
| | `LINEAR_AND_NONLINEAR` | nonlinear part not implemented (warning) |

### Formulations

With `m` the normalized observation, `P` the 3 × 4 camera, `X̃` the homogeneous point and `α` the `alpha` option:

- **`POSE`**: `(1−α)‖P₁₂X̃ − (p₃ᵀX̃) m‖² + α‖P₁₂X̃ − m‖²` (Hong & Zach).
- **`POSE_ROT`**: `POSE` plus the relative rotation terms `‖A_a A_bᵀ − R_ab‖²` of the view graph (Olsson & Nilsson).
- **`ROSE`**: the OSE term plus `α (p₃ᵀX̃ − 1)²`, a depth regularizer. The OSE minimum is strongly biased (about 11 px RMS on the 1 px test scene), so use it together with projective refinement.
- **`EXPOSE`**: the third residual is `√α · √(exp(−s₀)/2) · (s − 1 − s₀)`, with `s = m̃ᵀPX̃ / |m̃|` and `m̃ = [m; 1]`. The state `s₀` starts at `|m̃|` and, after `expose_warmup_iterations`, follows the current depth, as in InitFreeBA. Cameras with `det(A) < 0` are flipped first. This gives the smallest OSE bias of the formulations here (1.29 px against 1.40 px for `POSE` on the 1 px test scene).
- **`RPOSE`**: the radial version, `(1−α)(m⊥ᵀP₁₂X̃)²/|m|² + α‖P₁₂X̃ − m‖²` with `m⊥ = (−m₂, m₁)`. Only the first two camera rows are determined. The estimator rejects it, because no radial metric upgrade exists yet. The restart diagnostics report `ose_rms_error = −1`.

With `robust_loss_type = CAUCHY`, every VarPro iteration recomputes the IRLS weights `w = 1/(1 + ‖r‖²/c²)` of the observation residuals (`c = robust_loss_scale`), and the cost is `Σ ½c² log(1 + ‖r‖²/c²)`. The rotation terms are not robustified. With 10 % of the observations displaced by up to 150 px, the inlier median error after the OSE stage falls from 12.3 px to 1.1 px (`c` = 5 px). For robust or state-dependent formulations, the cost history is not monotone, because the weights change between iterations.

### Rotation weighting

`POSE_ROT` weights each relative rotation term with the scalar `rotation_weight`. With `rotation_weighting = TWO_VIEW_INFORMATION`, each pair instead gets the 9 × 9 weight `W_kl` of Olsson & Nilsson (Eq. 4), computed from the problem's own correspondences of the pair:

- The translation comes from `TwoViewInfo.position_2` or, if that is zero, from the epipolar constraint and cheirality.
- The points are triangulated with the midpoint method.
- The 3 × 3 rotation information `H` is formed with the translation and the points marginalized out.
- In the basis made of the tangent directions `[e_i]× R_ab` and the symmetric normal directions `S R_ab`, `W = diag(H/2, rotation_normal_weight · I₆)`. A tangent step `[ω]× R_ab` then costs `ωᵀHω`, the Gauss-Newton increase of the two-view reprojection error.

Pairs with fewer than `min_rotation_information_points` usable points keep the scalar weight. `pt.sfm.ComputeTwoViewRotationWeight(m_a, m_b, R_ab, t_ab)` exposes the computation.

Compared with the InitFreeBA reference code, `H` here is in observation units (not pixels) and has no extra 1/3 factor. The Jacobian assembly of the reference code also leaves some Jacobian buffers unfilled, so its weights cannot be reproduced exactly. On the synthetic test scenes (exact view-graph rotations, projective refinement enabled), the final rotation accuracy is the same as with scalar weights. The weighting matters most when the relative rotations are noisy and differ in quality, which those scenes do not test.

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

## Performance

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
- **Projective stage:** extend `RunProjectiveStages` (one pass per restart) and record the outcome in `InitFreeRestartDiagnostics`.
- **Upgrade:** add a stage in `InitFreeReconstructionEstimator::Estimate` and record its outcome in `InitFreeDiagnostics`.

## References

- J. H. Hong, C. Zach. *pOSE: Pseudo Object Space Error for Initialization-Free Bundle Adjustment.* CVPR 2018.
- J. H. Hong, C. Zach, A. Fitzgibbon. *Revisiting the Variable Projection Method for Separable Nonlinear Least Squares Problems.* CVPR 2017.
- S. Weber, J. H. Hong, D. Cremers. *Power Variable Projection for Initialization-Free Large-Scale Bundle Adjustment.* ECCV 2024.
- C. Olsson, A. Nilsson. *Towards Initialization-free Calibrated Bundle Adjustment.* [arXiv:2506.23808](https://arxiv.org/abs/2506.23808).
- S. Weber, M. de Mayo, J. H. Hong, C. Olsson, D. Cremers, R. Clark. *Initialization-Free Bundle Adjustment Revisited: A Controlled Experimental Study.* [arXiv:2608.18028](https://arxiv.org/abs/2608.18028). Code: [InitFreeBA](https://github.com/simonwebertum/InitFreeBA).
