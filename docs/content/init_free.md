# Initialization-free SfM (experimental)

`ReconstructionEstimatorType.INIT_FREE` estimates cameras and points **jointly from random starts**. It does not use rotation averaging, position estimation or incremental registration. It follows the object space error (OSE) line of work:

1. **Problem.** Tracks (and, for `POSE_ROT`, the relative rotations of the view graph) become an `OseProblem`.
2. **OSE stage.** Random cameras are sampled, and an OSE objective is minimized with variable projection (VarPro). The result is a projective reconstruction.
3. **Projective refinement** *(optional)*. Projective bundle adjustment with Ceres (cameras and homogeneous points on unit spheres) removes the bias that the α-weighted affine term of pOSE leaves in the OSE minimum. With several restarts, the winner is chosen by the reprojection RMS after refinement (`restart_selection = REPROJECTION_ERROR`, the default) or by the OSE cost.
4. **Metric upgrade.** Self-calibration with the absolute dual quadric: known intrinsics for `CALIBRATED_RAYS`, one focal length per camera for `PIXEL_ISOTROPIC`. A linear estimate, optionally refined nonlinearly.
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
| `restart_selection` | `REPROJECTION_ERROR`, `OSE_COST` | implemented |
| `metric_upgrade` | `LINEAR`, `LINEAR_AND_NONLINEAR` | implemented for both normalizations |
| `shared_focal_length` | `False`, `True` | implemented (`PIXEL_ISOTROPIC` only) |

### Formulations

With `m` the normalized observation, `P` the 3 × 4 camera, `X̃` the homogeneous point and `α` the `alpha` option:

- **`POSE`**: `(1−α)‖P₁₂X̃ − (p₃ᵀX̃) m‖² + α‖P₁₂X̃ − m‖²` (Hong & Zach).
- **`POSE_ROT`**: `POSE` plus the relative rotation terms `‖A_a A_bᵀ − R_ab‖²` of the view graph (Olsson & Nilsson).
- **`ROSE`**: the OSE term plus `α (p₃ᵀX̃ − 1)²`, a depth regularizer. The OSE minimum is strongly biased (about 11 px RMS on the 1 px test scene), so use it together with projective refinement.
- **`EXPOSE`**: the third residual is `√α · √(exp(−s₀)/2) · (s − 1 − s₀)`, with `s = m̃ᵀPX̃ / |m̃|` and `m̃ = [m; 1]`. The state `s₀` starts at `|m̃|` and, after `expose_warmup_iterations`, follows the current depth, as in InitFreeBA. Cameras with `det(A) < 0` are flipped first. This gives the smallest OSE bias of the formulations here (1.29 px against 1.40 px for `POSE` on the 1 px test scene).
- **`RPOSE`**: the radial version, `(1−α)(m⊥ᵀP₁₂X̃)²/|m|² + α‖P₁₂X̃ − m‖²` with `m⊥ = (−m₂, m₁)`. Only the first two camera rows are determined. The estimator runs the radial metric upgrade (`radial_cameras`, see *Metric upgrade*) and skips projective refinement, which is not defined for radial cameras; the metric bundle adjustment then refines the full cameras. The restart diagnostics report `ose_rms_error = −1`.

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

**Radial cameras** (`radial_cameras = True`, used by `RPOSE`) keep only rows 0 and 1 of each camera, and the metric camera has the form `P₁₂H = s[R₁₂ | t₁₂]` with orthonormal rows in `R₁₂`. The linear step uses two equations per camera (equal diagonal and zero off-diagonal entry of `P₁₂ΩP₁₂ᵀ`), so at least five cameras are needed. The optional refinement fits `Ω = BBᵀ` to the same constraint, without priors. Two candidates, `H` and `H·diag(1, 1, −1, 1)`, are built: per camera, the sign of the first two rows is chosen so that the 2D directions agree with the observations, and the forward translation `t₃` (and, for `PIXEL_ISOTROPIC`, the focal length) is fit by least squares to `m(r₃ᵀX + t₃) = f(R₁₂X + t₁₂)`. The candidate with more points in front is kept. Measured with the default `α = 0.05`: on a noise-free 15-view scene, the linear radial upgrade is off by up to 13° (the RPOSE affine term biases the OSE solution) and the refinement by up to 1.3°, with all observations in front; end to end with 1 px noise and `LINEAR_AND_NONLINEAR`, the worst relative rotation error after bundle adjustment is 0.25°. For `PIXEL_ISOTROPIC`, the focal error on noise-free data is 16 % at `α = 0.05` and 0.9 % at `α = 0.001`, because the same bias shrinks as `α` does.

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
- **Projective stage:** extend `RunProjectiveStages` (one pass per restart) and record the outcome in `InitFreeRestartDiagnostics`.
- **Upgrade:** add a stage in `InitFreeReconstructionEstimator::Estimate` and record its outcome in `InitFreeDiagnostics`.

## References

- J. H. Hong, C. Zach. *pOSE: Pseudo Object Space Error for Initialization-Free Bundle Adjustment.* CVPR 2018.
- J. H. Hong, C. Zach, A. Fitzgibbon. *Revisiting the Variable Projection Method for Separable Nonlinear Least Squares Problems.* CVPR 2017.
- S. Weber, J. H. Hong, D. Cremers. *Power Variable Projection for Initialization-Free Large-Scale Bundle Adjustment.* ECCV 2024.
- C. Olsson, A. Nilsson. *Towards Initialization-free Calibrated Bundle Adjustment.* [arXiv:2506.23808](https://arxiv.org/abs/2506.23808).
- S. Weber, M. de Mayo, J. H. Hong, C. Olsson, D. Cremers, R. Clark. *Initialization-Free Bundle Adjustment Revisited: A Controlled Experimental Study.* [arXiv:2608.18028](https://arxiv.org/abs/2608.18028). Code: [InitFreeBA](https://github.com/simonwebertum/InitFreeBA).
