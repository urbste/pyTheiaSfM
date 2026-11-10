# Initialization-free SfM (experimental)

`ReconstructionEstimatorType.INIT_FREE` estimates cameras and points **jointly from random starts**. It does not use rotation averaging, position estimation or incremental registration. It follows the object space error (OSE) line of work:

1. **Problem.** Tracks (and, for `POSE_ROT`, the relative rotations of the view graph) become an `OseProblem`.
2. **OSE stage.** Random cameras are sampled, and an OSE objective is minimized with variable projection (VarPro). The result is a projective reconstruction.
3. **Projective refinement** *(optional)*. Projective bundle adjustment with Ceres (cameras and homogeneous points on unit spheres) removes the bias that the α-weighted affine term of pOSE leaves in the OSE minimum. With several restarts, the winner is chosen by the reprojection RMS after refinement (`restart_selection = REPROJECTION_ERROR`, the default) or by the OSE cost.
4. **Metric upgrade.** A linear absolute-dual-quadric estimate for known intrinsics.
5. **Metric bundle adjustment.** The standard pyTheia BA, configured by `ReconstructionEstimatorOptions`.

The module is a testing ground. Every stage is exposed to Python on its own, so that new formulations, solvers and upgrades can be compared with the same data and diagnostics.

!!! warning "Experimental"
    The interface can still change. The calibrated pipeline (`POSE` / `POSE_ROT` with `CALIBRATED_RAYS` and the linear upgrade) works end to end. Options marked *not implemented* below fail with a clear message instead of silently falling back.

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
| `formulation` | `POSE`, `POSE_ROT` | implemented |
| | `ROSE`, `RPOSE`, `EXPOSE` | not implemented |
| `observation_normalization` | `CALIBRATED_RAYS` | implemented (needed by `POSE_ROT` and the upgrade) |
| | `PIXEL_ISOTROPIC` | problem building only; no uncalibrated upgrade yet |
| `camera_init` | `NORMAL`, `UNIFORM`, `UNIT_CIRCLE` | implemented |
| `varpro.linear_solver_type` | `DENSE_CHOLESKY` | implemented (CPU, Eigen) |
| | `CUDA_DENSE_CHOLESKY` | implemented, needs `-DTHEIA_WITH_CUDA_SOLVERS=ON` |
| | `POWER_SERIES` | implemented (CPU, matrix-free; `power_series_max_iterations`, `power_series_tolerance`) |
| | `SPARSE_CHOLESKY` | not implemented |
| `varpro.robust_loss_type` | `NONE` | implemented |
| | `CAUCHY` | not implemented |
| `projective_refinement.max_num_iterations` | `0` skips, `> 0` runs | implemented (`function_tolerance`, optional Huber `robust_loss_width`) |
| `restart_selection` | `REPROJECTION_ERROR`, `OSE_COST` | implemented |
| `metric_upgrade` | `LINEAR` | implemented for calibrated rays |
| | `LINEAR_AND_NONLINEAR` | nonlinear part not implemented (warning) |

`POSE_ROT` currently weights each relative rotation term with a scalar (`rotation_weight`). The 9×9 two-view weighting `W_kl` of Olsson & Nilsson is still open.

## CUDA backend

The reduced camera system of each VarPro step has the same structure as the Schur complement in bundle adjustment. `CUDA_DENSE_CHOLESKY` factorizes it with cuSOLVER. It uses the CUDA host libraries only, so no CUDA compiler is needed:

```bash
CMAKE_ARGS="-DTHEIA_WITH_CUDA_SOLVERS=ON" python setup.py bdist_wheel
python -c "import pytheia as pt; print(pt.sfm.CudaReducedCameraSolverAvailable())"
```

The option is off by default so that the published wheels stay portable. The Schur complement is still formed on the CPU, so the GPU helps once the O(n³) factorization dominates (several hundred cameras and more).

## Power series solver

`POWER_SERIES` never forms the reduced camera matrix. It writes the damped system as `S + λI = D_λ − M`, with `D_λ` the damped 12 × 12 camera blocks and `M = WᵀV⁻¹W − O`, and sums the Neumann series `Σ_k (D_λ⁻¹M)^k D_λ⁻¹ b` (PoBA / PoVar, Weber et al.). Each term costs one pass over the observations and rotation terms, so memory and time per term are linear in the problem size. For VarPro the series converges for every `λ > 0`: `WᵀV⁻¹W ⪯ D_obs`, and the pairwise rotation blocks satisfy `−D_rot ⪯ O ⪯ D_rot`. Truncating the series gives an inexact LM step. With 20 terms, it reaches the dense-solver cost to 1e-8 relative, in more but cheaper iterations: on a synthetic 300-camera scene (720k observations), VarPro takes 21 s instead of 60 s with `DENSE_CHOLESKY` (single-threaded CPU).

## Adding a method

- **Formulation:** implement `OseFormulation` (`ObservationModel`, `CameraJacobian`) in `src/theia/sfm/init_free/ose_formulation.cc`, add an enum value and a case in `CreateOseFormulation`. Every solver backend then works with it.
- **Linear solver:** implement `ReducedCameraSolver::Solve` on the block system `ReducedCameraSystem` (dense via `FormDenseSchurComplement`, or matrix-free from the `U`, `W`, `V⁻¹` blocks) and register it in `CreateReducedCameraSolver`.
- **Projective stage:** extend `RunProjectiveStages` (one pass per restart) and record the outcome in `InitFreeRestartDiagnostics`.
- **Upgrade:** add a stage in `InitFreeReconstructionEstimator::Estimate` and record its outcome in `InitFreeDiagnostics`.

## References

- J. H. Hong, C. Zach. *pOSE: Pseudo Object Space Error for Initialization-Free Bundle Adjustment.* CVPR 2018.
- J. H. Hong, C. Zach, A. Fitzgibbon. *Revisiting the Variable Projection Method for Separable Nonlinear Least Squares Problems.* CVPR 2017.
- S. Weber, J. H. Hong, D. Cremers. *Power Variable Projection for Initialization-Free Large-Scale Bundle Adjustment.* ECCV 2024.
- C. Olsson, A. Nilsson. *Towards Initialization-free Calibrated Bundle Adjustment.* [arXiv:2506.23808](https://arxiv.org/abs/2506.23808).
- S. Weber, M. de Mayo, J. H. Hong, C. Olsson, D. Cremers, R. Clark. *Initialization-Free Bundle Adjustment Revisited: A Controlled Experimental Study.* [arXiv:2608.18028](https://arxiv.org/abs/2608.18028). Code: [InitFreeBA](https://github.com/simonwebertum/InitFreeBA).
