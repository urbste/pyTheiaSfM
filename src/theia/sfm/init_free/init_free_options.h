// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_INIT_FREE_OPTIONS_H_
#define THEIA_SFM_INIT_FREE_INIT_FREE_OPTIONS_H_

// Options for initialization-free structure from motion (experimental).
//
// The pipeline follows the object space error (OSE) family of methods:
//   1. Build an OseProblem from the tracks (and, for rotation-aware
//      formulations, from the relative rotations of the view graph).
//   2. Sample random cameras and minimize an OSE objective with variable
//      projection (VarPro). The result is a projective reconstruction.
//   3. (Optional) Refine the projective reconstruction.
//   4. Upgrade to a metric reconstruction.
//   5. Hand over to the usual metric bundle adjustment.
//
// References:
//   pOSE:      Hong, Zach. "pOSE: Pseudo Object Space Error for
//              Initialization-Free Bundle Adjustment." CVPR 2018.
//   rOSE /     Iglesias, Olsson. "Radial distortion invariant factorization for
//   RpOSE:     structure from motion." ICCV 2021.
//   expOSE:    Iglesias, Nilsson, Olsson. "expOSE: Accurate Initialization-Free
//              Projective Factorization using Exponential Regularization."
//              CVPR 2023.
//   pOSE+rot:  Olsson, Nilsson. "Towards Initialization-free Calibrated Bundle
//              Adjustment." arXiv:2506.23808.
//   VarPro:    Hong, Zach, Fitzgibbon. "Revisiting the Variable Projection
//              Method for Separable Nonlinear Least Squares Problems." CVPR 2017.
//   PoVar:     Weber, Hong, Cremers. "Power Variable Projection for
//              Initialization-Free Large-Scale Bundle Adjustment." ECCV 2024.
//   Study:     Weber, de Mayo, Hong, Olsson, Cremers, Clark.
//              "Initialization-Free Bundle Adjustment Revisited: A Controlled
//              Experimental Study." arXiv:2608.18028.
//              Reference code: https://github.com/simonwebertum/InitFreeBA

namespace theia {

// The OSE objective minimized in the first stage.
// Residuals per observation m (X_h = [X; 1], p_k^T the rows of P):
//   OSE part (all but RPOSE):  sqrt(1 - alpha) * (p_{1:2}^T X_h - m p_3^T X_h)
//   POSE / POSE_ROT:  + sqrt(alpha) * (p_{1:2}^T X_h - m)         (affine)
//   ROSE:             + sqrt(alpha) * (p_3^T X_h - 1)             (depth)
//   EXPOSE:           + exponential depth regularizer
//                       alpha / 2 * exp(-[m; 1]^T P X_h / |[m; 1]|), minimized
//                       through its second-order expansion around the current
//                       depths (re-linearized every iteration after
//                       expose_warmup_iterations).
//   RPOSE (radial):   sqrt(1 - alpha) * m_perp^T p_{1:2}^T X_h / |m|
//                     + sqrt(alpha) * (p_{1:2}^T X_h - m); only the first two
//                     rows of P are estimated (the third row keeps its initial
//                     value), so no projective refinement or metric upgrade.
enum class OseFormulationType {
  POSE = 0,      // Pseudo object space error (affine regularizer).
  ROSE = 1,      // OSE with a depth regularizer (p_3^T X_h ~ 1).
  RPOSE = 2,     // Radial pOSE (1D radial cameras).
  EXPOSE = 3,    // Exponential regularization.
  POSE_ROT = 4,  // pOSE + pairwise relative rotation terms (calibrated).
};

// How the pairwise rotation terms of POSE_ROT are weighted.
enum class RotationWeightingType {
  // ||sqrt(rotation_weight) * vec(A_a A_b^T - R_ab)||^2.
  SCALAR = 0,
  // The 9x9 weight W_ab of Olsson & Nilsson: on the tangent space of SO(3) at
  // R_ab it is half the two-view reprojection information of the rotation
  // (translation and points marginalized, in the units of the observations);
  // on the 6 normal directions it is rotation_normal_weight. Scaled by
  // rotation_weight. Self terms keep the scalar weight.
  TWO_VIEW_INFORMATION = 1,
};

// How image observations are expressed inside the OSE problem.
enum class OseObservationNormalization {
  // Undistorted normalized image coordinates (K^-1 x) from the view's camera
  // intrinsics. Required by rotation-aware formulations and by the calibrated
  // metric upgrade.
  CALIBRATED_RAYS = 0,
  // Pixels shifted to the image center and scaled by half the larger image
  // side. For uncalibrated experiments.
  PIXEL_ISOTROPIC = 1,
};

// Distribution of the random starting cameras.
enum class InitFreeCameraInitType {
  NORMAL = 0,       // All 12 entries of P ~ N(0, 1).
  UNIFORM = 1,      // All 12 entries of P ~ U(-1, 1).
  UNIT_CIRCLE = 2,  // Cameras on the unit circle, looking at the origin.
};

// Linear solver for the damped reduced camera system of each VarPro step.
enum class ReducedCameraSolverType {
  DENSE_CHOLESKY = 0,       // Eigen LLT on the dense Schur complement (CPU).
  CUDA_DENSE_CHOLESKY = 1,  // cuSOLVER potrf/potrs. Needs a build with
                            // -DTHEIA_WITH_CUDA_SOLVERS=ON.
  SPARSE_CHOLESKY = 2,      // Not implemented yet.
  POWER_SERIES = 3,         // Matrix-free truncated power series of the
                            // Schur complement inverse (PoBA / PoVar).
  CUDA_POWER_SERIES = 4,    // POWER_SERIES with cuSPARSE on the GPU. Needs
                            // -DTHEIA_WITH_CUDA_SOLVERS=ON.
};

// How the best of several random restarts is chosen.
enum class InitFreeRestartSelection {
  // Lowest final OSE cost.
  OSE_COST = 0,
  // Lowest RMS reprojection error of the projective reconstruction (after
  // projective refinement if it is enabled). The study in arXiv:2608.18028
  // found this a better predictor of a correct metric result.
  REPROJECTION_ERROR = 1,
};

// Robust loss on the residual vector r of each observation, applied by
// iteratively reweighted least squares inside VarPro: the weights
// 1 / (1 + |r|^2 / c^2) are recomputed from the current cameras and points at
// the start of every iteration, and steps are accepted on the robust cost
// sum 0.5 c^2 log(1 + |r|^2 / c^2) (as in InitFreeBA). Rotation terms are not
// robustified.
enum class VarProRobustLossType {
  NONE = 0,
  CAUCHY = 1,
};

enum class MetricUpgradeType {
  // Keep the projective reconstruction. The reconstruction estimator fails in
  // this mode; use the stage-wise Python API to inspect projective results.
  NONE = 0,
  // Linear absolute-dual-quadric estimate. With CALIBRATED_RAYS the intrinsics
  // are known (K = I); the uncalibrated variant is not implemented yet.
  LINEAR = 1,
  // Linear estimate followed by nonlinear refinement (refinement not
  // implemented yet; behaves like LINEAR with a warning).
  LINEAR_AND_NONLINEAR = 2,
};

struct VarProOptions {
  int max_num_iterations = 200;

  // Stop when the relative cost decrease of an accepted step is below this.
  double function_tolerance = 1e-10;

  // Stop when the camera update is small relative to the camera parameters.
  double parameter_tolerance = 1e-10;

  // Levenberg damping, applied to the camera block only. The points are
  // eliminated exactly, which is what separates VarPro from joint LM.
  double initial_lambda = 1e-2;
  double min_lambda = 1e-12;
  double max_lambda = 1e12;
  double lambda_decrease_factor = 10.0;
  double lambda_increase_factor = 10.0;

  ReducedCameraSolverType linear_solver_type =
      ReducedCameraSolverType::DENSE_CHOLESKY;

  // POWER_SERIES: maximum number of series terms, and the relative size of a
  // term below which the series is truncated.
  int power_series_max_iterations = 20;
  double power_series_tolerance = 1e-6;

  VarProRobustLossType robust_loss_type = VarProRobustLossType::NONE;
  // Cauchy scale c in residual units (normalized image coordinates for
  // CALIBRATED_RAYS, so ~ pixels / focal length).
  double robust_loss_scale = 0.01;

  // Threads for building the reduced system, evaluating the cost and the
  // CPU power series. <= 0 uses InitFreeOptions::num_threads.
  int num_threads = 0;

  // Ridge added to each 3x3 point normal matrix before inversion.
  double point_ridge = 1e-12;

  bool verbose = false;
};

struct ProjectiveRefinementOptions {
  // 0 disables the stage.
  int max_num_iterations = 0;
  double function_tolerance = 1e-10;
  // Huber width in the units of the observations (normalized image
  // coordinates for CALIBRATED_RAYS, so ~ pixels / focal length). <= 0 means
  // plain least squares.
  double robust_loss_width = 0.0;
  bool verbose = false;
};

struct InitFreeOptions {
  OseFormulationType formulation = OseFormulationType::POSE_ROT;

  // Weight of the regularizer: (1 - alpha) * OSE + alpha * regularizer. This
  // is eta in Hong & Zach and alpha in the study (arXiv:2608.18028).
  double alpha = 0.05;

  // Weight beta of the relative rotation terms (POSE_ROT only).
  double rotation_weight = 1.0;

  RotationWeightingType rotation_weighting = RotationWeightingType::SCALAR;

  // TWO_VIEW_INFORMATION: weight of the 6 directions normal to SO(3).
  double rotation_normal_weight = 1.0;

  // TWO_VIEW_INFORMATION: pairs with fewer common tracks in the problem keep
  // the scalar weight.
  int min_rotation_information_points = 10;

  // EXPOSE: iterations with the regularizer expanded around unit depths
  // before it is re-linearized around the current depths (InitFreeBA: 5).
  int expose_warmup_iterations = 5;

  // Add R_i R_i^T = I terms for every camera (POSE_ROT only), as in
  // Olsson & Nilsson.
  bool include_self_rotation_terms = true;

  OseObservationNormalization observation_normalization =
      OseObservationNormalization::CALIBRATED_RAYS;

  // Tracks with fewer observations are dropped ("minimal-obs" in the study).
  int min_track_length = 3;

  // Views with fewer observations (after track filtering) are dropped.
  int min_observations_per_view = 8;

  InitFreeCameraInitType camera_init = InitFreeCameraInitType::UNIT_CIRCLE;

  // Each restart samples new random cameras; restart_selection picks the
  // result.
  int num_random_restarts = 1;

  // Seed for the camera sampler. Restart k uses seed + k.
  unsigned int seed = 42;

  VarProOptions varpro;

  // Stage 2: projective bundle adjustment of every restart's OSE solution.
  ProjectiveRefinementOptions projective_refinement;

  InitFreeRestartSelection restart_selection =
      InitFreeRestartSelection::REPROJECTION_ERROR;

  // Threads for VarPro (unless varpro.num_threads > 0) and projective
  // refinement. <= 0 uses ReconstructionEstimatorOptions::num_threads inside
  // the estimator, else 1.
  int num_threads = 0;

  MetricUpgradeType metric_upgrade = MetricUpgradeType::LINEAR;

  // Run the standard metric bundle adjustment (configured by the
  // ReconstructionEstimatorOptions) after the metric upgrade.
  bool run_metric_bundle_adjustment = true;
};

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_INIT_FREE_OPTIONS_H_
