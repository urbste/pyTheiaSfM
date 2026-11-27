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
//   dOSE:      Weber, de Mayo, Hong, Olsson, Cremers, Clark. "InitFree-SfM:
//              When Initialization-Free Bundle Adjustment Meets Learned
//              Structure-from-Motion." arXiv:2611.31516.

#include <vector>

#include "theia/sfm/bundle_adjustment/create_loss_function.h"

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
//                     value), so no projective refinement; the metric upgrade
//                     is the radial one (MetricUpgradeOptions::radial_cameras).
//   DOSE (calibrated, Euclidean cameras P = [R | t], y = R X + t, m~ = [m; 1],
//   d the depth prior of the observation, Feature::depth_prior()):
//                     (y_{1:2} - m y_3)
//                     + sqrt(lambda_d * w) * (m~^T y / (|m~|^2 d) - 1)
//                     with w the Huber IRLS weight of the depth residual.
//                     Rotations are updated on SO(3), translations
//                     additively; the first camera is held fixed (gauge),
//                     and the depth term fixes the scale. No projective
//                     refinement and no metric upgrade.
enum class OseFormulationType {
  POSE = 0,      // Pseudo object space error (affine regularizer).
  ROSE = 1,      // OSE with a depth regularizer (p_3^T X_h ~ 1).
  RPOSE = 2,     // Radial pOSE (1D radial cameras).
  EXPOSE = 3,    // Exponential regularization.
  POSE_ROT = 4,  // pOSE + pairwise relative rotation terms (calibrated).
  DOSE = 5,      // Depth-augmented OSE with Euclidean cameras (calibrated,
                 // needs depth priors).
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
  // Euclidean cameras [R | t]: R uniform on SO(3) (normalized Gaussian
  // quaternion), t ~ N(0, 0.5^2) per coordinate; camera 0 is [I | 0]
  // (arXiv:2611.31516). Intended for DOSE.
  EUCLIDEAN_RANDOM = 3,
};

// Linear solver for the damped reduced camera system of each VarPro step.
enum class ReducedCameraSolverType {
  DENSE_CHOLESKY = 0,       // Eigen LLT on the dense Schur complement (CPU).
  CUDA_DENSE_CHOLESKY = 1,  // cuSOLVER potrf/potrs. Needs a build with
                            // -DTHEIA_WITH_CUDA_SOLVERS=ON.
  SPARSE_CHOLESKY = 2,      // Eigen simplicial LLT (AMD) on the block-sparse
                            // Schur complement (CPU). For sequences and
                            // scenes where most camera pairs share no point.
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
  // are known (K = I); with PIXEL_ISOTROPIC one focal length per camera is
  // estimated (zero skew, unit aspect ratio, principal point at the image
  // center; Pollefeys et al. 2004).
  LINEAR = 1,
  // Linear estimate followed by nonlinear refinement of the rank-3 quadric
  // (and the focal lengths). See metric_upgrade.h.
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
  // Loss applied with robust_loss_width (ignored if the width is <= 0). Huber
  // keeps a linear influence of gross outliers; real tracks contain a few
  // percent of wrongly merged observations, for which a redescending loss
  // (CAUCHY, TUKEY) is more robust.
  LossFunctionType loss_function_type = LossFunctionType::HUBER;
  // Outlier rejection: after each solve, observations whose reprojection
  // error exceeds outlier_threshold (units of the observations) are removed
  // and the problem is solved again, for at most this many rounds (fewer if
  // no observation is removed). 0 disables it. Without it, wrongly merged
  // track observations can pull the projective reconstruction far from the
  // true one, because a projective camera (11 DOF) absorbs outliers that a
  // metric camera cannot.
  int outlier_rejection_rounds = 0;
  double outlier_threshold = 0.0;
  // Solve with one thread. The multithreaded Ceres evaluation sums in a
  // varying order; the differences (1e-16 relative) grow through the outlier
  // rejection, the restart ranking and the metric upgrade into different
  // results between runs (ETH3D office with SIFT: 0.76 to 1.5 deg). One
  // thread costs about a factor 2 in the refinement, which is a small part of
  // the run time.
  bool deterministic = true;
  bool verbose = false;
};

// Joint metric refinement after DOSE (see dose_refinement.h for the
// residuals). Weights are relative to the image residual in pixels.
struct DoseRefinementOptions {
  // 0 disables the stage (the DOSE solution then goes directly to the metric
  // bundle adjustment).
  int max_num_iterations = 100;
  double function_tolerance = 1e-10;
  double parameter_tolerance = 1e-8;
  double image_huber_width_px = 2.0;
  // Weight of the relative depth residual: 100 balances a 10 % depth error
  // against 1 px.
  double depth_weight = 100.0;
  double depth_huber_width = 0.1;
  bool optimize_depth_scales = true;
  // Normalize the image residual by alpha_i d_ij (Eq. 13 of the paper) instead
  // of d_ij. With alpha_i in the denominator, increasing alpha_i shrinks the
  // image residuals of camera i, which on real tracks with wrongly merged
  // observations drives the scales of outlier-heavy cameras up (ETH3D relief:
  // 28 mm median center error instead of 1.7 mm). Without it, alpha_i only
  // enters the depth residual.
  bool image_residual_uses_depth_scale = false;
  // Use the reprojection error f (y_{0,1} / y_2 - m) as the image residual
  // (for y_2 > 1e-6 d_ij; the object space form otherwise). The object space
  // form divided by the fixed d_ij (image_residual_uses_depth_scale = false)
  // decreases when the whole scene shrinks, which biases the scale (synthetic
  // scene with 5 % depth noise: 5 % too small); with alpha_i d_ij it has the
  // degeneracy described above. The reprojection error has neither. The
  // refinement starts from a DOSE solution with the points in front of the
  // cameras (see dose_check_before_refinement), so the division is safe.
  bool reprojection_image_residual = true;
  // Depth residual log(q^T y / (alpha_i d_ij)) instead of the relative
  // error (q^T y - alpha_i d_ij) / (alpha_i d_ij) of the paper. The relative
  // error saturates at -1 for large alpha_i, so a camera with inconsistent
  // depth priors can increase its scale at a bounded cost; the log ratio is
  // symmetric in over- and underestimation.
  bool log_ratio_depth_residual = true;
  double rotation_weight = 100.0;
  double translation_direction_weight = 100.0;
  // Paper: tau = 0.05, weight tau / |O| (cheirality_weight = 1).
  double cheirality_tau = 0.05;
  double cheirality_weight = 1.0;
  double scale_regularization_weight = 1.0;
  // Observations with a final reprojection error above this (pixels) are
  // marked as outliers; their tracks are left out of the metric bundle
  // adjustment. <= 0 disables it.
  double outlier_threshold_px = 5.0;
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

  // DOSE: weight lambda_d of the depth term (arXiv:2611.31516: 30).
  double dose_depth_weight = 30.0;
  // DOSE: Huber width of the relative depth residual m~^T y / (|m~|^2 d) - 1.
  // The paper does not state its value.
  double dose_depth_huber_width = 0.1;
  // DOSE: continuation of lambda_d. Stage s minimizes the objective with
  // dose_depth_weight * dose_depth_weight_schedule[s] for at most
  // dose_iterations_per_stage VarPro iterations, starting from the cameras of
  // stage s - 1 (paper: 5 %, 25 %, 100 %, 20 iterations each).
  std::vector<double> dose_depth_weight_schedule = {0.05, 0.25, 1.0};
  int dose_iterations_per_stage = 20;
  // DOSE: first stage with the Huber loss on the depth residual (IRLS,
  // weights updated every iteration); earlier stages are least squares.
  int dose_robust_from_stage = 1;
  // DOSE: joint metric refinement with per-image depth scales, run on every
  // restart before ranking (restarts are then ranked by its RMS reprojection
  // error).
  DoseRefinementOptions dose_refinement;
  // DOSE: the refinement pulls the solution towards the view-graph rotations
  // (pairwise rotation terms), so a wrong DOSE solution can be brought into
  // agreement with them and then pass the checks applied after the refinement
  // (ETH3D facade: accepted with 1 m error). If true, the checks of Estimate()
  // (min_fraction_in_front, max_mean_orthogonality_error,
  // max_relative_rotation_error_deg, max_translation_direction_error_deg, and
  // dropping of broken cameras) are applied to the DOSE solution before the
  // refinement. A restart that fails is not refined and is excluded from the
  // ranking; its reason is recorded in
  // InitFreeRestartDiagnostics::pre_refinement_check_message.
  bool dose_check_before_refinement = true;

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

  // Validity checks of the metric upgrade of each restart. The restarts are
  // ranked by restart_selection and upgraded in that order; the first restart
  // whose upgrade succeeds and passes both checks is used. If none does, the
  // estimator fails and writes no reconstruction. Ranking does not depend on
  // these checks.
  // Minimum fraction of the observations in front of the metric cameras.
  // Measured on ETH3D: correct restarts 0.998 to 1.0, wrong ones 0.58 to 0.98.
  double min_fraction_in_front = 0.9;
  // Maximum MetricUpgradeResult::mean_orthogonality_error (zero for an exactly
  // metric result). Compared with "!(error <= max)", so NaN fails.
  // Measured: correct restarts (after bundle adjustment, rotation error below
  // 0.3 deg) are at most 0.17 (ETH3D at 0.5 px noise, 20% outliers; 0.011 on
  // the 1 px synthetic scene); wrong restarts (rotation error above 25 deg)
  // are at least 0.63. The threshold sits between them.
  double max_mean_orthogonality_error = 0.3;
  // Minimum MetricUpgradeResult::conditioning: the fifth smallest singular
  // value of the self-calibration Jacobian over the largest (see
  // metric_upgrade.h). Small values mean an additional ambiguity of the
  // self-calibration (critical motions, e.g. varying focal lengths with all
  // cameras rotating about one axis), where the focal lengths are not unique.
  // 0 disables the check. Compared with "!(conditioning >= min)", so NaN fails.
  // Measured: well-posed scenes 0.018 to 0.56 (calibrated, PIXEL_ISOTROPIC
  // with varying and with shared focal length, RPOSE); critical pure
  // translation 1e-8 (noise free) to 5.7e-4 (2 px), with focal lengths off by
  // 20 to 60 %. The threshold sits in the gap, about 5 times above the largest
  // critical value and 6 times below the smallest well-posed one.
  double min_upgrade_conditioning = 3e-3;

  // If no refined restart passes the checks, upgrade the OSE solutions before
  // projective refinement, in the same order, with the same checks. The OSE
  // solution is biased towards an affine camera but is not pulled by outlier
  // observations like the refinement is; the metric bundle adjustment then
  // removes the bias. Has no effect without projective refinement.
  bool upgrade_unrefined_fallback = false;

  // Cameras whose own orthogonality error after the upgrade
  // (MetricUpgradeResult::camera_orthogonality_errors) exceeds this value are
  // dropped (left unestimated) instead of failing the restart, as long as at
  // most max_fraction_dropped_cameras of the cameras are dropped. The mean
  // orthogonality error and the fraction in front are then checked on the
  // remaining cameras. A camera with few observations can collapse during
  // projective refinement while all others are accurate (ETH3D relief and
  // meadow with SIFT: one camera at 2e4 to 2e5, all others below 0.11).
  // <= 0 disables dropping. Has no effect for radial cameras.
  double max_camera_orthogonality_error = 1.0;
  double max_fraction_dropped_cameras = 0.1;

  // Maximum median angle (degrees) between the relative rotations of the
  // upgraded cameras and the two-view relative rotations of the view graph
  // (the rotation terms of the problem; the check is skipped without them).
  // Independent of the projective fit: a wrong upgrade can reach a low
  // orthogonality error but does not reproduce the two-view rotations. 0
  // disables it.
  double max_relative_rotation_error_deg = 5.0;
  // The same for the direction of the camera centers: maximum median angle
  // between the upgraded and the two-view translation directions. Catches
  // upgrades with nearly right rotations but wrong centers. 0 disables it.
  double max_translation_direction_error_deg = 0.0;

  // Threads for VarPro (unless varpro.num_threads > 0) and projective
  // refinement. <= 0 uses ReconstructionEstimatorOptions::num_threads inside
  // the estimator, else 1.
  int num_threads = 0;

  MetricUpgradeType metric_upgrade = MetricUpgradeType::LINEAR;

  // PIXEL_ISOTROPIC only: estimate one focal length for all cameras in the
  // metric upgrade (all images from the same camera at the same zoom).
  bool shared_focal_length = false;

  // Run the standard metric bundle adjustment (configured by the
  // ReconstructionEstimatorOptions) after the metric upgrade.
  bool run_metric_bundle_adjustment = true;
};

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_INIT_FREE_OPTIONS_H_
