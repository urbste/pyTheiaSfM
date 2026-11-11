// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_INIT_FREE_RECONSTRUCTION_ESTIMATOR_H_
#define THEIA_SFM_INIT_FREE_INIT_FREE_RECONSTRUCTION_ESTIMATOR_H_

#include <string>
#include <vector>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_problem.h"
#include "theia/sfm/init_free/varpro_solver.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"

namespace theia {

struct InitFreeRestartDiagnostics {
  unsigned int seed = 0;

  // OSE stage.
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int num_iterations = 0;
  bool converged = false;
  double solve_time_seconds = 0.0;
  std::string message;
  // RMS reprojection error of the OSE solution (observation units); -1 for
  // radial formulations.
  double ose_rms_error = 0.0;

  // Projective refinement (-1 / 0 when the stage is disabled).
  double refined_rms_error = -1.0;
  // With outlier rejection: removed observations and the RMS of
  // min(error, outlier_threshold) over all observations (the ranking score).
  int refined_num_outlier_observations = 0;
  double refined_truncated_rms_error = -1.0;
  int refinement_iterations = 0;
  double refinement_time_seconds = 0.0;

  // Metric upgrade of this restart. Restarts are upgraded in ranked order until
  // one passes, so restarts after the used one may have been attempted too.
  bool upgrade_attempted = false;
  // The upgrade succeeded and passed the validity checks of InitFreeOptions.
  bool upgrade_success = false;
  // Valid only if the upgrade succeeded (otherwise 0).
  double upgrade_fraction_in_front = 0.0;
  double upgrade_mean_orthogonality_error = 0.0;
  // MetricUpgradeResult::conditioning (valid only if the upgrade succeeded).
  double upgrade_conditioning = 0.0;
  // Cameras dropped by InitFreeOptions::max_camera_orthogonality_error.
  int upgrade_num_dropped_cameras = 0;
  // Median angle between the upgraded relative rotations and the two-view
  // relative rotations (InitFreeOptions::max_relative_rotation_error_deg);
  // -1 if not computed.
  double upgrade_relative_rotation_error_deg = -1.0;
  // Median angle between the upgraded and the two-view translation directions
  // (InitFreeOptions::max_translation_direction_error_deg); -1 if not
  // computed.
  double upgrade_translation_direction_error_deg = -1.0;
  // The upgrade error, or the failed check ("rejected: ..."), or "ok".
  std::string upgrade_message;
  // InitFreeOptions::upgrade_unrefined_fallback: outcome of upgrading the OSE
  // solution of this restart without projective refinement.
  bool unrefined_upgrade_attempted = false;
  bool unrefined_upgrade_success = false;
  std::string unrefined_upgrade_message;
};

// Per-run diagnostics. The study in arXiv:2608.18028 shows that a low OSE cost
// does not guarantee a correct metric result, so all stages are recorded.
struct InitFreeDiagnostics {
  int num_cameras = 0;
  int num_points = 0;
  int num_observations = 0;
  int num_rotation_terms = 0;

  std::vector<InitFreeRestartDiagnostics> restarts;
  int best_restart = -1;

  bool metric_upgrade_success = false;
  // The used restart was upgraded from its unrefined OSE solution
  // (InitFreeOptions::upgrade_unrefined_fallback).
  bool used_unrefined_fallback = false;
  // Views left unestimated because their upgraded camera was broken
  // (InitFreeOptions::max_camera_orthogonality_error).
  std::vector<ViewId> dropped_view_ids;
  double fraction_in_front = 0.0;
  double mean_orthogonality_error = 0.0;
  // Before the nonlinear refinement (equal to the above without it).
  double linear_mean_orthogonality_error = 0.0;
  // Estimated focal lengths in pixels (PIXEL_ISOTROPIC only), in the order of
  // the OSE problem cameras.
  std::vector<double> focal_lengths;

  double problem_build_time = 0.0;
  double ose_time = 0.0;  // OSE and projective refinement of all restarts
  double metric_upgrade_time = 0.0;
  double bundle_adjustment_time = 0.0;
};

// A projective reconstruction in the numbering of an OseProblem.
struct ProjectiveReconstruction {
  OseCameraVector cameras;
  Eigen::Matrix4Xd points;  // homogeneous
};

// The projective reconstruction of one restart.
struct RankedProjectiveReconstruction {
  int restart = -1;  // index into InitFreeDiagnostics::restarts
  ProjectiveReconstruction reconstruction;
  // The OSE solution before projective refinement (empty if the refinement is
  // disabled, then `reconstruction` is the OSE solution).
  ProjectiveReconstruction unrefined;
  // Observations kept by the outlier rejection of the projective refinement
  // (ProjectiveRefinementResult::observation_inlier); empty if the rejection
  // is disabled or the refinement did not run.
  std::vector<bool> observation_inlier;
};

// Runs the projective stages for options.num_random_restarts random starts:
// OSE with VarPro, then (if enabled) projective refinement. The reconstructions
// of all restarts that produced a finite score are returned in ranked order
// (best first) according to options.restart_selection; ties keep the restart
// order. diagnostics->best_restart is the first ranked restart.
bool RunProjectiveStagesRanked(
    const OseProblem& problem,
    const InitFreeOptions& options,
    std::vector<RankedProjectiveReconstruction>* ranked,
    InitFreeDiagnostics* diagnostics,
    std::string* error);

// Like RunProjectiveStagesRanked, but returns only the best restart.
bool RunProjectiveStages(const OseProblem& problem,
                         const InitFreeOptions& options,
                         ProjectiveReconstruction* best,
                         InitFreeDiagnostics* diagnostics,
                         std::string* error);

// Initialization-free SfM (experimental). Uses no rotation averaging, position
// estimation or incremental registration: the cameras and points come from an
// OSE objective minimized from random starts, followed by a metric upgrade
// and the standard metric bundle adjustment. See init_free_options.h.
class InitFreeReconstructionEstimator : public ReconstructionEstimator {
 public:
  explicit InitFreeReconstructionEstimator(
      const ReconstructionEstimatorOptions& options);

  ReconstructionEstimatorSummary Estimate(
      ViewGraph* view_graph, Reconstruction* reconstruction) override;

  const InitFreeDiagnostics& Diagnostics() const { return diagnostics_; }

 private:
  ReconstructionEstimatorOptions options_;
  InitFreeDiagnostics diagnostics_;
};

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_INIT_FREE_RECONSTRUCTION_ESTIMATOR_H_
