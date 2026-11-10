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
  int refinement_iterations = 0;
  double refinement_time_seconds = 0.0;
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

// Runs the projective stages for options.num_random_restarts random starts:
// OSE with VarPro, then (if enabled) projective refinement. Returns the best
// restart according to options.restart_selection.
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
