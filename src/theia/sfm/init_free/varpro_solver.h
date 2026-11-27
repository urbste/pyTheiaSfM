// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_VARPRO_SOLVER_H_
#define THEIA_SFM_INIT_FREE_VARPRO_SOLVER_H_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_formulation.h"
#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

// Result of the OSE stage: projective cameras and Cartesian points in the
// numbering of the OseProblem.
struct OseSolution {
  OseCameraVector cameras;
  Eigen::Matrix3Xd points;

  // 0.5 * sum of squared residuals, or the robust cost with a robust loss.
  // For formulations that are re-linearized during the solve (expOSE) and for
  // robust losses, this is the cost of the current least-squares model, which
  // changes when the model is updated; cost_history is then not monotone.
  double initial_cost = 0.0;
  double final_cost = 0.0;
  std::vector<double> cost_history;  // cost after every accepted step

  int num_iterations = 0;
  bool converged = false;
  double solve_time_seconds = 0.0;
  std::string message;
};

// Minimizes the OSE objective with variable projection (Hong et al., CVPR
// 2017): for fixed cameras the points are solved exactly, and the cameras take
// Levenberg-Marquardt steps on the reduced problem using the RW2 Jacobian
// approximation. Damping is applied to the cameras only.
//
// Returns false (with `solution->message`) on invalid input or an unsupported
// option. Hitting the iteration limit is not a failure; check `converged`.
bool SolveOseVarPro(const OseProblem& problem,
                    const OseFormulation& formulation,
                    const VarProOptions& options,
                    const OseCameraVector& initial_cameras,
                    OseSolution* solution);

// DOSE with the depth-weight continuation of arXiv:2611.31516: for every
// stage s of options.dose_depth_weight_schedule, runs SolveOseVarPro with
// CreateDoseFormulation(options.dose_depth_weight * schedule[s],
// options.dose_depth_huber_width, s >= options.dose_robust_from_stage) and
// max_num_iterations = options.dose_iterations_per_stage, starting from the
// cameras of the previous stage. `varpro` supplies all other solver options.
// The solution holds the last stage's cameras, points and final cost, the
// first stage's initial cost, the concatenated cost history, the total number
// of iterations and run time; converged and message are those of the last
// stage. Returns false if a stage fails or the schedule is empty.
bool SolveDoseVarPro(const OseProblem& problem,
                     const InitFreeOptions& options,
                     const VarProOptions& varpro,
                     const OseCameraVector& initial_cameras,
                     OseSolution* solution);

// Solves every point exactly for fixed cameras and returns the cost
// 0.5 * ||r||^2 (including rotation terms if the formulation uses them), with
// unit weights and the formulation's initial state.
double EvaluateOseCost(const OseProblem& problem,
                       const OseFormulation& formulation,
                       const OseCameraVector& cameras,
                       double point_ridge,
                       Eigen::Matrix3Xd* points);

// options.varpro with num_threads resolved: varpro.num_threads if > 0, else
// options.num_threads if > 0, else 1.
VarProOptions ResolvedVarProOptions(const InitFreeOptions& options);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_VARPRO_SOLVER_H_
