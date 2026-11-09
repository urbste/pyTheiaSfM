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

  // 0.5 * sum of squared residuals.
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

// Solves every point exactly for fixed cameras and returns the cost
// 0.5 * ||r||^2 (including rotation terms if the formulation uses them).
double EvaluateOseCost(const OseProblem& problem,
                       const OseFormulation& formulation,
                       const OseCameraVector& cameras,
                       double point_ridge,
                       Eigen::Matrix3Xd* points);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_VARPRO_SOLVER_H_
