// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_PROJECTIVE_REFINEMENT_H_
#define THEIA_SFM_INIT_FREE_PROJECTIVE_REFINEMENT_H_

#include <Eigen/Core>
#include <string>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

struct ProjectiveRefinementResult {
  OseCameraVector cameras;  // unit Frobenius norm
  Eigen::Matrix4Xd points;  // homogeneous, unit norm

  // Root mean square reprojection error in the units of the problem's
  // observations (normalized image coordinates for CALIBRATED_RAYS).
  double initial_rms_error = 0.0;
  double final_rms_error = 0.0;

  int num_iterations = 0;
  bool converged = false;
  double solve_time_seconds = 0.0;
  std::string message;
};

// Projective bundle adjustment ("stage 2" in arXiv:2608.18028): minimizes
//   sum || m - dehom(P_i X_j) ||^2
// over projective cameras P_i (on the sphere in R^12) and homogeneous points
// X_j (on the sphere in R^4), starting from an OSE solution. Unlike the OSE
// objective this is the true reprojection error, so it removes the bias of the
// regularizer before the metric upgrade.
bool RefineProjectiveReconstruction(const OseProblem& problem,
                                    const OseCameraVector& cameras,
                                    const Eigen::Matrix4Xd& points,
                                    const ProjectiveRefinementOptions& options,
                                    int num_threads,
                                    ProjectiveRefinementResult* result);

// RMS reprojection error of a projective reconstruction (observations with a
// projective depth of exactly zero are skipped).
double ProjectiveRmsError(const OseProblem& problem,
                          const OseCameraVector& cameras,
                          const Eigen::Matrix4Xd& points);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_PROJECTIVE_REFINEMENT_H_
