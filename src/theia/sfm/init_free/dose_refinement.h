// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_DOSE_REFINEMENT_H_
#define THEIA_SFM_INIT_FREE_DOSE_REFINEMENT_H_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

// Joint metric refinement after the DOSE stage (arXiv:2611.31516, Sec. 4.4,
// Eq. 13 - 21), solved with Ceres (Levenberg-Marquardt). Unknowns: the
// Euclidean cameras (angle-axis rotation and translation, x_cam = R X + t),
// the points and one depth scale alpha_i per camera (log-parametrized, the
// monocular depths are only known up to a per-image scale). Camera 0 (pose and
// scale) is held fixed. With y = R_i X_j + t_i, m~ = [m; 1],
// q = m~ / |m~|^2, f_i the focal length in pixels and D = alpha_i d_ij:
//
//   image:           f_i (y_{0,1} / y_2 - m_{0,1}) (reprojection error, if
//                    reprojection_image_residual and y_2 > 1e-6 d_ij), else
//                    Eq. 13: f_i (y_{0,1} - m_{0,1} y_2) / D' [pixels]
//                    with D' = d_ij, or D' = D if
//                    image_residual_uses_depth_scale (as in the paper).
//                    Huber(image_huber_width_px). Observations without a
//                    depth prior use the median depth prior of camera i for
//                    d_ij.
//   depth (Eq. 14):  (q^T y - D) / D, or log(q^T y / D) if
//                    log_ratio_depth_residual; Huber(depth_huber_width), scaled by
//                    depth_weight; only observations with a depth prior.
//   rotation (15):   rotation_weight * ||R_a R_b^T - R_ab||_F^2 per pairwise
//                    rotation term of the problem (camera_a != camera_b).
//   direction (18):  translation_direction_weight * ||u(t_a - R_a R_b^T t_b)
//                    - u(-R_ab c_ab)||^2 with u(v) = v / |v| and
//                    c_ab = OseRotationTerm::center_a_in_b (skipped if zero).
//   cheirality (20): cheirality_weight * tau / |O| * sum
//                    softplus(-y_2 / (tau * max(D, 1e-4))).
//   scale (21):      scale_regularization_weight / (n - 1) * sum_i
//                    (log alpha_i - log alpha_i^init)^2.
//
// Not implemented: the metric pairwise translation and log-scale terms (Eq. 16,
// 17), which need the depth-based pairwise similarities of the paper.
struct DoseRefinementResult {
  std::vector<Eigen::Matrix3d> rotations;    // x_cam = R X + t
  std::vector<Eigen::Vector3d> translations;
  Eigen::Matrix3Xd points;
  std::vector<double> depth_scales;          // alpha_i, camera 0 is 1

  double initial_cost = 0.0;
  double final_cost = 0.0;
  int num_iterations = 0;
  // Reprojection error |f (y_{0,1} / y_2 - m)| in pixels after the
  // refinement: RMS over all observations with y_2 > 0, and the median.
  double rms_reprojection_error_px = 0.0;
  double median_reprojection_error_px = 0.0;
  // Median |q^T y / D - 1| over the observations with a depth prior.
  double median_relative_depth_error = 0.0;
  // Per observation: reprojection error <= outlier_threshold_px and y_2 > 0
  // (all true if outlier_threshold_px <= 0).
  std::vector<bool> observation_inlier;
  int num_outlier_observations = 0;

  double solve_time_seconds = 0.0;
  std::string message;
};

// `cameras` are Euclidean [R | t] (the DOSE solution), `points` Cartesian,
// both in the numbering of `problem`. Needs problem.focal_lengths and
// problem.observation_depths. Returns false (with result->message) on invalid
// input; a Ceres run that ends without convergence is not a failure.
bool RefineDoseReconstruction(const OseProblem& problem,
                              const OseCameraVector& cameras,
                              const Eigen::Matrix3Xd& points,
                              const DoseRefinementOptions& options,
                              int num_threads,
                              DoseRefinementResult* result);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_DOSE_REFINEMENT_H_
