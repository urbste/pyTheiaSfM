// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_
#define THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

struct MetricUpgradeOptions {
  // Refine the linear estimate by nonlinear least squares on the absolute dual
  // quadric (see UpgradeToMetric).
  bool nonlinear_refinement = false;

  // Uncalibrated observations only: one focal length for all cameras.
  bool shared_focal_length = false;

  // Radial cameras (RPOSE): only rows 0 and 1 of each camera are used, and
  // the third row is ignored. Needs at least five cameras. The radial path
  // ignores shared_focal_length. See UpgradeToMetric.
  bool radial_cameras = false;

  // Iterations of the nonlinear refinement.
  int max_num_iterations = 100;
};

struct MetricUpgradeResult {
  // Upgrade H with P_metric ~ P H and X_metric ~ H^{-1} X.
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();

  // Metric cameras x_cam = R X + t, in the numbering of the OseProblem.
  std::vector<Eigen::Matrix3d> rotations;
  std::vector<Eigen::Vector3d> translations;

  // Focal length of every camera in observation units: 1 for CALIBRATED_RAYS;
  // for PIXEL_ISOTROPIC, the focal length in pixels times the scale of the
  // isotropic normalization (problem.normalizations[i](0, 0)). The principal
  // point is the origin of the observations (the image center for
  // PIXEL_ISOTROPIC) and the skew is zero.
  std::vector<double> focal_lengths;

  Eigen::Matrix3Xd points;
  // Points mapped to (or near) infinity by H are invalid.
  std::vector<char> point_valid;

  // Fraction of observations with positive depth after the cheirality fix.
  double fraction_in_front = 0.0;

  // Mean over cameras of ||M M^T / s^2 - I||_F for the left 3x3 block M of
  // K^-1 P H. Zero for an exactly metric result.
  double mean_orthogonality_error = 0.0;

  // The same error for the linear estimate, before the nonlinear refinement.
  double linear_mean_orthogonality_error = 0.0;

  bool nonlinear_refinement_run = false;
};

// Metric upgrade of a projective reconstruction by self-calibration with the
// absolute dual quadric Omega (rank 3, P_i Omega P_i^T ~ K_i K_i^T).
//
// CALIBRATED_RAYS (K_i = I): five linear equations per camera.
//
// PIXEL_ISOTROPIC (K_i = diag(f_i, f_i, 1) in the normalized pixel frame:
// zero skew, unit aspect ratio, principal point at the image center): the
// weighted linear equations of Pollefeys et al. (IJCV 2004), with soft priors
// on the aspect ratio, skew, principal point and a weak prior f_i ~ 1,
// reweighted three times by 1 / (P_3 Omega P_3^T).
//
// The linear estimate is truncated to rank 3. With nonlinear_refinement,
// Omega = B B^T (B 4x3, so Omega stays positive semidefinite with rank 3) and
// the focal lengths are refined to minimize, per camera,
// || 3 E_i / trace(E_i) - I ||_F^2 with E_i = K_i^-1 P_i B B^T P_i^T K_i^-T.
// This drops the priors, so it removes their bias on exact data.
//
// Radial cameras (options.radial_cameras, RPOSE): the metric camera has the
// form P12 H = s [R12 | t12] with two orthonormal rows R12 (only rows 0 and 1
// of P are used). Linear: two equations per camera, P12 Omega P12^T with equal
// diagonal and zero off-diagonal entry. Nonlinear refinement (optional) fits
// Omega = B B^T to the same constraint, without priors. Then two candidates,
// H and H diag(1, 1, -1, 1), are built. Per camera, the sign of the first two
// rows is chosen so that the 2D directions R12 X + t12 agree with the
// observations, and t3 (and, for PIXEL_ISOTROPIC, the focal length f) is fit
// by least squares to m (r3 . X + t3) = f (R12 X + t12). The candidate with
// more points in front wins, ties broken by the residual of that fit. A
// candidate whose fit gives f <= 0 is discarded.
//
// The overall reflection is chosen so that most points lie in front of the
// cameras. `points` are homogeneous (4 x N); Cartesian OSE points can be
// passed as points.colwise().homogeneous().
bool UpgradeToMetric(const OseProblem& problem,
                     const OseCameraVector& cameras,
                     const Eigen::Matrix4Xd& points,
                     const MetricUpgradeOptions& options,
                     MetricUpgradeResult* result,
                     std::string* error);

// Linear upgrade for CALIBRATED_RAYS observations (UpgradeToMetric with
// default options).
bool UpgradeToMetricCalibrated(const OseProblem& problem,
                               const OseCameraVector& cameras,
                               const Eigen::Matrix4Xd& points,
                               MetricUpgradeResult* result,
                               std::string* error);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_
