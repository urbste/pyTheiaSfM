// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_
#define THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

struct MetricUpgradeResult {
  // Upgrade H with P_metric ~ P H and X_metric ~ H^{-1} X.
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();

  // Metric cameras x_cam = R X + t, in the numbering of the OseProblem.
  std::vector<Eigen::Matrix3d> rotations;
  std::vector<Eigen::Vector3d> translations;

  Eigen::Matrix3Xd points;
  // Points mapped to (or near) infinity by H are invalid.
  std::vector<char> point_valid;

  // Fraction of observations with positive depth after the cheirality fix.
  double fraction_in_front = 0.0;

  // Mean over cameras of ||M M^T / s^2 - I||_F for the left 3x3 block M of
  // P H. Zero for an exactly metric result.
  double mean_orthogonality_error = 0.0;
};

// Linear self-calibration for known intrinsics (observations in normalized
// image coordinates, K = I). Estimates the absolute dual quadric Omega from
// P_i Omega P_i^T ~ I (5 linear equations per camera), enforces rank 3 and
// decomposes Omega = H diag(1,1,1,0) H^T. The overall reflection is chosen so
// that most points lie in front of the cameras.
//
// The uncalibrated version (unknown focal lengths, Pollefeys et al. 1999) and
// the nonlinear refinement are not implemented yet.
bool UpgradeToMetricCalibrated(const OseProblem& problem,
                               const OseCameraVector& cameras,
                               const Eigen::Matrix3Xd& points,
                               MetricUpgradeResult* result,
                               std::string* error);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_METRIC_UPGRADE_H_
