// Copyright (C) 2026, The pyTheiaSfM authors. All rights reserved.
//
// Fast Iterative Five point Relative Pose Estimation based on Powell's Dogleg
// method, as described in:
//
//   J. Hedborg and M. Felsberg, "Fast Iterative Five point Relative Pose
//   Estimation", Computer Vision Laboratory, Linköping University.
//
// This solver is parameterized with 5 minimal angles w = [alpha, beta, gamma,
// theta, phi], where (alpha, beta, gamma) are Euler rotation angles and
// (theta, phi) define the unit translation vector in spherical coordinates.
//
// IMPORTANT: This solver is specifically designed and constrained for
// forward-facing trajectories (i.e. t ~ [0, 0, 1], R ~ I), such as consecutive
// video frames, automotive / visual odometry sequences, and forward camera
// motions. For arbitrary wide-baseline or pure sideways motions, use the
// closed-form 5-point solvers (FivePointRelativePoseSturm or
// FivePointRelativePose).

#ifndef THEIA_SFM_POSE_FAST_ITERATIVE_FIVE_POINT_H_
#define THEIA_SFM_POSE_FAST_ITERATIVE_FIVE_POINT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace theia {

struct RelativePose;

struct FastIterativeFivePointOptions {
  // Maximum number of Dogleg iterations. 5-8 iterations are typically sufficient
  // for convergence when initialized near forward motion.
  int max_iterations = 8;

  // Initial trust region radius Delta_0.
  double initial_trust_region_radius = 1.0;

  // Convergence thresholds.
  double gradient_tolerance = 1e-9;
  double residual_tolerance = 1e-9;
  double step_tolerance = 1e-10;
  double trust_region_tolerance = 1e-10;

  // Prior / initial relative pose. Defaults to identity rotation (zero Euler angles)
  // and forward unit translation in camera coordinates (t = [0, 0, -1], corresponding
  // to camera moving forward along the optical axis +Z).
  Eigen::Matrix3d prior_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d prior_translation = Eigen::Vector3d(0.0, 0.0, -1.0);
};

// Converts Euler angles (alpha, beta, gamma) and spherical coordinates (theta, phi)
// to a 3x3 rotation matrix R and 3D unit translation vector t.
void FastIterativePoseParametersToRotationAndTranslation(
    const Eigen::Matrix<double, 5, 1>& w,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation);

// Converts a 3x3 rotation matrix R and 3D translation vector t into the 5-angle
// parameter vector w = [alpha, beta, gamma, theta, phi].
Eigen::Matrix<double, 5, 1>
FastIterativeRotationAndTranslationToPoseParameters(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation);

// Solves for the relative pose / essential matrix between two calibrated cameras
// using the iterative Dogleg algorithm from at least 5 point correspondences.
//
// Params:
//   x1h: Homogeneous (or bearing) coordinates in camera 1 (size >= 5).
//   x2h: Homogeneous (or bearing) coordinates in camera 2 (size >= 5).
//   options: Solver options including forward trajectory prior and tolerances.
//   essential_matrices: Output vector containing the estimated essential matrix (if converged).
//   relative_poses: Optional output vector containing the decomposed RelativePose (R, position C).
//
// Returns:
//   Number of solutions found (1 on successful convergence, 0 otherwise).
int FastIterativeFivePoint(
    const std::vector<Eigen::Vector3d>& x1h,
    const std::vector<Eigen::Vector3d>& x2h,
    const FastIterativeFivePointOptions& options,
    std::vector<Eigen::Matrix3d>* essential_matrices,
    std::vector<RelativePose>* relative_poses = nullptr);

// Convenience overload using normalized 2D feature coordinates.
int FastIterativeFivePoint(
    const std::vector<Eigen::Vector2d>& image1_points,
    const std::vector<Eigen::Vector2d>& image2_points,
    const FastIterativeFivePointOptions& options,
    std::vector<Eigen::Matrix3d>* essential_matrices,
    std::vector<RelativePose>* relative_poses = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_POSE_FAST_ITERATIVE_FIVE_POINT_H_
