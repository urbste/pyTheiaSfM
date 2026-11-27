// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/dose_refinement.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "theia/util/timer.h"

namespace theia {

namespace {

// Pose block: [angle-axis (3), translation (3)], x_cam = R X + t.
// Point block: [X (3)]. Depth-scale block: [log alpha (1)].

// Image residual (Eq. 13), in pixels: f (y_{0,1} - m_{0,1} y_2) / D.
struct DoseImageResidual {
  DoseImageResidual(const Eigen::Vector2d& m,
                    const double focal,
                    const double depth_ref,
                    const bool use_depth_scale,
                    const bool reprojection)
      : m_(m),
        focal_(focal),
        depth_ref_(depth_ref),
        use_depth_scale_(use_depth_scale),
        reprojection_(reprojection) {}

  template <typename T>
  bool operator()(const T* pose,
                  const T* point,
                  const T* log_alpha,
                  T* residuals) const {
    T y[3];
    ceres::AngleAxisRotatePoint(pose, point, y);
    y[0] += pose[3];
    y[1] += pose[4];
    y[2] += pose[5];
    if (reprojection_ && y[2] > T(1e-6 * depth_ref_)) {
      residuals[0] = T(focal_) * (y[0] / y[2] - T(m_[0]));
      residuals[1] = T(focal_) * (y[1] / y[2] - T(m_[1]));
      return true;
    }
    // Object space form (Eq. 13); also used for points at or behind the
    // camera with the reprojection residual.
    const T depth = use_depth_scale_ ? ceres::exp(log_alpha[0]) * T(depth_ref_)
                                     : T(depth_ref_);
    residuals[0] = T(focal_) * (y[0] - T(m_[0]) * y[2]) / depth;
    residuals[1] = T(focal_) * (y[1] - T(m_[1]) * y[2]) / depth;
    return true;
  }

  const Eigen::Vector2d m_;
  const double focal_;
  const double depth_ref_;
  const bool use_depth_scale_;
  const bool reprojection_;
};

// Depth residual (Eq. 14): (q^T y - D) / D with q = m~ / |m~|^2.
struct DoseDepthResidual {
  DoseDepthResidual(const Eigen::Vector2d& m,
                    const double depth_prior,
                    const bool log_ratio)
      : depth_prior_(depth_prior), log_ratio_(log_ratio) {
    const Eigen::Vector3d mt(m[0], m[1], 1.0);
    q_ = mt / mt.squaredNorm();
  }

  template <typename T>
  bool operator()(const T* pose,
                  const T* point,
                  const T* log_alpha,
                  T* residuals) const {
    T y[3];
    ceres::AngleAxisRotatePoint(pose, point, y);
    y[0] += pose[3];
    y[1] += pose[4];
    y[2] += pose[5];
    const T depth = ceres::exp(log_alpha[0]) * T(depth_prior_);
    const T ray_depth = T(q_[0]) * y[0] + T(q_[1]) * y[1] + T(q_[2]) * y[2];
    if (log_ratio_) {
      // log(q^T y / D); a point at (or behind) the camera gets the residual
      // of 1e-3 D without gradient (the cheirality term acts there).
      const T ratio = ray_depth / depth;
      residuals[0] = ratio > T(1e-3) ? ceres::log(ratio) : T(std::log(1e-3));
    } else {
      residuals[0] = (ray_depth - depth) / depth;
    }
    return true;
  }

  Eigen::Vector3d q_;
  const double depth_prior_;
  const bool log_ratio_;
};

// Cheirality residual (Eq. 20): sqrt(c * softplus(-y_2 / (tau * max(D,
// 1e-4)))).
struct DoseCheiralityResidual {
  DoseCheiralityResidual(const double coefficient,
                         const double tau,
                         const double depth_ref)
      : coefficient_(coefficient), tau_(tau), depth_ref_(depth_ref) {}

  template <typename T>
  bool operator()(const T* pose,
                  const T* point,
                  const T* log_alpha,
                  T* residuals) const {
    T y[3];
    ceres::AngleAxisRotatePoint(pose, point, y);
    const T z = y[2] + pose[5];
    T depth = ceres::exp(log_alpha[0]) * T(depth_ref_);
    if (depth < T(1e-4)) depth = T(1e-4);
    const T x = -z / (T(tau_) * depth);
    // Stable softplus: log(1 + e^x) = x for large x.
    T s = x > T(30.0) ? x : ceres::log(T(1.0) + ceres::exp(x));
    // Keeps the square root differentiable when softplus underflows.
    if (s < T(1e-30)) s = T(1e-30);
    residuals[0] = ceres::sqrt(T(coefficient_) * s);
    return true;
  }

  const double coefficient_;
  const double tau_;
  const double depth_ref_;
};

// Rotation residual (Eq. 15): sqrt(w) (R_a R_b^T - R_ab), column-major.
struct DoseRotationResidual {
  DoseRotationResidual(const Eigen::Matrix3d& relative_rotation,
                       const double sqrt_weight)
      : relative_rotation_(relative_rotation), sqrt_weight_(sqrt_weight) {}

  template <typename T>
  bool operator()(const T* pose_a, const T* pose_b, T* residuals) const {
    T Ra[9], Rb[9];
    ceres::AngleAxisToRotationMatrix(pose_a, Ra);
    ceres::AngleAxisToRotationMatrix(pose_b, Rb);
    // Column-major: element (r, c) is at r + 3 c.
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        T sum = T(0.0);
        for (int k = 0; k < 3; ++k) {
          sum += Ra[r + 3 * k] * Rb[c + 3 * k];
        }
        residuals[r + 3 * c] =
            T(sqrt_weight_) * (sum - T(relative_rotation_(r, c)));
      }
    }
    return true;
  }

  const Eigen::Matrix3d relative_rotation_;
  const double sqrt_weight_;
};

// Translation direction residual (Eq. 18): with v = t_a - R_a R_b^T t_b,
// sqrt(w) (v / sqrt(|v|^2 + eps) - u), u the unit target direction.
struct DoseDirectionResidual {
  DoseDirectionResidual(const Eigen::Vector3d& target, const double sqrt_weight)
      : target_(target), sqrt_weight_(sqrt_weight) {}

  template <typename T>
  bool operator()(const T* pose_a, const T* pose_b, T* residuals) const {
    const T inverse_b[3] = {-pose_b[0], -pose_b[1], -pose_b[2]};
    T rb_t[3];
    ceres::AngleAxisRotatePoint(inverse_b, pose_b + 3, rb_t);  // R_b^T t_b
    T ra_rb_t[3];
    ceres::AngleAxisRotatePoint(pose_a, rb_t, ra_rb_t);
    T v[3];
    for (int i = 0; i < 3; ++i) v[i] = pose_a[3 + i] - ra_rb_t[i];
    const T norm =
        ceres::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + T(1e-12));
    const T scale = T(sqrt_weight_) / norm;
    for (int i = 0; i < 3; ++i) {
      residuals[i] = v[i] * scale - T(sqrt_weight_ * target_[i]);
    }
    return true;
  }

  const Eigen::Vector3d target_;
  const double sqrt_weight_;
};

// Scale regularization (Eq. 21): sqrt(w / (n - 1)) log alpha_i.
struct DoseScaleResidual {
  explicit DoseScaleResidual(const double sqrt_coefficient)
      : sqrt_coefficient_(sqrt_coefficient) {}

  template <typename T>
  bool operator()(const T* log_alpha, T* residuals) const {
    residuals[0] = T(sqrt_coefficient_) * log_alpha[0];
    return true;
  }

  const double sqrt_coefficient_;
};

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

}  // namespace

bool RefineDoseReconstruction(const OseProblem& problem,
                              const OseCameraVector& cameras,
                              const Eigen::Matrix3Xd& points,
                              const DoseRefinementOptions& options,
                              const int num_threads,
                              DoseRefinementResult* result) {
  CHECK_NOTNULL(result);
  *result = DoseRefinementResult();
  Timer timer;
  const int num_cameras = problem.NumCameras();
  const int num_points = problem.NumPoints();
  const int num_observations = problem.NumObservations();
  if (static_cast<int>(cameras.size()) != num_cameras) {
    result->message = "Cameras do not match the problem.";
    return false;
  }
  if (points.cols() != num_points) {
    result->message = "Points do not match the problem.";
    return false;
  }
  if (static_cast<int>(problem.focal_lengths.size()) != num_cameras) {
    result->message = "The problem needs one focal length per camera.";
    return false;
  }
  if (static_cast<int>(problem.observation_depths.size()) != num_observations) {
    result->message = "The problem needs one depth prior per observation.";
    return false;
  }
  if (options.max_num_iterations <= 0) {
    result->message = "max_num_iterations must be positive.";
    return false;
  }
  if (!(options.image_huber_width_px > 0.0) ||
      !(options.depth_huber_width > 0.0)) {
    result->message = "Huber widths must be positive.";
    return false;
  }
  if (num_cameras == 0 || num_points == 0 || num_observations == 0) {
    result->message = "The problem is empty.";
    return false;
  }

  // Parameter storage. The vectors are not resized after this point, so the
  // pointers handed to Ceres stay valid.
  std::vector<std::array<double, 6>> poses(num_cameras);
  std::vector<std::array<double, 3>> point_params(num_points);
  std::vector<std::array<double, 1>> log_alphas(num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    const Eigen::Matrix3d R = cameras[i].leftCols<3>();
    double angle_axis[3];
    ceres::RotationMatrixToAngleAxis(R.data(), angle_axis);
    poses[i] = {angle_axis[0],
                angle_axis[1],
                angle_axis[2],
                cameras[i](0, 3),
                cameras[i](1, 3),
                cameras[i](2, 3)};
    log_alphas[i][0] = 0.0;
  }
  for (int j = 0; j < num_points; ++j) {
    point_params[j] = {points(0, j), points(1, j), points(2, j)};
  }

  // Depth prior per camera (median over its observations with a prior), and
  // the median over all observations for cameras without any prior.
  std::vector<double> all_depths;
  std::vector<std::vector<double>> camera_depths(num_cameras);
  for (int k = 0; k < num_observations; ++k) {
    const double depth = problem.observation_depths[k];
    if (depth > 0.0 && std::isfinite(depth)) {
      all_depths.push_back(depth);
      camera_depths[problem.observation_camera[k]].push_back(depth);
    }
  }
  const double global_median = all_depths.empty() ? 1.0 : Median(all_depths);
  std::vector<double> camera_median(num_cameras, global_median);
  for (int i = 0; i < num_cameras; ++i) {
    if (!camera_depths[i].empty()) {
      camera_median[i] = Median(camera_depths[i]);
    }
  }
  // Observation depth used by the image and cheirality residuals.
  std::vector<double> depth_ref(num_observations);
  std::vector<bool> has_depth(num_observations, false);
  for (int k = 0; k < num_observations; ++k) {
    const double depth = problem.observation_depths[k];
    has_depth[k] = depth > 0.0 && std::isfinite(depth);
    depth_ref[k] =
        has_depth[k] ? depth : camera_median[problem.observation_camera[k]];
  }

  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem ceres_problem(problem_options);

  for (auto& pose : poses) {
    ceres_problem.AddParameterBlock(pose.data(), 6);
  }
  for (auto& point : point_params) {
    ceres_problem.AddParameterBlock(point.data(), 3);
  }
  for (auto& log_alpha : log_alphas) {
    ceres_problem.AddParameterBlock(log_alpha.data(), 1);
  }

  std::unique_ptr<ceres::LossFunction> image_loss(
      new ceres::HuberLoss(options.image_huber_width_px));
  std::unique_ptr<ceres::LossFunction> depth_loss;
  if (options.depth_weight > 0.0) {
    depth_loss.reset(
        new ceres::ScaledLoss(new ceres::HuberLoss(options.depth_huber_width),
                              options.depth_weight,
                              ceres::TAKE_OWNERSHIP));
  }

  // Image and cheirality residuals, depth residuals.
  const double cheirality_coefficient = options.cheirality_weight *
                                        options.cheirality_tau /
                                        static_cast<double>(num_observations);
  const bool use_cheirality = options.cheirality_weight > 0.0;
  for (int k = 0; k < num_observations; ++k) {
    const int c = problem.observation_camera[k];
    const int j = problem.observation_point[k];
    const Eigen::Vector2d& m = problem.observations.col(k);
    double* pose = poses[c].data();
    double* point = point_params[j].data();
    double* log_alpha = log_alphas[c].data();

    using ImageCost =
        ceres::AutoDiffCostFunction<DoseImageResidual, 2, 6, 3, 1>;
    ceres_problem.AddResidualBlock(
        new ImageCost(
            new DoseImageResidual(m,
                                  problem.focal_lengths[c],
                                  depth_ref[k],
                                  options.image_residual_uses_depth_scale,
                                  options.reprojection_image_residual)),
        image_loss.get(),
        pose,
        point,
        log_alpha);

    if (has_depth[k] && depth_loss != nullptr) {
      using DepthCost =
          ceres::AutoDiffCostFunction<DoseDepthResidual, 1, 6, 3, 1>;
      ceres_problem.AddResidualBlock(new DepthCost(new DoseDepthResidual(
                                         m, problem.observation_depths[k],
                                         options.log_ratio_depth_residual)),
                                     depth_loss.get(),
                                     pose,
                                     point,
                                     log_alpha);
    }

    if (use_cheirality) {
      using CheiralityCost =
          ceres::AutoDiffCostFunction<DoseCheiralityResidual, 1, 6, 3, 1>;
      ceres_problem.AddResidualBlock(
          new CheiralityCost(new DoseCheiralityResidual(
              cheirality_coefficient, options.cheirality_tau, depth_ref[k])),
          nullptr,
          pose,
          point,
          log_alpha);
    }
  }

  // Pairwise rotation and translation direction terms (camera pairs only).
  const double sqrt_rotation_weight = std::sqrt(options.rotation_weight);
  const double sqrt_direction_weight =
      std::sqrt(options.translation_direction_weight);
  for (const OseRotationTerm& term : problem.rotation_terms) {
    const int a = term.camera_a;
    const int b = term.camera_b;
    if (a == b) continue;
    if (options.rotation_weight > 0.0) {
      using RotationCost =
          ceres::AutoDiffCostFunction<DoseRotationResidual, 9, 6, 6>;
      ceres_problem.AddResidualBlock(
          new RotationCost(new DoseRotationResidual(term.relative_rotation,
                                                    sqrt_rotation_weight)),
          nullptr,
          poses[a].data(),
          poses[b].data());
    }
    if (options.translation_direction_weight > 0.0 &&
        term.center_a_in_b.norm() > 0.0) {
      // With c_a the center of camera a, R_b c_a + t_b = -R_ab^T v and
      // v = t_a - R_a R_b^T t_b, so the target direction is u(-R_ab c_ab).
      const Eigen::Vector3d target =
          (-term.relative_rotation * term.center_a_in_b).normalized();
      using DirectionCost =
          ceres::AutoDiffCostFunction<DoseDirectionResidual, 3, 6, 6>;
      ceres_problem.AddResidualBlock(
          new DirectionCost(
              new DoseDirectionResidual(target, sqrt_direction_weight)),
          nullptr,
          poses[a].data(),
          poses[b].data());
    }
  }

  // Gauge: camera 0 (pose and scale) is fixed.
  ceres_problem.SetParameterBlockConstant(poses[0].data());
  ceres_problem.SetParameterBlockConstant(log_alphas[0].data());
  if (!options.optimize_depth_scales) {
    for (auto& log_alpha : log_alphas) {
      ceres_problem.SetParameterBlockConstant(log_alpha.data());
    }
  } else if (options.scale_regularization_weight > 0.0 && num_cameras >= 2) {
    const double coefficient = options.scale_regularization_weight /
                               static_cast<double>(num_cameras - 1);
    for (int i = 1; i < num_cameras; ++i) {
      using ScaleCost = ceres::AutoDiffCostFunction<DoseScaleResidual, 1, 1>;
      ceres_problem.AddResidualBlock(
          new ScaleCost(new DoseScaleResidual(std::sqrt(coefficient))),
          nullptr,
          log_alphas[i].data());
    }
  }

  // Points are eliminated first (Schur complement on the cameras).
  auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();
  for (auto& point : point_params) {
    ordering->AddElementToGroup(point.data(), 0);
  }
  for (auto& pose : poses) {
    ordering->AddElementToGroup(pose.data(), 1);
  }
  for (auto& log_alpha : log_alphas) {
    ordering->AddElementToGroup(log_alpha.data(), 1);
  }

  ceres::Solver::Options solver_options;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  solver_options.linear_solver_ordering = ordering;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = options.function_tolerance;
  solver_options.parameter_tolerance = options.parameter_tolerance;
  solver_options.num_threads = std::max(1, num_threads);
  solver_options.minimizer_progress_to_stdout = options.verbose;
  solver_options.logging_type =
      options.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &ceres_problem, &summary);
  result->initial_cost = summary.initial_cost;
  result->final_cost = summary.final_cost;
  result->num_iterations = static_cast<int>(summary.iterations.size());
  result->message = summary.BriefReport();

  // Output parameters.
  result->rotations.resize(num_cameras);
  result->translations.resize(num_cameras);
  result->depth_scales.resize(num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    ceres::AngleAxisToRotationMatrix(poses[i].data(),
                                     result->rotations[i].data());
    result->translations[i] =
        Eigen::Vector3d(poses[i][3], poses[i][4], poses[i][5]);
    result->depth_scales[i] = std::exp(log_alphas[i][0]);
  }
  result->points.resize(3, num_points);
  for (int j = 0; j < num_points; ++j) {
    result->points.col(j) = Eigen::Vector3d(
        point_params[j][0], point_params[j][1], point_params[j][2]);
  }

  // Reprojection and depth statistics.
  std::vector<double> reprojection_errors;
  std::vector<double> relative_depth_errors;
  reprojection_errors.reserve(num_observations);
  result->observation_inlier.assign(num_observations, true);
  double squared_sum = 0.0;
  int num_valid = 0;
  for (int k = 0; k < num_observations; ++k) {
    const int c = problem.observation_camera[k];
    const int j = problem.observation_point[k];
    const Eigen::Vector2d& m = problem.observations.col(k);
    const Eigen::Vector3d y =
        result->rotations[c] * result->points.col(j) + result->translations[c];
    bool inlier = true;
    if (y.z() > 0.0) {
      const double e =
          problem.focal_lengths[c] *
          (Eigen::Vector2d(y.x() / y.z(), y.y() / y.z()) - m).norm();
      reprojection_errors.push_back(e);
      squared_sum += e * e;
      ++num_valid;
      inlier = e <= options.outlier_threshold_px;
    } else {
      inlier = false;
    }
    if (options.outlier_threshold_px <= 0.0) inlier = true;
    result->observation_inlier[k] = inlier;
    if (!inlier) ++result->num_outlier_observations;

    if (has_depth[k]) {
      const Eigen::Vector3d mt(m[0], m[1], 1.0);
      const double ray_depth = (mt / mt.squaredNorm()).dot(y);
      const double depth = depth_ref[k] * result->depth_scales[c];
      relative_depth_errors.push_back(std::abs(ray_depth / depth - 1.0));
    }
  }
  result->rms_reprojection_error_px =
      num_valid > 0 ? std::sqrt(squared_sum / num_valid) : 0.0;
  result->median_reprojection_error_px = Median(reprojection_errors);
  result->median_relative_depth_error = Median(relative_depth_errors);

  result->solve_time_seconds = timer.ElapsedTimeInSeconds();
  return summary.IsSolutionUsable();
}

}  // namespace theia
