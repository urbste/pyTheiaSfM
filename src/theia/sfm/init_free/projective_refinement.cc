// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/projective_refinement.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <memory>
#include <cmath>
#include <limits>
#include <vector>

#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/util/timer.h"

namespace theia {

namespace {

// Camera parameters are vec(P) in row-major order.
struct ProjectiveReprojectionError {
  explicit ProjectiveReprojectionError(const Eigen::Vector2d& m) : m_(m) {}

  template <typename T>
  bool operator()(const T* P, const T* X, T* residuals) const {
    const T x0 = P[0] * X[0] + P[1] * X[1] + P[2] * X[2] + P[3] * X[3];
    const T x1 = P[4] * X[0] + P[5] * X[1] + P[6] * X[2] + P[7] * X[3];
    const T x2 = P[8] * X[0] + P[9] * X[1] + P[10] * X[2] + P[11] * X[3];
    residuals[0] = x0 / x2 - T(m_[0]);
    residuals[1] = x1 / x2 - T(m_[1]);
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& m) {
    return new ceres::AutoDiffCostFunction<ProjectiveReprojectionError, 2, 12,
                                           4>(
        new ProjectiveReprojectionError(m));
  }

  const Eigen::Vector2d m_;
};

}  // namespace

double ProjectiveRmsError(const OseProblem& problem,
                          const OseCameraVector& cameras,
                          const Eigen::Matrix4Xd& points) {
  double sum = 0.0;
  int count = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const Eigen::Vector3d x = cameras[problem.observation_camera[k]] *
                              points.col(problem.observation_point[k]);
    if (x.z() == 0.0) continue;
    sum += (x.hnormalized() - problem.observations.col(k)).squaredNorm();
    ++count;
  }
  return count > 0 ? std::sqrt(sum / count) : 0.0;
}

Eigen::VectorXd ProjectiveReprojectionErrors(const OseProblem& problem,
                                             const OseCameraVector& cameras,
                                             const Eigen::Matrix4Xd& points) {
  Eigen::VectorXd errors(problem.NumObservations());
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const Eigen::Vector3d x = cameras[problem.observation_camera[k]] *
                              points.col(problem.observation_point[k]);
    errors[k] = x.z() == 0.0
                    ? std::numeric_limits<double>::infinity()
                    : (x.hnormalized() - problem.observations.col(k)).norm();
  }
  return errors;
}

bool RefineProjectiveReconstruction(const OseProblem& problem,
                                    const OseCameraVector& cameras,
                                    const Eigen::Matrix4Xd& points,
                                    const ProjectiveRefinementOptions& options,
                                    const int num_threads,
                                    ProjectiveRefinementResult* result) {
  CHECK_NOTNULL(result);
  *result = ProjectiveRefinementResult();
  Timer timer;
  if (static_cast<int>(cameras.size()) != problem.NumCameras() ||
      points.cols() != problem.NumPoints()) {
    result->message = "Cameras or points do not match the problem.";
    return false;
  }

  // Unit-norm copies; the projective scale of each block is a gauge freedom
  // removed by the sphere manifolds.
  std::vector<std::array<double, 12>> camera_params(cameras.size());
  for (size_t i = 0; i < cameras.size(); ++i) {
    const Eigen::Matrix<double, 3, 4, Eigen::RowMajor> P =
        cameras[i] / cameras[i].norm();
    std::copy(P.data(), P.data() + 12, camera_params[i].data());
  }
  Eigen::Matrix4Xd point_params = points.colwise().normalized();

  ceres::Problem::Options problem_options;
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.enable_fast_removal = true;
  ceres::Problem ceres_problem(problem_options);
  ceres::SphereManifold<12> camera_manifold;
  ceres::SphereManifold<4> point_manifold;
  std::unique_ptr<ceres::LossFunction> loss;
  if (options.robust_loss_width > 0.0) {
    loss = CreateLossFunction(options.loss_function_type,
                              options.robust_loss_width);
  }
  std::vector<ceres::ResidualBlockId> blocks(problem.NumObservations());
  for (int k = 0; k < problem.NumObservations(); ++k) {
    blocks[k] = ceres_problem.AddResidualBlock(
        ProjectiveReprojectionError::Create(problem.observations.col(k)),
        loss.get(),
        camera_params[problem.observation_camera[k]].data(),
        point_params.col(problem.observation_point[k]).data());
  }
  for (auto& P : camera_params) {
    ceres_problem.SetManifold(P.data(), &camera_manifold);
  }
  for (int j = 0; j < point_params.cols(); ++j) {
    ceres_problem.SetManifold(point_params.col(j).data(), &point_manifold);
  }

  // Points first in the elimination order (Schur complement on cameras).
  auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();
  for (int j = 0; j < point_params.cols(); ++j) {
    ordering->AddElementToGroup(point_params.col(j).data(), 0);
  }
  for (auto& P : camera_params) {
    ordering->AddElementToGroup(P.data(), 1);
  }

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  solver_options.linear_solver_ordering = ordering;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = options.function_tolerance;
  solver_options.num_threads =
      options.deterministic ? 1 : std::max(1, num_threads);
  solver_options.minimizer_progress_to_stdout = options.verbose;
  solver_options.logging_type =
      options.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;

  result->initial_rms_error =
      ProjectiveRmsError(problem, cameras, points);
  const auto copy_out = [&]() {
    result->cameras.resize(cameras.size());
    for (size_t i = 0; i < cameras.size(); ++i) {
      result->cameras[i] =
          Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
              camera_params[i].data());
    }
    result->points = point_params;
  };

  const bool reject = options.outlier_rejection_rounds > 0 &&
                      options.outlier_threshold > 0.0;
  result->observation_inlier.assign(problem.NumObservations(), true);
  ceres::Solver::Summary summary;
  int num_iterations = 0;
  ceres::Solve(solver_options, &ceres_problem, &summary);
  num_iterations += static_cast<int>(summary.iterations.size());
  for (int round = 0; reject && summary.IsSolutionUsable() &&
                      round < options.outlier_rejection_rounds;
       ++round) {
    copy_out();
    const Eigen::VectorXd errors =
        ProjectiveReprojectionErrors(problem, result->cameras, result->points);
    int removed = 0;
    for (int k = 0; k < problem.NumObservations(); ++k) {
      if (result->observation_inlier[k] &&
          !(errors[k] <= options.outlier_threshold)) {
        ceres_problem.RemoveResidualBlock(blocks[k]);
        result->observation_inlier[k] = false;
        ++removed;
      }
    }
    if (removed == 0) break;
    ++result->num_rejection_rounds;
    VLOG(1) << "Projective refinement: rejection round " << round + 1
            << " removed " << removed << " observations.";
    // A point that lost all its residuals is no longer in the problem;
    // re-adding is not needed (it keeps its value).
    ceres::Solve(solver_options, &ceres_problem, &summary);
    num_iterations += static_cast<int>(summary.iterations.size());
  }

  copy_out();
  result->final_rms_error =
      ProjectiveRmsError(problem, result->cameras, result->points);
  const Eigen::VectorXd errors =
      ProjectiveReprojectionErrors(problem, result->cameras, result->points);
  double inlier_sum = 0.0, truncated_sum = 0.0;
  int inliers = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    if (result->observation_inlier[k]) {
      if (std::isfinite(errors[k])) inlier_sum += errors[k] * errors[k];
      ++inliers;
    }
    const double e =
        reject ? std::min(errors[k], options.outlier_threshold) : errors[k];
    if (std::isfinite(e)) truncated_sum += e * e;
  }
  result->num_outlier_observations = problem.NumObservations() - inliers;
  result->final_inlier_rms_error =
      inliers > 0 ? std::sqrt(inlier_sum / inliers) : 0.0;
  result->final_truncated_rms_error =
      reject && problem.NumObservations() > 0
          ? std::sqrt(truncated_sum / problem.NumObservations())
          : result->final_rms_error;
  result->num_iterations = num_iterations;
  result->converged = summary.termination_type == ceres::CONVERGENCE;
  result->message = summary.BriefReport();
  result->solve_time_seconds = timer.ElapsedTimeInSeconds();
  return summary.IsSolutionUsable();
}

}  // namespace theia
