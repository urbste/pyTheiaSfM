// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/varpro_solver.h"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <glog/logging.h>

#include <algorithm>
#include <memory>

#include "theia/sfm/init_free/reduced_camera_solver.h"
#include "theia/util/timer.h"

namespace theia {

namespace {

using RotationJacobian =
    Eigen::Matrix<double, 9, kOseCameraParameters, Eigen::RowMajor>;

double RotationTermsCost(const OseProblem& problem,
                         const OseFormulation& formulation,
                         const OseCameraVector& cameras) {
  if (!formulation.UsesRotationTerms()) {
    return 0.0;
  }
  double cost = 0.0;
  Eigen::Matrix<double, 9, 1> r;
  RotationJacobian Ja, Jb;
  for (const OseRotationTerm& term : problem.rotation_terms) {
    OseFormulation::EvaluateRotationTerm(term,
                                         cameras[term.camera_a],
                                         cameras[term.camera_b],
                                         &r,
                                         &Ja,
                                         &Jb);
    cost += r.squaredNorm();
  }
  return 0.5 * cost;
}

// Builds the block system and the reduced right-hand side
//   rhs = -(g_u - sum_j W_j^T V_j^{-1} g_v_j).
// At the exact point solution g_v_j is (numerically) zero; it is kept so that
// the step stays correct if the point solve is regularized.
void BuildReducedSystem(const OseProblem& problem,
                        const OseFormulation& formulation,
                        const OseCameraVector& cameras,
                        const Eigen::Matrix3Xd& points,
                        const double point_ridge,
                        ReducedCameraSystem* system,
                        Eigen::VectorXd* rhs) {
  constexpr int b = ReducedCameraSystem::kBlockSize;
  const int n = problem.NumCameras();
  system->num_cameras = n;
  system->U_diagonal.assign(n, Matrix12d::Zero());
  system->U_off_diagonal.clear();
  system->V_inverse.resize(problem.NumPoints());
  system->point_offsets = problem.point_offsets;
  system->W_camera = problem.observation_camera;
  system->W_blocks.resize(problem.NumObservations());
  ++system->revision;

  Eigen::VectorXd g = Eigen::VectorXd::Zero(b * n);
  Eigen::VectorXd correction = Eigen::VectorXd::Zero(b * n);
  OseObservationModel model;
  OseCameraJacobian J;
  for (int j = 0; j < problem.NumPoints(); ++j) {
    const Eigen::Vector3d X = points.col(j);
    Eigen::Matrix3d V = point_ridge * Eigen::Matrix3d::Identity();
    Eigen::Vector3d g_v = Eigen::Vector3d::Zero();
    for (int k = problem.point_offsets[j]; k < problem.point_offsets[j + 1];
         ++k) {
      const int c = problem.observation_camera[k];
      const Eigen::Vector2d m = problem.observations.col(k);
      formulation.ObservationModel(cameras[c], m, &model);
      formulation.CameraJacobian(cameras[c], m, X, &J);
      const auto r = (model.G * X - model.z).eval();
      system->U_diagonal[c].noalias() += J.transpose() * J;
      g.segment<b>(b * c).noalias() += J.transpose() * r;
      system->W_blocks[k].noalias() = model.G.transpose() * J;
      V.noalias() += model.G.transpose() * model.G;
      g_v.noalias() += model.G.transpose() * r;
    }
    system->V_inverse[j] = V.inverse();
    const Eigen::Vector3d Vinv_gv = system->V_inverse[j] * g_v;
    for (int k = problem.point_offsets[j]; k < problem.point_offsets[j + 1];
         ++k) {
      correction.segment<b>(b * problem.observation_camera[k]).noalias() +=
          system->W_blocks[k].transpose() * Vinv_gv;
    }
  }

  if (formulation.UsesRotationTerms()) {
    Eigen::Matrix<double, 9, 1> r;
    RotationJacobian Ja, Jb;
    for (const OseRotationTerm& term : problem.rotation_terms) {
      const int a = term.camera_a;
      const int c = term.camera_b;
      OseFormulation::EvaluateRotationTerm(
          term, cameras[a], cameras[c], &r, &Ja, &Jb);
      system->U_diagonal[a].noalias() += Ja.transpose() * Ja;
      g.segment<b>(b * a).noalias() += Ja.transpose() * r;
      if (a == c) {
        continue;
      }
      system->U_diagonal[c].noalias() += Jb.transpose() * Jb;
      g.segment<b>(b * c).noalias() += Jb.transpose() * r;
      const Matrix12d ab = Ja.transpose() * Jb;
      system->U_off_diagonal.push_back({a, c, ab});
      system->U_off_diagonal.push_back({c, a, ab.transpose()});
    }
  }

  *rhs = -(g - correction);
}

}  // namespace

double EvaluateOseCost(const OseProblem& problem,
                       const OseFormulation& formulation,
                       const OseCameraVector& cameras,
                       const double point_ridge,
                       Eigen::Matrix3Xd* points) {
  points->resize(3, problem.NumPoints());
  double cost = 0.0;
  OseObservationModel model;
  for (int j = 0; j < problem.NumPoints(); ++j) {
    Eigen::Matrix3d V = point_ridge * Eigen::Matrix3d::Identity();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    const int begin = problem.point_offsets[j];
    const int end = problem.point_offsets[j + 1];
    for (int k = begin; k < end; ++k) {
      formulation.ObservationModel(cameras[problem.observation_camera[k]],
                                   problem.observations.col(k),
                                   &model);
      V.noalias() += model.G.transpose() * model.G;
      rhs.noalias() += model.G.transpose() * model.z;
    }
    const Eigen::Vector3d X = V.ldlt().solve(rhs);
    points->col(j) = X;
    for (int k = begin; k < end; ++k) {
      formulation.ObservationModel(cameras[problem.observation_camera[k]],
                                   problem.observations.col(k),
                                   &model);
      cost += 0.5 * (model.G * X - model.z).squaredNorm();
    }
  }
  return cost + RotationTermsCost(problem, formulation, cameras);
}

bool SolveOseVarPro(const OseProblem& problem,
                    const OseFormulation& formulation,
                    const VarProOptions& options,
                    const OseCameraVector& initial_cameras,
                    OseSolution* solution) {
  CHECK_NOTNULL(solution);
  *solution = OseSolution();
  Timer timer;
  if (static_cast<int>(initial_cameras.size()) != problem.NumCameras()) {
    solution->message = "Number of initial cameras does not match problem.";
    return false;
  }
  if (options.robust_loss_type != VarProRobustLossType::NONE) {
    solution->message = "Robust VarPro losses are not implemented yet.";
    return false;
  }
  std::string error;
  std::unique_ptr<ReducedCameraSolver> linear_solver =
      CreateReducedCameraSolver(options, &error);
  if (linear_solver == nullptr) {
    solution->message = error;
    return false;
  }

  constexpr int b = ReducedCameraSystem::kBlockSize;
  OseCameraVector cameras = initial_cameras;
  Eigen::Matrix3Xd points;
  double cost = EvaluateOseCost(
      problem, formulation, cameras, options.point_ridge, &points);
  solution->initial_cost = cost;
  solution->cost_history.push_back(cost);

  ReducedCameraSystem system;
  Eigen::VectorXd rhs, step;
  OseCameraVector candidate(cameras.size());
  Eigen::Matrix3Xd candidate_points;
  double lambda = options.initial_lambda;
  std::string termination = "Maximum number of iterations reached.";

  int iteration = 0;
  for (; iteration < options.max_num_iterations; ++iteration) {
    BuildReducedSystem(problem,
                       formulation,
                       cameras,
                       points,
                       options.point_ridge,
                       &system,
                       &rhs);

    bool accepted = false;
    double relative_decrease = 0.0;
    while (lambda <= options.max_lambda) {
      if (!linear_solver->Solve(system, lambda, rhs, &step, &error)) {
        VLOG(2) << "Linear solve failed at lambda " << lambda << ": " << error;
        lambda *= options.lambda_increase_factor;
        continue;
      }
      for (size_t i = 0; i < cameras.size(); ++i) {
        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> delta =
            Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
                step.data() + b * i);
        candidate[i] = cameras[i] + delta;
      }
      const double candidate_cost = EvaluateOseCost(problem,
                                                    formulation,
                                                    candidate,
                                                    options.point_ridge,
                                                    &candidate_points);
      if (std::isfinite(candidate_cost) && candidate_cost < cost) {
        relative_decrease = (cost - candidate_cost) / std::max(cost, 1e-300);
        cameras.swap(candidate);
        points.swap(candidate_points);
        cost = candidate_cost;
        lambda = std::max(lambda / options.lambda_decrease_factor,
                          options.min_lambda);
        accepted = true;
        break;
      }
      lambda *= options.lambda_increase_factor;
    }

    if (!accepted) {
      termination = "Damping exceeded max_lambda without a cost decrease.";
      break;
    }
    solution->cost_history.push_back(cost);
    if (options.verbose) {
      LOG(INFO) << "VarPro iter " << iteration << ": cost " << cost
                << ", lambda " << lambda << ", |step| " << step.norm();
    }

    double camera_norm = 0.0;
    for (const OseCamera& P : cameras) {
      camera_norm += P.squaredNorm();
    }
    camera_norm = std::sqrt(camera_norm);
    if (relative_decrease < options.function_tolerance) {
      solution->converged = true;
      termination = "Function tolerance reached.";
      ++iteration;
      break;
    }
    if (step.norm() <
        options.parameter_tolerance * (camera_norm + options.parameter_tolerance)) {
      solution->converged = true;
      termination = "Parameter tolerance reached.";
      ++iteration;
      break;
    }
  }

  solution->cameras = std::move(cameras);
  solution->points = std::move(points);
  solution->final_cost = cost;
  solution->num_iterations = iteration;
  solution->message = termination;
  solution->solve_time_seconds = timer.ElapsedTimeInSeconds();
  return true;
}

}  // namespace theia
