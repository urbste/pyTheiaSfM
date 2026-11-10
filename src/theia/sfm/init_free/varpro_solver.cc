// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/varpro_solver.h"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>

#include "theia/sfm/init_free/parallel_for.h"
#include "theia/sfm/init_free/reduced_camera_solver.h"
#include "theia/util/timer.h"

namespace theia {

namespace {

constexpr int kB = ReducedCameraSystem::kBlockSize;
constexpr int kMinPointsPerChunk = 256;
constexpr int kMinCamerasPerChunk = 4;

// Per-observation quantities that the least-squares model depends on but that
// are held fixed within one VarPro iteration: IRLS weights of the robust loss
// and the formulation's linearization state (expOSE).
struct ObservationContext {
  std::vector<double> weights;  // empty means 1
  std::vector<double> states;   // empty means 0
  VarProRobustLossType robust_loss = VarProRobustLossType::NONE;
  double robust_scale = 1.0;

  double Weight(const int k) const { return weights.empty() ? 1.0 : weights[k]; }
  double State(const int k) const { return states.empty() ? 0.0 : states[k]; }

  // Cost of an observation with unweighted squared residual norm s.
  double Cost(const double s) const {
    if (robust_loss == VarProRobustLossType::CAUCHY) {
      const double c2 = robust_scale * robust_scale;
      return 0.5 * c2 * std::log1p(s / c2);
    }
    return 0.5 * s;
  }

  double RobustWeight(const double s) const {
    if (robust_loss == VarProRobustLossType::CAUCHY) {
      return 1.0 / (1.0 + s / (robust_scale * robust_scale));
    }
    return 1.0;
  }
};

double RotationTermsCost(const OseProblem& problem,
                         const OseFormulation& formulation,
                         const OseCameraVector& cameras) {
  if (!formulation.UsesRotationTerms()) {
    return 0.0;
  }
  double cost = 0.0;
  Eigen::Matrix<double, 9, 1> r;
  OseRotationJacobian Ja, Jb;
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

// Solves every point by weighted linear least squares and returns the cost.
// Per-point costs are summed in a fixed order, so the result does not depend
// on the number of threads.
double EvaluateCost(const OseProblem& problem,
                    const OseFormulation& formulation,
                    const OseCameraVector& cameras,
                    const double point_ridge,
                    const ObservationContext& context,
                    const int num_threads,
                    Eigen::Matrix3Xd* points) {
  const int num_points = problem.NumPoints();
  points->resize(3, num_points);
  std::vector<double> point_cost(num_points, 0.0);
  ParallelFor(
      num_threads,
      num_points,
      kMinPointsPerChunk,
      [&](const int begin, const int end, int) {
        OseObservationModel model;
        for (int j = begin; j < end; ++j) {
          Eigen::Matrix3d V = point_ridge * Eigen::Matrix3d::Identity();
          Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
          const int first = problem.point_offsets[j];
          const int last = problem.point_offsets[j + 1];
          for (int k = first; k < last; ++k) {
            formulation.ObservationModel(
                cameras[problem.observation_camera[k]],
                problem.observations.col(k),
                context.State(k),
                &model);
            const double w = context.Weight(k);
            V.noalias() += w * model.G.transpose() * model.G;
            rhs.noalias() += w * model.G.transpose() * model.z;
          }
          const Eigen::Vector3d X = V.ldlt().solve(rhs);
          points->col(j) = X;
          double cost = 0.0;
          for (int k = first; k < last; ++k) {
            formulation.ObservationModel(
                cameras[problem.observation_camera[k]],
                problem.observations.col(k),
                context.State(k),
                &model);
            cost += context.Cost((model.G * X - model.z).squaredNorm());
          }
          point_cost[j] = cost;
        }
      });
  return std::accumulate(point_cost.begin(), point_cost.end(), 0.0) +
         RotationTermsCost(problem, formulation, cameras);
}

// Recomputes the IRLS weights (robust loss) and/or the linearization states
// from the current cameras and points. Returns true if anything changed.
bool UpdateObservationContext(const OseProblem& problem,
                              const OseFormulation& formulation,
                              const OseCameraVector& cameras,
                              const Eigen::Matrix3Xd& points,
                              const bool update_weights,
                              const bool update_states,
                              const int num_threads,
                              ObservationContext* context) {
  if (!update_weights && !update_states) {
    return false;
  }
  const int num_points = problem.NumPoints();
  ParallelFor(
      num_threads,
      num_points,
      kMinPointsPerChunk,
      [&](const int begin, const int end, int) {
        OseObservationModel model;
        for (int j = begin; j < end; ++j) {
          const Eigen::Vector3d X = points.col(j);
          for (int k = problem.point_offsets[j];
               k < problem.point_offsets[j + 1];
               ++k) {
            const OseCamera& P = cameras[problem.observation_camera[k]];
            const Eigen::Vector2d m = problem.observations.col(k);
            if (update_weights) {
              formulation.ObservationModel(P, m, context->State(k), &model);
              context->weights[k] = context->RobustWeight(
                  (model.G * X - model.z).squaredNorm());
            }
            if (update_states) {
              context->states[k] = formulation.UpdatedState(P, m, X);
            }
          }
        }
      });
  return true;
}

// Builds the block system and the reduced right-hand side
//   rhs = -(g_u - sum_j W_j^T V_j^{-1} g_v_j).
// At the exact point solution g_v_j is (numerically) zero; it is kept so that
// the step stays correct if the point solve is regularized.
//
// Pass 1 (parallel over points) forms V_j^-1, the W blocks and V_j^-1 g_v_j;
// pass 2 (parallel over cameras) accumulates the diagonal camera blocks and
// the gradient from the camera-major index. Both passes write only to their
// own points / cameras, so the result does not depend on the thread count.
// The structure fields of `system` (offsets, W_camera, W_point, camera index)
// must already be set.
void BuildReducedSystem(const OseProblem& problem,
                        const OseFormulation& formulation,
                        const OseCameraVector& cameras,
                        const Eigen::Matrix3Xd& points,
                        const double point_ridge,
                        const ObservationContext& context,
                        ReducedCameraSystem* system,
                        Eigen::VectorXd* rhs) {
  const int n = problem.NumCameras();
  const int num_points = problem.NumPoints();
  const int num_threads = system->num_threads;
  system->V_inverse.resize(num_points);
  system->W_blocks.resize(problem.NumObservations());
  ++system->revision;

  Eigen::Matrix3Xd Vinv_gv(3, num_points);
  ParallelFor(
      num_threads,
      num_points,
      kMinPointsPerChunk,
      [&](const int begin, const int end, int) {
        OseObservationModel model;
        OseCameraJacobian J;
        for (int j = begin; j < end; ++j) {
          const Eigen::Vector3d X = points.col(j);
          Eigen::Matrix3d V = point_ridge * Eigen::Matrix3d::Identity();
          Eigen::Vector3d g_v = Eigen::Vector3d::Zero();
          for (int k = problem.point_offsets[j];
               k < problem.point_offsets[j + 1];
               ++k) {
            const int c = problem.observation_camera[k];
            const Eigen::Vector2d m = problem.observations.col(k);
            const double state = context.State(k);
            const double w = context.Weight(k);
            formulation.ObservationModel(cameras[c], m, state, &model);
            formulation.CameraJacobian(cameras[c], m, X, state, &J);
            const auto r = (model.G * X - model.z).eval();
            system->W_blocks[k].noalias() = w * model.G.transpose() * J;
            V.noalias() += w * model.G.transpose() * model.G;
            g_v.noalias() += w * model.G.transpose() * r;
          }
          system->V_inverse[j] = V.inverse();
          Vinv_gv.col(j) = system->V_inverse[j] * g_v;
        }
      });

  system->U_diagonal.resize(n);
  Eigen::VectorXd g(kB * n);
  ParallelFor(
      num_threads,
      n,
      kMinCamerasPerChunk,
      [&](const int begin, const int end, int) {
        OseObservationModel model;
        OseCameraJacobian J;
        for (int c = begin; c < end; ++c) {
          Matrix12d U = Matrix12d::Zero();
          Eigen::Matrix<double, kB, 1> gc = Eigen::Matrix<double, kB, 1>::Zero();
          for (int idx = system->camera_offsets[c];
               idx < system->camera_offsets[c + 1];
               ++idx) {
            const int k = system->camera_W_index[idx];
            const int j = problem.observation_point[k];
            const Eigen::Vector3d X = points.col(j);
            const Eigen::Vector2d m = problem.observations.col(k);
            const double state = context.State(k);
            const double w = context.Weight(k);
            formulation.ObservationModel(cameras[c], m, state, &model);
            formulation.CameraJacobian(cameras[c], m, X, state, &J);
            const auto r = (model.G * X - model.z).eval();
            U.noalias() += w * J.transpose() * J;
            gc.noalias() += w * J.transpose() * r;
            // Reduced gradient: subtract W_k^T V_j^-1 g_v_j.
            gc.noalias() -= system->W_blocks[k].transpose() * Vinv_gv.col(j);
          }
          system->U_diagonal[c] = U;
          g.segment<kB>(kB * c) = gc;
        }
      });

  system->U_off_diagonal.clear();
  if (formulation.UsesRotationTerms()) {
    Eigen::Matrix<double, 9, 1> r;
    OseRotationJacobian Ja, Jb;
    for (const OseRotationTerm& term : problem.rotation_terms) {
      const int a = term.camera_a;
      const int c = term.camera_b;
      OseFormulation::EvaluateRotationTerm(
          term, cameras[a], cameras[c], &r, &Ja, &Jb);
      system->U_diagonal[a].noalias() += Ja.transpose() * Ja;
      g.segment<kB>(kB * a).noalias() += Ja.transpose() * r;
      if (a == c) {
        continue;
      }
      system->U_diagonal[c].noalias() += Jb.transpose() * Jb;
      g.segment<kB>(kB * c).noalias() += Jb.transpose() * r;
      const Matrix12d ab = Ja.transpose() * Jb;
      system->U_off_diagonal.push_back({a, c, ab});
      system->U_off_diagonal.push_back({c, a, ab.transpose()});
    }
    std::stable_sort(system->U_off_diagonal.begin(),
                     system->U_off_diagonal.end(),
                     [](const auto& x, const auto& y) {
                       return x.camera_a < y.camera_a;
                     });
  }
  system->off_diagonal_offsets.assign(n + 1, 0);
  for (const auto& pair : system->U_off_diagonal) {
    ++system->off_diagonal_offsets[pair.camera_a + 1];
  }
  for (int i = 0; i < n; ++i) {
    system->off_diagonal_offsets[i + 1] += system->off_diagonal_offsets[i];
  }

  *rhs = -g;
}

}  // namespace

double EvaluateOseCost(const OseProblem& problem,
                       const OseFormulation& formulation,
                       const OseCameraVector& cameras,
                       const double point_ridge,
                       Eigen::Matrix3Xd* points) {
  ObservationContext context;
  if (formulation.HasState()) {
    context.states.resize(problem.NumObservations());
    for (int k = 0; k < problem.NumObservations(); ++k) {
      context.states[k] =
          formulation.InitialState(problem.observations.col(k));
    }
  }
  return EvaluateCost(
      problem, formulation, cameras, point_ridge, context, 1, points);
}

VarProOptions ResolvedVarProOptions(const InitFreeOptions& options) {
  VarProOptions varpro = options.varpro;
  if (varpro.num_threads <= 0) {
    varpro.num_threads = std::max(1, options.num_threads);
  }
  return varpro;
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
  if (options.robust_loss_type != VarProRobustLossType::NONE &&
      !(options.robust_loss_scale > 0.0)) {
    solution->message = "robust_loss_scale must be positive.";
    return false;
  }
  std::string error;
  std::unique_ptr<ReducedCameraSolver> linear_solver =
      CreateReducedCameraSolver(options, &error);
  if (linear_solver == nullptr) {
    solution->message = error;
    return false;
  }
  const int num_threads = std::max(1, options.num_threads);
  const bool robust = options.robust_loss_type != VarProRobustLossType::NONE;

  OseCameraVector cameras = initial_cameras;
  for (OseCamera& P : cameras) {
    formulation.NormalizeInitialCamera(&P);
  }

  ObservationContext context;
  context.robust_loss = options.robust_loss_type;
  context.robust_scale = options.robust_loss_scale;
  if (robust) {
    context.weights.assign(problem.NumObservations(), 1.0);
  }
  if (formulation.HasState()) {
    context.states.resize(problem.NumObservations());
    for (int k = 0; k < problem.NumObservations(); ++k) {
      context.states[k] =
          formulation.InitialState(problem.observations.col(k));
    }
  }

  Eigen::Matrix3Xd points;
  double cost = EvaluateCost(problem,
                             formulation,
                             cameras,
                             options.point_ridge,
                             context,
                             num_threads,
                             &points);
  solution->initial_cost = cost;
  solution->cost_history.push_back(cost);

  // Structure of the reduced system; the values are rebuilt every iteration.
  ReducedCameraSystem system;
  system.num_cameras = problem.NumCameras();
  system.num_threads = num_threads;
  system.point_offsets = problem.point_offsets;
  system.W_camera = problem.observation_camera;
  system.W_point = problem.observation_point;
  system.BuildCameraIndex();

  Eigen::VectorXd rhs, step;
  OseCameraVector candidate(cameras.size());
  Eigen::Matrix3Xd candidate_points;
  double lambda = options.initial_lambda;
  std::string termination = "Maximum number of iterations reached.";

  int iteration = 0;
  for (; iteration < options.max_num_iterations; ++iteration) {
    // Re-linearize the least-squares model (IRLS weights, expOSE state) at
    // the current cameras and points; the step is then taken on that model.
    const bool update_states =
        formulation.HasState() &&
        iteration >= formulation.UpdateStateFromIteration();
    if (UpdateObservationContext(problem,
                                 formulation,
                                 cameras,
                                 points,
                                 robust,
                                 update_states,
                                 num_threads,
                                 &context)) {
      cost = EvaluateCost(problem,
                          formulation,
                          cameras,
                          options.point_ridge,
                          context,
                          num_threads,
                          &points);
    }
    BuildReducedSystem(problem,
                       formulation,
                       cameras,
                       points,
                       options.point_ridge,
                       context,
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
                step.data() + kB * i);
        candidate[i] = cameras[i] + delta;
      }
      const double candidate_cost = EvaluateCost(problem,
                                                 formulation,
                                                 candidate,
                                                 options.point_ridge,
                                                 context,
                                                 num_threads,
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
