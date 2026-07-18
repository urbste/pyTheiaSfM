// Copyright (c) 2023, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib) robust/optim/lm_impl.h,
// commit fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// Dense Levenberg-Marquardt for small geometric models (RANSAC LO).
//
// Problem must provide:
//   int NumParams() const;
//   using Model = ...;   (or pass Model explicitly)
//   double ComputeResidual(NormalAccumulator& acc, const Model& model);
//   void ComputeJacobian(NormalAccumulator& acc, const Model& model);
//   Model Step(const Eigen::VectorXd& dp, const Model& model) const;

#ifndef THEIA_MATH_LMLSQ_LM_OPTIMIZER_H_
#define THEIA_MATH_LMLSQ_LM_OPTIMIZER_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>

#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/math/lmlsq/robust_loss.h"

namespace theia {

template <typename Problem, typename Model = typename Problem::Model>
LmStats MinimizeLM(Problem& problem, Model* parameters, const LmOptions& opt) {
  LmStats stats;
  NormalAccumulator acc;
  if (opt.loss_scale > 0.0) {
    acc.Initialize(problem.NumParams(),
                   std::make_shared<TruncatedLoss>(opt.loss_scale));
  } else {
    acc.InitializeTrivial(problem.NumParams());
  }

  acc.ResetResidual();
  stats.final_cost = problem.ComputeResidual(acc, *parameters);
  stats.initial_cost = stats.final_cost;
  stats.lambda = opt.initial_lambda;
  double nu = 2.0;

  bool recompute_jac = true;
  for (stats.iterations = 0; stats.iterations < opt.max_iterations;
       ++stats.iterations) {
    if (recompute_jac) {
      acc.ResetJacobian();
      problem.ComputeJacobian(acc, *parameters);
      stats.grad_norm = acc.GradNorm();
      if (stats.grad_norm < opt.gradient_tol) {
        break;
      }
    }

    const Eigen::VectorXd sol = acc.Solve(stats.lambda);
    stats.step_norm = sol.norm();
    if (stats.step_norm < opt.step_tol) {
      break;
    }

    const Model parameters_new = problem.Step(sol, *parameters);
    acc.ResetResidual();
    const double cost_new = problem.ComputeResidual(acc, parameters_new);

    if (cost_new < stats.final_cost) {
      const double cost_decrease = stats.final_cost - cost_new;
      *parameters = parameters_new;
      stats.final_cost = cost_new;
      recompute_jac = true;

      // Nielsen lambda update.
      const double predicted = acc.PredictedDecrease(sol, stats.lambda);
      if (predicted > 0.0) {
        const double rho = cost_decrease / predicted;
        const double factor = 1.0 - std::pow(2.0 * rho - 1.0, 3);
        stats.lambda *= std::max(1.0 / 3.0, factor);
      } else {
        stats.lambda *= 1.0 / 3.0;
      }
      nu = 2.0;
      stats.lambda = std::max(opt.min_lambda, stats.lambda);

      if (stats.final_cost > 0.0 &&
          cost_decrease / stats.final_cost < opt.relative_cost_tol) {
        break;
      }
    } else {
      ++stats.invalid_steps;
      recompute_jac = false;
      stats.lambda *= nu;
      nu *= 2.0;
      stats.lambda = std::min(opt.max_lambda, stats.lambda);
    }
  }
  return stats;
}

}  // namespace theia

#endif  // THEIA_MATH_LMLSQ_LM_OPTIMIZER_H_
