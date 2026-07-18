// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Options / stats for the dense Levenberg-Marquardt local optimizer used by
// RANSAC RefineModel.

#ifndef THEIA_MATH_LMLSQ_LM_OPTIONS_H_
#define THEIA_MATH_LMLSQ_LM_OPTIONS_H_

#include <cstddef>

namespace theia {

struct LmOptions {
  size_t max_iterations = 25;
  // Truncation threshold on squared residual (matches RANSAC error_thresh for
  // Sampson distance). Set <= 0 to use a trivial (non-robust) loss.
  double loss_scale = 1.0;
  double gradient_tol = 1e-12;
  double step_tol = 1e-8;
  double relative_cost_tol = 1e-10;
  double initial_lambda = 1e-3;
  double min_lambda = 1e-10;
  double max_lambda = 1e10;
};

struct LmStats {
  size_t iterations = 0;
  size_t invalid_steps = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double lambda = 0.0;
  double grad_norm = -1.0;
  double step_norm = -1.0;
};

}  // namespace theia

#endif  // THEIA_MATH_LMLSQ_LM_OPTIONS_H_
