// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "theia/math/lmlsq/lm_optimizer.h"
#include "theia/math/lmlsq/normal_accumulator.h"

namespace theia {
namespace {

// Fit y ≈ a*x + b by least squares; Model = [a, b].
class LinearFitProblem {
 public:
  using Model = Eigen::Vector2d;

  LinearFitProblem(const std::vector<double>& x, const std::vector<double>& y)
      : x_(x), y_(y) {}

  int NumParams() const { return 2; }

  double ComputeResidual(NormalAccumulator& acc, const Model& m) {
    for (size_t i = 0; i < x_.size(); ++i) {
      acc.AddResidual(m(0) * x_[i] + m(1) - y_[i]);
    }
    return acc.Cost();
  }

  void ComputeJacobian(NormalAccumulator& acc, const Model& m) {
    for (size_t i = 0; i < x_.size(); ++i) {
      const double r = m(0) * x_[i] + m(1) - y_[i];
      Eigen::Matrix<double, 1, 2> J;
      J << x_[i], 1.0;
      acc.AddJacobian(r, J);
    }
  }

  Model Step(const Eigen::VectorXd& dp, const Model& m) const {
    return m + dp;
  }

 private:
  const std::vector<double>& x_;
  const std::vector<double>& y_;
};

TEST(LmOptimizer, FitsNoisyLine) {
  // Ground truth: y = 2x + 3
  std::vector<double> x, y;
  for (int i = 0; i < 20; ++i) {
    x.push_back(static_cast<double>(i));
    y.push_back(2.0 * i + 3.0);
  }

  LinearFitProblem problem(x, y);
  Eigen::Vector2d model(0.0, 0.0);
  LmOptions opt;
  opt.max_iterations = 50;
  opt.loss_scale = -1.0;  // trivial loss
  opt.gradient_tol = 1e-14;

  const LmStats stats = MinimizeLM(problem, &model, opt);

  EXPECT_LT(stats.final_cost, stats.initial_cost);
  EXPECT_NEAR(model(0), 2.0, 1e-8);
  EXPECT_NEAR(model(1), 3.0, 1e-8);
  EXPECT_LT(stats.grad_norm, 1e-8);
}

TEST(LmOptimizer, TruncatedLossIgnoresOutliers) {
  std::vector<double> x, y;
  for (int i = 0; i < 20; ++i) {
    x.push_back(static_cast<double>(i));
    y.push_back(2.0 * i + 3.0);
  }
  // Gross outliers.
  x.push_back(100.0);
  y.push_back(-1000.0);

  LinearFitProblem problem(x, y);
  // Start near the inlier solution so truncated IRLS has support.
  Eigen::Vector2d model(1.5, 2.5);
  LmOptions opt;
  opt.max_iterations = 50;
  opt.loss_scale = 1.0;  // truncate r² > 1

  const LmStats stats = MinimizeLM(problem, &model, opt);
  EXPECT_LE(stats.final_cost, stats.initial_cost);
  EXPECT_NEAR(model(0), 2.0, 1e-2);
  EXPECT_NEAR(model(1), 3.0, 1e-2);
}

}  // namespace
}  // namespace theia
