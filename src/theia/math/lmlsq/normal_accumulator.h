// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib)
// robust/optim/jacobian_accumulator.h, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// Aggregates normal equations (JᵀJ) δ = -Jᵀr for small dense LM problems.

#ifndef THEIA_MATH_LMLSQ_NORMAL_ACCUMULATOR_H_
#define THEIA_MATH_LMLSQ_NORMAL_ACCUMULATOR_H_

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "theia/math/lmlsq/robust_loss.h"

namespace theia {

class NormalAccumulator {
 public:
  void Initialize(int num_params, std::shared_ptr<TruncatedLoss> loss) {
    num_params_ = num_params;
    JtJ_.resize(num_params, num_params);
    Jtr_.resize(num_params);
    loss_ = std::move(loss);
    if (!loss_) {
      // Non-robust: emulate trivial loss via a huge truncation threshold.
      loss_ = std::make_shared<TruncatedLoss>(
          std::numeric_limits<double>::infinity());
    }
  }

  void InitializeTrivial(int num_params) {
    Initialize(num_params, nullptr);
  }

  void ResetResidual() {
    residual_acc_ = 0.0;
    residual_count_ = 0;
  }

  void AddResidual(double res, double w = 1.0) {
    residual_acc_ += w * loss_->Loss(res * res);
    ++residual_count_;
  }

  template <int ResidualDim>
  void AddResidual(const Eigen::Matrix<double, ResidualDim, 1>& res,
                   double w = 1.0) {
    residual_acc_ += w * loss_->Loss(res.squaredNorm());
    ++residual_count_;
  }

  double Cost() const { return residual_acc_ * ResidualScale(); }

  void ResetJacobian() {
    residual_count_ = 0;
    JtJ_.setZero();
    Jtr_.setZero();
  }

  // 1-D residual.
  template <int ParamsDim>
  void AddJacobian(double res,
                   const Eigen::Matrix<double, 1, ParamsDim>& jac,
                   double w = 1.0) {
    const double r2 = res * res;
    const double weight = w * loss_->Weight(r2);
    if (weight == 0.0) {
      return;
    }
    for (int i = 0; i < jac.cols(); ++i) {
      for (int j = 0; j <= i; ++j) {
        JtJ_(i, j) += weight * (jac(i) * jac(j));
      }
    }
    Jtr_ += (weight * res) * jac.transpose();
    ++residual_count_;
  }

  // Dynamic-width 1-D residual Jacobian.
  void AddJacobian(double res, const Eigen::RowVectorXd& jac, double w = 1.0) {
    const double r2 = res * res;
    const double weight = w * loss_->Weight(r2);
    if (weight == 0.0) {
      return;
    }
    for (int i = 0; i < jac.cols(); ++i) {
      for (int j = 0; j <= i; ++j) {
        JtJ_(i, j) += weight * (jac(i) * jac(j));
      }
    }
    Jtr_ += (weight * res) * jac.transpose();
    ++residual_count_;
  }

  template <int ResidualDim, int ParamsDim>
  void AddJacobian(const Eigen::Matrix<double, ResidualDim, 1>& res,
                   const Eigen::Matrix<double, ResidualDim, ParamsDim>& jac,
                   double w = 1.0) {
    const double r2 = res.squaredNorm();
    const double weight = w * loss_->Weight(r2);
    if (weight == 0.0) {
      return;
    }
    for (int i = 0; i < jac.cols(); ++i) {
      for (int j = 0; j <= i; ++j) {
        JtJ_(i, j) += weight * (jac.col(i).dot(jac.col(j)));
      }
    }
    Jtr_ += jac.transpose() * (weight * res);
    ++residual_count_;
  }

  // Dynamic Jacobian (e.g. 2 x n for monodepth reprojection).
  template <int ResidualDim>
  void AddJacobian(const Eigen::Matrix<double, ResidualDim, 1>& res,
                   const Eigen::MatrixXd& jac,
                   double w = 1.0) {
    const double r2 = res.squaredNorm();
    const double weight = w * loss_->Weight(r2);
    if (weight == 0.0) {
      return;
    }
    for (int i = 0; i < jac.cols(); ++i) {
      for (int j = 0; j <= i; ++j) {
        JtJ_(i, j) += weight * (jac.col(i).dot(jac.col(j)));
      }
    }
    Jtr_ += jac.transpose() * (weight * res);
    ++residual_count_;
  }

  double GradNorm() const { return ResidualScale() * Jtr_.norm(); }

  Eigen::VectorXd Solve(double lambda) const {
    const double scale = ResidualScale();
    Eigen::MatrixXd scaled_JtJ = scale * JtJ_;
    for (int i = 0; i < scaled_JtJ.cols(); ++i) {
      scaled_JtJ(i, i) += lambda;
    }
    return scaled_JtJ.selfadjointView<Eigen::Lower>().llt().solve(
        -(scale * Jtr_));
  }

  double PredictedDecrease(const Eigen::VectorXd& step, double lambda) const {
    const double scale = ResidualScale();
    return -step.dot(lambda * step + scale * Jtr_);
  }

  const Eigen::MatrixXd& JtJ() const { return JtJ_; }

 private:
  double ResidualScale() const {
    return 1.0 / std::max(1.0, static_cast<double>(residual_count_));
  }

  int num_params_ = 0;
  double residual_acc_ = 0.0;
  size_t residual_count_ = 0;
  Eigen::MatrixXd JtJ_;
  Eigen::VectorXd Jtr_;
  std::shared_ptr<TruncatedLoss> loss_;
};

}  // namespace theia

#endif  // THEIA_MATH_LMLSQ_NORMAL_ACCUMULATOR_H_
