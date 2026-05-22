// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_ERRORS_H_
#define THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_ERRORS_H_

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Sophus/sophus/sim3.hpp>

namespace theia {

// Full Sim(3) anchor: log(S_meas^{-1} * S_run) with fixed PnP measurement S_meas.
struct Sim3AbsoluteAnchorPoseErrorTerm {
  explicit Sim3AbsoluteAnchorPoseErrorTerm(const Sophus::Sim3d& S_meas,
                                           double weight)
      : S_meas_(S_meas), weight_(weight) {}

  template <typename T>
  bool operator()(const T* const lie_run, T* residuals) const {
    const Sophus::Sim3<T> S_run =
        Sophus::Sim3<T>::exp(Eigen::Map<const Eigen::Matrix<T, 7, 1>>(lie_run));
    const Sophus::Sim3<T> S_meas_T = S_meas_.cast<T>();
    const Sophus::Sim3<T> error = S_meas_T.inverse() * S_run;
    Eigen::Map<Eigen::Matrix<T, 7, 1>> r(residuals);
    r = T(weight_) * error.log();
    return true;
  }

  static ceres::CostFunction* Create(const Sophus::Sim3d& S_meas, double weight) {
    return new ceres::AutoDiffCostFunction<Sim3AbsoluteAnchorPoseErrorTerm, 7, 7>(
        new Sim3AbsoluteAnchorPoseErrorTerm(S_meas, weight));
  }

  const Sophus::Sim3d S_meas_;
  const double weight_;
};

struct Sim3ScaleSmoothnessErrorTerm {
  explicit Sim3ScaleSmoothnessErrorTerm(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T* const lie_i, const T* const lie_j, T* residual) const {
    const Sophus::Sim3<T> S_i = Sophus::Sim3<T>::exp(
        Eigen::Map<const Eigen::Matrix<T, 7, 1>>(lie_i));
    const Sophus::Sim3<T> S_j = Sophus::Sim3<T>::exp(
        Eigen::Map<const Eigen::Matrix<T, 7, 1>>(lie_j));
    residual[0] = T(weight_) * (S_j.scale() - S_i.scale());
    return true;
  }

  static ceres::CostFunction* Create(double weight) {
    return new ceres::AutoDiffCostFunction<Sim3ScaleSmoothnessErrorTerm, 1, 7, 7>(
        new Sim3ScaleSmoothnessErrorTerm(weight));
  }

  const double weight_;
};

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_CROSS_RECONSTRUCTION_POSE_GRAPH_ERRORS_H_
