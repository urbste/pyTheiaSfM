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
namespace sim3_pose_graph_errors {

// Jet-safe rotation residual: 2 * imaginary part of R_pred * R_meas^T.
template <typename T>
inline Eigen::Matrix<T, 3, 1> RotationResidualFromSim3Poses(
    const Sophus::Sim3<T>& S_pred, const Sophus::Sim3<T>& S_meas) {
  const Eigen::Matrix<T, 3, 3> R_err =
      S_pred.rotationMatrix() * S_meas.rotationMatrix().transpose();
  const Eigen::Quaternion<T> q_err(R_err);
  return T(2.0) * q_err.vec();
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> RotationResidualFromSim3Error(
    const Sophus::Sim3<T>& error) {
  const Eigen::Quaternion<T> q_err(error.rotationMatrix());
  return T(2.0) * q_err.vec();
}

}  // namespace sim3_pose_graph_errors

// Scale-free sequential odometry: rotation + translation direction + weak
// magnitude tied to sigma_i. Preserves local trajectory shape while allowing
// global scale to adapt via per-view sigma and anchor constraints.
struct ScaleFreeSequentialSim3ErrorTerm {
  ScaleFreeSequentialSim3ErrorTerm(
      const Sophus::Sim3d& S_meas_ji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information,
      double translation_magnitude_sqrt_weight)
      : S_meas_ji_(S_meas_ji),
        sqrt_information_(sqrt_information),
        translation_magnitude_sqrt_weight_(translation_magnitude_sqrt_weight) {}

  template <typename T>
  bool operator()(const T* const lie_i, const T* const lie_j, T* residual) const {
    const Sophus::Sim3<T> S_i = Sophus::Sim3<T>::exp(
        Eigen::Map<const Eigen::Matrix<T, 7, 1>>(lie_i));
    const Sophus::Sim3<T> S_j = Sophus::Sim3<T>::exp(
        Eigen::Map<const Eigen::Matrix<T, 7, 1>>(lie_j));
    const Sophus::Sim3<T> S_pred = S_i.inverse() * S_j;
    const Sophus::Sim3<T> S_meas = S_meas_ji_.cast<T>();

    Eigen::Map<Eigen::Matrix<T, 7, 1>> residuals(residual);
    residuals.setZero();

    residuals.template head<3>() =
        sim3_pose_graph_errors::RotationResidualFromSim3Poses(S_pred, S_meas);

    const Eigen::Matrix<T, 3, 1> t_pred = S_pred.translation();
    const Eigen::Matrix<T, 3, 1> t_meas = S_meas.translation();
    const T eps = T(1e-8);
    const T pred_norm = t_pred.norm();
    const T meas_norm = t_meas.norm();
    if (pred_norm > eps && meas_norm > eps) {
      const Eigen::Matrix<T, 3, 1> dir_pred = t_pred / pred_norm;
      const Eigen::Matrix<T, 3, 1> dir_meas = t_meas / meas_norm;
      residuals.template segment<3>(3) = dir_pred.cross(dir_meas);
      const T sigma_i = lie_i[6];
      residuals(6) = T(translation_magnitude_sqrt_weight_) *
                     ceres::log(pred_norm / (ceres::exp(sigma_i) * meas_norm));
    }

    residuals = sqrt_information_.cast<T>() * residuals;
    return true;
  }

  static ceres::CostFunction* Create(
      const Sophus::Sim3d& S_meas_ji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information,
      double translation_magnitude_sqrt_weight) {
    return new ceres::AutoDiffCostFunction<ScaleFreeSequentialSim3ErrorTerm,
                                           7, 7, 7>(
        new ScaleFreeSequentialSim3ErrorTerm(S_meas_ji,
                                             sqrt_information,
                                             translation_magnitude_sqrt_weight));
  }

  const Sophus::Sim3d S_meas_ji_;
  const Eigen::Matrix<double, 7, 7> sqrt_information_;
  const double translation_magnitude_sqrt_weight_;
};

// Jet-safe absolute anchor: rotation + translation + log-scale vs PnP target.
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
    r.template head<3>() =
        T(weight_) *
        sim3_pose_graph_errors::RotationResidualFromSim3Error(error);
    r.template segment<3>(3) = T(weight_) * error.translation();

    const T scale_error = error.scale();
    const T eps = T(1e-8);
    if (scale_error > eps) {
      r(6) = T(weight_) * ceres::log(scale_error);
    } else {
      r(6) = T(0);
    }
    return true;
  }

  static ceres::CostFunction* Create(const Sophus::Sim3d& S_meas, double weight) {
    return new ceres::AutoDiffCostFunction<Sim3AbsoluteAnchorPoseErrorTerm, 7, 7>(
        new Sim3AbsoluteAnchorPoseErrorTerm(S_meas, weight));
  }

  const Sophus::Sim3d S_meas_;
  const double weight_;
};

// Log-scale smoothness on lie[6] (sigma), Jet-safe.
struct Sim3ScaleSmoothnessErrorTerm {
  explicit Sim3ScaleSmoothnessErrorTerm(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T* const lie_i, const T* const lie_j, T* residual) const {
    residual[0] = T(weight_) * (lie_j[6] - lie_i[6]);
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
