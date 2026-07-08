// Copyright (C) 2026 Steffen Urban
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_SCALED_RELATIVE_POSE_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_SCALED_RELATIVE_POSE_ERROR_H_

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Sophus/sophus/se3.hpp>

#include "theia/sfm/camera/camera.h"

namespace theia {

// Scale-aware relative SE3 pose-to-pose error between two cameras.
//
// Rotation is penalized with the SE3 log rotation component (3 residuals).
// Translation is split into:
//   - direction: cross(predicted, measured) on normalized translations (3)
//   - magnitude: log(|predicted| / |measured|) (1)
// so local trajectory shape is preserved while scale may vary along the chain.
struct ScaledRelativePoseError {
 public:
  ScaledRelativePoseError(const Sophus::SE3d& measured_i_to_j,
                          double rotation_sqrt_weight,
                          double translation_direction_sqrt_weight,
                          double translation_magnitude_sqrt_weight)
      : measured_i_to_j_(measured_i_to_j),
        rotation_sqrt_weight_(rotation_sqrt_weight),
        translation_direction_sqrt_weight_(translation_direction_sqrt_weight),
        translation_magnitude_sqrt_weight_(translation_magnitude_sqrt_weight) {}

  template <typename T>
  bool operator()(const T* extrinsics_i,
                  const T* extrinsics_j,
                  T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> center_i(extrinsics_i +
                                                      Camera::POSITION);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> aa_i(extrinsics_i +
                                                  Camera::ORIENTATION);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> center_j(extrinsics_j +
                                                      Camera::POSITION);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> aa_j(extrinsics_j +
                                                  Camera::ORIENTATION);

    for (int k = 0; k < 3; ++k) {
      if (!ceres::isfinite(aa_i[k]) || !ceres::isfinite(aa_j[k]) ||
          !ceres::isfinite(center_i[k]) || !ceres::isfinite(center_j[k])) {
        return false;
      }
    }
    // A per-element isfinite check is NOT sufficient: a huge-but-finite
    // angle-axis (e.g. a diverging LM step of order 1e258) passes isfinite yet
    // overflows squaredNorm() to +Inf inside Sophus::SO3::exp -> sqrt(Inf)=Inf
    // -> sin/cos(Inf)=NaN -> NaN quaternion -> SOPHUS_ENSURE aborts the whole
    // process. A sane rotation angle-axis has magnitude <= a few * pi, so bound
    // the squared norm well below the overflow threshold and reject otherwise.
    constexpr double kMaxAngleAxisSqNorm = 1e12;  // |omega| ~ 1e6 rad
    if (aa_i.squaredNorm() > T(kMaxAngleAxisSqNorm) ||
        aa_j.squaredNorm() > T(kMaxAngleAxisSqNorm)) {
      return false;
    }

    const Sophus::SO3<T> R_i = Sophus::SO3<T>::exp(aa_i);
    const Sophus::SO3<T> R_j = Sophus::SO3<T>::exp(aa_j);
    const Sophus::SE3<T> g_i(R_i, -(R_i * center_i));
    const Sophus::SE3<T> g_j(R_j, -(R_j * center_j));

    const Sophus::SE3<T> predicted_i_to_j = g_j * g_i.inverse();
    const Sophus::SE3<T> error =
        predicted_i_to_j * measured_i_to_j_.cast<T>().inverse();

    const Eigen::Matrix<T, 6, 1> log_error = error.log();
    for (int k = 0; k < 6; ++k) {
      if (!ceres::isfinite(log_error[k])) {
        return false;
      }
    }

    Eigen::Map<Eigen::Matrix<T, 7, 1>> residuals(residual);
    residuals.setZero();

    residuals.template head<3>() =
        T(rotation_sqrt_weight_) * log_error.template tail<3>();

    const Eigen::Matrix<T, 3, 1> t_pred = predicted_i_to_j.translation();
    const Eigen::Matrix<T, 3, 1> t_meas =
        measured_i_to_j_.translation().cast<T>();

    const T eps = T(1e-8);
    const T pred_norm = t_pred.norm();
    const T meas_norm = t_meas.norm();
    if (pred_norm > eps && meas_norm > eps) {
      const Eigen::Matrix<T, 3, 1> dir_pred = t_pred / pred_norm;
      const Eigen::Matrix<T, 3, 1> dir_meas = t_meas / meas_norm;
      residuals.template segment<3>(3) =
          T(translation_direction_sqrt_weight_) * dir_pred.cross(dir_meas);
      residuals(6) = T(translation_magnitude_sqrt_weight_) *
                     ceres::log(pred_norm / meas_norm);
    }

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Sophus::SE3d& measured_i_to_j,
      double rotation_sqrt_weight,
      double translation_direction_sqrt_weight,
      double translation_magnitude_sqrt_weight) {
    static const int kParameterSize = 6;
    static const int kNumResiduals = 7;
    return new ceres::AutoDiffCostFunction<ScaledRelativePoseError,
                                           kNumResiduals,
                                           kParameterSize,
                                           kParameterSize>(
        new ScaledRelativePoseError(measured_i_to_j,
                                    rotation_sqrt_weight,
                                    translation_direction_sqrt_weight,
                                    translation_magnitude_sqrt_weight));
  }

 private:
  const Sophus::SE3d measured_i_to_j_;
  const double rotation_sqrt_weight_;
  const double translation_direction_sqrt_weight_;
  const double translation_magnitude_sqrt_weight_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_SCALED_RELATIVE_POSE_ERROR_H_
