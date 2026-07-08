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
//
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_RELATIVE_POSE_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_RELATIVE_POSE_ERROR_H_

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Sophus/sophus/se3.hpp>

#include "theia/sfm/camera/camera.h"

namespace theia {

// Relative SE3 pose-to-pose error term between two cameras. Acts directly on the
// two cameras' extrinsics parameter blocks (layout [POSITION(3), ORIENTATION(3)]
// where ORIENTATION is the world->cam angle-axis and POSITION the camera center).
//
// Given the world->cam poses g_i, g_j built from the extrinsics, the predicted
// relative pose g_j * g_i^{-1} (cam_i -> cam_j) is compared to a fixed measured
// relative pose (typically snapshotted from the current trajectory, i.e. the
// run's own odometry). The residual is the SE3 logarithm of the discrepancy,
// weighted by a sqrt-information matrix (Sophus tangent order: [translation(3),
// rotation(3)]). This stiffens the local trajectory shape so that absolute
// anchor pulls propagate smoothly along the chain instead of being absorbed
// locally by the structure.
struct RelativePoseError {
 public:
  RelativePoseError(const Sophus::SE3d& measured_i_to_j,
                    const Eigen::Matrix<double, 6, 6>& sqrt_information)
      : measured_i_to_j_(measured_i_to_j),
        sqrt_information_(sqrt_information) {}

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

    // Guard against non-finite trial steps: Sophus::SO3/SE3::exp/log hard-abort
    // via SOPHUS_ENSURE on NaN/Inf input (e.g. a divergent Levenberg-Marquardt
    // step). Returning false lets Ceres reject the step and shrink the trust
    // region instead of aborting the whole process.
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
    // world->cam: x_cam = R (x_world - center) => translation = -R * center.
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

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);
    residuals = sqrt_information_.cast<T>() * log_error;
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Sophus::SE3d& measured_i_to_j,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    static const int kParameterSize = 6;
    static const int kNumResiduals = 6;
    return new ceres::AutoDiffCostFunction<RelativePoseError,
                                           kNumResiduals,
                                           kParameterSize,
                                           kParameterSize>(
        new RelativePoseError(measured_i_to_j, sqrt_information));
  }

 private:
  const Sophus::SE3d measured_i_to_j_;
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_RELATIVE_POSE_ERROR_H_
