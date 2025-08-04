// Copyright (C) 2025 Steffen Urban
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
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_ORIENTATION_PRIOR_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_ORIENTATION_PRIOR_ERROR_H_

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Sophus/sophus/se3.hpp>
#include <Sophus/sophus/sim3.hpp>

namespace theia {

// Orientation Prior Error Term 
struct OrientationPriorError {
 public:
  explicit OrientationPriorError(
      const Eigen::Vector3d& prior_orientation,
      const Eigen::Matrix3d& sqrt_information)
      : prior_orientation_(prior_orientation), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* camera_extrinsics, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> current_orientation(
      camera_extrinsics + Camera::ORIENTATION);
    Sophus::SO3<T> current_orientation_so3 = Sophus::SO3<T>::exp(current_orientation);
    Sophus::SO3<T> prior_orientation_so3 = Sophus::SO3<T>::exp(prior_orientation_.cast<T>());
    Sophus::SO3<T> error = current_orientation_so3 * prior_orientation_so3.inverse();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residual);
    residuals = sqrt_information_.cast<T>() *  error.log();

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& prior_orientation,
      const Eigen::Matrix3d& sqrt_information) {
    static const int kParameterSize = 6;
    static const int kNumResiduals = 3;
    return new ceres::
        AutoDiffCostFunction<OrientationPriorError, kNumResiduals, kParameterSize>(
            new OrientationPriorError(prior_orientation, sqrt_information));
  }

 private:
  const Eigen::Vector3d prior_orientation_;
  const Eigen::Matrix3d sqrt_information_;
};
}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_ORIENTATION_PRIOR_ERROR_H_
