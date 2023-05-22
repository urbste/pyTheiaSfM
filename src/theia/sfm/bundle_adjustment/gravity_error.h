// Copyright (C) 2023 Steffen Urban
// All rights reserved.
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

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_GRAVITY_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_GRAVITY_ERROR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>

namespace theia {

struct GravityError {
 public:
  explicit GravityError(const Eigen::Vector3d& gravity_prior,
                        const Eigen::Matrix3d& gravity_prior_sqrt_information)
      : gravity_prior_(gravity_prior),
        gravity_prior_sqrt_information_(gravity_prior_sqrt_information) {}

  template <typename T>
  bool operator()(const T* camera_extrinsics, T* residual) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residual);
    Eigen::Matrix<T, 3, 1> g_world(T(0), T(0), T(-1));
    Eigen::Matrix<T, 3, 1> gravity_in_camera;
    // g_camera = R_c_w * g_world = R_c_w * (0,0,-1)
    ceres::AngleAxisRotatePoint(camera_extrinsics + Camera::ORIENTATION,
                                g_world.data(),
                                gravity_in_camera.data());

    res = gravity_prior_sqrt_information_.cast<T>() * (
          gravity_in_camera - gravity_prior_.cast<T>());
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& gravity_prior,
      const Eigen::Matrix3d& gravity_prior_sqrt_information) {
    static const int kParameterSize = 6;
    static const int kNumResiduals = 3;
    return new ceres::
        AutoDiffCostFunction<GravityError, kNumResiduals, kParameterSize>(
            new GravityError(gravity_prior, gravity_prior_sqrt_information));
  }

 private:
  const Eigen::Vector3d gravity_prior_;
  const Eigen::Matrix3d gravity_prior_sqrt_information_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_GRAVITY_ERROR_H_
