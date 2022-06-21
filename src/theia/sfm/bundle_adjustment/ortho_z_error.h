// Copyright (C) 2022 Steffen Urban
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

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_ORTHOZ_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_ORTHOZ_ERROR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>

namespace theia {

struct OrthographicZConstrain {
 public:
  explicit OrthographicZConstrain(const double& ortho_z_sqrt_information)
      : ortho_z_sqrt_information_(ortho_z_sqrt_information) {}

  template <typename T>
  bool operator()(const T* camera_extrinsics, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_w_c(camera_extrinsics +
                                                      Camera::POSITION);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> r_c_w(camera_extrinsics +
                                                      Camera::ORIENTATION);
    Eigen::Matrix<T, 3, 3> R_c_w;       
    ceres::AngleAxisToRotationMatrix(r_c_w.data(), R_c_w.data());                                  
    Eigen::Matrix<T, 3, 1> t_c_w = -R_c_w * p_w_c;
    // z should stay zero
    residual[0] = T(ortho_z_sqrt_information_) * t_c_w(2);
    return true;
  }

  static ceres::CostFunction* Create(
      const double& ortho_z_sqrt_information) {
    static const int kParameterSize = 6;
    static const int kNumResiduals = 1;
    return new ceres::
        AutoDiffCostFunction<OrthographicZConstrain, kNumResiduals, kParameterSize>(
            new OrthographicZConstrain(ortho_z_sqrt_information));
  }

 private:
  const double ortho_z_sqrt_information_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_ORTHOZ_ERROR_H_
