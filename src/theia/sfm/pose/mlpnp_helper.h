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
// Author: Steffen Urban (urbste@gmail.com)

#ifndef THEIA_SFM_POSE_MLPNP_HELPER_H_
#define THEIA_SFM_POSE_MLPNP_HELPER_H_

#include <Eigen/Core>
#include <vector>

#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_absolute_pose.h"

namespace theia {

// Whitened tangent-plane residual r = W B^T y_hat with Z = R X + t.
Eigen::Matrix2d SqrtInformation2x2(const Eigen::Matrix2d& covariance);

Eigen::Matrix2d ProjectedFeatureSqrtInformation(
    const Eigen::Matrix<double, 3, 2>& basis,
    const Eigen::Matrix3d& feature_covariance,
    bool has_covariance);

// LM state: Z = R X + t (world-to-camera, same as MLPnP linear solver output).
using MLPnPLMState = AbsolutePoseLMState;

class MLPnPTangentRefiner {
 public:
  using Model = MLPnPLMState;
  static constexpr int kNumParams = 6;

  MLPnPTangentRefiner(const std::vector<Eigen::Vector2d>& norm_feature_points,
                      const std::vector<Eigen::Matrix3d>& feature_covariances,
                      const std::vector<Eigen::Vector3d>& world_points);

  int NumParams() const { return kNumParams; }

  double ComputeResidual(NormalAccumulator& acc, const Model& pose);
  void ComputeJacobian(NormalAccumulator& acc, const Model& pose);
  Model Step(const Eigen::VectorXd& dp, const Model& pose) const;

  void AccumulatePointResiduals(NormalAccumulator& acc, const Model& pose,
                                bool compute_jacobian) const;

 private:
  std::vector<Eigen::Vector3d> bearings_;
  std::vector<Eigen::Matrix<double, 3, 2>> bases_;
  std::vector<Eigen::Matrix2d> sqrt_information_;
  const std::vector<Eigen::Vector3d>& world_points_;
};

// Refines world-to-camera (R, t) in place using whitened tangent residuals.
// loss_scale <= 0 selects a non-robust (pure ML) refinement.
bool MLPnPGaussNewton(const std::vector<Eigen::Vector2d>& norm_feature_points,
                      const std::vector<Eigen::Matrix3d>& feature_covariances,
                      const std::vector<Eigen::Vector3d>& world_points,
                      int max_iterations,
                      Eigen::Matrix3d* rotation,
                      Eigen::Vector3d* translation,
                      double loss_scale = 0.0,
                      LmStats* stats = nullptr);

}  // namespace theia

#endif  // THEIA_SFM_POSE_MLPNP_HELPER_H_
