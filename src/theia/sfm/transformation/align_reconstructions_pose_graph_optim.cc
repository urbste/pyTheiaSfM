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

#include "theia/sfm/transformation/align_reconstructions_pose_graph_optim.h"

namespace theia {

bool Sim3Parameterization::Plus(const double* x,
                                const double* delta,
                                double* x_plus_delta) const {
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > lie_x(x);
  Eigen::Matrix<double, 7, 1> lie_delta;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> updated(x_plus_delta);

  for (int i = 0; i < 7; i++) {
    lie_delta[i] = delta[i];
  }
  // make sure scale not too small, exp(scale) > 1e-5
  lie_delta[6] = std::max(lie_delta[6], -20.);

  Sophus::Sim3d sim_x = Sophus::Sim3d::exp(lie_x);
  Sophus::Sim3d sim_delta = Sophus::Sim3d::exp(lie_delta);
  updated = (sim_x * sim_delta).log();
  return true;
}

bool Sim3Parameterization::ComputeJacobian(const double* x,
                                           double* jacobian) const {
  ceres::MatrixRef(jacobian, 7, 7) = ceres::Matrix::Identity(7, 7);
  return true;
}


} // namespace theia