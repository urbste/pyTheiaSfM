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

#ifndef THEIA_SFM_TRANSFORMATION_ALIGN_RECONSTRUCTIONS_POSE_GRAPH_OPTIM_H_
#define THEIA_SFM_TRANSFORMATION_ALIGN_RECONSTRUCTIONS_POSE_GRAPH_OPTIM_H_

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <Sophus/sophus/sim3.hpp>

namespace theia {


class CERES_EXPORT Sim3Parameterization : public ceres::LocalParameterization {
 public:
  virtual ~Sim3Parameterization() {}
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const;
  bool ComputeJacobian(const double* x, double* jacobian) const;
  int GlobalSize() const { return 7; }
  int LocalSize() const { return 7; }
};

/**
 *  Sim3 Jacobian Calculation Reference:
 *  https://github.com/b51/CeresSim3Optimize.git
 *  <num residuals 7, parameters 1 num 7, parameters 2 num 7>
 */
struct SelfEdgesErrorTerm {
 public:
  explicit SelfEdgesErrorTerm(
      const Sophus::Sim3d& Sji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information)
      : Sji_(Sji), sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* params_i, const T* params_j, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_i(params_i);
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_j(params_j);

    Sophus::Sim3<T> S_i_w = Sophus::Sim3<T>::exp(lie_i);
    Sophus::Sim3<T> S_j_w = Sophus::Sim3<T>::exp(lie_j);
    Sophus::Sim3<T> S_j_i = S_i_w.inverse() * S_j_w;
    Sophus::Sim3<T> error = S_j_i * Sji_.inverse().cast<T>();
    Eigen::Map<Eigen::Matrix<T, 7, 1>> residuals(residual);
    residuals = error.log();

    residuals = sqrt_information_.cast<T>() * residuals;
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Sophus::Sim3d& Sji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information) {
    static const int kParameterSize = 7;
    static const int kNumResiduals = 7;
    return new ceres::
        AutoDiffCostFunction<SelfEdgesErrorTerm, kNumResiduals, kParameterSize, kParameterSize>(
            new SelfEdgesErrorTerm(Sji, sqrt_information));
  }

 private:
  const Sophus::Sim3d Sji_;
  const Eigen::Matrix<double, 7, 7> sqrt_information_;
};


struct CrossEdgesErrorTerm {
 public:
  explicit CrossEdgesErrorTerm(
      const Sophus::Sim3d& S_ref,
      const Eigen::Matrix<double, 7, 7>& sqrt_information)
      : S_ref_(S_ref), sqrt_information_(sqrt_information) {}

   template <typename T>
   bool operator()(const T* params, T* residual)const {
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> lie_qry(params);

    Sophus::Sim3<T> S_qry = Sophus::Sim3<T>::exp(lie_qry);
    Sophus::Sim3<T> error = S_qry * S_ref_.inverse().cast<T>();
    Eigen::Map<Eigen::Matrix<T, 7, 1>> residuals(residual);
    residuals = error.log();

    residuals = sqrt_information_.cast<T>() * residuals;
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Sophus::Sim3d& S_ref,
      const Eigen::Matrix<double, 7, 7>& sqrt_information) {
    static const int kParameterSize = 7;
    static const int kNumResiduals = 7;
    return new ceres::
        AutoDiffCostFunction<CrossEdgesErrorTerm, kNumResiduals, kParameterSize>(
            new CrossEdgesErrorTerm(S_ref, sqrt_information));
  }

 private:
  const Sophus::Sim3d S_ref_;
  const Eigen::Matrix<double, 7, 7> sqrt_information_;
};

}

#endif // THEIA_SFM_TRANSFORMATION_ALIGN_RECONSTRUCTIONS_POSE_GRAPH_OPTIM_H_
