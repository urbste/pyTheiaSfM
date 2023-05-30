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
class SelfEdgesErrorTerm : public ceres::SizedCostFunction<7, 7, 7> {
 public:
  SelfEdgesErrorTerm(
      const Sophus::Sim3d& Sji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information)
      : Sji_(Sji), sqrt_information_(sqrt_information) {}

  virtual bool Evaluate(double const* const* parameters_ptr,
                        double* residuals_ptr, double** jacobians_ptr) const {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_j(*parameters_ptr);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_i(*(parameters_ptr + 1));

    Sophus::Sim3d Si = Sophus::Sim3d::exp(lie_i);
    Sophus::Sim3d Sj = Sophus::Sim3d::exp(lie_j);
    Sophus::Sim3d error = Sji_ * Si * Sj.inverse();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> residuals(residuals_ptr);
    residuals = error.log();

    if (jacobians_ptr) {
      Eigen::Matrix<double, 7, 7> Jacobian_i;
      Eigen::Matrix<double, 7, 7> Jacobian_j;
      Eigen::Matrix<double, 7, 7> Jr = Eigen::Matrix<double, 7, 7>::Zero();

      Jr.block<3, 3>(0, 0) = Sophus::RxSO3d::hat(residuals.tail(4));
      Jr.block<3, 3>(0, 3) = Sophus::SO3d::hat(residuals.head(3));
      Jr.block<3, 1>(0, 6) = -residuals.head(3);
      Jr.block<3, 3>(3, 3) = Sophus::SO3d::hat(residuals.block<3, 1>(3, 0));
      Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
      Jr = sqrt_information_ * (I + 0.5 * Jr + 1.0 / 12. * (Jr * Jr));

      Jacobian_i = Jr * Sj.Adj();
      Jacobian_j = -Jacobian_i;
      int k = 0;
      for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; ++j) {
          if (jacobians_ptr[0]) jacobians_ptr[0][k] = Jacobian_j(i, j);
          if (jacobians_ptr[1]) jacobians_ptr[1][k] = Jacobian_i(i, j);
          k++;
        }
      }
    }

    residuals = sqrt_information_ * residuals;
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction* Create(
      const Sophus::Sim3d& Sji,
      const Eigen::Matrix<double, 7, 7>& sqrt_information) {
    return new SelfEdgesErrorTerm(Sji, sqrt_information);
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
    Sophus::Sim3<T> error = S_qry * S_ref_.inverse().cast<T>(); // * S_ref_.inverse(); //.cast<T>().inverse();
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
