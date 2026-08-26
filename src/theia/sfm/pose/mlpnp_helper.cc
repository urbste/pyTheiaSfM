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

#include "theia/sfm/pose/mlpnp_helper.h"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <glog/logging.h>

#include "theia/math/lmlsq/lm_optimizer.h"
#include "theia/math/nullspace.h"
#include "theia/sfm/pose/util.h"

namespace theia {
namespace {

constexpr double kCovarianceFloor = 1e-14;

}  // namespace

Eigen::Matrix2d SqrtInformation2x2(const Eigen::Matrix2d& covariance) {
  const Eigen::Matrix2d symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(symmetric);
  const Eigen::Vector2d values =
      eigensolver.eigenvalues().cwiseMax(kCovarianceFloor).cwiseSqrt().cwiseInverse();
  return eigensolver.eigenvectors() * values.asDiagonal() *
         eigensolver.eigenvectors().transpose();
}

Eigen::Matrix2d ProjectedFeatureSqrtInformation(
    const Eigen::Matrix<double, 3, 2>& basis,
    const Eigen::Matrix3d& feature_covariance,
    bool has_covariance) {
  if (!has_covariance) {
    return Eigen::Matrix2d::Identity();
  }
  const Eigen::Matrix2d projected =
      basis.transpose() * feature_covariance * basis;
  return SqrtInformation2x2(projected);
}

MLPnPTangentRefiner::MLPnPTangentRefiner(
    const std::vector<Eigen::Vector2d>& norm_feature_points,
    const std::vector<Eigen::Matrix3d>& feature_covariances,
    const std::vector<Eigen::Vector3d>& world_points)
    : world_points_(world_points) {
  const size_t num_points = norm_feature_points.size();
  bearings_.resize(num_points);
  bases_.resize(num_points);
  sqrt_information_.resize(num_points);
  const bool has_covariance = feature_covariances.size() == num_points;
  for (size_t i = 0; i < num_points; ++i) {
    bearings_[i] = norm_feature_points[i].homogeneous().normalized();
    nullS_3x2_templated<double>(bearings_[i], bases_[i]);
    const Eigen::Matrix3d feature_covariance =
        has_covariance ? feature_covariances[i] : Eigen::Matrix3d::Identity();
    sqrt_information_[i] = ProjectedFeatureSqrtInformation(
        bases_[i], feature_covariance, has_covariance);
  }
}

void MLPnPTangentRefiner::AccumulatePointResiduals(NormalAccumulator& acc,
                                                     const Model& pose,
                                                     bool compute_jacobian) const {
  const Eigen::Matrix3d& rotation = pose.rotation;
  for (size_t i = 0; i < world_points_.size(); ++i) {
    const Eigen::Vector3d rotated_point = rotation * world_points_[i];
    const Eigen::Vector3d camera_point = rotated_point + pose.translation;
    const double norm = camera_point.norm();
    if (norm < 1e-12) {
      continue;
    }
    const Eigen::Vector3d prediction = camera_point / norm;
    const Eigen::Vector2d residual =
        sqrt_information_[i] * bases_[i].transpose() * prediction;
    if (compute_jacobian) {
      const Eigen::Matrix3d normalization_jacobian =
          (Eigen::Matrix3d::Identity() - prediction * prediction.transpose()) /
          norm;
      Eigen::Matrix<double, 3, 6> point_jacobian;
      point_jacobian.leftCols<3>() = -CrossProductMatrix(rotated_point);
      point_jacobian.rightCols<3>() = Eigen::Matrix3d::Identity();
      const Eigen::Matrix<double, 2, 6> jacobian =
          sqrt_information_[i] * bases_[i].transpose() * normalization_jacobian *
          point_jacobian;
      acc.AddJacobian(residual, jacobian);
    } else {
      acc.AddResidual(residual);
    }
  }
}

double MLPnPTangentRefiner::ComputeResidual(NormalAccumulator& acc,
                                            const Model& pose) {
  AccumulatePointResiduals(acc, pose, false);
  return acc.Cost();
}

void MLPnPTangentRefiner::ComputeJacobian(NormalAccumulator& acc,
                                          const Model& pose) {
  AccumulatePointResiduals(acc, pose, true);
}

MLPnPTangentRefiner::Model MLPnPTangentRefiner::Step(
    const Eigen::VectorXd& dp, const Model& pose) const {
  Model pose_new;
  pose_new.rotation = pose.rotation * ExpSO3(dp.head<3>());
  pose_new.translation = pose.translation + pose.rotation * dp.tail<3>();
  return pose_new;
}

bool MLPnPGaussNewton(
    const std::vector<Eigen::Vector2d>& norm_feature_points,
    const std::vector<Eigen::Matrix3d>& feature_covariances,
    const std::vector<Eigen::Vector3d>& world_points,
    int max_iterations,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double loss_scale,
    LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(translation);
  CHECK_EQ(norm_feature_points.size(), world_points.size());
  if (norm_feature_points.size() < 3 || max_iterations <= 0) {
    return false;
  }

  MLPnPLMState state;
  state.rotation = *rotation;
  state.translation = *translation;

  MLPnPTangentRefiner refiner(norm_feature_points, feature_covariances,
                              world_points);
  LmOptions opt;
  opt.max_iterations = static_cast<size_t>(max_iterations);
  opt.loss_scale = loss_scale;

  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *translation = state.translation;
  return local_stats.final_cost <= local_stats.initial_cost;
}

}  // namespace theia
