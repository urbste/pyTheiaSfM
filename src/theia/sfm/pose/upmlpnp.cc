// Copyright (C) 2026 Steffen Urban
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

#include "theia/sfm/pose/upmlpnp.h"

#include <cmath>
#include <limits>

#include <glog/logging.h>

#include "theia/math/lmlsq/lm_optimizer.h"
#include "theia/math/nullspace.h"
#include "theia/sfm/pose/mlpnp_helper.h"
#include "theia/sfm/pose/util.h"

namespace theia {
namespace {

constexpr double kCovarianceFloor = 1e-14;

Eigen::Matrix3d YawMatrix(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d rotation;
  rotation << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return rotation;
}

void ConvertInputs(
    const std::vector<Eigen::Vector2d>& norm_feature_points,
    const std::vector<Eigen::Matrix3d>& feature_covariances,
    std::vector<Eigen::Vector3d>* bearings,
    std::vector<Eigen::Matrix2d>* covariances) {
  const size_t num_points = norm_feature_points.size();
  bearings->resize(num_points);
  covariances->resize(num_points);
  const bool has_covariance = feature_covariances.size() == num_points;
  for (size_t i = 0; i < num_points; ++i) {
    (*bearings)[i] = norm_feature_points[i].homogeneous().normalized();
    Eigen::Matrix<double, 3, 2> basis;
    nullS_3x2_templated<double>((*bearings)[i], basis);
    if (has_covariance) {
      (*covariances)[i] = basis.transpose() * feature_covariances[i] * basis;
    } else {
      (*covariances)[i] = Eigen::Matrix2d::Identity();
    }
  }
}

bool InitializeUpright(const std::vector<Eigen::Vector3d>& bearings,
                       const std::vector<Eigen::Matrix2d>& covariances,
                       const std::vector<Eigen::Vector3d>& world_points,
                       const Eigen::Vector3d& gravity_camera,
                       const Eigen::Vector3d& gravity_world,
                       const Eigen::Matrix2d& gravity_covariance,
                       bool marginalize_gravity,
                       MLPnPLMState* pose) {
  const int num_points = static_cast<int>(bearings.size());
  const Eigen::Matrix3d camera_alignment =
      Eigen::Quaterniond::FromTwoVectors(gravity_camera.normalized(),
                                         Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  const Eigen::Matrix3d world_alignment =
      Eigen::Quaterniond::FromTwoVectors(gravity_world.normalized(),
                                         Eigen::Vector3d::UnitY())
          .toRotationMatrix();

  Eigen::MatrixXd design(2 * num_points, 5);
  Eigen::VectorXd right(2 * num_points);
  std::vector<Eigen::Vector3d> bearings_u(num_points);
  std::vector<Eigen::Vector3d> points_u(num_points);
  std::vector<Eigen::Matrix2d> covariances_u(num_points);
  for (int i = 0; i < num_points; ++i) {
    const Eigen::Vector3d bearing = bearings[i].normalized();
    bearings_u[i] = (camera_alignment * bearing).normalized();
    points_u[i] = world_alignment * world_points[i];
    Eigen::Matrix<double, 3, 2> old_basis;
    nullS_3x2_templated<double>(bearing, old_basis);
    Eigen::Matrix<double, 3, 2> basis;
    nullS_3x2_templated<double>(bearings_u[i], basis);
    const Eigen::Matrix2d basis_change =
        basis.transpose() * camera_alignment * old_basis;
    covariances_u[i] = basis_change * covariances[i] * basis_change.transpose();
    const Eigen::Matrix2d weight = SqrtInformation2x2(covariances_u[i]);

    const Eigen::Vector3d cosine_part(points_u[i].x(), 0.0, points_u[i].z());
    const Eigen::Vector3d sine_part(points_u[i].z(), 0.0, -points_u[i].x());
    const Eigen::Vector3d fixed_part(0.0, points_u[i].y(), 0.0);
    Eigen::Matrix<double, 3, 5> components;
    components.col(0) = cosine_part;
    components.col(1) = sine_part;
    components.rightCols<3>() = Eigen::Matrix3d::Identity();
    design.middleRows<2>(2 * i) = weight * basis.transpose() * components;
    right.segment<2>(2 * i) = -weight * basis.transpose() * fixed_part;
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
  if (qr.rank() < 5) {
    return false;
  }
  const Eigen::VectorXd linear = qr.solve(right);
  if (!linear.allFinite() || linear.head<2>().norm() < 1e-12) {
    return false;
  }
  double yaw = std::atan2(linear(1), linear(0));
  Eigen::Matrix3d rotation_u = YawMatrix(yaw);

  Eigen::MatrixXd translation_design(2 * num_points, 3);
  Eigen::VectorXd translation_right(2 * num_points);
  for (int i = 0; i < num_points; ++i) {
    Eigen::Matrix<double, 3, 2> basis;
    nullS_3x2_templated<double>(bearings_u[i], basis);
    const Eigen::Matrix2d weight = SqrtInformation2x2(covariances_u[i]);
    translation_design.middleRows<2>(2 * i) = weight * basis.transpose();
    translation_right.segment<2>(2 * i) =
        -weight * basis.transpose() * rotation_u * points_u[i];
  }
  Eigen::Vector3d translation_u =
      translation_design.colPivHouseholderQr().solve(translation_right);
  if (!translation_u.allFinite()) {
    return false;
  }

  if (marginalize_gravity && gravity_covariance.norm() > kCovarianceFloor) {
    const Eigen::Matrix<double, 3, 2> aligned_gravity_derivatives =
        (Eigen::Matrix<double, 3, 2>() << 0.0, -1.0, 0.0, 0.0, 1.0, 0.0)
            .finished();
    Eigen::Matrix<double, 3, 2> gravity_basis;
    nullS_3x2_templated<double>(gravity_camera, gravity_basis);
    const Eigen::Matrix2d gravity_jacobian =
        gravity_basis.transpose() * camera_alignment.transpose() *
        aligned_gravity_derivatives;
    if (std::abs(gravity_jacobian.determinant()) > 1e-12) {
      const Eigen::Matrix2d gravity_jacobian_inverse =
          gravity_jacobian.inverse();
      const Eigen::Matrix2d tilt_covariance =
          gravity_jacobian_inverse * gravity_covariance *
          gravity_jacobian_inverse.transpose();
      Eigen::Matrix<double, 3, 2> tilt_axes;
      tilt_axes << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

      for (int iteration = 0; iteration < 3; ++iteration) {
        Eigen::MatrixXd normalized_design(2 * num_points, 5);
        Eigen::VectorXd normalized_right(2 * num_points);
        Eigen::MatrixXd tilt_jacobian(2 * num_points, 2);
        Eigen::MatrixXd effective_covariance =
            Eigen::MatrixXd::Zero(2 * num_points, 2 * num_points);
        for (int i = 0; i < num_points; ++i) {
          Eigen::Matrix<double, 3, 2> basis;
          nullS_3x2_templated<double>(bearings_u[i], basis);
          const Eigen::Vector3d cosine_part(points_u[i].x(), 0.0,
                                            points_u[i].z());
          const Eigen::Vector3d sine_part(points_u[i].z(), 0.0,
                                            -points_u[i].x());
          const Eigen::Vector3d fixed_part(0.0, points_u[i].y(), 0.0);
          Eigen::Matrix<double, 3, 5> components;
          components.col(0) = cosine_part;
          components.col(1) = sine_part;
          components.rightCols<3>() = Eigen::Matrix3d::Identity();

          const Eigen::Vector3d rotated_point = rotation_u * points_u[i];
          const Eigen::Vector3d camera_point = rotated_point + translation_u;
          const double range = camera_point.norm();
          if (range < 1e-12) {
            return false;
          }
          const Eigen::Vector3d prediction = camera_point / range;
          const Eigen::Matrix3d normalization_jacobian =
              (Eigen::Matrix3d::Identity() -
               prediction * prediction.transpose()) /
              range;
          normalized_design.middleRows<2>(2 * i) =
              basis.transpose() * components / range;
          normalized_right.segment<2>(2 * i) =
              -basis.transpose() * fixed_part / range;
          tilt_jacobian.middleRows<2>(2 * i) =
              -basis.transpose() * normalization_jacobian *
              CrossProductMatrix(rotated_point) * tilt_axes;
          effective_covariance.block<2, 2>(2 * i, 2 * i) = covariances_u[i];
        }
        effective_covariance +=
            tilt_jacobian * tilt_covariance * tilt_jacobian.transpose();
        effective_covariance.diagonal().array() += kCovarianceFloor;
        Eigen::LDLT<Eigen::MatrixXd> covariance_ldlt(effective_covariance);
        if (covariance_ldlt.info() != Eigen::Success) {
          break;
        }
        const Eigen::MatrixXd information_design =
            covariance_ldlt.solve(normalized_design);
        const Eigen::VectorXd information_right =
            covariance_ldlt.solve(normalized_right);
        const Eigen::Matrix<double, 5, 5> normal =
            normalized_design.transpose() * information_design;
        const Eigen::Matrix<double, 5, 1> rhs =
            normalized_design.transpose() * information_right;
        const Eigen::Matrix<double, 5, 1> marginalized =
            normal.ldlt().solve(rhs);
        if (!marginalized.allFinite() ||
            marginalized.head<2>().norm() < 1e-12) {
          break;
        }
        yaw = std::atan2(marginalized(1), marginalized(0));
        rotation_u = YawMatrix(yaw);

        Eigen::MatrixXd translation_matrix(2 * num_points, 3);
        Eigen::VectorXd translation_vector(2 * num_points);
        for (int i = 0; i < num_points; ++i) {
          Eigen::Matrix<double, 3, 2> basis;
          nullS_3x2_templated<double>(bearings_u[i], basis);
          const double range = (rotation_u * points_u[i] + translation_u).norm();
          translation_matrix.middleRows<2>(2 * i) =
              basis.transpose() / range;
          translation_vector.segment<2>(2 * i) =
              -basis.transpose() * rotation_u * points_u[i] / range;
        }
        const Eigen::MatrixXd information_translation =
            covariance_ldlt.solve(translation_matrix);
        const Eigen::VectorXd information_translation_right =
            covariance_ldlt.solve(translation_vector);
        translation_u = (translation_matrix.transpose() * information_translation)
                            .ldlt()
                            .solve(translation_matrix.transpose() *
                                   information_translation_right);
        if (!translation_u.allFinite()) {
          return false;
        }
      }
    }
  }

  pose->rotation =
      camera_alignment.transpose() * rotation_u * world_alignment;
  pose->translation = camera_alignment.transpose() * translation_u;
  return true;
}

class UPMLPnPTangentRefiner {
 public:
  using Model = MLPnPLMState;
  static constexpr int kNumParams = 6;

  UPMLPnPTangentRefiner(
      const std::vector<Eigen::Vector2d>& norm_feature_points,
      const std::vector<Eigen::Matrix3d>& feature_covariances,
      const std::vector<Eigen::Vector3d>& world_points,
      const Eigen::Vector3d& gravity_camera,
      const Eigen::Vector3d& gravity_world,
      const Eigen::Matrix2d& gravity_covariance)
      : points_refiner_(norm_feature_points, feature_covariances, world_points),
        gravity_camera_(gravity_camera.normalized()),
        gravity_world_(gravity_world.normalized()),
        gravity_sqrt_information_(SqrtInformation2x2(gravity_covariance)) {
    nullS_3x2_templated<double>(gravity_camera_, gravity_basis_);
  }

  int NumParams() const { return kNumParams; }

  double ComputeResidual(NormalAccumulator& acc, const Model& pose) {
    points_refiner_.AccumulatePointResiduals(acc, pose, false);
    AccumulateGravityResidual(acc, pose, false);
    return acc.Cost();
  }

  void ComputeJacobian(NormalAccumulator& acc, const Model& pose) {
    points_refiner_.AccumulatePointResiduals(acc, pose, true);
    AccumulateGravityResidual(acc, pose, true);
  }

  Model Step(const Eigen::VectorXd& dp, const Model& pose) const {
    Model pose_new;
    pose_new.rotation = pose.rotation * ExpSO3(dp.head<3>());
    pose_new.translation = pose.translation + pose.rotation * dp.tail<3>();
    return pose_new;
  }

 private:
  void AccumulateGravityResidual(NormalAccumulator& acc, const Model& pose,
                                 bool compute_jacobian) const {
    const Eigen::Vector3d predicted_gravity =
        (pose.rotation * gravity_world_).normalized();
    const Eigen::Vector2d gravity_residual =
        gravity_sqrt_information_ * gravity_basis_.transpose() *
        predicted_gravity;
    if (compute_jacobian) {
      Eigen::Matrix<double, 2, 6> gravity_jacobian =
          Eigen::Matrix<double, 2, 6>::Zero();
      gravity_jacobian.leftCols<3>() =
          -gravity_sqrt_information_ * gravity_basis_.transpose() *
          CrossProductMatrix(predicted_gravity);
      acc.AddJacobian(gravity_residual, gravity_jacobian);
    } else {
      acc.AddResidual(gravity_residual);
    }
  }

  MLPnPTangentRefiner points_refiner_;
  Eigen::Vector3d gravity_camera_;
  Eigen::Vector3d gravity_world_;
  Eigen::Matrix<double, 3, 2> gravity_basis_;
  Eigen::Matrix2d gravity_sqrt_information_;
};

bool RefineUPMLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
                   const std::vector<Eigen::Matrix3d>& feature_covariances,
                   const std::vector<Eigen::Vector3d>& world_points,
                   const Eigen::Vector3d& gravity_camera,
                   const Eigen::Vector3d& gravity_world,
                   const Eigen::Matrix2d& gravity_covariance,
                   int max_iterations,
                   MLPnPLMState* pose,
                   LmStats* stats = nullptr) {
  UPMLPnPTangentRefiner refiner(norm_feature_points, feature_covariances,
                                world_points, gravity_camera, gravity_world,
                                gravity_covariance);
  LmOptions opt;
  opt.max_iterations = static_cast<size_t>(max_iterations);
  opt.loss_scale = 0.0;
  const LmStats local_stats = MinimizeLM(refiner, pose, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }
  return local_stats.final_cost <= local_stats.initial_cost;
}

bool ComputePoseCovariance(
    const std::vector<Eigen::Vector2d>& norm_feature_points,
    const std::vector<Eigen::Matrix3d>& feature_covariances,
    const std::vector<Eigen::Vector3d>& world_points,
    const Eigen::Vector3d& gravity_camera,
    const Eigen::Vector3d& gravity_world,
    const Eigen::Matrix2d& gravity_covariance,
    const MLPnPLMState& pose,
    Eigen::Matrix<double, 6, 6>* pose_covariance) {
  UPMLPnPTangentRefiner refiner(norm_feature_points, feature_covariances,
                                world_points, gravity_camera, gravity_world,
                                gravity_covariance);
  NormalAccumulator acc;
  acc.InitializeTrivial(refiner.NumParams());
  acc.ResetJacobian();
  refiner.ComputeJacobian(acc, pose);
  Eigen::Matrix<double, 6, 6> hessian = acc.JtJ();
  for (int i = 0; i < 6; ++i) {
    hessian(i, i) = std::max(hessian(i, i), kCovarianceFloor);
  }
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(hessian);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  *pose_covariance = ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
  *pose_covariance = 0.5 * (*pose_covariance + pose_covariance->transpose());
  return pose_covariance->allFinite();
}

bool ValidateInputs(const std::vector<Eigen::Vector2d>& norm_feature_points,
                    const std::vector<Eigen::Matrix3d>& feature_covariances,
                    const std::vector<Eigen::Vector3d>& world_points,
                    const Eigen::Vector3d& gravity_camera,
                    const Eigen::Vector3d& gravity_world) {
  if (norm_feature_points.size() <
          static_cast<size_t>(kUPMLPnPMinimumPoints) ||
      norm_feature_points.size() != world_points.size()) {
    return false;
  }
  if (!feature_covariances.empty() &&
      feature_covariances.size() != norm_feature_points.size()) {
    return false;
  }
  if (gravity_camera.norm() < 1e-12 || gravity_world.norm() < 1e-12) {
    return false;
  }
  return true;
}

}  // namespace

bool UPMLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
             const std::vector<Eigen::Matrix3d>& feature_covariances,
             const std::vector<Eigen::Vector3d>& world_points,
             const Eigen::Vector3d& gravity_camera,
             const Eigen::Vector3d& gravity_world,
             const Eigen::Matrix2d& gravity_covariance,
             Eigen::Matrix3d* rotation,
             Eigen::Vector3d* translation,
             bool run_refinement) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(translation);
  if (!ValidateInputs(norm_feature_points, feature_covariances, world_points,
                      gravity_camera, gravity_world)) {
    return false;
  }

  std::vector<Eigen::Vector3d> bearings;
  std::vector<Eigen::Matrix2d> covariances;
  ConvertInputs(norm_feature_points, feature_covariances, &bearings,
                &covariances);

  MLPnPLMState pose;
  if (!InitializeUpright(bearings, covariances, world_points, gravity_camera,
                         gravity_world, gravity_covariance, false, &pose)) {
    return false;
  }

  if (run_refinement) {
    RefineUPMLPnP(norm_feature_points, feature_covariances, world_points,
                  gravity_camera, gravity_world, gravity_covariance, 25, &pose);
  }

  *rotation = pose.rotation;
  *translation = pose.translation;
  return rotation->allFinite() && translation->allFinite();
}

bool UPMLPnPWithCovariance(
    const std::vector<Eigen::Vector2d>& norm_feature_points,
    const std::vector<Eigen::Matrix3d>& feature_covariances,
    const std::vector<Eigen::Vector3d>& world_points,
    const Eigen::Vector3d& gravity_camera,
    const Eigen::Vector3d& gravity_world,
    const Eigen::Matrix2d& gravity_covariance,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    Eigen::Matrix<double, 6, 6>* pose_covariance,
    bool run_refinement) {
  CHECK_NOTNULL(pose_covariance);
  if (!UPMLPnP(norm_feature_points, feature_covariances, world_points,
               gravity_camera, gravity_world, gravity_covariance, rotation,
               translation, run_refinement)) {
    return false;
  }
  MLPnPLMState pose;
  pose.rotation = *rotation;
  pose.translation = *translation;
  return ComputePoseCovariance(norm_feature_points, feature_covariances,
                               world_points, gravity_camera, gravity_world,
                               gravity_covariance, pose, pose_covariance);
}

}  // namespace theia
