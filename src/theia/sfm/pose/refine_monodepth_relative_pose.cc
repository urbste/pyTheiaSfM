// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib robust/optim/monodepth_relpose.h, commit
// fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// Reference: Y. Ding et al., "RePoseD", ICCV 2025.

#include "theia/sfm/pose/refine_monodepth_relative_pose.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <glog/logging.h>

#include "theia/math/lmlsq/lm_optimizer.h"
#include "theia/sfm/pose/util.h"

namespace theia {
namespace {

void DerivEssentialWrtTranslation(const Eigen::Matrix3d& R,
                                  Eigen::Matrix<double, 9, 3>* dt) {
  // Each column is vec(skew(e_k) * R) in column-major order.
  for (int k = 0; k < 3; ++k) {
    Eigen::Matrix3d dEk = Eigen::Matrix3d::Zero();
    if (k == 0) {
      dEk.row(1) = -R.row(2);
      dEk.row(2) = R.row(1);
    } else if (k == 1) {
      dEk.row(0) = R.row(2);
      dEk.row(2) = -R.row(0);
    } else {
      dEk.row(0) = -R.row(1);
      dEk.row(1) = R.row(0);
    }
    dt->col(k) = Eigen::Map<const Eigen::Matrix<double, 9, 1>>(dEk.data());
  }
}

void DerivEssentialWrtRotation(const Eigen::Matrix3d& E,
                               Eigen::Matrix<double, 9, 3>* dR) {
  dR->block<3, 1>(0, 0).setZero();
  dR->block<3, 1>(0, 1) = -E.col(2);
  dR->block<3, 1>(0, 2) = E.col(1);
  dR->block<3, 1>(3, 0) = E.col(2);
  dR->block<3, 1>(3, 1).setZero();
  dR->block<3, 1>(3, 2) = -E.col(0);
  dR->block<3, 1>(6, 0) = -E.col(1);
  dR->block<3, 1>(6, 1) = E.col(0);
  dR->block<3, 1>(6, 2).setZero();
}

double ComputeSampsonJacobian(const Eigen::Vector2d& x1,
                              const Eigen::Vector2d& x2,
                              const Eigen::Matrix3d& F,
                              Eigen::Matrix<double, 1, 9>* dF) {
  const Eigen::Vector3d x1h = x1.homogeneous();
  const Eigen::Vector3d x2h = x2.homogeneous();
  const double C = x2h.dot(F * x1h);

  Eigen::Vector4d J_C;
  J_C << F.block<3, 2>(0, 0).transpose() * x2h, F.block<2, 3>(0, 0) * x1h;
  const double nJ_C = J_C.norm();
  const double inv_nJ_C = 1.0 / nJ_C;
  const double r = C * inv_nJ_C;

  (*dF) << x1(0) * x2(0), x1(0) * x2(1), x1(0), x1(1) * x2(0), x1(1) * x2(1),
      x1(1), x2(0), x2(1), 1.0;
  const double s = C * inv_nJ_C * inv_nJ_C;
  (*dF)(0) -= s * (J_C(2) * x1(0) + J_C(0) * x2(0));
  (*dF)(1) -= s * (J_C(3) * x1(0) + J_C(0) * x2(1));
  (*dF)(2) -= s * (J_C(0));
  (*dF)(3) -= s * (J_C(2) * x1(1) + J_C(1) * x2(0));
  (*dF)(4) -= s * (J_C(3) * x1(1) + J_C(1) * x2(1));
  (*dF)(5) -= s * (J_C(1));
  (*dF)(6) -= s * (J_C(2));
  (*dF)(7) -= s * (J_C(3));
  (*dF) *= inv_nJ_C;
  return r;
}

double ComputeSampsonResidual(const Eigen::Vector2d& x1,
                              const Eigen::Vector2d& x2,
                              const Eigen::Matrix3d& F) {
  const Eigen::Vector3d x1h = x1.homogeneous();
  const Eigen::Vector3d x2h = x2.homogeneous();
  const double C = x2h.dot(F * x1h);
  const double nJc_sq =
      (F.block<2, 3>(0, 0) * x1h).squaredNorm() +
      (F.block<3, 2>(0, 0).transpose() * x2h).squaredNorm();
  return C / std::sqrt(nJc_sq);
}

// Recover metric translation magnitude + depth scale from unit position.
void RecoverMetricTranslationAndScale(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& position,
    double shift1,
    double shift2,
    double inv_focal1,
    double inv_focal2,
    Eigen::Vector3d* translation,
    double* scale) {
  const Eigen::Vector3d t_dir = (-rotation * position).normalized();
  std::vector<double> m_estimates;
  std::vector<double> scale_estimates;
  m_estimates.reserve(x1.size());
  scale_estimates.reserve(x1.size());

  for (size_t i = 0; i < x1.size(); ++i) {
    if (depth1[i] <= 0.0 || depth2[i] <= 0.0) {
      continue;
    }
    Eigen::Vector3d x1h = x1[i].homogeneous();
    x1h.x() *= inv_focal1;
    x1h.y() *= inv_focal1;
    Eigen::Vector3d x2h = x2[i].homogeneous();
    x2h.x() *= inv_focal2;
    x2h.y() *= inv_focal2;

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = t_dir;
    A.col(1) = -(depth2[i] + shift2) * x2h;
    const Eigen::Vector3d b = -rotation * ((depth1[i] + shift1) * x1h);
    const Eigen::Vector2d sol = A.colPivHouseholderQr().solve(b);
    if (sol(1) > 0.0) {
      m_estimates.push_back(sol(0));
      scale_estimates.push_back(sol(1));
    }
  }

  double m = 1.0;
  *scale = 1.0;
  if (!scale_estimates.empty()) {
    std::nth_element(scale_estimates.begin(),
                     scale_estimates.begin() + scale_estimates.size() / 2,
                     scale_estimates.end());
    *scale = scale_estimates[scale_estimates.size() / 2];
    std::nth_element(m_estimates.begin(),
                     m_estimates.begin() + m_estimates.size() / 2,
                     m_estimates.end());
    m = m_estimates[m_estimates.size() / 2];
  }
  *translation = m * t_dir;
}

}  // namespace

// ---------------------------------------------------------------------------
// Calibrated monodepth refiner
// ---------------------------------------------------------------------------

MonoDepthRelativePoseRefiner::MonoDepthRelativePoseRefiner(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double scale_reproj,
    double weight_sampson,
    bool refine_shift)
    : x1_(x1),
      x2_(x2),
      depth1_(depth1),
      depth2_(depth2),
      scale_reproj_(scale_reproj),
      weight_sampson_(weight_sampson),
      refine_shift_(refine_shift),
      num_params_(refine_shift ? 9 : 7) {}

double MonoDepthRelativePoseRefiner::ComputeResidual(
    NormalAccumulator& acc, const Model& geometry) {
  const double scale = geometry.scale;
  const double shift_1 = geometry.shift1;
  const double shift_2 = geometry.shift2;
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Vector3d& t = geometry.translation;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E =
      CrossProductMatrix(t) * R;

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (weight_sampson_ > 0.0) {
      acc.AddResidual(ComputeSampsonResidual(x1_[i], x2_[i], E),
                      weight_sampson_);
    }
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d Z1 =
          R * ((depth1_[i] + shift_1) * x1_[i].homogeneous()) + t;
      const Eigen::Vector3d Z2 =
          R.transpose() *
          (scale * (depth2_[i] + shift_2) * x2_[i].homogeneous() - t);

      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        Eigen::Vector2d res;
        res << Z1(0) * inv_z - x2_[i](0), Z1(1) * inv_z - x2_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        Eigen::Vector2d res;
        res << Z2(0) * inv_z - x1_[i](0), Z2(1) * inv_z - x1_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
    }
  }
  return acc.Cost();
}

void MonoDepthRelativePoseRefiner::ComputeJacobian(
    NormalAccumulator& acc, const Model& geometry) {
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Matrix3d Rt = R.transpose();
  const double scale = geometry.scale;
  const double shift_1 = geometry.shift1;
  const double shift_2 = geometry.shift2;
  const int np = num_params_;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E =
      CrossProductMatrix(geometry.translation) * R;

  Eigen::Matrix<double, 9, 3> dR, dt;
  DerivEssentialWrtRotation(E, &dR);
  DerivEssentialWrtTranslation(R, &dt);

  Eigen::Matrix<double, 2, 3> Jproj;
  Jproj.setZero();

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d X1o = x1_[i].homogeneous();
      const Eigen::Vector3d X1i = (depth1_[i] + shift_1) * X1o;
      const Eigen::Vector3d Z1 = R * X1i + geometry.translation;

      const Eigen::Vector3d X2o = x2_[i].homogeneous();
      const Eigen::Vector3d X2s = (depth2_[i] + shift_2) * X2o;
      const Eigen::Vector3d X2i = scale * X2s;
      const Eigen::Vector3d Z2 = Rt * (X2i - geometry.translation);

      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        Eigen::Vector2d res;
        res << Z1(0) * inv_z - x2_[i](0), Z1(1) * inv_z - x2_[i](1);

        Jproj(0, 0) = inv_z;
        Jproj(1, 1) = inv_z;
        Jproj(0, 2) = -Z1(0) * inv_z * inv_z;
        Jproj(1, 2) = -Z1(1) * inv_z * inv_z;

        Eigen::MatrixXd J(2, np);
        J.setZero();
        Eigen::Matrix<double, 2, 3> dZ = Jproj * R;
        J.col(0) = -X1i(2) * dZ.col(1) + X1i(1) * dZ.col(2);
        J.col(1) = X1i(2) * dZ.col(0) - X1i(0) * dZ.col(2);
        J.col(2) = -X1i(1) * dZ.col(0) + X1i(0) * dZ.col(1);
        J.block<2, 3>(0, 3) = Jproj;
        if (refine_shift_) {
          J.col(7) = Jproj * R * X1o;
        }
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }

      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        Eigen::Vector2d res;
        res << Z2(0) * inv_z - x1_[i](0), Z2(1) * inv_z - x1_[i](1);

        Jproj(0, 0) = inv_z;
        Jproj(1, 1) = inv_z;
        Jproj(0, 2) = -Z2(0) * inv_z * inv_z;
        Jproj(1, 2) = -Z2(1) * inv_z * inv_z;

        Eigen::MatrixXd J(2, np);
        J.setZero();
        const Eigen::Vector3d X2t = X2i - geometry.translation;
        Eigen::Matrix3d dZdr;
        dZdr.setZero();
        dZdr(1, 0) = X2t.dot(R.col(2));
        dZdr(2, 0) = -X2t.dot(R.col(1));
        dZdr(0, 1) = -X2t.dot(R.col(2));
        dZdr(2, 1) = X2t.dot(R.col(0));
        dZdr(0, 2) = X2t.dot(R.col(1));
        dZdr(1, 2) = -X2t.dot(R.col(0));
        J.block<2, 3>(0, 0) = Jproj * dZdr;
        J.block<2, 3>(0, 3) = -Jproj * Rt;
        J.col(6) = Jproj * Rt * X2s;
        if (refine_shift_) {
          J.col(8) = scale * Jproj * Rt * X2o;
        }
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }
    }

    if (weight_sampson_ > 0.0) {
      Eigen::Matrix<double, 1, 9> dF;
      const double r = ComputeSampsonJacobian(x1_[i], x2_[i], E, &dF);
      Eigen::RowVectorXd J_sam(np);
      J_sam.setZero();
      J_sam.segment<3>(0) = dF * dR;
      J_sam.segment<3>(3) = dF * dt;
      acc.AddJacobian(r, J_sam, weight_sampson_);
    }
  }
}

MonoDepthRelativePoseRefiner::Model MonoDepthRelativePoseRefiner::Step(
    const Eigen::VectorXd& dp, const Model& geometry) const {
  Model geometry_new = geometry;
  geometry_new.rotation = geometry.rotation * ExpSO3(dp.head<3>());
  geometry_new.translation = geometry.translation + dp.segment<3>(3);
  geometry_new.scale = geometry.scale + dp(6);
  if (refine_shift_) {
    geometry_new.shift1 = geometry.shift1 + dp(7);
    geometry_new.shift2 = geometry.shift2 + dp(8);
  }
  return geometry_new;
}

// ---------------------------------------------------------------------------
// Shared-focal monodepth refiner
// ---------------------------------------------------------------------------

MonoDepthSharedFocalRelativePoseRefiner::MonoDepthSharedFocalRelativePoseRefiner(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double scale_reproj,
    double weight_sampson)
    : x1_(x1),
      x2_(x2),
      depth1_(depth1),
      depth2_(depth2),
      scale_reproj_(scale_reproj),
      weight_sampson_(weight_sampson) {}

double MonoDepthSharedFocalRelativePoseRefiner::ComputeResidual(
    NormalAccumulator& acc, const Model& geometry) {
  const double scale = geometry.scale;
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Vector3d& t = geometry.translation;
  const double f = geometry.focal1;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E = CrossProductMatrix(t) * R;
  const Eigen::DiagonalMatrix<double, 3> K_inv(1.0, 1.0, f);
  const Eigen::Matrix3d F = K_inv * E * K_inv;

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (weight_sampson_ > 0.0) {
      acc.AddResidual(ComputeSampsonResidual(x1_[i], x2_[i], F),
                      weight_sampson_);
    }
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d b1(x1_[i](0) / f, x1_[i](1) / f, 1.0);
      const Eigen::Vector3d b2(x2_[i](0) / f, x2_[i](1) / f, 1.0);
      const Eigen::Vector3d Z1 = R * (depth1_[i] * b1) + t;
      const Eigen::Vector3d Z2 =
          R.transpose() * (scale * depth2_[i] * b2 - t);
      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        Eigen::Vector2d res;
        res << f * Z1(0) * inv_z - x2_[i](0), f * Z1(1) * inv_z - x2_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        Eigen::Vector2d res;
        res << f * Z2(0) * inv_z - x1_[i](0), f * Z2(1) * inv_z - x1_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
    }
  }
  return acc.Cost();
}

void MonoDepthSharedFocalRelativePoseRefiner::ComputeJacobian(
    NormalAccumulator& acc, const Model& geometry) {
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Matrix3d Rt = R.transpose();
  const double scale = geometry.scale;
  const double f = geometry.focal1;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E =
      CrossProductMatrix(geometry.translation) * R;
  const Eigen::DiagonalMatrix<double, 3> K_inv(1.0, 1.0, f);
  const Eigen::Matrix3d F = K_inv * E * K_inv;

  Eigen::Matrix<double, 9, 3> dR_E, dt_E;
  DerivEssentialWrtRotation(E, &dR_E);
  DerivEssentialWrtTranslation(R, &dt_E);

  Eigen::Matrix<double, 9, 3> dR_F = dR_E, dt_F = dt_E;
  dR_F.row(2) *= f;
  dR_F.row(5) *= f;
  dR_F.row(6) *= f;
  dR_F.row(7) *= f;
  dR_F.row(8) *= f * f;
  dt_F.row(2) *= f;
  dt_F.row(5) *= f;
  dt_F.row(6) *= f;
  dt_F.row(7) *= f;
  dt_F.row(8) *= f * f;

  Eigen::Matrix<double, 9, 1> df_F;
  df_F << 0.0, 0.0, E(2, 0), 0.0, 0.0, E(2, 1), E(0, 2), E(1, 2),
      2 * E(2, 2) * f;

  Eigen::Matrix<double, 2, 3> Jproj;
  Jproj.setZero();

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d b1(x1_[i](0) / f, x1_[i](1) / f, 1.0);
      const Eigen::Vector3d b2(x2_[i](0) / f, x2_[i](1) / f, 1.0);
      const Eigen::Vector3d X1i = depth1_[i] * b1;
      const Eigen::Vector3d Z1 = R * X1i + geometry.translation;
      const Eigen::Vector3d X2s = depth2_[i] * b2;
      const Eigen::Vector3d X2i = scale * X2s;
      const Eigen::Vector3d Z2 = Rt * (X2i - geometry.translation);

      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        const Eigen::Vector2d xp_cal(Z1(0) * inv_z, Z1(1) * inv_z);
        Eigen::Vector2d res = f * xp_cal - x2_[i];

        Jproj(0, 0) = f * inv_z;
        Jproj(1, 1) = f * inv_z;
        Jproj(0, 2) = -f * Z1(0) * inv_z * inv_z;
        Jproj(1, 2) = -f * Z1(1) * inv_z * inv_z;

        Eigen::Matrix<double, 2, 8> J;
        Eigen::Matrix<double, 2, 3> dZ = Jproj * R;
        J.col(0) = -X1i(2) * dZ.col(1) + X1i(1) * dZ.col(2);
        J.col(1) = X1i(2) * dZ.col(0) - X1i(0) * dZ.col(2);
        J.col(2) = -X1i(1) * dZ.col(0) + X1i(0) * dZ.col(1);
        J.block<2, 3>(0, 3) = Jproj;
        J.col(6).setZero();
        const Eigen::Vector3d dX1_df =
            depth1_[i] *
            Eigen::Vector3d(-x1_[i](0) / (f * f), -x1_[i](1) / (f * f), 0.0);
        J.col(7) = xp_cal + Jproj * (R * dX1_df);
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }

      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        const Eigen::Vector2d xp_cal(Z2(0) * inv_z, Z2(1) * inv_z);
        Eigen::Vector2d res = f * xp_cal - x1_[i];

        Jproj(0, 0) = f * inv_z;
        Jproj(1, 1) = f * inv_z;
        Jproj(0, 2) = -f * Z2(0) * inv_z * inv_z;
        Jproj(1, 2) = -f * Z2(1) * inv_z * inv_z;

        Eigen::Matrix<double, 2, 8> J;
        const Eigen::Vector3d X2t = X2i - geometry.translation;
        Eigen::Matrix3d dZdr;
        dZdr.setZero();
        dZdr(1, 0) = X2t.dot(R.col(2));
        dZdr(2, 0) = -X2t.dot(R.col(1));
        dZdr(0, 1) = -X2t.dot(R.col(2));
        dZdr(2, 1) = X2t.dot(R.col(0));
        dZdr(0, 2) = X2t.dot(R.col(1));
        dZdr(1, 2) = -X2t.dot(R.col(0));
        J.block<2, 3>(0, 0) = Jproj * dZdr;
        J.block<2, 3>(0, 3) = -Jproj * Rt;
        J.col(6) = Jproj * Rt * X2s;
        const Eigen::Vector3d dX2_df =
            scale * depth2_[i] *
            Eigen::Vector3d(-x2_[i](0) / (f * f), -x2_[i](1) / (f * f), 0.0);
        J.col(7) = xp_cal + Jproj * (Rt * dX2_df);
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }
    }

    if (weight_sampson_ > 0.0) {
      Eigen::Matrix<double, 1, 9> dF_mat;
      const double r = ComputeSampsonJacobian(x1_[i], x2_[i], F, &dF_mat);
      Eigen::Matrix<double, 1, 8> J_sam;
      J_sam.block<1, 3>(0, 0) = dF_mat * dR_F;
      J_sam.block<1, 3>(0, 3) = dF_mat * dt_F;
      J_sam(0, 6) = 0.0;
      J_sam(0, 7) = (dF_mat * df_F)(0, 0);
      acc.AddJacobian(r, J_sam, weight_sampson_);
    }
  }
}

MonoDepthSharedFocalRelativePoseRefiner::Model
MonoDepthSharedFocalRelativePoseRefiner::Step(const Eigen::VectorXd& dp,
                                              const Model& geometry) const {
  Model result = geometry;
  result.rotation = geometry.rotation * ExpSO3(dp.head<3>());
  result.translation = geometry.translation + dp.segment<3>(3);
  result.scale = geometry.scale + dp(6);
  result.focal1 = geometry.focal1 + dp(7);
  result.focal2 = result.focal1;
  return result;
}

// ---------------------------------------------------------------------------
// Varying-focal monodepth refiner
// ---------------------------------------------------------------------------

MonoDepthVaryingFocalRelativePoseRefiner::
    MonoDepthVaryingFocalRelativePoseRefiner(
        const std::vector<Eigen::Vector2d>& x1,
        const std::vector<Eigen::Vector2d>& x2,
        const std::vector<double>& depth1,
        const std::vector<double>& depth2,
        double scale_reproj,
        double weight_sampson)
    : x1_(x1),
      x2_(x2),
      depth1_(depth1),
      depth2_(depth2),
      scale_reproj_(scale_reproj),
      weight_sampson_(weight_sampson) {}

double MonoDepthVaryingFocalRelativePoseRefiner::ComputeResidual(
    NormalAccumulator& acc, const Model& geometry) {
  const double scale = geometry.scale;
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Vector3d& t = geometry.translation;
  const double f1 = geometry.focal1;
  const double f2 = geometry.focal2;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E = CrossProductMatrix(t) * R;
  const Eigen::DiagonalMatrix<double, 3> K1_inv(1.0, 1.0, f1);
  const Eigen::DiagonalMatrix<double, 3> K2_inv(1.0, 1.0, f2);
  const Eigen::Matrix3d F = K2_inv * E * K1_inv;

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (weight_sampson_ > 0.0) {
      acc.AddResidual(ComputeSampsonResidual(x1_[i], x2_[i], F),
                      weight_sampson_);
    }
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d b1(x1_[i](0) / f1, x1_[i](1) / f1, 1.0);
      const Eigen::Vector3d b2(x2_[i](0) / f2, x2_[i](1) / f2, 1.0);
      const Eigen::Vector3d Z1 = R * (depth1_[i] * b1) + t;
      const Eigen::Vector3d Z2 =
          R.transpose() * (scale * depth2_[i] * b2 - t);
      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        Eigen::Vector2d res;
        res << f2 * Z1(0) * inv_z - x2_[i](0), f2 * Z1(1) * inv_z - x2_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        Eigen::Vector2d res;
        res << f1 * Z2(0) * inv_z - x1_[i](0), f1 * Z2(1) * inv_z - x1_[i](1);
        res *= sr;
        acc.AddResidual(res);
      }
    }
  }
  return acc.Cost();
}

void MonoDepthVaryingFocalRelativePoseRefiner::ComputeJacobian(
    NormalAccumulator& acc, const Model& geometry) {
  const Eigen::Matrix3d R = geometry.rotation;
  const Eigen::Matrix3d Rt = R.transpose();
  const double scale = geometry.scale;
  const double f1 = geometry.focal1;
  const double f2 = geometry.focal2;
  const double sr = std::sqrt(scale_reproj_);
  const Eigen::Matrix3d E =
      CrossProductMatrix(geometry.translation) * R;
  const Eigen::DiagonalMatrix<double, 3> K1_inv(1.0, 1.0, f1);
  const Eigen::DiagonalMatrix<double, 3> K2_inv(1.0, 1.0, f2);
  const Eigen::Matrix3d F = K2_inv * E * K1_inv;

  Eigen::Matrix<double, 9, 3> dR_E, dt_E;
  DerivEssentialWrtRotation(E, &dR_E);
  DerivEssentialWrtTranslation(R, &dt_E);

  // F = K2_inv * E * K1_inv  → scale rows/cols of dE by focals.
  Eigen::Matrix<double, 9, 3> dR_F = dR_E, dt_F = dt_E;
  for (int c = 0; c < 3; ++c) {
    dR_F(2, c) *= f1;
    dR_F(5, c) *= f1;
    dR_F(6, c) *= f2;
    dR_F(7, c) *= f2;
    dR_F(8, c) *= f1 * f2;
    dt_F(2, c) *= f1;
    dt_F(5, c) *= f1;
    dt_F(6, c) *= f2;
    dt_F(7, c) *= f2;
    dt_F(8, c) *= f1 * f2;
  }

  Eigen::Matrix<double, 9, 1> df1_F, df2_F;
  df1_F << 0, 0, E(2, 0), 0, 0, E(2, 1), 0, 0, f2 * E(2, 2);
  df2_F << 0, 0, 0, 0, 0, 0, E(0, 2), E(1, 2), f1 * E(2, 2);

  Eigen::Matrix<double, 2, 3> Jproj;
  Jproj.setZero();

  for (size_t i = 0; i < x1_.size(); ++i) {
    if (scale_reproj_ > 0.0) {
      const Eigen::Vector3d b1(x1_[i](0) / f1, x1_[i](1) / f1, 1.0);
      const Eigen::Vector3d b2(x2_[i](0) / f2, x2_[i](1) / f2, 1.0);
      const Eigen::Vector3d X1i = depth1_[i] * b1;
      const Eigen::Vector3d Z1 = R * X1i + geometry.translation;
      const Eigen::Vector3d X2s = depth2_[i] * b2;
      const Eigen::Vector3d X2i = scale * X2s;
      const Eigen::Vector3d Z2 = Rt * (X2i - geometry.translation);

      if (Z1(2) > 0) {
        const double inv_z = 1.0 / Z1(2);
        const Eigen::Vector2d xp_cal(Z1(0) * inv_z, Z1(1) * inv_z);
        Eigen::Vector2d res = f2 * xp_cal - x2_[i];

        Jproj(0, 0) = f2 * inv_z;
        Jproj(1, 1) = f2 * inv_z;
        Jproj(0, 2) = -f2 * Z1(0) * inv_z * inv_z;
        Jproj(1, 2) = -f2 * Z1(1) * inv_z * inv_z;

        Eigen::Matrix<double, 2, 9> J;
        Eigen::Matrix<double, 2, 3> dZ = Jproj * R;
        J.col(0) = -X1i(2) * dZ.col(1) + X1i(1) * dZ.col(2);
        J.col(1) = X1i(2) * dZ.col(0) - X1i(0) * dZ.col(2);
        J.col(2) = -X1i(1) * dZ.col(0) + X1i(0) * dZ.col(1);
        J.block<2, 3>(0, 3) = Jproj;
        J.col(6).setZero();
        const Eigen::Vector3d dX1_df1 =
            depth1_[i] * Eigen::Vector3d(-x1_[i](0) / (f1 * f1),
                                         -x1_[i](1) / (f1 * f1), 0.0);
        J.col(7) = Jproj * (R * dX1_df1);
        J.col(8) = xp_cal;
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }

      if (Z2(2) > 0) {
        const double inv_z = 1.0 / Z2(2);
        const Eigen::Vector2d xp_cal(Z2(0) * inv_z, Z2(1) * inv_z);
        Eigen::Vector2d res = f1 * xp_cal - x1_[i];

        Jproj(0, 0) = f1 * inv_z;
        Jproj(1, 1) = f1 * inv_z;
        Jproj(0, 2) = -f1 * Z2(0) * inv_z * inv_z;
        Jproj(1, 2) = -f1 * Z2(1) * inv_z * inv_z;

        Eigen::Matrix<double, 2, 9> J;
        const Eigen::Vector3d X2t = X2i - geometry.translation;
        Eigen::Matrix3d dZdr;
        dZdr.setZero();
        dZdr(1, 0) = X2t.dot(R.col(2));
        dZdr(2, 0) = -X2t.dot(R.col(1));
        dZdr(0, 1) = -X2t.dot(R.col(2));
        dZdr(2, 1) = X2t.dot(R.col(0));
        dZdr(0, 2) = X2t.dot(R.col(1));
        dZdr(1, 2) = -X2t.dot(R.col(0));
        J.block<2, 3>(0, 0) = Jproj * dZdr;
        J.block<2, 3>(0, 3) = -Jproj * Rt;
        J.col(6) = Jproj * Rt * X2s;
        J.col(7) = xp_cal;
        const Eigen::Vector3d dX2_df2 =
            scale * depth2_[i] *
            Eigen::Vector3d(-x2_[i](0) / (f2 * f2), -x2_[i](1) / (f2 * f2),
                            0.0);
        J.col(8) = Jproj * (Rt * dX2_df2);
        res *= sr;
        J *= sr;
        acc.AddJacobian(res, J);
      }
    }

    if (weight_sampson_ > 0.0) {
      Eigen::Matrix<double, 1, 9> dF_mat;
      const double r = ComputeSampsonJacobian(x1_[i], x2_[i], F, &dF_mat);
      Eigen::Matrix<double, 1, 9> J_sam;
      J_sam.block<1, 3>(0, 0) = dF_mat * dR_F;
      J_sam.block<1, 3>(0, 3) = dF_mat * dt_F;
      J_sam(0, 6) = 0.0;
      J_sam(0, 7) = (dF_mat * df1_F)(0, 0);
      J_sam(0, 8) = (dF_mat * df2_F)(0, 0);
      acc.AddJacobian(r, J_sam, weight_sampson_);
    }
  }
}

MonoDepthVaryingFocalRelativePoseRefiner::Model
MonoDepthVaryingFocalRelativePoseRefiner::Step(const Eigen::VectorXd& dp,
                                               const Model& geometry) const {
  Model result = geometry;
  result.rotation = geometry.rotation * ExpSO3(dp.head<3>());
  result.translation = geometry.translation + dp.segment<3>(3);
  result.scale = geometry.scale + dp(6);
  result.focal1 = geometry.focal1 + dp(7);
  result.focal2 = geometry.focal2 + dp(8);
  return result;
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

bool RefineMonoDepthRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* shift1,
    double* shift2,
    LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(position);
  CHECK_NOTNULL(scale);
  CHECK_NOTNULL(shift1);
  CHECK_NOTNULL(shift2);
  CHECK_EQ(x1.size(), x2.size());
  CHECK_EQ(x1.size(), depth1.size());
  CHECK_EQ(x1.size(), depth2.size());
  if (x1.size() < 3) {
    return false;
  }

  MonoDepthLMState state;
  state.rotation = *rotation;
  state.shift1 = *shift1;
  state.shift2 = *shift2;
  RecoverMetricTranslationAndScale(x1, x2, depth1, depth2, *rotation, *position,
                                   *shift1, *shift2, 1.0, 1.0,
                                   &state.translation, scale);
  state.scale = *scale;

  // scale_reproj weights reprojection vs Sampson; match PoseLib defaults when
  // only a Sampson threshold is available.
  const double scale_reproj = 1.0;
  const double weight_sampson = 1.0;
  MonoDepthRelativePoseRefiner refiner(
      x1, x2, depth1, depth2, scale_reproj, weight_sampson, /*refine_shift=*/true);

  LmOptions opt;
  opt.max_iterations = 25;
  opt.loss_scale = squared_error_thresh;
  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *position = -state.rotation.transpose() * state.translation;
  const double n = position->norm();
  if (n < 1e-12) {
    return false;
  }
  *position /= n;
  *scale = state.scale;
  *shift1 = state.shift1;
  *shift2 = state.shift2;
  return local_stats.final_cost < local_stats.initial_cost;
}

bool RefineMonoDepthSharedFocalRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* focal_length,
    LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(position);
  CHECK_NOTNULL(scale);
  CHECK_NOTNULL(focal_length);
  if (x1.size() < 3 || *focal_length <= 0.0) {
    return false;
  }

  MonoDepthLMState state;
  state.rotation = *rotation;
  state.focal1 = *focal_length;
  state.focal2 = *focal_length;
  RecoverMetricTranslationAndScale(x1, x2, depth1, depth2, *rotation, *position,
                                   0.0, 0.0, 1.0 / (*focal_length),
                                   1.0 / (*focal_length), &state.translation,
                                   scale);
  state.scale = *scale;

  MonoDepthSharedFocalRelativePoseRefiner refiner(
      x1, x2, depth1, depth2, /*scale_reproj=*/1.0, /*weight_sampson=*/1.0);
  LmOptions opt;
  opt.max_iterations = 25;
  opt.loss_scale = squared_error_thresh;
  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *position = -state.rotation.transpose() * state.translation;
  const double n = position->norm();
  if (n < 1e-12 || state.focal1 <= 0.0) {
    return false;
  }
  *position /= n;
  *scale = state.scale;
  *focal_length = state.focal1;
  return local_stats.final_cost < local_stats.initial_cost;
}

bool RefineMonoDepthVaryingFocalRelativePose(
    const std::vector<Eigen::Vector2d>& x1,
    const std::vector<Eigen::Vector2d>& x2,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    double squared_error_thresh,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* position,
    double* scale,
    double* focal_length1,
    double* focal_length2,
    LmStats* stats) {
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(position);
  CHECK_NOTNULL(scale);
  CHECK_NOTNULL(focal_length1);
  CHECK_NOTNULL(focal_length2);
  if (x1.size() < 3 || *focal_length1 <= 0.0 || *focal_length2 <= 0.0) {
    return false;
  }

  MonoDepthLMState state;
  state.rotation = *rotation;
  state.focal1 = *focal_length1;
  state.focal2 = *focal_length2;
  RecoverMetricTranslationAndScale(x1, x2, depth1, depth2, *rotation, *position,
                                   0.0, 0.0, 1.0 / (*focal_length1),
                                   1.0 / (*focal_length2), &state.translation,
                                   scale);
  state.scale = *scale;

  MonoDepthVaryingFocalRelativePoseRefiner refiner(
      x1, x2, depth1, depth2, /*scale_reproj=*/1.0, /*weight_sampson=*/1.0);
  LmOptions opt;
  opt.max_iterations = 25;
  opt.loss_scale = squared_error_thresh;
  const LmStats local_stats = MinimizeLM(refiner, &state, opt);
  if (stats != nullptr) {
    *stats = local_stats;
  }

  *rotation = state.rotation;
  *position = -state.rotation.transpose() * state.translation;
  const double n = position->norm();
  if (n < 1e-12 || state.focal1 <= 0.0 || state.focal2 <= 0.0) {
    return false;
  }
  *position /= n;
  *scale = state.scale;
  *focal_length1 = state.focal1;
  *focal_length2 = state.focal2;
  return local_stats.final_cost < local_stats.initial_cost;
}

}  // namespace theia
