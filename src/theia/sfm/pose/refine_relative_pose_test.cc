// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_relative_pose.h"
#include "theia/sfm/pose/util.h"
#include "theia/util/random.h"

namespace theia {
namespace {

void MakeSyntheticCorrespondences(const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t,
                                  int num_points,
                                  double noise_std,
                                  RandomNumberGenerator* rng,
                                  std::vector<Eigen::Vector2d>* x1,
                                  std::vector<Eigen::Vector2d>* x2) {
  x1->clear();
  x2->clear();
  x1->reserve(num_points);
  x2->reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    Eigen::Vector3d X(rng->RandDouble(-1.0, 1.0),
                      rng->RandDouble(-1.0, 1.0),
                      rng->RandDouble(2.0, 5.0));
    const Eigen::Vector3d x1h = X;
    const Eigen::Vector3d x2h = R * X + t;
    Eigen::Vector2d p1 = x1h.hnormalized();
    Eigen::Vector2d p2 = x2h.hnormalized();
    if (noise_std > 0.0) {
      p1 += Eigen::Vector2d(rng->RandGaussian(0.0, noise_std),
                            rng->RandGaussian(0.0, noise_std));
      p2 += Eigen::Vector2d(rng->RandGaussian(0.0, noise_std),
                            rng->RandGaussian(0.0, noise_std));
    }
    x1->push_back(p1);
    x2->push_back(p2);
  }
}

TEST(RefineRelativePose, JacobianMatchesFiniteDifferences) {
  RandomNumberGenerator rng(42);
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d t(0.1, -0.05, 0.8);
  t.normalize();

  std::vector<Eigen::Vector2d> x1, x2;
  MakeSyntheticCorrespondences(R, t, 30, 0.0, &rng, &x1, &x2);

  RelativePoseLMState pose;
  pose.rotation = R;
  pose.translation = t;

  RelativePoseSampsonRefiner refiner(x1, x2);
  NormalAccumulator acc;
  acc.InitializeTrivial(5);
  acc.ResetJacobian();
  refiner.ComputeJacobian(acc, pose);

  // Finite-difference Jᵀr via residual cost gradients is awkward; instead
  // compare the analytic residual Jacobian columns using Step.
  const double delta = 1e-6;
  Eigen::Matrix<double, Eigen::Dynamic, 5> J_fd(x1.size(), 5);
  Eigen::VectorXd r0(x1.size());

  auto residuals = [&](const RelativePoseLMState& p) {
    Eigen::VectorXd r(x1.size());
    const Eigen::Matrix3d E = EssentialFromMotion(p);
    for (size_t k = 0; k < x1.size(); ++k) {
      const Eigen::Vector3d x1h = x1[k].homogeneous();
      const Eigen::Vector3d x2h = x2[k].homogeneous();
      const double C = x2h.dot(E * x1h);
      const double nJc_sq =
          (E.block<2, 3>(0, 0) * x1h).squaredNorm() +
          (E.block<3, 2>(0, 0).transpose() * x2h).squaredNorm();
      r(k) = C / std::sqrt(nJc_sq);
    }
    return r;
  };

  r0 = residuals(pose);
  SetupTangentBasis(pose.translation, &refiner.tangent_basis_);

  // Build analytic J by accumulating one point at a time into a dense matrix.
  Eigen::MatrixXd J_analytic(x1.size(), 5);
  {
    const Eigen::Matrix3d E = EssentialFromMotion(pose);
    Eigen::Matrix<double, 9, 3> dR;
    Eigen::Matrix<double, 9, 2> dt;
    DerivEssentialWrtPose(E, pose.rotation, refiner.tangent_basis_, &dR, &dt);
    for (size_t k = 0; k < x1.size(); ++k) {
      const Eigen::Vector3d x1h = x1[k].homogeneous();
      const Eigen::Vector3d x2h = x2[k].homogeneous();
      const double C = x2h.dot(E * x1h);
      Eigen::Vector4d J_C;
      J_C << E.block<3, 2>(0, 0).transpose() * x2h, E.block<2, 3>(0, 0) * x1h;
      const double inv_nJ_C = 1.0 / J_C.norm();
      Eigen::Matrix<double, 1, 9> dF;
      dF << x1[k](0) * x2[k](0), x1[k](0) * x2[k](1), x1[k](0),
          x1[k](1) * x2[k](0), x1[k](1) * x2[k](1), x1[k](1), x2[k](0),
          x2[k](1), 1.0;
      const double s = C * inv_nJ_C * inv_nJ_C;
      dF(0) -= s * (J_C(2) * x1[k](0) + J_C(0) * x2[k](0));
      dF(1) -= s * (J_C(3) * x1[k](0) + J_C(0) * x2[k](1));
      dF(2) -= s * (J_C(0));
      dF(3) -= s * (J_C(2) * x1[k](1) + J_C(1) * x2[k](0));
      dF(4) -= s * (J_C(3) * x1[k](1) + J_C(1) * x2[k](1));
      dF(5) -= s * (J_C(1));
      dF(6) -= s * (J_C(2));
      dF(7) -= s * (J_C(3));
      dF *= inv_nJ_C;
      Eigen::Matrix<double, 1, 5> J;
      J.block<1, 3>(0, 0) = dF * dR;
      J.block<1, 2>(0, 3) = dF * dt;
      J_analytic.row(k) = J;
    }
  }

  for (int j = 0; j < 5; ++j) {
    Eigen::VectorXd dp = Eigen::VectorXd::Zero(5);
    dp(j) = delta;
    const RelativePoseLMState pose_fwd = refiner.Step(dp, pose);
    dp(j) = -delta;
    const RelativePoseLMState pose_bwd = refiner.Step(dp, pose);
    const Eigen::VectorXd r_fwd = residuals(pose_fwd);
    const Eigen::VectorXd r_bwd = residuals(pose_bwd);
    J_fd.col(j) = (r_fwd - r_bwd) / (2.0 * delta);
  }

  const double rel_err =
      (J_analytic - J_fd).norm() / std::max(1e-12, J_analytic.norm());
  EXPECT_LT(rel_err, 1e-4) << "analytic:\n"
                           << J_analytic << "\nfd:\n"
                           << J_fd;
}

TEST(RefineRelativePose, ReducesCostFromPerturbedPose) {
  RandomNumberGenerator rng(7);
  Eigen::Matrix3d R_gt =
      Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.3, 0.8, 0.2).normalized())
          .toRotationMatrix();
  Eigen::Vector3d t_gt(0.2, 0.1, 1.0);
  t_gt.normalize();

  std::vector<Eigen::Vector2d> x1, x2;
  MakeSyntheticCorrespondences(R_gt, t_gt, 50, 1e-3, &rng, &x1, &x2);

  // Perturb GT.
  Eigen::Matrix3d R =
      R_gt * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Vector3d t = (t_gt + Eigen::Vector3d(0.02, -0.03, 0.0)).normalized();
  Eigen::Vector3d position = -R.transpose() * t;

  LmStats stats;
  const bool improved =
      RefineRelativePoseSampson(x1, x2, 1e-4, &R, &position, &stats);
  EXPECT_TRUE(improved);
  EXPECT_LT(stats.final_cost, stats.initial_cost);

  const Eigen::Vector3d t_est = -R * position;
  // Rotation / translation should move closer to GT.
  const double rot_err =
      Eigen::AngleAxisd(R.transpose() * R_gt).angle();
  const double t_err =
      std::min((t_est.normalized() - t_gt).norm(),
               (t_est.normalized() + t_gt).norm());
  EXPECT_LT(rot_err, 0.05);
  EXPECT_LT(t_err, 0.05);
}

}  // namespace
}  // namespace theia
