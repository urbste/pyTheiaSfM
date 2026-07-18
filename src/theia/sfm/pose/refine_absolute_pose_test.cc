// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_absolute_pose.h"
#include "theia/util/random.h"

namespace theia {
namespace {

void MakeSyntheticPnP(const Eigen::Matrix3d& R,
                      const Eigen::Vector3d& t,
                      int num_points,
                      double noise_std,
                      RandomNumberGenerator* rng,
                      std::vector<Eigen::Vector2d>* features,
                      std::vector<Eigen::Vector3d>* world_points) {
  features->clear();
  world_points->clear();
  for (int i = 0; i < num_points; ++i) {
    const Eigen::Vector3d X(rng->RandDouble(-2.0, 2.0),
                            rng->RandDouble(-2.0, 2.0),
                            rng->RandDouble(4.0, 10.0));
    Eigen::Vector2d x = (R * X + t).hnormalized();
    if (noise_std > 0.0) {
      x += Eigen::Vector2d(rng->RandGaussian(0.0, noise_std),
                           rng->RandGaussian(0.0, noise_std));
    }
    world_points->push_back(X);
    features->push_back(x);
  }
}

TEST(RefineAbsolutePose, JacobianMatchesFiniteDifferences) {
  RandomNumberGenerator rng(19);
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d(0.1, 0.8, 0.3).normalized())
          .toRotationMatrix();
  Eigen::Vector3d t(0.1, -0.2, 0.05);

  std::vector<Eigen::Vector2d> features;
  std::vector<Eigen::Vector3d> world_points;
  MakeSyntheticPnP(R, t, 40, 0.0, &rng, &features, &world_points);

  AbsolutePoseLMState pose;
  pose.rotation = R;
  pose.translation = t;

  AbsolutePoseReprojectionRefiner refiner(features, world_points);

  auto residuals = [&](const AbsolutePoseLMState& p) {
    Eigen::VectorXd r(2 * features.size());
    r.setZero();
    for (size_t i = 0; i < features.size(); ++i) {
      const Eigen::Vector3d Z = p.rotation * world_points[i] + p.translation;
      if (Z(2) < 0.0) {
        continue;
      }
      const Eigen::Vector2d res = Z.hnormalized() - features[i];
      r.segment<2>(2 * i) = res;
    }
    return r;
  };

  Eigen::MatrixXd J_analytic(2 * features.size(), 6);
  J_analytic.setZero();
  {
    Eigen::Matrix<double, 2, 3> Jproj;
    for (size_t i = 0; i < features.size(); ++i) {
      const Eigen::Vector3d& Xi = world_points[i];
      const Eigen::Vector3d Z = pose.rotation * Xi + pose.translation;
      if (Z(2) < 0.0) {
        continue;
      }
      const Eigen::Vector2d zp = Z.hnormalized();
      Jproj << 1.0 / Z(2), 0.0, -zp(0) / Z(2), 0.0, 1.0 / Z(2), -zp(1) / Z(2);
      const Eigen::Matrix<double, 2, 3> dZ = Jproj * pose.rotation;
      Eigen::Matrix<double, 2, 6> J;
      J.col(0) = -Xi(2) * dZ.col(1) + Xi(1) * dZ.col(2);
      J.col(1) = Xi(2) * dZ.col(0) - Xi(0) * dZ.col(2);
      J.col(2) = -Xi(1) * dZ.col(0) + Xi(0) * dZ.col(1);
      J.block<2, 3>(0, 3) = dZ;
      J_analytic.block<2, 6>(2 * i, 0) = J;
    }
  }

  const double delta = 1e-6;
  Eigen::MatrixXd J_fd(2 * features.size(), 6);
  for (int j = 0; j < 6; ++j) {
    Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
    dp(j) = delta;
    const AbsolutePoseLMState pose_fwd = refiner.Step(dp, pose);
    dp(j) = -delta;
    const AbsolutePoseLMState pose_bwd = refiner.Step(dp, pose);
    J_fd.col(j) =
        (residuals(pose_fwd) - residuals(pose_bwd)) / (2.0 * delta);
  }

  const double rel_err =
      (J_analytic - J_fd).norm() / std::max(1e-12, J_analytic.norm());
  EXPECT_LT(rel_err, 1e-4);
}

TEST(RefineAbsolutePose, ReducesCostFromPerturbedPose) {
  RandomNumberGenerator rng(5);
  Eigen::Matrix3d R_gt =
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d t_gt(0.05, 0.1, -0.02);
  Eigen::Vector3d position_gt = -R_gt.transpose() * t_gt;

  std::vector<Eigen::Vector2d> features;
  std::vector<Eigen::Vector3d> world_points;
  MakeSyntheticPnP(R_gt, t_gt, 50, 1e-3, &rng, &features, &world_points);

  Eigen::Matrix3d R =
      R_gt * Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Vector3d position =
      position_gt + Eigen::Vector3d(0.02, -0.015, 0.01);

  LmStats stats;
  // Truncation width must cover the initial (inlier-scale) residuals so LM
  // has non-zero Jacobian support — same regime as RANSAC LO on inliers.
  const bool improved = RefineAbsolutePoseReprojection(
      features, world_points, 1e-2, &R, &position, &stats);
  EXPECT_TRUE(improved);
  EXPECT_LT(stats.final_cost, stats.initial_cost);

  const double rot_err = Eigen::AngleAxisd(R.transpose() * R_gt).angle();
  const double pos_err = (position - position_gt).norm();
  EXPECT_LT(rot_err, 0.02);
  EXPECT_LT(pos_err, 0.02);
}

}  // namespace
}  // namespace theia
