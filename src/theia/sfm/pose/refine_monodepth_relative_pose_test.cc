// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "theia/math/lmlsq/lm_options.h"
#include "theia/math/lmlsq/normal_accumulator.h"
#include "theia/sfm/pose/refine_monodepth_relative_pose.h"
#include "theia/util/random.h"

namespace theia {
namespace {

void MakeMonodepthCorrespondences(const Eigen::Matrix3d& R,
                                  const Eigen::Vector3d& t,
                                  double scale,
                                  int num_points,
                                  double noise_std,
                                  RandomNumberGenerator* rng,
                                  std::vector<Eigen::Vector2d>* x1,
                                  std::vector<Eigen::Vector2d>* x2,
                                  std::vector<double>* d1,
                                  std::vector<double>* d2) {
  x1->clear();
  x2->clear();
  d1->clear();
  d2->clear();
  for (int i = 0; i < num_points; ++i) {
    const Eigen::Vector3d X(rng->RandDouble(-1.0, 1.0),
                            rng->RandDouble(-1.0, 1.0),
                            rng->RandDouble(2.0, 6.0));
    const Eigen::Vector3d X2 = R * X + t;
    Eigen::Vector2d p1 = X.hnormalized();
    Eigen::Vector2d p2 = X2.hnormalized();
    if (noise_std > 0.0) {
      p1 += Eigen::Vector2d(rng->RandGaussian(0.0, noise_std),
                            rng->RandGaussian(0.0, noise_std));
      p2 += Eigen::Vector2d(rng->RandGaussian(0.0, noise_std),
                            rng->RandGaussian(0.0, noise_std));
    }
    x1->push_back(p1);
    x2->push_back(p2);
    // Affine depth corruption: d_est = a * d_true + b with a=scale on view2.
    d1->push_back(X.z());
    d2->push_back(X2.z() / scale);
  }
}

TEST(RefineMonoDepthRelativePose, JacobianStepDecreasesCost) {
  RandomNumberGenerator rng(11);
  Eigen::Matrix3d R_gt =
      Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d t_gt(0.05, -0.02, 0.4);
  const double scale_gt = 1.3;

  std::vector<Eigen::Vector2d> x1, x2;
  std::vector<double> d1, d2;
  MakeMonodepthCorrespondences(
      R_gt, t_gt, scale_gt, 25, 0.0, &rng, &x1, &x2, &d1, &d2);

  // Perturb away from GT so the Gauss-Newton step is nontrivial.
  MonoDepthLMState state;
  state.rotation =
      R_gt * Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX()).toRotationMatrix();
  state.translation = t_gt + Eigen::Vector3d(0.02, -0.01, 0.0);
  state.scale = scale_gt * 0.9;
  state.shift1 = 0.0;
  state.shift2 = 0.0;

  MonoDepthRelativePoseRefiner refiner(
      x1, x2, d1, d2, /*scale_reproj=*/1.0, /*weight_sampson=*/1.0,
      /*refine_shift=*/false);

  auto cost = [&](const MonoDepthLMState& s) {
    NormalAccumulator acc;
    acc.InitializeTrivial(refiner.NumParams());
    acc.ResetResidual();
    return refiner.ComputeResidual(acc, s);
  };

  const double c0 = cost(state);
  NormalAccumulator acc;
  acc.InitializeTrivial(refiner.NumParams());
  acc.ResetJacobian();
  refiner.ComputeJacobian(acc, state);
  const Eigen::VectorXd step = acc.Solve(/*lambda=*/1e-6);
  const MonoDepthLMState state_new = refiner.Step(step, state);
  EXPECT_LT(cost(state_new), c0);
}

TEST(RefineMonoDepthRelativePose, ReducesCostFromPerturbedPose) {
  RandomNumberGenerator rng(3);
  Eigen::Matrix3d R_gt =
      Eigen::AngleAxisd(0.12, Eigen::Vector3d(0.2, 0.9, 0.1).normalized())
          .toRotationMatrix();
  Eigen::Vector3d t_gt(0.15, 0.05, 0.5);
  const double scale_gt = 1.25;

  std::vector<Eigen::Vector2d> x1, x2;
  std::vector<double> d1, d2;
  MakeMonodepthCorrespondences(
      R_gt, t_gt, scale_gt, 40, 5e-4, &rng, &x1, &x2, &d1, &d2);

  Eigen::Matrix3d R =
      R_gt * Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d position = -R.transpose() * t_gt.normalized();
  double scale = 1.0;
  double shift1 = 0.0;
  double shift2 = 0.0;

  LmStats stats;
  const bool improved = RefineMonoDepthRelativePose(
      x1, x2, d1, d2, 1e-3, &R, &position, &scale, &shift1, &shift2, &stats);
  EXPECT_TRUE(improved);
  EXPECT_LT(stats.final_cost, stats.initial_cost);
  EXPECT_NEAR(scale, scale_gt, 0.15);
}

}  // namespace
}  // namespace theia
