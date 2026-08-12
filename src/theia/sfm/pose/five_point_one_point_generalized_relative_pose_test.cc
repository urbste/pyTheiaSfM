// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "theia/sfm/pose/five_point_one_point_generalized_relative_pose.h"
#include "theia/sfm/pose/four_point_upright_generalized_relative_pose.h"
#include "theia/sfm/estimators/estimate_relative_rig_info.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/random.h"

namespace theia {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

RandomNumberGenerator rng(42);

GeneralizedRayCorrespondence MakeCorr(const Vector3d& X,
                                      const Vector3d& o1,
                                      const Vector3d& o2,
                                      const Matrix3d& R,
                                      const Vector3d& t) {
  GeneralizedRayCorrespondence c;
  c.origin1 = o1;
  c.direction1 = (X - o1).normalized();
  const Vector3d X2 = R * X + t;
  c.origin2 = o2;
  c.direction2 = (X2 - o2).normalized();
  return c;
}

TEST(FivePointOnePointGeneralizedRelativePose, StereoMetricTranslation) {
  // Left/right sensors in each rig frame.
  const Vector3d left(-0.06, 0.0, 0.0);
  const Vector3d right(0.06, 0.0, 0.0);

  const Matrix3d R =
      AngleAxisd(0.15, Vector3d::UnitY()).toRotationMatrix();
  const Vector3d t(0.35, 0.02, 0.08);  // metric

  std::vector<Vector3d> points;
  for (int i = 0; i < 20; ++i) {
    points.emplace_back(rng.RandDouble(-1.0, 1.0),
                        rng.RandDouble(-0.8, 0.8),
                        rng.RandDouble(2.0, 6.0));
  }

  // 5 left↔left + 1 left↔right (scale).
  std::vector<GeneralizedRayCorrespondence> sample;
  for (int i = 0; i < 5; ++i) {
    sample.push_back(MakeCorr(points[i], left, left, R, t));
  }
  sample.push_back(MakeCorr(points[5], left, right, R, t));

  std::vector<RigidTransformation> solutions;
  ASSERT_GT(FivePointOnePointGeneralizedRelativePose(sample, &solutions), 0);

  bool found = false;
  for (const auto& pose : solutions) {
    const double rot_err =
        Eigen::AngleAxisd(pose.rotation.transpose() * R).angle();
    const double t_err = (pose.translation - t).norm();
    if (rot_err < 1e-2 && t_err < 1e-2) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST(FourPointUprightGeneralizedRelativePose, RecoversYawAndTranslation) {
  // Mirror the existing FourPointRelativePosePartialRotation BasicTest geometry
  // with rotation constrained to gravity (Y).
  const Vector3d gravity = Vector3d::UnitY();
  const Matrix3d R = AngleAxisd(0.4, gravity).toRotationMatrix();
  const Vector3d t(-2.0, 3.0, -5.0);

  const Vector3d points[4] = {Vector3d(-1.0, 3.0, 3.0),
                              Vector3d(1.0, -1.0, 2.0),
                              Vector3d(2.0, 1.0, 3.0),
                              Vector3d(4.0, 3.0, 5.0)};
  const Vector3d o1[4] = {Vector3d(-1.0, 0.0, 0.0),
                          Vector3d(0.0, 0.0, 0.0),
                          Vector3d(2.0, 0.0, 0.0),
                          Vector3d(3.0, 0.0, 0.0)};
  const Vector3d o2[4] = {Vector3d(0.0, 1.0, 0.0),
                          Vector3d(0.0, 0.0, 0.0),
                          Vector3d(0.0, 2.0, 0.0),
                          Vector3d(0.0, 3.0, 0.0)};

  std::vector<GeneralizedRayCorrespondence> sample;
  for (int i = 0; i < 4; ++i) {
    sample.push_back(MakeCorr(points[i], o1[i], o2[i], R, t));
  }

  std::vector<RigidTransformation> solutions;
  ASSERT_GT(
      FourPointUprightGeneralizedRelativePose(gravity, sample, &solutions), 0);

  bool found = false;
  for (const auto& pose : solutions) {
    const double rot_err =
        Eigen::AngleAxisd(pose.rotation.transpose() * R).angle();
    const double t_err = (pose.translation - t).norm();
    if (rot_err < 1e-4 && t_err < 1e-3) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST(EstimateRelativeRigInfo, RansacRecoversPose) {
  const Vector3d left(-0.06, 0.0, 0.0);
  const Vector3d right(0.06, 0.0, 0.0);
  const Matrix3d R =
      AngleAxisd(0.12, Vector3d::UnitY()).toRotationMatrix();
  const Vector3d t(0.4, -0.05, 0.15);

  std::vector<GeneralizedRayCorrespondence> central, generalized;
  for (int i = 0; i < 40; ++i) {
    const Vector3d X(rng.RandDouble(-1.0, 1.0),
                     rng.RandDouble(-0.7, 0.7),
                     rng.RandDouble(2.0, 7.0));
    central.push_back(MakeCorr(X, left, left, R, t));
  }
  for (int i = 0; i < 30; ++i) {
    const Vector3d X(rng.RandDouble(-1.0, 1.0),
                     rng.RandDouble(-0.7, 0.7),
                     rng.RandDouble(2.0, 7.0));
    generalized.push_back(MakeCorr(X, left, right, R, t));
  }

  RansacParameters params;
  // Algebraic residual; keep threshold loose enough for RANSAC scoring.
  params.error_thresh = 1e-4;
  params.min_iterations = 100;
  params.max_iterations = 1000;
  params.failure_probability = 0.01;
  params.rng = std::make_shared<RandomNumberGenerator>(7);

  RelativeRigInfo info;
  RansacSummary summary;
  ASSERT_TRUE(EstimateRelativeRigInfo(
      params, central, generalized, &info, &summary));
  EXPECT_GE(summary.inliers.size(), 50u);

  const double rot_err =
      Eigen::AngleAxisd(info.rotation.transpose() * R).angle();
  EXPECT_LT(rot_err, 1e-2);
  EXPECT_LT((info.translation - t).norm(), 5e-2);

  TwoViewInfo twoview;
  info.ToTwoViewInfo(&twoview);
  EXPECT_GT(twoview.scale_estimate, 0.1);
}

}  // namespace
}  // namespace theia
