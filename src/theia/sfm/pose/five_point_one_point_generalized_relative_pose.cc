// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/pose/five_point_one_point_generalized_relative_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <glog/logging.h>
#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/pose/essential_matrix_utils.h"
#include "theia/sfm/pose/five_point_relative_pose.h"
#include "theia/sfm/pose/five_point_relative_pose_sturm.h"

namespace theia {
namespace {

bool OriginsMatch(const Eigen::Vector3d& a,
                  const Eigen::Vector3d& b,
                  const double tol = 1e-9) {
  return (a - b).squaredNorm() <= tol * tol;
}

}  // namespace

int FivePointOnePointGeneralizedRelativePose(
    const std::vector<GeneralizedRayCorrespondence>& correspondences,
    std::vector<RigidTransformation>* solutions) {
  CHECK_NOTNULL(solutions)->clear();
  if (correspondences.size() != 6) {
    return 0;
  }

  // First five must share origins (central pair).
  for (int i = 1; i < 5; ++i) {
    if (!OriginsMatch(correspondences[i].origin1, correspondences[0].origin1) ||
        !OriginsMatch(correspondences[i].origin2, correspondences[0].origin2)) {
      return 0;
    }
  }

  std::vector<Eigen::Vector3d> x1(5), x2(5);
  for (int i = 0; i < 5; ++i) {
    x1[i] = correspondences[i].direction1.normalized();
    x2[i] = correspondences[i].direction2.normalized();
  }

  std::vector<Eigen::Matrix3d> essential_matrices;
  if (FivePointRelativePoseSturm(x1, x2, &essential_matrices) == 0) {
    // Fallback to Stewenius polynomial solver.
    std::vector<Eigen::Vector2d> img1(5), img2(5);
    for (int i = 0; i < 5; ++i) {
      // Project unit rays to normalized image plane z=1.
      if (std::abs(x1[i].z()) < 1e-12 || std::abs(x2[i].z()) < 1e-12) {
        return 0;
      }
      img1[i] = x1[i].hnormalized();
      img2[i] = x2[i].hnormalized();
    }
    if (!FivePointRelativePose(img1, img2, &essential_matrices)) {
      return 0;
    }
  }

  // Build central FeatureCorrespondences for cheirality disambiguation of R,t.
  std::vector<FeatureCorrespondence> central_corrs;
  central_corrs.reserve(5);
  for (int i = 0; i < 5; ++i) {
    FeatureCorrespondence corr;
    corr.feature1 = Feature(x1[i].hnormalized());
    corr.feature2 = Feature(x2[i].hnormalized());
    central_corrs.push_back(corr);
  }

  const Eigen::Vector3d& p1 = correspondences[0].origin1;
  const Eigen::Vector3d& p2 = correspondences[0].origin2;
  const GeneralizedRayCorrespondence& scale_corr = correspondences[5];

  solutions->reserve(essential_matrices.size());
  for (const Eigen::Matrix3d& E : essential_matrices) {
    Eigen::Matrix3d R;
    Eigen::Vector3d position;
    if (GetBestPoseFromEssentialMatrix(E, central_corrs, &R, &position) < 1) {
      continue;
    }
    // Convert camera-center convention back to PoseLib t: X2 = R X1 + t
    // with C2 = -R^T t  =>  t = -R * C2.
    const Eigen::Vector3d t_unit = -R * position;

    // t = (p2 - R p1) + gamma * t_unit
    const Eigen::Vector3d a = p2 - R * p1;
    const Eigen::Vector3d b = t_unit;

    const Eigen::Vector3d w =
        scale_corr.direction2.normalized().cross(R * scale_corr.direction1.normalized());
    const double c1 = w.dot(b);
    if (std::abs(c1) < 1e-12) {
      continue;
    }
    const double c0 = w.dot(scale_corr.origin2 - R * scale_corr.origin1 - a);
    const double gamma = c0 / c1;

    RigidTransformation pose;
    pose.rotation = R;
    pose.translation = a + gamma * b;
    solutions->push_back(pose);
  }

  return static_cast<int>(solutions->size());
}

}  // namespace theia
