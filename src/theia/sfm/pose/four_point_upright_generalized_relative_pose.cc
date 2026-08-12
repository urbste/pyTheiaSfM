// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/pose/four_point_upright_generalized_relative_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <vector>

#include "theia/sfm/pose/four_point_relative_pose_partial_rotation.h"

namespace theia {

int FourPointUprightGeneralizedRelativePose(
    const Eigen::Vector3d& gravity_axis,
    const std::vector<GeneralizedRayCorrespondence>& correspondences,
    std::vector<RigidTransformation>* solutions) {
  CHECK_NOTNULL(solutions)->clear();
  if (correspondences.size() != 4) {
    return 0;
  }
  if (gravity_axis.squaredNorm() < 1e-16) {
    return 0;
  }

  const Eigen::Vector3d axis = gravity_axis.normalized();
  Eigen::Vector3d dirs1[4], origins1[4], dirs2[4], origins2[4];
  for (int i = 0; i < 4; ++i) {
    dirs1[i] = correspondences[i].direction1.normalized();
    origins1[i] = correspondences[i].origin1;
    dirs2[i] = correspondences[i].direction2.normalized();
    origins2[i] = correspondences[i].origin2;
  }

  std::vector<Eigen::Quaterniond> rotations;
  std::vector<Eigen::Vector3d> translations;
  FourPointRelativePosePartialRotation(
      axis, dirs1, origins1, dirs2, origins2, &rotations, &translations);

  solutions->reserve(rotations.size());
  for (size_t i = 0; i < rotations.size(); ++i) {
    RigidTransformation pose;
    pose.rotation = rotations[i].toRotationMatrix();
    pose.translation = translations[i];
    solutions->push_back(pose);
  }
  return static_cast<int>(solutions->size());
}

}  // namespace theia
