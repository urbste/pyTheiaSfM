// Copyright (C) 2017 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (sweeney.chris.m@gmail.com)

#include "theia/math/rotation.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <limits>

#include "theia/math/util.h"
#include "theia/util/map_util.h"

namespace theia {

// Use Ceres to perform a stable composition of rotations. This is not as
// efficient as directly composing angle axis vectors (see the old
// implementation commented above) but is more stable.
Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation1_mat, rotation2_mat;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation1_mat.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation2_mat.data());

  const Eigen::Matrix3d rotation = rotation1_mat * rotation2_mat;
  Eigen::Vector3d rotation_aa;
  ceres::RotationMatrixToAngleAxis(rotation.data(), rotation_aa.data());
  return rotation_aa;
}

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& position1,
    const Eigen::Vector3d& position2,
    const Eigen::Vector3d& rotation1) {
  Eigen::Matrix3d rotation_matrix1;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  const Eigen::Vector3d relative_translation =
      rotation_matrix1 * (position2 - position1).normalized();
  return relative_translation;
}

// AlignRotations is implemented in sfm/transformation/align_rotations.cc
// (single definition; duplicate in this file broke linking with --whole-archive).

void AlignOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations) {
  // Collect all rotations into a vector.
  std::vector<Eigen::Vector3d> gt_rot, rot;
  std::unordered_map<int, int> index_to_view_id;
  int current_index = 0;
  for (const auto& gt_rotation : gt_rotations) {
    gt_rot.emplace_back(gt_rotation.second);
    rot.emplace_back(FindOrDie(*rotations, gt_rotation.first));

    index_to_view_id[current_index] = gt_rotation.first;
    ++current_index;
  }

  AlignRotations(gt_rot, &rot);

  for (unsigned int i = 0; i < rot.size(); i++) {
    const ViewId view_id = FindOrDie(index_to_view_id, i);
    (*rotations)[view_id] = rot[i];
  }
}

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd& M) {
  // Compute the SVD of M.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  double detU = svd.matrixU().determinant();
  double detV = svd.matrixV().determinant();

  if (detU * detV > 0) {
    return svd.matrixU() * svd.matrixV().transpose();
  } else {
    Eigen::MatrixXd U_prime = svd.matrixU();
    U_prime.col(U_prime.cols() - 1) *= -1;
    return U_prime * svd.matrixV().transpose();
  }
}

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Computes R_ij = R_j * R_i^t. With noise.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2,
    const double noise,
    theia::RandomNumberGenerator& rng) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(noisy_rotation * rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

Eigen::Vector3d ApplyRelativeRotation(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& relative_rotation) {
  Eigen::Vector3d rotation2;
  Eigen::Matrix3d rotation1_matrix, relative_rotation_matrix;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation1_matrix.data()));
  ceres::AngleAxisToRotationMatrix(
      relative_rotation.data(),
      ceres::ColumnMajorAdapter3x3(relative_rotation_matrix.data()));

  const Eigen::Matrix3d rotation2_matrix =
      relative_rotation_matrix * rotation1_matrix;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation2_matrix.data()), rotation2.data());
  return rotation2;
}

}  // namespace theia
