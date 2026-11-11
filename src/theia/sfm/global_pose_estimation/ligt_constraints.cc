// Copyright (C) 2015 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// Based on: Cai, Zhang, Wu, Yu, Hu, "A Pose-only Solution to Visual
// Reconstruction and Navigation", IEEE TPAMI 45(1):73-86, 2023.

#include "theia/sfm/global_pose_estimation/ligt_constraints.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <unordered_map>
#include <vector>
#include <utility>

#include "theia/sfm/feature.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"

namespace theia {

Eigen::Matrix3d GetSkew(const Eigen::Vector3d& f) {
  Eigen::Matrix3d skew_mat;
  skew_mat << 0.0, -f(2), f(1), f(2), 0.0, -f(0), -f(1), f(0), 0.0;
  return skew_mat;
}

Eigen::Matrix3d GetRij(const Eigen::Matrix3d& i, const Eigen::Matrix3d& j) {
  return j * i.transpose();
}

double GetThetaSq(const Eigen::Vector3d& feat_i,
                  const Eigen::Vector3d& feat_j,
                  const Eigen::Matrix3d& Rij) {
  return (GetSkew(feat_j) * Rij * feat_i).squaredNorm();
}

Eigen::Vector3d Get_aij(const Eigen::Matrix3d& Rij,
                        const Eigen::Vector3d Xi,
                        const Eigen::Vector3d Xj) {
  return (GetSkew(Rij * Xi) * Xj).transpose() * GetSkew(Xj);
}

Eigen::Vector3d GetNormalizedFeatureRay(const View& view, TrackId track_id) {
  const Feature feature = *view.GetFeature(track_id);
  const Camera& camera = view.Camera();
  return camera.PixelToNormalizedCoordinates(feature.point_)
      .hnormalized()
      .homogeneous();
}

void ComputeLiGTBC(const Eigen::Vector3d& x_i,
                   const Eigen::Vector3d& x_j,
                   const Eigen::Vector3d& x_l,
                   const Eigen::Matrix3d& R_i,
                   const Eigen::Matrix3d& R_j,
                   const Eigen::Matrix3d& R_l,
                   Eigen::Matrix3d* B,
                   Eigen::Matrix3d* C) {
  CHECK_NOTNULL(B);
  CHECK_NOTNULL(C);
  const Eigen::Matrix3d R_ij = GetRij(R_i, R_j);
  const Eigen::Matrix3d R_il = GetRij(R_i, R_l);

  const Eigen::Vector3d a_il = Get_aij(R_il, x_i, x_l);

  const Eigen::Matrix3d skew_x_j = GetSkew(x_j);
  // Equation 18 of the paper.
  *B = skew_x_j * R_ij * x_i * a_il.transpose() * R_l;

  const double theta_sq = GetThetaSq(x_i, x_l, R_il);
  *C = theta_sq * skew_x_j * R_j;
}

std::pair<ViewId, ViewId> SelectBestBaseViews(
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewId, Eigen::Matrix3d>& rotations,
    TrackId track_id) {
  const Track* track = reconstruction.Track(track_id);
  std::vector<ViewId> view_ids;
  for (const ViewId view_id : track->ViewIds()) {
    if (ContainsKey(rotations, view_id)) {
      view_ids.push_back(view_id);
    }
  }
  double theta_max = 0.0;
  std::pair<ViewId, ViewId> base_views;
  for (size_t i = 0; i < view_ids.size(); ++i) {
    ViewId id1 = view_ids[i];
    const View* view1 = reconstruction.View(id1);
    const Eigen::Vector3d feature1 = GetNormalizedFeatureRay(*view1, track_id);
    const Eigen::Matrix3d R1 = rotations.at(id1);

    for (size_t j = i + 1; j < view_ids.size(); ++j) {
      ViewId id2 = view_ids[j];
      const View* view2 = reconstruction.View(id2);
      const Eigen::Vector3d feature2 = GetNormalizedFeatureRay(*view2, track_id);

      const Eigen::Matrix3d R2 = rotations.at(id2);
      const Eigen::Matrix3d R12 = GetRij(R1, R2);
      const double theta = GetThetaSq(feature1, feature2, R12);
      if (theta > theta_max) {
        base_views = std::make_pair(id1, id2);
        theta_max = theta;
      }
    }
  }
  return base_views;
}

bool SolveTwoViewTranslation(const std::vector<Eigen::Vector3d>& x_i,
                             const std::vector<Eigen::Vector3d>& x_j,
                             const Eigen::Matrix3d& R_i,
                             const Eigen::Matrix3d& R_j,
                             const double min_eigen_ratio,
                             Eigen::Vector3d* t_ij,
                             double* eigen_ratio) {
  if (x_i.empty() || x_i.size() != x_j.size()) {
    return false;
  }
  const Eigen::Matrix3d R_ij = GetRij(R_i, R_j);
  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  for (size_t k = 0; k < x_i.size(); ++k) {
    const Eigen::Vector3d theta = (R_ij * x_i[k]).cross(x_j[k]);
    P += theta * theta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(P);
  if (eig.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Vector3d values = eig.eigenvalues();  // ascending
  if (!(values(2) > 0.0 && values(1) > min_eigen_ratio * values(2))) {
    return false;
  }
  Eigen::Vector3d t = eig.eigenvectors().col(0);
  // Chirality: z_j x_j = z_i R_ij x_i + t, solved for (z_i, z_j) per track;
  // the sign of t is the majority sign of the depths.
  int num_positive = 0;
  for (size_t k = 0; k < x_i.size(); ++k) {
    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = R_ij * x_i[k];
    A.col(1) = -x_j[k];
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(
        A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector2d z = svd.solve(-t);
    if (z(0) > 0.0 && z(1) > 0.0) {
      ++num_positive;
    }
  }
  if (2 * num_positive < static_cast<int>(x_i.size())) {
    t = -t;
  }
  if (!t.allFinite()) {
    return false;
  }
  *t_ij = t;
  if (eigen_ratio != nullptr) {
    *eigen_ratio = values(0) / values(1);
  }
  return true;
}

namespace {

// Returns true if the vector R1 * (c2 - c1) is in the same direction as t_12.
// Same test as in LiGTPositionEstimator.
bool VectorsAreSameDirection(const Eigen::Vector3d& position1,
                             const Eigen::Vector3d& position2,
                             const Eigen::Matrix3d& rotation1,
                             const Eigen::Vector3d& relative_position12) {
  const Eigen::Vector3d global_relative_position =
      (position2 - position1).normalized();
  const Eigen::Vector3d rotated_relative_position =
      rotation1 * global_relative_position;

  return rotated_relative_position.dot(relative_position12) > 0;
}

}  // namespace

bool PositionsHaveWrongSign(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Matrix3d>& rotations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& positions) {
  // If this value is below zero, then we should flip the sign.
  int correct_sign_votes = 0;
  for (const auto& view_pair : view_pairs) {
    // Only count the votes for edges where both positions were successfully
    // estimated.
    const Eigen::Vector3d* position1 =
        FindOrNull(positions, view_pair.first.first);
    const Eigen::Vector3d* position2 =
        FindOrNull(positions, view_pair.first.second);
    if (position1 == nullptr || position2 == nullptr) {
      continue;
    }

    // Check the relative translation of views 1 and 2 in the triplet.
    if (VectorsAreSameDirection(
            *position1,
            *position2,
            FindOrDieNoPrint(rotations, view_pair.first.first),
            view_pair.second.position_2)) {
      correct_sign_votes += 1;
    } else {
      correct_sign_votes -= 1;
    }
  }

  if (correct_sign_votes < 0) {
    const int num_correct_votes =
        (view_pairs.size() + correct_sign_votes) / 2;
    VLOG(2) << "Sign of the positions was incorrect: " << num_correct_votes
            << " of " << view_pairs.size()
            << " relative translations had the correct sign. "
               "Flipping the sign of the camera positions.";
    return true;
  }
  return false;
}

}  // namespace theia
