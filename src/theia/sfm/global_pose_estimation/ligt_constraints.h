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

// Shared constraint math of the LiGT pose-only position estimator. Used by
// LiGT and by estimators that build the same triplet constraints.
//
// Based on: Cai, Zhang, Wu, Yu, Hu, "A Pose-only Solution to Visual
// Reconstruction and Navigation", IEEE TPAMI 45(1):73-86, 2023.

#ifndef THEIA_SFM_GLOBAL_POSE_ESTIMATION_LIGT_CONSTRAINTS_H_
#define THEIA_SFM_GLOBAL_POSE_ESTIMATION_LIGT_CONSTRAINTS_H_

#include <Eigen/Core>
#include <vector>
#include <unordered_map>
#include <utility>

#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/sfm/reconstruction.h"

namespace theia {

// Skew-symmetric matrix [f]_x such that [f]_x v = f x v.
Eigen::Matrix3d GetSkew(const Eigen::Vector3d& f);

// Relative rotation R_ij = R_j * R_i^T from world-to-camera rotations R_i, R_j.
Eigen::Matrix3d GetRij(const Eigen::Matrix3d& i, const Eigen::Matrix3d& j);

// Squared norm of [feat_j]_x * R_ij * feat_i (note: the squared norm).
double GetThetaSq(const Eigen::Vector3d& feat_i,
                  const Eigen::Vector3d& feat_j,
                  const Eigen::Matrix3d& Rij);

// Row vector a_ij^T = ([R_ij X_i]_x X_j)^T [X_j]_x.
Eigen::Vector3d Get_aij(const Eigen::Matrix3d& Rij,
                        const Eigen::Vector3d Xi,
                        const Eigen::Vector3d Xj);

// Returns the feature of `track_id` in `view` as a homogeneous unit-depth
// normalized image ray (x, y, 1), i.e. with the camera intrinsics removed.
Eigen::Vector3d GetNormalizedFeatureRay(const View& view, TrackId track_id);

// Computes the LiGT constraint blocks of Eq. 8 of the paper for a triplet with
// normalized features x_i, x_j, x_l and world-to-camera rotations R_i, R_j,
// R_l. The constraint on the world camera centers c is
//   (B + C) c_i - B c_l - C c_j = 0,
// where B = [x_j]_x R_ij x_i a_il^T R_l and C = thetaSq_il [x_j]_x R_j.
void ComputeLiGTBC(const Eigen::Vector3d& x_i,
                   const Eigen::Vector3d& x_j,
                   const Eigen::Vector3d& x_l,
                   const Eigen::Matrix3d& R_i,
                   const Eigen::Matrix3d& R_j,
                   const Eigen::Matrix3d& R_l,
                   Eigen::Matrix3d* B,
                   Eigen::Matrix3d* C);

// Selects the pair of views (among the views of `track_id` that have a
// rotation in `rotations`) with the largest triangulation angle, eq. 29 of the
// paper. Returns a default-constructed pair if fewer than two views qualify.
std::pair<ViewId, ViewId> SelectBestBaseViews(
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewId, Eigen::Matrix3d>& rotations,
    TrackId track_id);

// Two-view translation with known rotations (TRRM, Li/Cai/Wu 2026, Eq. 11):
// t_ij = R_j (c_i - c_j) in the frame of camera j is orthogonal to every
// theta_k = (R_ij x_ik) x x_jk, so it is the smallest eigenvector of
// P = sum_k theta_k theta_k^T. Its sign is the majority sign of the two-view
// depths (chirality). Returns false if P is degenerate (second eigenvalue
// <= min_eigen_ratio * largest). eigen_ratio (optional) receives
// lambda_0 / lambda_1 of P, a measure of how well t_ij is determined.
bool SolveTwoViewTranslation(const std::vector<Eigen::Vector3d>& x_i,
                             const std::vector<Eigen::Vector3d>& x_j,
                             const Eigen::Matrix3d& R_i,
                             const Eigen::Matrix3d& R_j,
                             double min_eigen_ratio,
                             Eigen::Vector3d* t_ij,
                             double* eigen_ratio);

// Votes on the sign of the estimated positions: each view pair whose relative
// translation agrees with the estimated positions counts +1, otherwise -1.
// Returns true if the vote is negative, i.e. all positions should be flipped.
// Only pairs with both positions estimated are counted.
bool PositionsHaveWrongSign(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Matrix3d>& rotations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& positions);

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_POSE_ESTIMATION_LIGT_CONSTRAINTS_H_
