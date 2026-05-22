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

#include "theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <ceres/rotation.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "theia/math/constrained_l1_solver.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"
#include "theia/util/util.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

Vector3d GetRotatedTranslation(const Vector3d& rotation_angle_axis,
                               const Vector3d& translation) {
  Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(
      rotation_angle_axis.data(),
      ceres::ColumnMajorAdapter3x3(rotation.data()));
  return rotation.transpose() * translation;
}

}  // namespace

LeastUnsquaredDeviationPositionEstimator::
    LeastUnsquaredDeviationPositionEstimator(
        const LeastUnsquaredDeviationPositionEstimator::Options& options)
    : options_(options) {
  CHECK_GT(options_.max_num_iterations, 0);
  CHECK_GT(options_.max_num_reweighted_iterations, 0);
}

bool LeastUnsquaredDeviationPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    std::unordered_map<ViewId, Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();

  InitializeIndexMapping(view_pairs, orientations);
  const int num_views = static_cast<int>(view_id_to_index_.size());
  const int num_free_scales =
      static_cast<int>(view_id_pair_to_index_.size());

  Eigen::VectorXd b;
  SetupConstraintMatrix(view_pairs, orientations, &b);

  Eigen::VectorXd solution;
  solution.setZero(constraint_matrix_.cols());

  // For edges without a fixed scale, enforce auxiliary scale >= 1.
  Eigen::SparseMatrix<double> geq_mat(num_free_scales,
                                      constraint_matrix_.cols());
  for (int i = 0; i < num_free_scales; i++) {
    geq_mat.insert(i, 3 * (num_views - 1) + i) = 1.0;
  }
  Eigen::VectorXd geq_vec(num_free_scales);
  geq_vec.setConstant(1.0);

  // Use `ConstrainedL1Solver` defaults for ADMM (matches pre-change LUD).
  ConstrainedL1Solver::Options l1_options;
  ConstrainedL1Solver solver(l1_options, constraint_matrix_, b, geq_mat,
                             geq_vec);
  solver.Solve(&solution);

  for (const auto& view_id_index : view_id_to_index_) {
    const int index = view_id_index.second;
    const ViewId view_id = view_id_index.first;
    if (index == kConstantViewIndex) {
      (*positions)[view_id] = Eigen::Vector3d::Zero();
    } else {
      (*positions)[view_id] = solution.segment<3>(index);
    }
  }

  return true;
}

void LeastUnsquaredDeviationPositionEstimator::InitializeIndexMapping(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations) {
  view_id_to_index_.clear();
  view_id_pair_to_index_.clear();

  std::unordered_set<ViewId> views;
  for (const auto& view_pair : view_pairs) {
    if (ContainsKey(orientations, view_pair.first.first) &&
        ContainsKey(orientations, view_pair.first.second)) {
      views.insert(view_pair.first.first);
      views.insert(view_pair.first.second);
    }
  }

  int index = kConstantViewIndex;
  view_id_to_index_.reserve(views.size());
  for (const ViewId view_id : views) {
    view_id_to_index_[view_id] = index;
    index += 3;
  }

  view_id_pair_to_index_.reserve(view_pairs.size());
  for (const auto& view_pair : view_pairs) {
    if (!ContainsKey(view_id_to_index_, view_pair.first.first) ||
        !ContainsKey(view_id_to_index_, view_pair.first.second)) {
      continue;
    }
    const bool fixed_scale = options_.use_scale_estimates &&
                             view_pair.second.scale_estimate >
                                 options_.min_valid_scale_estimate;
    if (!fixed_scale) {
      view_id_pair_to_index_[view_pair.first] = index;
      ++index;
    }
  }
}

void LeastUnsquaredDeviationPositionEstimator::SetupConstraintMatrix(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    Eigen::VectorXd* b) {
  CHECK_NOTNULL(b);

  const int num_views = static_cast<int>(view_id_to_index_.size());
  const int num_position_cols = 3 * (num_views - 1);
  const int num_free_scales =
      static_cast<int>(view_id_pair_to_index_.size());
  // Legacy LUD used `view_pairs.size()` for the width, which can exceed the
  // number of free-scale columns; unused columns remain all-zero (singular
  // padding). Preserve that shape when not using fixed-scale edges so behavior
  // matches older builds.
  const int num_cols =
      num_position_cols +
      (options_.use_scale_estimates
           ? num_free_scales
           : static_cast<int>(view_pairs.size()));

  int num_edges = 0;
  for (const auto& view_pair : view_pairs) {
    const ViewIdPair& view_id_pair = view_pair.first;
    if (!ContainsKey(view_id_to_index_, view_id_pair.first) ||
        !ContainsKey(view_id_to_index_, view_id_pair.second)) {
      continue;
    }
    ++num_edges;
  }

  const int num_rows = 3 * num_edges;
  constraint_matrix_.resize(num_rows, num_cols);
  b->resize(num_rows);
  b->setZero();

  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(12 * num_edges);
  int row = 0;
  for (const auto& view_pair : view_pairs) {
    const ViewIdPair view_id_pair = view_pair.first;
    if (!ContainsKey(view_id_to_index_, view_id_pair.first) ||
        !ContainsKey(view_id_to_index_, view_id_pair.second)) {
      continue;
    }

    const int view1_index = FindOrDie(view_id_to_index_, view_id_pair.first);
    const int view2_index = FindOrDie(view_id_to_index_, view_id_pair.second);
    const bool fixed_scale = options_.use_scale_estimates &&
                             view_pair.second.scale_estimate >
                                 options_.min_valid_scale_estimate;

    const Vector3d translation_direction =
        GetRotatedTranslation(FindOrDie(orientations, view_id_pair.first),
                              view_pair.second.position_2);

    if (view1_index != kConstantViewIndex) {
      triplet_list.emplace_back(row + 0, view1_index + 0, -1.0);
      triplet_list.emplace_back(row + 1, view1_index + 1, -1.0);
      triplet_list.emplace_back(row + 2, view1_index + 2, -1.0);
    }

    if (view2_index != kConstantViewIndex) {
      triplet_list.emplace_back(row + 0, view2_index + 0, 1.0);
      triplet_list.emplace_back(row + 1, view2_index + 1, 1.0);
      triplet_list.emplace_back(row + 2, view2_index + 2, 1.0);
    }

    if (fixed_scale) {
      const double s = view_pair.second.scale_estimate;
      (*b)[row + 0] = s * translation_direction[0];
      (*b)[row + 1] = s * translation_direction[1];
      (*b)[row + 2] = s * translation_direction[2];
    } else {
      const int scale_index =
          FindOrDieNoPrint(view_id_pair_to_index_, view_id_pair);
      triplet_list.emplace_back(row + 0, scale_index, -translation_direction[0]);
      triplet_list.emplace_back(row + 1, scale_index, -translation_direction[1]);
      triplet_list.emplace_back(row + 2, scale_index, -translation_direction[2]);
    }

    row += 3;
  }

  CHECK_EQ(row, num_rows);
  constraint_matrix_.setFromTriplets(triplet_list.begin(), triplet_list.end());

  VLOG(2) << num_edges
          << " camera to camera constraints were added "
             "to the position estimation problem.";
}

std::unordered_map<ViewId, Eigen::Vector3d>
LeastUnsquaredDeviationPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation) {
  std::unordered_map<ViewId, Eigen::Vector3d> positions;
  EstimatePositions(view_pairs, orientation, &positions);
  return positions;
}

}  // namespace theia
