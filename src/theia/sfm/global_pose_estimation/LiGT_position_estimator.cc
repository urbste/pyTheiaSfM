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

// Author: Steffen Urban (steffen.urban@googlemail.com), March 2021

#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "spectra/include/SymEigsShiftSolver.h"

#include "theia/math/graph/connected_components.h"
#include "theia/math/graph/triplet_extractor.h"
#include "theia/math/matrix/sparse_cholesky_llt.h"
#include "theia/math/matrix/spectra_linear_operator.h"
#include "theia/sfm/find_common_tracks_in_views.h"
#include "theia/sfm/global_pose_estimation/compute_triplet_baseline_ratios.h"
#include "theia/sfm/global_pose_estimation/ligt_constraints.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_triplet.h"
#include "theia/util/map_util.h"
#include "theia/util/parallel_for.h"
#include "theia/util/threadpool.h"

namespace theia {

using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

namespace {

void AddTripletConstraintToSymmetricMatrix(
    const std::vector<Matrix3d>& constraints,
    const std::vector<int>& view_indices,
    const double weight,
    std::unordered_map<std::pair<int, int>, double>* sparse_matrix_entries) {

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      // Skip any block entries that correspond to the lower triangular portion
      // of the matrix.
      // if (view_indices[i] > view_indices[j]) {
      //   continue;
      // }

      // Compute the A^t * B, etc. matrix.
      const Matrix3d symmetric_constraint =
          weight * constraints[i].transpose() * constraints[j];

      // Add to the 3x3 block corresponding to (i, j)
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          const std::pair<int, int> row_col(view_indices[i] + r,
                                            view_indices[j] + c);
          (*sparse_matrix_entries)[row_col] += symmetric_constraint(r, c);
        }
      }
    }
  }
}

Matrix3d AngleAxisToRotationMatrix(const Vector3d angle_axis) {
  Matrix3d rotation_matrix;
  ceres::AngleAxisToRotationMatrix(angle_axis.data(), rotation_matrix.data());
  return rotation_matrix;
}

// Follows the rig constraints of `view_id` (c_view = c_anchor + delta) until a
// view without a constraint is reached, accumulating the offsets so that
// c_view = c_root + offset. Returns true if that root is part of the system
// (`in_system` has a count() for it). Cycles and broken chains return false.
template <typename SystemViews>
bool ResolveRigChain(
    const std::unordered_map<ViewId, RigOffsetConstraint>& rig_constraints,
    const ViewId view_id,
    const SystemViews& in_system,
    ViewId* root,
    Vector3d* offset) {
  ViewId current = view_id;
  Vector3d total = Vector3d::Zero();
  // A chain without a cycle has at most rig_constraints.size() links.
  for (size_t step = 0; step <= rig_constraints.size(); ++step) {
    const auto it = rig_constraints.find(current);
    if (it == rig_constraints.end()) {
      if (in_system.count(current) == 0) {
        return false;
      }
      *root = current;
      *offset = total;
      return true;
    }
    total += it->second.delta;
    current = it->second.anchor;
  }
  return false;
}

}  // namespace

LiGTPositionEstimator::LiGTPositionEstimator(
    const Options& options, const Reconstruction& reconstruction)
    : options_(options), reconstruction_(reconstruction) {
  CHECK_GT(options.num_threads, 0);
}

bool LiGTPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    std::unordered_map<ViewId, Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();

  num_triplets_for_view_.clear();
  linear_system_index_.clear();
  resolved_rig_constraints_.clear();
  BCDs_.clear();
  triplets_for_tracks_.clear();
  orientations_.clear();

  view_pairs_ = &view_pairs;
  // convert orientations to matrices
  for (const auto& ori : orientations) {
    orientations_[ori.first] = AngleAxisToRotationMatrix(ori.second);
  }

  constraint_weights_.clear();
  RemoveWeaklyConstrainedViews();
  VLOG(2) << "Extracting triplets from tracks and calculating BCDs for tracks.";
  FindTripletsForTracks();
  VLOG(2) << "Finished extracting triplets from tracks and calculating BCDs.";

  BuildLinearSystemIndex();
  // Rig-constrained views count too, although they have no unknowns.
  const size_t num_system_views =
      linear_system_index_.size() + resolved_rig_constraints_.size();
  if (num_system_views < 3) {
    LOG(WARNING) << "LiGT needs tracks that connect at least 3 views, found "
                 << num_system_views << ".";
    return false;
  }
  // With rig constraints the scale is metric, so the scale constraint of
  // ShapeFit is not needed.
  if (resolved_rig_constraints_.empty()) {
    ComputeScaleConstraint();
  }

  VLOG(2) << "Building the constraint matrix...";
  // Create the linear system based on triplet constraints. One position is
  // fixed at the origin, so the (N-1)*3 system gives the positions up to scale
  // (without rig constraints) or exactly (with them).
  Eigen::VectorXd solution;
  Eigen::SparseMatrix<double> constraint_matrix;
  // With fewer than two free views there is no unknown left to solve for; the
  // rig-constrained views then follow the origin view.
  if (linear_system_index_.size() >= 2) {
    CreateLinearSystem(&constraint_matrix);
    solution = SolveLinearSystem(constraint_matrix);
    if (solution.size() != constraint_matrix.cols()) {
      return false;
    }

    // Robustification (IRLS): re-weight the track constraints with the
    // residuals of the current solution and solve again.
    for (int iter = 0; iter < options_.num_irls_iterations; ++iter) {
      const double scale = UpdateConstraintWeights(solution);
      if (!(scale > 0.0)) {
        break;  // perfectly consistent (or degenerate) constraints
      }
      CreateLinearSystem(&constraint_matrix);
      Eigen::VectorXd new_solution = SolveLinearSystem(constraint_matrix);
      if (new_solution.size() != constraint_matrix.cols()) {
        break;  // keep the previous solution
      }
      // The eigenvector sign is arbitrary: align it with the previous solution.
      // Inhomogeneous (rig) solutions have a fixed sign.
      if (resolved_rig_constraints_.empty() &&
          new_solution.dot(solution) < 0.0) {
        new_solution *= -1.0;
      }
      const double change = (new_solution - solution).norm();
      solution = new_solution;
      VLOG(2) << "LiGT IRLS iteration " << iter << ": robust scale " << scale
              << ", solution change " << change;
      if (change < options_.irls_convergence_threshold) {
        break;
      }
    }
  }

  // Add the solutions to the output. Set the position with an index of -1 to
  // be at the origin.
  for (const auto &view_index : linear_system_index_) {
    if (view_index.second < 0) {
      (*positions)[view_index.first].setZero();
    } else {
      (*positions)[view_index.first] =
          solution.segment<3>(view_index.second * 3);
    }
  }

  // Rig-constrained views are not unknowns: their center is the root center
  // plus the known offset (c_view = c_root + offset). The root is always a
  // system view, so its position has been set above.
  for (const auto& resolved : resolved_rig_constraints_) {
    (*positions)[resolved.first] =
        (*positions)[resolved.second.root] + resolved.second.offset;
  }

  // The sign of the eigenvector is arbitrary, which is what the sign check
  // corrects. A metric rig-constrained solution has no such ambiguity.
  if (resolved_rig_constraints_.empty()) {
    FlipSignOfPositionsIfNecessary(positions);
  }

  return true;
}

std::pair<ViewId, ViewId> LiGTPositionEstimator::GetBestBaseViews(
    const TrackId& track_id) {
  return SelectBestBaseViews(reconstruction_, orientations_, track_id);
}

void LiGTPositionEstimator::CalculateBCDForTrack(
    const theia::ViewId& vid_1,
    const theia::ViewId& vid_2,
    const theia::ViewId& vid_3,
    const TrackId& track_id,
    std::tuple<Matrix3d, Matrix3d, Matrix3d>& BCD) {
  const Vector3d feature1 =
      GetNormalizedFeatureRay(*reconstruction_.View(vid_1), track_id);
  const Vector3d feature2 =
      GetNormalizedFeatureRay(*reconstruction_.View(vid_2), track_id);
  const Vector3d feature3 =
      GetNormalizedFeatureRay(*reconstruction_.View(vid_3), track_id);

  const Matrix3d R1 = orientations_.at(vid_1);
  const Matrix3d R2 = orientations_.at(vid_2);
  const Matrix3d R3 = orientations_.at(vid_3);

  // Paper notation: j = vid_1, l = vid_2, i = vid_3 (the central view).
  Matrix3d B, C;
  ComputeLiGTBC(feature3, feature1, feature2, R3, R1, R2, &B, &C);
  std::get<0>(BCD) = B;
  std::get<1>(BCD) = C;
  std::get<2>(BCD) = -(std::get<0>(BCD) + std::get<1>(BCD));
}

void LiGTPositionEstimator::FindTripletsForTracks() {
  const auto track_ids = reconstruction_.TrackIds();
  using TrackConstraints =
      std::vector<std::pair<ViewIdTriplet, std::tuple<Matrix3d, Matrix3d, Matrix3d>>>;
  // Computed in parallel per track, inserted serially in track order, so the
  // result does not depend on the thread count.
  std::vector<TrackConstraints> per_track(track_ids.size());
  ParallelFor(options_.num_threads, track_ids.size(), [&](const size_t t) {
    const TrackId t_id = track_ids[t];
    // Views can lack an orientation, e.g. if they are not part of the view
    // graph any more.
    std::vector<ViewId> view_ids_for_track;
    for (const ViewId view_id : reconstruction_.Track(t_id)->ViewIds()) {
      if (ContainsKey(orientations_, view_id)) {
        view_ids_for_track.push_back(view_id);
      }
    }
    if (view_ids_for_track.size() < 3) {
      return;
    }
    // Implements equation 29 of the paper: the base views of the point.
    const std::pair<ViewId, ViewId> base_views = GetBestBaseViews(t_id);
    // One constraint per remaining observation.
    for (const ViewId cur_id : view_ids_for_track) {
      if (cur_id == base_views.first || cur_id == base_views.second) {
        continue;
      }
      std::tuple<Matrix3d, Matrix3d, Matrix3d> BCD;
      CalculateBCDForTrack(base_views.first, cur_id, base_views.second, t_id,
                           BCD);
      per_track[t].emplace_back(
          std::make_tuple(base_views.first, cur_id, base_views.second), BCD);
    }
  });
  for (size_t t = 0; t < track_ids.size(); ++t) {
    for (const auto& constraint : per_track[t]) {
      triplets_for_tracks_[track_ids[t]].push_back(constraint.first);
      BCDs_[track_ids[t]].push_back(constraint.second);
    }
  }
}

void LiGTPositionEstimator::RemoveWeaklyConstrainedViews() {
  if (options_.min_tracks_per_view <= 0) {
    return;
  }
  const auto track_ids = reconstruction_.TrackIds();
  std::vector<ViewId> oriented_views;
  int num_removed = 0;
  bool changed = true;
  while (changed) {
    changed = false;
    std::unordered_map<ViewId, int> num_tracks;
    for (const TrackId t_id : track_ids) {
      oriented_views.clear();
      for (const ViewId view_id : reconstruction_.Track(t_id)->ViewIds()) {
        if (ContainsKey(orientations_, view_id)) {
          oriented_views.push_back(view_id);
        }
      }
      if (oriented_views.size() < 3) {
        continue;
      }
      for (const ViewId view_id : oriented_views) {
        ++num_tracks[view_id];
      }
    }
    for (auto it = orientations_.begin(); it != orientations_.end();) {
      if (!ContainsKey(rig_constraints_, it->first) &&
          FindWithDefault(num_tracks, it->first, 0) <
              options_.min_tracks_per_view) {
        it = orientations_.erase(it);
        ++num_removed;
        changed = true;
      } else {
        ++it;
      }
    }
  }
  if (num_removed > 0) {
    VLOG(2) << "LiGT: removed " << num_removed
            << " views with fewer than " << options_.min_tracks_per_view
            << " tracks of length >= 3.";
  }
}

void LiGTPositionEstimator::BuildLinearSystemIndex() {
  ConnectedComponents<ViewId> components;
  for (const auto& entry : triplets_for_tracks_) {
    for (const ViewIdTriplet& triplet : entry.second) {
      components.AddEdge(std::get<0>(triplet), std::get<1>(triplet));
      components.AddEdge(std::get<0>(triplet), std::get<2>(triplet));
    }
  }
  std::unordered_map<ViewId, std::unordered_set<ViewId>> sets;
  components.Extract(&sets);

  // Views in different components have no common scale, and each extra
  // component adds a null vector to the system. Ties go to the lowest root id.
  const std::unordered_set<ViewId>* largest = nullptr;
  ViewId largest_root = kInvalidViewId;
  for (const auto& set : sets) {
    if (largest == nullptr || set.second.size() > largest->size() ||
        (set.second.size() == largest->size() && set.first < largest_root)) {
      largest = &set.second;
      largest_root = set.first;
    }
  }
  if (largest == nullptr) {
    return;
  }

  // All views of a constraint are connected, so checking one of them is enough.
  for (auto it = triplets_for_tracks_.begin();
       it != triplets_for_tracks_.end();) {
    auto& triplets = it->second;
    auto& BCDs = BCDs_.at(it->first);
    size_t num_kept = 0;
    for (size_t i = 0; i < triplets.size(); ++i) {
      if (largest->count(std::get<0>(triplets[i])) > 0) {
        triplets[num_kept] = triplets[i];
        BCDs[num_kept] = BCDs[i];
        ++num_kept;
      }
    }
    triplets.resize(num_kept);
    BCDs.resize(num_kept);
    if (num_kept == 0) {
      BCDs_.erase(it->first);
      it = triplets_for_tracks_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& entry : triplets_for_tracks_) {
    for (const ViewIdTriplet& triplet : entry.second) {
      num_triplets_for_view_[std::get<0>(triplet)] += 1;
      num_triplets_for_view_[std::get<1>(triplet)] += 1;
      num_triplets_for_view_[std::get<2>(triplet)] += 1;
    }
  }

  std::vector<ViewId> view_ids;
  view_ids.reserve(num_triplets_for_view_.size());
  for (const auto& entry : num_triplets_for_view_) {
    view_ids.push_back(entry.first);
  }
  std::sort(view_ids.begin(), view_ids.end());

  // Rig-constrained views are tied to a root view of the system. Only views
  // whose anchor chain ends at a view of the system are resolved.
  resolved_rig_constraints_.clear();
  for (const auto& rig : rig_constraints_) {
    ResolvedRigConstraint resolved;
    if (ResolveRigChain(rig_constraints_, rig.first, num_triplets_for_view_,
                        &resolved.root, &resolved.offset)) {
      resolved_rig_constraints_[rig.first] = resolved;
    }
  }

  // The position at index -1 is held constant (and not part of the system).
  // The best constrained view is the safest one to fix. Rig-constrained views
  // are not candidates: they are not unknowns.
  ViewId constant_view = kInvalidViewId;
  for (const ViewId view_id : view_ids) {
    if (ContainsKey(resolved_rig_constraints_, view_id)) {
      continue;
    }
    if (constant_view == kInvalidViewId ||
        num_triplets_for_view_[view_id] >
            num_triplets_for_view_[constant_view]) {
      constant_view = view_id;
    }
  }
  linear_system_index_[constant_view] = kConstantPositionIndex;
  int index = 0;
  for (const ViewId view_id : view_ids) {
    if (view_id != constant_view &&
        !ContainsKey(resolved_rig_constraints_, view_id)) {
      linear_system_index_[view_id] = index++;
    }
  }
}

void LiGTPositionEstimator::ResolveViewBlock(const ViewId view_id, int* block,
                                             Vector3d* offset) const {
  const ResolvedRigConstraint* resolved =
      FindOrNull(resolved_rig_constraints_, view_id);
  const ViewId lookup = resolved == nullptr ? view_id : resolved->root;
  *block = 3 * FindOrDie(linear_system_index_, lookup);
  *offset = resolved == nullptr ? Vector3d::Zero() : resolved->offset;
}

void LiGTPositionEstimator::ComputeScaleConstraint() {
  scale_constraint_ =
      Eigen::VectorXd::Zero(3 * (linear_system_index_.size() - 1));
  for (const auto& view_pair : *view_pairs_) {
    const int* index1 = FindOrNull(linear_system_index_, view_pair.first.first);
    const int* index2 =
        FindOrNull(linear_system_index_, view_pair.first.second);
    if (index1 == nullptr || index2 == nullptr) {
      continue;
    }
    // position_2 is the direction from view 1 to view 2 in the frame of view 1.
    const Vector3d direction =
        orientations_.at(view_pair.first.first).transpose() *
        view_pair.second.position_2.normalized();
    if (*index1 != kConstantPositionIndex) {
      scale_constraint_.segment<3>(3 * *index1) -= direction;
    }
    if (*index2 != kConstantPositionIndex) {
      scale_constraint_.segment<3>(3 * *index2) += direction;
    }
  }
}


// Sets up the linear system with the constraints that each triplet adds.
void LiGTPositionEstimator::CreateLinearSystem(
    Eigen::SparseMatrix<double>* constraint_matrix) {
  const int num_views = linear_system_index_.size();

    std::unordered_map<std::pair<int, int>, double> sparse_matrix_entries;
    sparse_matrix_entries.reserve(27 * num_triplets_for_view_.size());

    rhs_ = Eigen::VectorXd::Zero(3 * (num_views - 1));
    for (const auto& triplet_vector : triplets_for_tracks_) {
        const TrackId t_id = triplet_vector.first;
        const std::vector<ViewIdTriplet> triplet_v = triplet_vector.second;
        for (size_t i = 0; i < triplet_v.size(); ++i) {
            const ViewId &view_id1 = std::get<0>(triplet_v[i]);
            const ViewId &view_id2 = std::get<1>(triplet_v[i]);
            const ViewId &view_id3 = std::get<2>(triplet_v[i]);
            const auto weights_it = constraint_weights_.find(t_id);
            const double weight = weights_it == constraint_weights_.end()
                                      ? 1.0
                                      : weights_it->second[i];
            AddTripletConstraintToSparseMatrix(view_id1, view_id2, view_id3,
                                               BCDs_[t_id][i], weight,
                                               &sparse_matrix_entries, &rhs_);

        }
    }

    // Set the sparse matrix from the container of the accumulated entries.
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(sparse_matrix_entries.size());
    for (const auto &sparse_matrix_entry : sparse_matrix_entries) {
      // Skip this entry if the indices are invalid. This only occurs when we
      // encounter a constraint with the constant camera (which has a view index
      // of -1).
      if (sparse_matrix_entry.first.first < 0 ||
          sparse_matrix_entry.first.second < 0) {
        continue;
      }
      triplet_list.emplace_back(sparse_matrix_entry.first.first,
                                sparse_matrix_entry.first.second,
                                sparse_matrix_entry.second);
    }

    // We construct the constraint matrix A^t * A directly, which is an
    // N - 1 x N - 1 matrix where N is the number of cameras (and 3 entries per
    // camera, corresponding to the camera position entries).

    constraint_matrix->resize((num_views - 1) * 3, (num_views - 1) * 3);
    constraint_matrix->setFromTriplets(triplet_list.begin(), triplet_list.end());
    triplet_list.clear();
}

// Adds a triplet constraint to the linear system. The weight of the constraint
// (w), the global orientations, baseline (ratios), and view triplet information
// are needed to form the constraint.
void LiGTPositionEstimator::AddTripletConstraintToSparseMatrix(
    const ViewId view_id0,
    const ViewId view_id1,
    const ViewId view_id2,
    const std::tuple<Matrix3d, Matrix3d, Matrix3d>& BCD,
    const double weight,
    std::unordered_map<std::pair<int, int>, double>* sparse_matrix_entries,
    Eigen::VectorXd* rhs) {
//  // Weight each term by the inverse of the # of triplet that the nodes
//  // participate in.
//  const double w =
//      1.0 / std::sqrt(std::min({num_triplets_for_view_[view_id0],
//                                num_triplets_for_view_[view_id1],
//                                num_triplets_for_view_[view_id2]}));

  // Get the index of each camera in the sparse matrix, and its rig offset.
  // Rig-constrained views use the block of their root view.
  int block0 = 0, block1 = 0, block2 = 0;
  Vector3d offset0, offset1, offset2;
  ResolveViewBlock(view_id0, &block0, &offset0);
  ResolveViewBlock(view_id1, &block1, &offset1);
  ResolveViewBlock(view_id2, &block2, &offset2);
  const std::vector<int> view_indices = {block0, block1, block2};

  // important to use index 1 0 2 here.
  // 0 ist the central camera and equation 17 is using it that way
  std::vector<Matrix3d> constraints = {std::get<1>(BCD),std::get<0>(BCD),std::get<2>(BCD)};

  // Row normalization: weight by 1 / ||[B C D]||_F^2 (the weight multiplies the
  // squared residual).
  double w = weight;
  if (options_.normalize_constraints) {
    const double norm_sq = std::get<0>(BCD).squaredNorm() +
                           std::get<1>(BCD).squaredNorm() +
                           std::get<2>(BCD).squaredNorm();
    if (norm_sq > 1e-24) {
      w /= norm_sq;
    }
  }
  AddTripletConstraintToSymmetricMatrix(
      constraints, view_indices, w, sparse_matrix_entries);

  // Substitution of rig-constrained views, c_i = c_root + offset_i: the residual
  // of the triplet is A x + b with b = sum_i C_i offset_i. The normal equations
  // M x = rhs get -w A^T b, i.e. -w C_i^T b in the block of view i's root.
  const std::vector<Vector3d> offsets = {offset0, offset1, offset2};
  Vector3d b = Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    b += constraints[i] * offsets[i];
  }
  if (b.squaredNorm() > 0.0) {
    for (int i = 0; i < 3; ++i) {
      if (view_indices[i] >= 0) {
        rhs->segment<3>(view_indices[i]) -= w * constraints[i].transpose() * b;
      }
    }
  }
}

Eigen::VectorXd LiGTPositionEstimator::SolveLinearSystem(
    const Eigen::SparseMatrix<double>& constraint_matrix) const {
  Eigen::VectorXd solution;
  if (!resolved_rig_constraints_.empty()) {
    // The rig offsets make the system inhomogeneous: the metric solution is
    // the least squares solution of M x = rhs_, no normalization needed.
    if (SolveWithRigConstraints(constraint_matrix, &solution)) {
      return solution;
    }
    LOG(WARNING) << "LiGT: the rig-constrained solve failed.";
    return Eigen::VectorXd();
  }

  if (options_.use_scale_constraint) {
    if (SolveWithScaleConstraint(constraint_matrix, &solution)) {
      return solution;
    }
    LOG(WARNING) << "LiGT: the scale-constrained solve failed, falling back to "
                    "the smallest eigenvector.";
  }

  if (static_cast<int>(orientations_.size()) > options_.max_num_views_svd) {
    VLOG(2) << "Solving for positions from the sparse eigenvalue problem...";
    std::shared_ptr<SparseCholeskyLLt> linear_solver =
        std::make_shared<SparseCholeskyLLt>();
    SparseSymShiftSolveLLT<double> op(linear_solver, constraint_matrix);
    Spectra::SymEigsShiftSolver<SparseSymShiftSolveLLT<double>> eigs(
        op, 1, 6, 0.0);
    eigs.init();
    eigs.compute(Spectra::SortRule::LargestMagn, 1000, 1e-4);
    solution = eigs.eigenvectors().col(0);
  } else {
    VLOG(2) << "Solving for positions from the eigenvalue problem using "
               "BDCSVD...";
    Eigen::MatrixXd constraint_matrix_dense(constraint_matrix);
    Eigen::BDCSVD<Eigen::MatrixXd> svd(
        constraint_matrix_dense, Eigen::ComputeThinU | Eigen::ComputeThinV);
    solution = svd.matrixV().col(svd.matrixV().cols() - 1);
  }
  return solution;
}

bool LiGTPositionEstimator::SolveWithRigConstraints(
    const Eigen::SparseMatrix<double>& constraint_matrix,
    Eigen::VectorXd* solution) const {
  const int size = constraint_matrix.rows();
  if (size == 0 || rhs_.size() != size) {
    return false;
  }

  // Same regularization as in SolveWithScaleConstraint.
  Eigen::SparseMatrix<double> identity(size, size);
  identity.setIdentity();
  const double trace = constraint_matrix.diagonal().sum();
  const Eigen::SparseMatrix<double> regularized =
      constraint_matrix + (1e-10 * trace / size) * identity;

  SparseCholeskyLLt linear_solver;
  linear_solver.Compute(regularized);
  if (linear_solver.Info() != Eigen::Success) {
    return false;
  }
  *solution = linear_solver.Solve(rhs_);
  return linear_solver.Info() == Eigen::Success && solution->allFinite();
}

bool LiGTPositionEstimator::SolveWithScaleConstraint(
    const Eigen::SparseMatrix<double>& constraint_matrix,
    Eigen::VectorXd* solution) const {
  if (scale_constraint_.squaredNorm() < 1e-24) {
    return false;
  }

  // M is singular up to noise for consistent constraints, so a tiny diagonal
  // term keeps the factorization positive definite. It is far below the
  // entries of M and does not change the solution in any measurable way.
  const int size = constraint_matrix.rows();
  Eigen::SparseMatrix<double> identity(size, size);
  identity.setIdentity();
  const double trace = constraint_matrix.diagonal().sum();
  const Eigen::SparseMatrix<double> regularized =
      constraint_matrix + (1e-10 * trace / size) * identity;

  SparseCholeskyLLt linear_solver;
  linear_solver.Compute(regularized);
  if (linear_solver.Info() != Eigen::Success) {
    return false;
  }
  *solution = linear_solver.Solve(scale_constraint_);
  if (linear_solver.Info() != Eigen::Success || !solution->allFinite() ||
      solution->squaredNorm() < 1e-24) {
    return false;
  }
  solution->normalize();
  return true;
}

double LiGTPositionEstimator::UpdateConstraintWeights(
    const Eigen::VectorXd& solution) {
  auto position_of = [&](const ViewId view_id) -> Vector3d {
    int block = 0;
    Vector3d offset;
    ResolveViewBlock(view_id, &block, &offset);
    if (block < 0) {
      return offset;  // constant view (offset is zero unless rig-constrained)
    }
    return Vector3d(solution.segment<3>(block) + offset);
  };

  // Normalized residual of every constraint: |B c0 + C c1 + D c2| divided by
  // the size of the constraint matrices, so that constraints from near- and
  // far-baseline tracks are comparable.
  std::vector<double> all_residuals;
  std::unordered_map<TrackId, std::vector<double>> residuals;
  for (const auto& entry : triplets_for_tracks_) {
    const TrackId t_id = entry.first;
    const auto& triplets = entry.second;
    auto& res = residuals[t_id];
    res.assign(triplets.size(), -1.0);
    for (size_t i = 0; i < triplets.size(); ++i) {
      const auto& BCD = BCDs_.at(t_id)[i];
      const double norm = std::sqrt(std::get<0>(BCD).squaredNorm() +
                                    std::get<1>(BCD).squaredNorm() +
                                    std::get<2>(BCD).squaredNorm());
      if (norm < 1e-12) {
        continue;  // degenerate (all-zero) constraint: leave weight at 1
      }
      // Same ordering as AddTripletConstraintToSparseMatrix: {BCD1, BCD0, BCD2}
      // applied to the positions of views {0, 1, 2} of the triplet.
      const Vector3d r = std::get<1>(BCD) * position_of(std::get<0>(triplets[i])) +
                         std::get<0>(BCD) * position_of(std::get<1>(triplets[i])) +
                         std::get<2>(BCD) * position_of(std::get<2>(triplets[i]));
      res[i] = r.norm() / norm;
      all_residuals.push_back(res[i]);
    }
  }
  if (all_residuals.empty()) {
    return 0.0;
  }

  const size_t mid = all_residuals.size() / 2;
  std::nth_element(all_residuals.begin(), all_residuals.begin() + mid,
                   all_residuals.end());
  const double scale = options_.irls_scale_multiplier * 1.4826 *
                       all_residuals[mid];
  if (!(scale > 0.0)) {
    return 0.0;
  }

  constraint_weights_.clear();
  for (const auto& entry : residuals) {
    auto& weights = constraint_weights_[entry.first];
    weights.resize(entry.second.size(), 1.0);
    for (size_t i = 0; i < entry.second.size(); ++i) {
      if (entry.second[i] >= 0.0) {
        const double u = entry.second[i] / scale;
        weights[i] = u <= 1.0 ? 1.0 : 1.0 / u;  // Huber IRLS weight
      }
    }
  }
  return scale;
}

void LiGTPositionEstimator::FlipSignOfPositionsIfNecessary(
    std::unordered_map<ViewId, Vector3d>* positions) {
  if (PositionsHaveWrongSign(*view_pairs_, orientations_, *positions)) {
    for (auto& position : *positions) {
      position.second *= -1.0;
    }
  }
}

std::unordered_map<ViewId, Vector3d>
LiGTPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientation) {
  std::unordered_map<ViewId, Vector3d> positions;
  EstimatePositions(view_pairs, orientation, &positions);
  return positions;
}

}  // namespace theia
