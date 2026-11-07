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
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_triplet.h"
#include "theia/util/map_util.h"
#include "theia/util/threadpool.h"

namespace theia {

using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

namespace {

Matrix3d GetSkew(const Vector3d& f) {
  Matrix3d skew_mat;
  skew_mat << 0.0, -f(2), f(1), f(2), 0.0, -f(0), -f(1), f(0), 0.0;
  return skew_mat;
}

Matrix3d GetRij(const Matrix3d& i, const Matrix3d& j) {
  return j * i.transpose();
}

double GetThetaSq(const Vector3d& feat_i,
                  const Vector3d& feat_j,
                  const Matrix3d& Rij) {
  return (GetSkew(feat_j) * Rij * feat_i).squaredNorm();
}

Vector3d Get_aij(const Matrix3d& Rij,
                        const Vector3d Xi,
                        const Vector3d Xj) {
  return (GetSkew(Rij * Xi) * Xj).transpose() * GetSkew(Xj);
}


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

// Returns true if the vector R1 * (c2 - c1) is in the same direction as t_12.
bool VectorsAreSameDirection(const Vector3d& position1,
                             const Vector3d& position2,
                             const Matrix3d& rotation1,
                             const Vector3d& relative_position12) {
  const Vector3d global_relative_position =
      (position2 - position1).normalized();
  Vector3d rotated_relative_position = rotation1 * global_relative_position;

  return rotated_relative_position.dot(relative_position12) > 0;
}

// Returns the features as a unit-norm pixel ray after camera intrinsics
// (i.e. focal length an principal point) have been removed.
Feature GetNormalizedFeature(const View& view, const TrackId track_id) {
  Feature feature = *view.GetFeature(track_id);
  const Camera& camera = view.Camera();
  Vector3d ray = camera.PixelToNormalizedCoordinates(feature.point_);
  Feature normalized_Feature(ray.hnormalized());
  // todo normalized covariance?
  return normalized_Feature;
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
  BCDs_.clear();
  triplets_for_tracks_.clear();
  orientations_.clear();

  view_pairs_ = &view_pairs;
  // convert orientations to matrices
  for (const auto& ori : orientations) {
    orientations_[ori.first] = AngleAxisToRotationMatrix(ori.second);
  }

  constraint_weights_.clear();
  VLOG(2) << "Extracting triplets from tracks and calculating BCDs for tracks.";
  FindTripletsForTracks();
  VLOG(2) << "Finished extracting triplets from tracks and calculating BCDs.";

  BuildLinearSystemIndex();
  if (linear_system_index_.size() < 3) {
    LOG(WARNING) << "LiGT needs tracks that connect at least 3 views, found "
                 << linear_system_index_.size() << ".";
    return false;
  }
  ComputeScaleConstraint();

  VLOG(2) << "Building the constraint matrix...";
  // Create the linear system based on triplet constraints. One position is
  // fixed at the origin, so the (N-1)*3 system gives the positions up to scale.
  Eigen::SparseMatrix<double> constraint_matrix;
  CreateLinearSystem(&constraint_matrix);
  Eigen::VectorXd solution = SolveLinearSystem(constraint_matrix);

  // Robustification (IRLS): re-weight the track constraints with the residuals
  // of the current solution and solve again.
  for (int iter = 0; iter < options_.num_irls_iterations; ++iter) {
    const double scale = UpdateConstraintWeights(solution);
    if (!(scale > 0.0)) {
      break;  // perfectly consistent (or degenerate) constraints
    }
    CreateLinearSystem(&constraint_matrix);
    Eigen::VectorXd new_solution = SolveLinearSystem(constraint_matrix);
    // The eigenvector sign is arbitrary: align it with the previous solution.
    if (new_solution.dot(solution) < 0.0) {
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

  // Compute with power iterations.
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

  FlipSignOfPositionsIfNecessary(positions);

  return true;
}

std::pair<ViewId, ViewId> LiGTPositionEstimator::GetBestBaseViews(
    const TrackId& track_id) {

    const Track* track = reconstruction_.Track(track_id);
    std::vector<ViewId> view_ids;
    for (const ViewId view_id : track->ViewIds()) {
      if (ContainsKey(orientations_, view_id)) {
        view_ids.push_back(view_id);
      }
    }
    double theta_max = 0.0;
    std::pair<ViewId, ViewId> base_views;
    for (size_t i = 0; i < view_ids.size(); ++i) {     
      ViewId id1 = view_ids[i];
      const View* view1 = reconstruction_.View(id1);
      const Vector3d feature1 = GetNormalizedFeature(*view1, track_id).point_.homogeneous();; 
      const Matrix3d R1 = orientations_.at(id1);

      for (size_t j = i+1; j < view_ids.size(); ++j) {
        ViewId id2 = view_ids[j];
        const View* view2 = reconstruction_.View(id2);
        const Vector3d feature2 = GetNormalizedFeature(*view2, track_id).point_.homogeneous();; 

        const Matrix3d R2 = orientations_.at(id2);
        const Matrix3d R12 = GetRij(R1, R2);
        const double theta = GetThetaSq(feature1, feature2, R12);
        if (theta > theta_max) {
            base_views = std::make_pair(id1, id2);
            theta_max = theta;
        }
      }
    }
    return base_views;
}

void LiGTPositionEstimator::CalculateBCDForTrack(
    const theia::ViewId& vid_1,
    const theia::ViewId& vid_2,
    const theia::ViewId& vid_3,
    const TrackId& track_id,
    std::tuple<Matrix3d, Matrix3d, Matrix3d>& BCD) {
      
  const auto view1 = reconstruction_.View(vid_1);
  const auto view2 = reconstruction_.View(vid_2);
  const auto view3 = reconstruction_.View(vid_3);

  const Vector3d feature1 = GetNormalizedFeature(*view1, track_id).point_.homogeneous();
  const Vector3d feature2 = GetNormalizedFeature(*view2, track_id).point_.homogeneous();
  const Vector3d feature3 = GetNormalizedFeature(*view3, track_id).point_.homogeneous();

  const Matrix3d R1 = orientations_.at(vid_1);
  const Matrix3d R2 = orientations_.at(vid_2);
  const Matrix3d R3 = orientations_.at(vid_3);

  const Matrix3d R31 = GetRij(R3, R1);
  const Matrix3d R32 = GetRij(R3, R2);

  const Vector3d a32 = Get_aij(R32, feature3, feature2);

  const Matrix3d skew_feat1 = GetSkew(feature1);
  // equation 18
  std::get<0>(BCD) = skew_feat1 * R31 * feature3 * a32.transpose() * R2;

  const double theta = GetThetaSq(feature3, feature2, R32);
  std::get<1>(BCD) = theta * skew_feat1 * R1;

  std::get<2>(BCD) = -(std::get<0>(BCD) + std::get<1>(BCD));
}

void LiGTPositionEstimator::FindTripletsForTracks() {
  auto track_ids = reconstruction_.TrackIds();

  #pragma omp parallel for shared(triplets_for_tracks_, BCDs_)
  for (size_t t = 0; t < track_ids.size(); ++t) {
    // parallelize this loop
    const auto t_id = track_ids[t];

    // Views can lack an orientation, e.g. if they are not part of the view
    // graph any more.
    std::vector<ViewId> view_ids_for_track;
    for (const ViewId view_id : reconstruction_.Track(t_id)->ViewIds()) {
      if (ContainsKey(orientations_, view_id)) {
        view_ids_for_track.push_back(view_id);
      }
    }
    if (view_ids_for_track.size() < 3) {
      continue;
    }
    // implements equation 29 from paper. Get base views for point
    std::pair<ViewId, ViewId> base_views = GetBestBaseViews(t_id);
    // now iterate all other observations beside the base views
    for (const ViewId cur_id : view_ids_for_track) {
      // check if the current id is one of the base views
      if (cur_id == base_views.first || cur_id == base_views.second) {
          continue;
      }

      std::tuple<Matrix3d, Matrix3d, Matrix3d> BCD;
      CalculateBCDForTrack(base_views.first, cur_id, base_views.second, t_id, BCD);

      // const auto view1 = reconstruction_.View(base_views.first);
      // const auto view2 = reconstruction_.View(cur_id);
      // const auto view3 = reconstruction_.View(base_views.second);
//      Vector3d shouldbezero = std::get<1>(BCD)*view1->Camera().GetPosition() +
//              std::get<0>(BCD)*view2->Camera().GetPosition() +
//              std::get<2>(BCD)*view3->Camera().GetPosition();
//      std::cout<<"shouldbezero: "<<shouldbezero<<"\n";

      
      #pragma omp critical
      {
        const ViewIdTriplet triplet = std::make_tuple(base_views.first, cur_id, base_views.second);
        triplets_for_tracks_[t_id].push_back(triplet);
        BCDs_[t_id].push_back(BCD);
      }
    }
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

  // The position at index -1 is held constant (and not part of the system).
  // The best constrained view is the safest one to fix.
  ViewId constant_view = view_ids.front();
  for (const ViewId view_id : view_ids) {
    if (num_triplets_for_view_[view_id] >
        num_triplets_for_view_[constant_view]) {
      constant_view = view_id;
    }
  }
  linear_system_index_[constant_view] = kConstantPositionIndex;
  int index = 0;
  for (const ViewId view_id : view_ids) {
    if (view_id != constant_view) {
      linear_system_index_[view_id] = index++;
    }
  }
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
                                               &sparse_matrix_entries);

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
    std::unordered_map<std::pair<int, int>, double>* sparse_matrix_entries) {
//  // Weight each term by the inverse of the # of triplet that the nodes
//  // participate in.
//  const double w =
//      1.0 / std::sqrt(std::min({num_triplets_for_view_[view_id0],
//                                num_triplets_for_view_[view_id1],
//                                num_triplets_for_view_[view_id2]}));

  // Get the index of each camera in the sparse matrix.
  const std::vector<int> view_indices = {
      static_cast<int>(3 * FindOrDie(linear_system_index_, view_id0)),
      static_cast<int>(3 * FindOrDie(linear_system_index_, view_id1)),
      static_cast<int>(3 * FindOrDie(linear_system_index_, view_id2))};

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
}

Eigen::VectorXd LiGTPositionEstimator::SolveLinearSystem(
    const Eigen::SparseMatrix<double>& constraint_matrix) const {
  Eigen::VectorXd solution;
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
    const int index = FindOrDie(linear_system_index_, view_id);
    return index < 0 ? Vector3d(Vector3d::Zero())
                     : Vector3d(solution.segment<3>(3 * index));
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
  // If this value is below zero, then we should flip the sign.
  int correct_sign_votes = 0;
  for (const auto& view_pair : *view_pairs_) {
    // Only count the votes for edges where both positions were successfully
    // estimated.
    const Vector3d* position1 = FindOrNull(*positions, view_pair.first.first);
    const Vector3d* position2 = FindOrNull(*positions, view_pair.first.second);
    if (position1 == nullptr || position2 == nullptr) {
      continue;
    }

    // Check the relative translation of views 1 and 2 in the triplet.
    if (VectorsAreSameDirection(
            *position1,
            *position2,
            FindOrDieNoPrint(orientations_, view_pair.first.first),
            view_pair.second.position_2)) {
      correct_sign_votes += 1;
    } else {
      correct_sign_votes -= 1;
    }
  }

  // If the sign of the votes is below zero, we must flip the sign of all
  // position estimates.
  if (correct_sign_votes < 0) {
    const int num_correct_votes =
        (view_pairs_->size() + correct_sign_votes) / 2;
    VLOG(2) << "Sign of the positions was incorrect: " << num_correct_votes
            << " of " << view_pairs_->size()
            << " relative translations had the correct sign. "
               "Flipping the sign of the camera positions.";
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
