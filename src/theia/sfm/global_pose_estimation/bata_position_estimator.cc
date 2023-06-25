// Copyright (C) 2023, Steffen Urban
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

// The implementation is taken from here https://github.com/DiantaoTu/BATA-CXX/blob/main/BATA.cpp
// The original code Matlab code can be found here: https://bbzh.github.io/document/BATA.zip

#include "theia/sfm/global_pose_estimation/bata_position_estimator.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <limits>
#include <unordered_map>
#include <vector>
#include <iostream>

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

Eigen::SparseMatrix<double> SpliceMatrix(
  const Eigen::SparseMatrix<double>& A, 
  const Eigen::SparseMatrix<double>& B, 
  const Eigen::SparseMatrix<double>& C, 
  const Eigen::SparseMatrix<double>& D)
{
  assert(!A.IsRowMajor && !B.IsRowMajor && !C.IsRowMajor && !D.IsRowMajor);
  Eigen::SparseMatrix<double> At = A.transpose();
  Eigen::SparseMatrix<double> Bt = B.transpose();
  Eigen::SparseMatrix<double> Ct = C.transpose();
  Eigen::SparseMatrix<double> Dt = D.transpose();
  Eigen::SparseMatrix<double> AtCt(At.rows(), At.cols() + Ct.cols());
  Eigen::SparseMatrix<double> BtDt(Bt.rows(), Bt.cols() + Dt.cols());
  AtCt.leftCols(At.cols()) = At;
  AtCt.rightCols(Ct.cols()) = Ct;
  BtDt.leftCols(Bt.cols()) = Bt;
  BtDt.rightCols(Dt.cols()) = Dt;
  Eigen::SparseMatrix<double> spliced(A.rows() + C.rows(), A.cols() + B.cols());
  spliced.leftCols(AtCt.rows()) = AtCt.transpose();
  spliced.rightCols(BtDt.rows()) = BtDt.transpose();
  return spliced;
}

}  // namespace

RevisedLeastUnsquaredDeviationPositionEstimator::
  RevisedLeastUnsquaredDeviationPositionEstimator(
      const RevisedLeastUnsquaredDeviationPositionEstimator::Options& options)
  : options_(options) {
CHECK_GT(options_.max_num_iterations, 0);
CHECK_GT(options_.max_num_reweighted_iterations, 0);
}

bool RevisedLeastUnsquaredDeviationPositionEstimator::EstimatePositions(
  const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
  const std::unordered_map<ViewId, Vector3d>& orientations,
  std::unordered_map<ViewId, Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();

  InitializeIndexMapping(view_pairs, orientations);
  const int num_views = view_id_to_index_.size();
  const int num_view_pairs = view_id_pair_to_index_.size();

  // Set up the linear system.
  SetupConstraintMatrix(view_pairs, orientations);

  Eigen::VectorXd tij_sq_sum = tij_all_.array().square().colwise().sum();

  Aeq1_ = Aeq1_ * At0_full_;

  Eigen::SparseMatrix<double> Aeq(4, Aeq1_.cols());
  Aeq.reserve(2 * Aeq1_.cols());
  for(size_t i = 0; i < Aeq.cols(); i++) {
      Aeq.insert(0, i) = Aeq1_.col(i).sum();
      Aeq.insert(i % 3 + 1, i) = 1;
  }

  Eigen::SparseMatrix<double> Aeq_transpose = Aeq.transpose();
  Eigen::SparseMatrix<double> beq(4,1);
  beq.insert(0,0) = view_pairs.size();
  std::cout<<"beq"<<beq<<"\n";

  // Eigen::VectorXd Svec = Eigen::VectorXd::Random(num_view_pairs).array().abs();
  // Svec *= num_view_pairs / Svec.sum();
  // Eigen::MatrixXd S = Svec.replicate(1,3).reshaped<Eigen::RowMajor>();

  // Eigen::VectorXd W = Eigen::VectorXd::Ones(3 * num_view_pairs);
  // W = W.array().sqrt();
  Eigen::VectorXd t;

  for(int iter = 0; iter < options_.num_inner_iterations; ++iter)
  {
    Eigen::SparseMatrix<double> A(3 * num_view_pairs, 3 * num_view_pairs);
    A.setIdentity();
    //A = A * W.asDiagonal() * At0_full_;
    A = A * At0_full_;
    Eigen::SparseMatrix<double> At = A.adjoint();
    //Eigen::MatrixXd B = W.array() * S.array() * tij_all_.reshaped<Eigen::ColMajor>().array();
    Eigen::MatrixXd B = tij_all_.reshaped<Eigen::ColMajor>().array();
    Eigen::SparseMatrix<double> A_full(A.cols() + 4, A.cols() + 4);
    A_full = SpliceMatrix(2 * At * A, Aeq_transpose, Aeq, Eigen::SparseMatrix<double>(4,4));
    std::cout<<"A_full"<<A_full<<"\n";
    Eigen::SparseMatrix<double> b = SpliceMatrix(
      2 * At * Eigen::SparseMatrix<double>(B.sparseView()), 
      Eigen::SparseMatrix<double>(0,0), beq, Eigen::SparseMatrix<double>(0,0));
    

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A_full);
    if(solver.info() != Eigen::Success)
      std::cout << "decomposition failed" << std::endl;
    Eigen::VectorXd X;
    X = solver.solve(b);
    if(solver.info() != Eigen::Success)
      std::cout << "solving failed" << std::endl;

    t = X.head(3 * num_views);
    Eigen::MatrixXd Aij = (At0_full_ * t).reshaped(3, num_view_pairs);
    std::cout<<"Aij: "<<Aij<<std::endl;
    std::cout<<"tij_all_.array()): "<<tij_all_<<std::endl;
    // Svec = (Aij.array() * tij_all_.array()).colwise().sum();
    // Svec = Svec.array() / tij_sq_sum.array();  
    // S = Svec.replicate(1,3).reshaped<Eigen::RowMajor>();

    // Eigen::MatrixXd tmp = At0_full_ * t;
    // tmp = (tmp.array() - S.array() * tij_all_.reshaped<Eigen::ColMajor>().array()).reshaped(3, num_view_pairs);
    // tmp = tmp.array().square();
    // Eigen::MatrixXd tmp_col_sum = tmp.colwise().sum();
    // Eigen::MatrixXd Wvec = (tmp_col_sum.array() + options_.delta).pow(-0.5);
    // W = Wvec.replicate(3,1).reshaped<Eigen::ColMajor>();
    // W = W.array().sqrt();
  }
  Eigen::MatrixXd pose = t.reshaped(3, num_views);

  // for(size_t i = 0; i < num_views; i++) {
  //    pose.col(i) -= pose.col(0);
  // }
  // Set the estimated positions.
  for (const auto& view_id_index : view_id_to_index_) {
    const int index = view_id_index.second;
    const ViewId view_id = view_id_index.first;
    (*positions)[view_id] = pose.col((int)(index/3));
  }

  return true;
}

void RevisedLeastUnsquaredDeviationPositionEstimator::InitializeIndexMapping(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations) {
  std::unordered_set<ViewId> views;
  for (const auto& view_pair : view_pairs) {
    if (ContainsKey(orientations, view_pair.first.first) &&
        ContainsKey(orientations, view_pair.first.second)) {
      views.insert(view_pair.first.first);
      views.insert(view_pair.first.second);
    }
  }

  // Create a mapping from the view id to the index of the linear system.
  int index = 0; //kConstantViewIndex;
  view_id_to_index_.reserve(views.size());
  for (const ViewId view_id : views) {
    view_id_to_index_[view_id] = index;
    index += 3;
  }

  // Create a mapping from the view id pair to the index of the linear system.
  view_id_pair_to_index_.reserve(view_pairs.size());
  for (const auto& view_pair : view_pairs) {
    if (ContainsKey(view_id_to_index_, view_pair.first.first) &&
        ContainsKey(view_id_to_index_, view_pair.first.second)) {
      view_id_pair_to_index_[view_pair.first] = index;
      ++index;
    }
  }
}

void RevisedLeastUnsquaredDeviationPositionEstimator::SetupConstraintMatrix(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations) {

  const size_t num_view_pairs = view_pairs.size();
  At0_full_.resize(
      3 * num_view_pairs,
      3 * view_id_to_index_.size());
  Aeq1_.resize(num_view_pairs, 3 * num_view_pairs);
  Aeq1_.reserve(num_view_pairs * 3); 

  tij_all_.resize(3, view_id_pair_to_index_.size());
  // Add the camera to camera constraints.
  std::vector<Eigen::Triplet<double> > triplet_list;
  triplet_list.reserve(6 * view_pairs.size());
  int row = 0;
  for (const auto& view_pair : view_pairs) {
    const ViewIdPair view_id_pair = view_pair.first;
    if (!ContainsKey(view_id_to_index_, view_id_pair.first) ||
        !ContainsKey(view_id_to_index_, view_id_pair.second)) {
      continue;
    }

    const int view1_index = FindOrDie(view_id_to_index_, view_id_pair.first);
    const int view2_index = FindOrDie(view_id_to_index_, view_id_pair.second);
    const int scale_index =
        FindOrDieNoPrint(view_id_pair_to_index_, view_id_pair);

    // Rotate the relative translation so that it is aligned to the global
    // orientation frame.
    const Vector3d translation_direction =
        GetRotatedTranslation(FindOrDie(orientations, view_id_pair.first),
                              view_pair.second.position_2);

    triplet_list.emplace_back(row + 0, view1_index + 0, -1.0);
    triplet_list.emplace_back(row + 1, view1_index + 1, -1.0);
    triplet_list.emplace_back(row + 2, view1_index + 2, -1.0);

    triplet_list.emplace_back(row + 0, view2_index + 0, 1.0);
    triplet_list.emplace_back(row + 1, view2_index + 1, 1.0);
    triplet_list.emplace_back(row + 2, view2_index + 2, 1.0);

    int idx_obs = row / 3;
    Aeq1_.insert(idx_obs, row + 0) = translation_direction[0];
    Aeq1_.insert(idx_obs, row + 1) = translation_direction[1];
    Aeq1_.insert(idx_obs, row + 2) = translation_direction[2];

    tij_all_.col(idx_obs) = translation_direction;
    row += 3;
  }

  At0_full_.setFromTriplets(triplet_list.begin(), triplet_list.end());

  triplet_list.clear();

  VLOG(2) << view_pairs.size()
          << " camera to camera constraints were added "
             "to the position estimation problem.";
}

std::unordered_map<ViewId, Eigen::Vector3d>
RevisedLeastUnsquaredDeviationPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation) {
  std::unordered_map<ViewId, Eigen::Vector3d> positions;
  EstimatePositions(view_pairs, orientation, &positions);
  return positions;
}

}  // namespace theia
