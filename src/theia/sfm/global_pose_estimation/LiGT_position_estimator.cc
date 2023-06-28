
// Copyright (C) 2023 Steffen Urban
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

// Author: Steffen Urban (steffen.urban@googlemail.com), March 2021

#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "spectra/include/SymEigsShiftSolver.h"
#include "spectra/include/MatOp/SparseSymShiftSolve.h"

#include "theia/math/graph/triplet_extractor.h"
#include "theia/math/matrix/spectra_linear_operator.h"
#include "theia/sfm/find_common_tracks_in_views.h"
#include "theia/sfm/global_pose_estimation/compute_triplet_baseline_ratios.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_triplet.h"
#include "theia/util/map_util.h"
#include "theia/util/threadpool.h"

namespace theia {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

Eigen::Matrix3d GetSkew(const Eigen::Vector3d& f) {
  Eigen::Matrix3d skew_mat;
  skew_mat << 0.0, -f(2), f(1), f(2), 0.0, -f(0), -f(1), f(0), 0.0;
  return skew_mat;
}

Eigen::Matrix3d GetRij(const Eigen::Matrix3d& i, const Eigen::Matrix3d& j) {
  return j * i.transpose();
}


Eigen::Vector3d GetTheta(const Eigen::Vector3d& feat_i,
                         const Eigen::Vector3d& feat_j,
                         const Eigen::Matrix3d& Rij) {
  return GetSkew(feat_j) * Rij * feat_i;
}

double GetThetaSq(const Eigen::Vector3d& feat_i,
                  const Eigen::Vector3d& feat_j,
                  const Eigen::Matrix3d& Rij) {
  return GetTheta(feat_i, feat_j, Rij).squaredNorm();
}



Eigen::Vector3d Get_aij(const Eigen::Matrix3d& Rij,
                        const Eigen::Vector3d Xi,
                        const Eigen::Vector3d Xj) {
  return (GetSkew(Rij * Xi) * Xj).transpose() * GetSkew(Xj);
}

inline Matrix3d AngleAxisToRotationMatrix(const Vector3d angle_axis) {
  const double angle = angle_axis.norm();
  const Eigen::AngleAxisd rotation_aa(angle, angle_axis / angle);
  return rotation_aa.toRotationMatrix();
}

// Returns true if the vector R1 * (c2 - c1) is in the same direction as t_12.
bool VectorsAreSameDirection(const Vector3d& position1,
                             const Vector3d& position2,
                             const Vector3d& rotation1,
                             const Vector3d& relative_position12) {
  const Vector3d global_relative_position =
      (position2 - position1).normalized();
  Vector3d rotated_relative_position;
  ceres::AngleAxisRotatePoint(rotation1.data(),
                              global_relative_position.data(),
                              rotated_relative_position.data());
  return rotated_relative_position.dot(relative_position12) > 0;
}

// Returns the features as a unit-norm pixel ray after camera intrinsics
// (i.e. focal length an principal point) have been removed.
Feature GetNormalizedFeature(const View& view, const TrackId track_id) {
  Feature feature = *view.GetFeature(track_id);
  const Camera& camera = view.Camera();
  Eigen::Vector3d ray = camera.PixelToNormalizedCoordinates(feature.point_);
  Feature normalized_Feature(ray.hnormalized());
  // todo normalized covariance?
  return normalized_Feature;
}

std::pair<ViewId, ViewId> GetBestBaseViews(
    const Reconstruction& reconstruction,
    const TrackId& track_id) {

    const Track* track = reconstruction.Track(track_id);
    std::vector<ViewId> view_ids(track->ViewIds().begin(), track->ViewIds().end());
    double theta_max = 0.0;
    std::pair<ViewId, ViewId> base_views;
    for (size_t i = 0; i < view_ids.size(); ++i) {
      ViewId id1 = view_ids[i];
      const View* view1 = reconstruction.View(id1);
      const Vector3d feature1 =
          GetNormalizedFeature(*view1, track_id).point_.homogeneous();
      const Eigen::Matrix3d R1 = view1->Camera().GetOrientationAsRotationMatrix();

      for (size_t j = i+1; j < view_ids.size(); ++j) {
        ViewId id2 = view_ids[j];
        const View* view2 = reconstruction.View(id2);
        const Vector3d feature2 =
            GetNormalizedFeature(*view2, track_id).point_.homogeneous();

        const Eigen::Matrix3d R2 = view2->Camera().GetOrientationAsRotationMatrix();
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

  const size_t num_views = reconstruction_.ViewIds().size();
  const size_t num_tracks = reconstruction_.TrackIds().size();

  int index = kConstantPositionIndex;
  linear_system_index_.reserve(num_views);
  for (const auto& vid : reconstruction_.ViewIds()) {
    linear_system_index_[vid] = index;
    index += 3;
  }

  VLOG(2) << "Building the constraint matrix...";
  // Create the linear system based on triplet constraints.
  Eigen::MatrixXd A_lr = Eigen::MatrixXd::Zero(num_tracks, 3 * num_views);
  Eigen::MatrixXd LtL = CreateLinearSystem(&A_lr);

  // Solve for positions by examining the smallest eigenvalues. Since we have
  // set one position constant at the origin, we only need to solve for the
  // eigenvector corresponding to the smallest eigenvalue. This can be done
  // efficiently with inverse power iterations.
  VLOG(2) << "Solving for positions from the sparse eigenvalue problem...";

  Spectra::DenseSymShiftSolve<double> op(LtL);
  Spectra::SymEigsShiftSolver<Spectra::DenseSymShiftSolve<double>>
      eigs(op, 1, 8, 0.0);
  eigs.init();
  eigs.compute(Spectra::SortRule::LargestMagn);

  if (eigs.info() != Spectra::CompInfo::Successful) {
    LOG(ERROR) << "Solve for SVD Failed!";
    return false;
  }
  const Eigen::VectorXd eigen_values = eigs.eigenvalues();
  LOG(INFO) << "Eigenvalues: " << eigen_values.transpose();

  Eigen::VectorXd eigen_vectors = Eigen::VectorXd::Zero(3 * num_views);
  eigen_vectors.bottomRows(3 * (num_views - 1)) = eigs.eigenvectors();

  // Identify the sign 
  const Eigen::VectorXd judge_value = A_lr * (eigen_vectors);
  const int positive_count = (judge_value.array() > 0.0).cast<int>().sum();
  const int negative_count = judge_value.rows() - positive_count;

  if (positive_count < negative_count) {
    eigen_vectors *= -1;
  }

  // Set the estimated positions.
  for (const auto& view_id_idx : linear_system_index_) {
    const int index = view_id_idx.second;
    const ViewId vid = view_id_idx.first;
    (*positions)[vid] = eigen_vectors.segment<3>(index + 3);
  }

  return true;
}

// Sets up the linear system with the constraints that each triplet adds.
Eigen::MatrixXd LiGTPositionEstimator::CreateLinearSystem(
    Eigen::MatrixXd* A_lr) {
  const int num_views = linear_system_index_.size();

  Eigen::MatrixXd LtL =
      Eigen::MatrixXd::Zero(3 * (num_views - 1), 3 * (num_views - 1));

  //#pragma omp parallel for shared(A_lr, LtL)
  auto track_ids = reconstruction_.TrackIds();
  uint32_t total_nr_triplets = 0;
  for (size_t t = 0; t < track_ids.size(); ++t) {
    const auto t_id = track_ids[t];
    const auto view_ids_for_track = reconstruction_.Track(t_id)->ViewIds();

    if (view_ids_for_track.size() < 3) {
      continue;
    }

    // implements equation 29 from paper. Get base views for point
    std::pair<ViewId, ViewId> base_views = GetBestBaseViews(reconstruction_, t_id);
    std::cout<<"base_views"<<base_views.first<<" "<<base_views.second<<"\n";

    const View* view_l = reconstruction_.View(base_views.first);
    const View* view_r = reconstruction_.View(base_views.second);
    
    const int left_view_idx = linear_system_index_[base_views.first];
    const int right_view_idx = linear_system_index_[base_views.second];

    const Eigen::Matrix3d R_l = view_l->Camera().GetOrientationAsRotationMatrix();
    const Eigen::Matrix3d R_r = view_r->Camera().GetOrientationAsRotationMatrix();

    const Eigen::Vector3d Xl =
        GetNormalizedFeature(*view_l, t_id).point_.homogeneous();
    const Eigen::Vector3d Xr =
        GetNormalizedFeature(*view_r, t_id).point_.homogeneous();
        
    const Eigen::Matrix3d R_lr = R_r * R_l.transpose(); // GetRij(R_l, R_r);
    const Eigen::Vector3d theta_lr = GetTheta(Xl, Xr, R_lr);
    const double theta_lr_norm = theta_lr.norm();

    const Eigen::Matrix<double, 1, 3> a_lr = -theta_lr.transpose() * GetSkew(Xr);

    (*A_lr).row(t).block<1, 3>(0, left_view_idx + 3) = a_lr * R_r;
    (*A_lr).row(t).block<1, 3>(0, right_view_idx + 3) = -a_lr * R_r;

    Eigen::MatrixXd local_coefficient_mat = Eigen::MatrixXd::Zero(3, 3 * num_views);
    for (size_t v = 0; v < view_ids_for_track.size(); ++v) {
      ViewId cur_id = *std::next(view_ids_for_track.begin(), v);

      // check if the current id is one of the base views
      if (cur_id == base_views.first || cur_id == base_views.second) {
          continue;
      }
      const View* view_k = reconstruction_.View(cur_id);

      const Vector3d Xk =
        GetNormalizedFeature(*view_k, t_id).point_.homogeneous();

      const Eigen::Matrix3d R_k = view_k->Camera().GetOrientationAsRotationMatrix();

      const Matrix3d R_lk = R_k * R_l.transpose(); //; GetRij(R_l, R_k);

      const Matrix3d skew_feat_k = GetSkew(Xk);
      // equation 18
      const Eigen::Matrix3d B = skew_feat_k * R_lk * Xl * a_lr * R_r;      
      const Eigen::Matrix3d C = theta_lr_norm * theta_lr_norm * skew_feat_k * R_k;
      const Eigen::Matrix3d D = -(B + C);

      const int cur_view_idx = linear_system_index_[cur_id];

      local_coefficient_mat.setZero();
      local_coefficient_mat.block<3, 3>(0, left_view_idx + 3) += D;
      local_coefficient_mat.block<3, 3>(0, cur_view_idx + 3) += C;
      local_coefficient_mat.block<3, 3>(0, right_view_idx + 3) += B;


      const Eigen::MatrixXd Ltl_l = D.transpose() * local_coefficient_mat;
      const Eigen::MatrixXd Ltl_k = C.transpose() * local_coefficient_mat;
      const Eigen::MatrixXd Ltl_r = B.transpose() * local_coefficient_mat;
      
      // #pragma omp critical
      // {
        if (left_view_idx > 0) {
          LtL.middleRows<3>(left_view_idx) += Ltl_l.rightCols(Ltl_l.cols() - 3);
        }

        if (cur_view_idx > 0) {
          LtL.middleRows<3>(cur_view_idx) += Ltl_k.rightCols(Ltl_k.cols() - 3);
        }

        if (right_view_idx > 0) {
          LtL.middleRows<3>(right_view_idx) += Ltl_r.rightCols(Ltl_r.cols() - 3);
        }
      //}
    }
  }
  // std::cout<<"LtL"<<LtL<<"\n";
  // std::cout<<"A_lr"<<(*A_lr)<<"\n";

  return LtL;
}

std::unordered_map<ViewId, Eigen::Vector3d>
LiGTPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientation) {
  std::unordered_map<ViewId, Eigen::Vector3d> positions;
  EstimatePositions(view_pairs, orientation, &positions);
  return positions;
}

}  // namespace theia