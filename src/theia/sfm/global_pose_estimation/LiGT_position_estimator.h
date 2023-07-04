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

#ifndef THEIA_SFM_GLOBAL_POSE_ESTIMATION_LiGT_POSITION_ESTIMATOR_H_
#define THEIA_SFM_GLOBAL_POSE_ESTIMATION_LiGT_POSITION_ESTIMATOR_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <unordered_map>
#include <vector>

#include "theia/sfm/feature.h"
#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_triplet.h"
#include "theia/util/util.h"
#include "theia/sfm/reconstruction.h"

namespace theia {
class TwoViewInfo;
class View;

// Estimates the camera position of views given global orientations and
// normalized image correspondeces of the same scene point across image triplets.
// The constraints formed by each triplet are used to create a sparse
// linear system to solve for the positions.

// Implements the paper:
//@article{cai2021pose,
//  title={A Pose-only Solution to Visual Reconstruction and Navigation},
//  author={Cai, Qi and Zhang, Lilian and Wu, Yuanxin and Yu, Wenxian and Hu, Dewen},
//  journal={arXiv preprint arXiv:2103.01530},
//  year={2021}
//}
class LiGTPositionEstimator : public PositionEstimator {
 public:
  struct Options {
    int num_threads = 1;

    // Maximum number of inverse power iterations to perform while extracting
    // the eigenvector corresponding to the smallest eigenvalue.
    int max_power_iterations = 1000;
    // The threshold at which to the iterative eigensolver method is considered
    // to be converged.
    double eigensolver_threshold = 1e-8;

    // threshold for using sparse solver instead of SVD
    int max_num_views_svd = 500;
  };

  LiGTPositionEstimator(const Options& options,
                        const Reconstruction& reconstruction);

  // Estimate the positions
  bool EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions);

  // python
  std::unordered_map<ViewId, Eigen::Vector3d> EstimatePositionsWrapper(
      const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
      const std::unordered_map<ViewId, Eigen::Vector3d>& orientation);

 private:
   // Store the triplet.
  void AddTripletConstraint(const ViewIdTriplet& view_triplet);
  
  // Sets up the linear system with the constraints that each triplet adds.
  void CreateLinearSystem(Eigen::SparseMatrix<double>* constraint_matrix);

  // Implements equation 29 from the paper. Get base views for point
  std::pair<ViewId, ViewId> GetBestBaseViews(const TrackId& track_id);

  void CalculateBCDForTrack(
      const theia::ViewId& vid_1,
      const theia::ViewId& vid_2,
      const theia::ViewId& vid_3,
      const TrackId& track_id,
      std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d>& BCD);

  //void CreateLinearSystem(Eigen::MatrixXd& constraint_matrix);
  void AddTripletConstraintToSparseMatrix(const ViewId view_id1,
      const ViewId view_id2,
      const ViewId view_id3,
      const std::tuple<Matrix3d, Matrix3d, Matrix3d> &BCD,
      std::unordered_map<std::pair<int, int>, double>* sparse_matrix_entries);

  void FindTripletsForTracks();

  Eigen::Vector3d GetNormalizedHomFeature(
    const ViewId& view_id, const TrackId& track_id) const;
  // Positions are estimated from an eigenvector that is unit-norm with an
  // ambiguous sign. To ensure that the sign of the camera positions is correct,
  // we measure the relative translations from estimated camera positions and
  // compare that to the relative positions. If the sign is incorrect, we flip
  // the sign of all camera positions.
  void FlipSignOfPositionsIfNecessary(
      std::unordered_map<ViewId, Eigen::Vector3d>* positions);

  const Options options_;
  const theia::Reconstruction& reconstruction_;
  const std::unordered_map<ViewIdPair, TwoViewInfo>* view_pairs_;
  std::unordered_map<ViewId, Eigen::Matrix3d> orientations_;
  
  // save a view triplet for each track, eq. 29
  std::unordered_map<TrackId, std::vector<ViewIdTriplet>> triplets_for_tracks_;
  std::unordered_map<TrackId,
                     std::vector<std::tuple<Matrix3d, Matrix3d, Matrix3d>>> BCDs_;

  // We keep one of the positions as constant to remove the ambiguity of the
  // origin of the linear system.
  static const int kConstantPositionIndex = -1;

  std::unordered_map<ViewId, int> num_triplets_for_view_;
  std::unordered_map<ViewId, int> linear_system_index_;

  DISALLOW_COPY_AND_ASSIGN(LiGTPositionEstimator);
};

}  // namespace theia

#endif  // THEIA_SFM_GLOBAL_POSE_ESTIMATION_LiGT_POSITION_ESTIMATOR_H_
