// Copyright (C) 2014 The Regents of the University of California (Regents).
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

#include "theia/sfm/estimators/estimate_relative_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <memory>
#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/pose/essential_matrix_utils.h"
#include "theia/sfm/pose/five_point_relative_pose.h"
#include "theia/sfm/pose/five_point_relative_pose_sturm.h"
#include "theia/sfm/pose/refine_relative_pose.h"
#include "theia/sfm/pose/util.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/util.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// An estimator for computing the relative pose from 5 feature
// correspondences. The feature correspondences should be normalized
// by the focal length with the principal point at (0, 0).
class RelativePoseEstimator
    : public Estimator<FeatureCorrespondence, RelativePose> {
 public:
  explicit RelativePoseEstimator(const bool use_sturm_5pt = true)
      : use_sturm_5pt_(use_sturm_5pt) {}

  // 5 correspondences are needed to determine an essential matrix and thus a
  // relative pose..
  double SampleSize() const { return 5; }

  // Estimates candidate relative poses from correspondences. Only the
  // essential matrix is filled in (Sampson-only scoring, COLMAP-style): the
  // expensive per-candidate SVD decomposition + 4-way cheirality check is
  // deferred to RefineModel/the caller, since scoring only needs the E
  // matrix. rotation/position are left at their default (identity/zero)
  // until RefineModel or the final decomposition in EstimateRelativePose()
  // fills them in.
  bool EstimateModel(const std::vector<FeatureCorrespondence>& correspondences,
                     std::vector<RelativePose>* relative_poses) const {
    std::vector<Matrix3d> essential_matrices;
    // The ported Sturm-sequence-based solver (theia/sfm/pose/
    // five_point_relative_pose_sturm.h) is strictly minimal, so it is only
    // used for exactly-5-point samples (the only case RANSAC ever calls this
    // with); larger, non-minimal samples always use FivePointRelativePose.
    if (use_sturm_5pt_ && correspondences.size() == 5) {
      std::vector<Vector3d> x1h, x2h;
      x1h.reserve(5);
      x2h.reserve(5);
      for (int i = 0; i < 5; i++) {
        x1h.emplace_back(correspondences[i].feature1.point_.homogeneous());
        x2h.emplace_back(correspondences[i].feature2.point_.homogeneous());
      }
      if (FivePointRelativePoseSturm(x1h, x2h, &essential_matrices) == 0) {
        return false;
      }
    } else {
      std::vector<Eigen::Vector2d> image1_points, image2_points;
      image1_points.reserve(correspondences.size());
      image2_points.reserve(correspondences.size());
      for (int i = 0; i < correspondences.size(); i++) {
        image1_points.emplace_back(correspondences[i].feature1.point_);
        image2_points.emplace_back(correspondences[i].feature2.point_);
      }
      if (!FivePointRelativePose(
              image1_points, image2_points, &essential_matrices)) {
        return false;
      }
    }

    relative_poses->reserve(essential_matrices.size());
    for (const Eigen::Matrix3d& essential_matrix : essential_matrices) {
      RelativePose relative_pose;
      relative_pose.essential_matrix = essential_matrix;
      relative_pose.rotation.setIdentity();
      relative_pose.position.setZero();
      relative_poses->push_back(relative_pose);
    }
    return relative_poses->size() > 0;
  }

  bool RefineModel(
    const std::vector<FeatureCorrespondence>& correspondences,
    const double error_thresh,
    RelativePose* relative_pose) const {
    // Local optimization needs an actual rotation/position. Decompose the
    // essential matrix lazily here (once per LO call, on the current inlier
    // set), rather than for every candidate from EstimateModel.
    GetBestPoseFromEssentialMatrix(relative_pose->essential_matrix,
                                   correspondences,
                                   &relative_pose->rotation,
                                   &relative_pose->position);

    std::vector<Eigen::Vector2d> x1(correspondences.size());
    std::vector<Eigen::Vector2d> x2(correspondences.size());
    for (size_t i = 0; i < correspondences.size(); ++i) {
      x1[i] = correspondences[i].feature1.point_;
      x2[i] = correspondences[i].feature2.point_;
    }

    // Dense 5-DoF Sampson LM (PoseLib-style); avoids Ceres problem setup.
    if (!RefineRelativePoseSampson(x1,
                                   x2,
                                   error_thresh,
                                   &relative_pose->rotation,
                                   &relative_pose->position)) {
      return false;
    }

    // Keep essential_matrix consistent with the refined pose for scoring.
    const Eigen::Vector3d translation =
        -relative_pose->rotation * relative_pose->position;
    relative_pose->essential_matrix =
        CrossProductMatrix(translation) * relative_pose->rotation;
    return true;
  }

  // The error for a correspondences given a model. This is the squared
  // sampson error. Cheirality is deliberately not checked here: scoring
  // purely on Sampson distance (no per-candidate SVD/cheirality) is the
  // COLMAP-style approach and is significantly cheaper; the winning model's
  // inliers are re-filtered by cheirality once in EstimateRelativePose().
  double Error(const FeatureCorrespondence& correspondence,
               const RelativePose& relative_pose) const {
    return SquaredSampsonDistance(relative_pose.essential_matrix,
                                  correspondence.feature1.point_,
                                  correspondence.feature2.point_);
  }

  // Vectorized Sampson residuals for all correspondences at once. Packs the
  // homogeneous point coordinates into 3xN matrices lazily on first call and
  // reuses them across RANSAC iterations (the `data` vector passed in by
  // SampleConsensusEstimator::Estimate is the same object every iteration).
  std::vector<double> Residuals(
      const std::vector<FeatureCorrespondence>& correspondences,
      const RelativePose& relative_pose) const override {
    if (cached_correspondences_ != &correspondences ||
        cached_x1_.cols() != static_cast<int>(correspondences.size())) {
      cached_x1_.resize(3, correspondences.size());
      cached_x2_.resize(3, correspondences.size());
      for (int i = 0; i < correspondences.size(); i++) {
        cached_x1_.col(i) = correspondences[i].feature1.point_.homogeneous();
        cached_x2_.col(i) = correspondences[i].feature2.point_.homogeneous();
      }
      cached_correspondences_ = &correspondences;
    }
    return SquaredSampsonDistances(
        relative_pose.essential_matrix, cached_x1_, cached_x2_);
  }

 private:
  const bool use_sturm_5pt_;
  mutable Eigen::Matrix3Xd cached_x1_, cached_x2_;
  mutable const std::vector<FeatureCorrespondence>* cached_correspondences_ =
      nullptr;

  DISALLOW_COPY_AND_ASSIGN(RelativePoseEstimator);
};

}  // namespace

bool EstimateRelativePose(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences,
    RelativePose* relative_pose,
    RansacSummary* ransac_summary) {
  RelativePoseEstimator relative_pose_estimator(ransac_params.use_sturm_5pt);
  std::unique_ptr<SampleConsensusEstimator<RelativePoseEstimator> > ransac =
      CreateAndInitializeRansacVariant(
          ransac_type, ransac_params, relative_pose_estimator);
  // Estimate the relative pose.
  if (!ransac->Estimate(
          normalized_correspondences, relative_pose, ransac_summary)) {
    return false;
  }

  // RelativePoseEstimator scores purely on Sampson distance and never
  // decomposes the essential matrix into a pose (Phase 1.1), so do the
  // single, final decomposition here on the winning inlier set. If LO-RANSAC
  // ran, this simply reproduces the pose RefineModel's BA already computed;
  // if it did not run, this is the only decomposition performed.
  std::vector<FeatureCorrespondence> inlier_correspondences;
  inlier_correspondences.reserve(ransac_summary->inliers.size());
  for (const int inlier_index : ransac_summary->inliers) {
    inlier_correspondences.push_back(
        normalized_correspondences[inlier_index]);
  }
  GetBestPoseFromEssentialMatrix(relative_pose->essential_matrix,
                                 inlier_correspondences,
                                 &relative_pose->rotation,
                                 &relative_pose->position);

  // Re-filter inliers by cheirality w.r.t. the final pose, preserving the
  // previous behavior where RANSAC inliers were guaranteed to triangulate in
  // front of both cameras.
  std::vector<int> cheiral_inliers;
  cheiral_inliers.reserve(ransac_summary->inliers.size());
  for (const int inlier_index : ransac_summary->inliers) {
    if (IsTriangulatedPointInFrontOfCameras(
            normalized_correspondences[inlier_index],
            relative_pose->rotation,
            relative_pose->position)) {
      cheiral_inliers.push_back(inlier_index);
    }
  }
  ransac_summary->inliers = std::move(cheiral_inliers);

  return true;
}

}  // namespace theia
