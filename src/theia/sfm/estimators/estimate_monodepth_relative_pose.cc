// Copyright (C) 2024 The Regents of the University of California (Regents).
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

#include "theia/sfm/estimators/estimate_monodepth_relative_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "theia/matching/feature_correspondence.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/pose/refine_monodepth_relative_pose.h"
#include "theia/sfm/pose/relative_pose_monodepth_3pt.h"
#include "theia/sfm/pose/util.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/util.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// Essential matrix consistent with the RelativePose/TwoViewInfo convention
// position = -rotation^T * translation (Sampson distance is invariant to the
// overall scale of the essential matrix, so the fact that `translation` here
// may not be unit-norm does not matter for scoring).
Matrix3d EssentialMatrixFromRotationPosition(const Matrix3d& rotation,
                                             const Vector3d& position) {
  const Vector3d translation = -rotation * position;
  return CrossProductMatrix(translation) * rotation;
}

// Fundamental matrix F = K2^-T E K1^-1 for scoring the uncalibrated (shared
// or varying focal) estimators against pp-centered pixel correspondences.
Matrix3d FundamentalMatrixFromResult(const MonoDepthRelativePoseResult& result) {
  const Matrix3d essential_matrix =
      EssentialMatrixFromRotationPosition(result.rotation, result.position);
  const Matrix3d k1_inv = Eigen::DiagonalMatrix<double, 3>(
      1.0 / result.focal_length1, 1.0 / result.focal_length1, 1.0);
  const Matrix3d k2_inv = Eigen::DiagonalMatrix<double, 3>(
      1.0 / result.focal_length2, 1.0 / result.focal_length2, 1.0);
  return k2_inv.transpose() * essential_matrix * k1_inv;
}

void ExtractPointsAndDepths(
    const std::vector<FeatureCorrespondence>& correspondences,
    std::vector<Eigen::Vector2d>* x1,
    std::vector<Eigen::Vector2d>* x2,
    std::vector<double>* depth1,
    std::vector<double>* depth2) {
  x1->resize(correspondences.size());
  x2->resize(correspondences.size());
  depth1->resize(correspondences.size());
  depth2->resize(correspondences.size());
  for (size_t i = 0; i < correspondences.size(); ++i) {
    (*x1)[i] = correspondences[i].feature1.point_;
    (*x2)[i] = correspondences[i].feature2.point_;
    (*depth1)[i] = correspondences[i].feature1.depth_prior_;
    (*depth2)[i] = correspondences[i].feature2.depth_prior_;
  }
}

// ---------------------------------------------------------------------------
// Calibrated estimator.
// ---------------------------------------------------------------------------
class MonoDepthRelativePoseEstimator
    : public Estimator<FeatureCorrespondence, MonoDepthRelativePoseResult> {
 public:
  MonoDepthRelativePoseEstimator() {}

  double SampleSize() const { return 3; }

  bool EstimateModel(
      const std::vector<FeatureCorrespondence>& correspondences,
      std::vector<MonoDepthRelativePoseResult>* results) const {
    std::vector<Vector3d> x1h(3), x2h(3);
    std::vector<double> depth1(3), depth2(3);
    for (int i = 0; i < 3; i++) {
      x1h[i] = correspondences[i].feature1.point_.homogeneous();
      x2h[i] = correspondences[i].feature2.point_.homogeneous();
      depth1[i] = correspondences[i].feature1.depth_prior_;
      depth2[i] = correspondences[i].feature2.depth_prior_;
    }

    std::vector<MonoDepthRelativePose> poses;
    if (MonoDepthRelativePose3pt(x1h, x2h, depth1, depth2, &poses) == 0) {
      return false;
    }

    results->reserve(poses.size());
    for (const MonoDepthRelativePose& pose : poses) {
      if (pose.translation.norm() < 1e-12) {
        continue;
      }
      MonoDepthRelativePoseResult result;
      result.rotation = pose.rotation;
      result.position =
          -pose.rotation.transpose() * pose.translation.normalized();
      result.scale = pose.scale;
      result.shift1 = pose.shift1;
      result.shift2 = pose.shift2;
      results->push_back(result);
    }
    return results->size() > 0;
  }

  bool RefineModel(const std::vector<FeatureCorrespondence>& correspondences,
                   const double error_thresh,
                   MonoDepthRelativePoseResult* result) const {
    std::vector<Eigen::Vector2d> x1, x2;
    std::vector<double> depth1, depth2;
    ExtractPointsAndDepths(correspondences, &x1, &x2, &depth1, &depth2);
    return RefineMonoDepthRelativePose(x1,
                                       x2,
                                       depth1,
                                       depth2,
                                       error_thresh,
                                       &result->rotation,
                                       &result->position,
                                       &result->scale,
                                       &result->shift1,
                                       &result->shift2);
  }

  double Error(const FeatureCorrespondence& correspondence,
              const MonoDepthRelativePoseResult& result) const {
    const Matrix3d essential_matrix =
        EssentialMatrixFromRotationPosition(result.rotation, result.position);
    return SquaredSampsonDistance(essential_matrix,
                                  correspondence.feature1.point_,
                                  correspondence.feature2.point_);
  }

  std::vector<double> Residuals(
      const std::vector<FeatureCorrespondence>& correspondences,
      const MonoDepthRelativePoseResult& result) const override {
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
    const Matrix3d essential_matrix =
        EssentialMatrixFromRotationPosition(result.rotation, result.position);
    return SquaredSampsonDistances(essential_matrix, cached_x1_, cached_x2_);
  }

 private:
  mutable Eigen::Matrix3Xd cached_x1_, cached_x2_;
  mutable const std::vector<FeatureCorrespondence>* cached_correspondences_ =
      nullptr;

  DISALLOW_COPY_AND_ASSIGN(MonoDepthRelativePoseEstimator);
};

// ---------------------------------------------------------------------------
// Uncalibrated, shared-focal estimator.
// ---------------------------------------------------------------------------
class MonoDepthRelativePoseSharedFocalEstimator
    : public Estimator<FeatureCorrespondence, MonoDepthRelativePoseResult> {
 public:
  MonoDepthRelativePoseSharedFocalEstimator() {}

  double SampleSize() const { return 3; }

  bool EstimateModel(
      const std::vector<FeatureCorrespondence>& correspondences,
      std::vector<MonoDepthRelativePoseResult>* results) const {
    std::vector<Vector3d> x1h(3), x2h(3);
    std::vector<double> depth1(3), depth2(3);
    for (int i = 0; i < 3; i++) {
      x1h[i] = correspondences[i].feature1.point_.homogeneous();
      x2h[i] = correspondences[i].feature2.point_.homogeneous();
      depth1[i] = correspondences[i].feature1.depth_prior_;
      depth2[i] = correspondences[i].feature2.depth_prior_;
    }

    std::vector<MonoDepthRelativePose> poses;
    if (MonoDepthRelativePose3ptSharedFocal(
            x1h, x2h, depth1, depth2, &poses) == 0) {
      return false;
    }

    results->reserve(poses.size());
    for (const MonoDepthRelativePose& pose : poses) {
      if (pose.translation.norm() < 1e-12 || pose.focal_length1 <= 0.0) {
        continue;
      }
      MonoDepthRelativePoseResult result;
      result.rotation = pose.rotation;
      result.position =
          -pose.rotation.transpose() * pose.translation.normalized();
      result.scale = pose.scale;
      result.focal_length1 = pose.focal_length1;
      result.focal_length2 = pose.focal_length2;
      results->push_back(result);
    }
    return results->size() > 0;
  }

  bool RefineModel(const std::vector<FeatureCorrespondence>& correspondences,
                   const double error_thresh,
                   MonoDepthRelativePoseResult* result) const {
    std::vector<Eigen::Vector2d> x1, x2;
    std::vector<double> depth1, depth2;
    ExtractPointsAndDepths(correspondences, &x1, &x2, &depth1, &depth2);
    const bool improved = RefineMonoDepthSharedFocalRelativePose(
        x1,
        x2,
        depth1,
        depth2,
        error_thresh,
        &result->rotation,
        &result->position,
        &result->scale,
        &result->focal_length1);
    result->focal_length2 = result->focal_length1;
    return improved;
  }

  double Error(const FeatureCorrespondence& correspondence,
              const MonoDepthRelativePoseResult& result) const {
    const Matrix3d fundamental_matrix = FundamentalMatrixFromResult(result);
    return SquaredSampsonDistance(fundamental_matrix,
                                  correspondence.feature1.point_,
                                  correspondence.feature2.point_);
  }

  std::vector<double> Residuals(
      const std::vector<FeatureCorrespondence>& correspondences,
      const MonoDepthRelativePoseResult& result) const override {
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
    const Matrix3d fundamental_matrix = FundamentalMatrixFromResult(result);
    return SquaredSampsonDistances(fundamental_matrix, cached_x1_, cached_x2_);
  }

 private:
  mutable Eigen::Matrix3Xd cached_x1_, cached_x2_;
  mutable const std::vector<FeatureCorrespondence>* cached_correspondences_ =
      nullptr;

  DISALLOW_COPY_AND_ASSIGN(MonoDepthRelativePoseSharedFocalEstimator);
};

// ---------------------------------------------------------------------------
// Uncalibrated, varying-focal estimator.
// ---------------------------------------------------------------------------
class MonoDepthRelativePoseVaryingFocalEstimator
    : public Estimator<FeatureCorrespondence, MonoDepthRelativePoseResult> {
 public:
  MonoDepthRelativePoseVaryingFocalEstimator() {}

  double SampleSize() const { return 3; }

  bool EstimateModel(
      const std::vector<FeatureCorrespondence>& correspondences,
      std::vector<MonoDepthRelativePoseResult>* results) const {
    std::vector<Vector3d> x1h(3), x2h(3);
    std::vector<double> depth1(3), depth2(3);
    for (int i = 0; i < 3; i++) {
      x1h[i] = correspondences[i].feature1.point_.homogeneous();
      x2h[i] = correspondences[i].feature2.point_.homogeneous();
      depth1[i] = correspondences[i].feature1.depth_prior_;
      depth2[i] = correspondences[i].feature2.depth_prior_;
    }

    std::vector<MonoDepthRelativePose> poses;
    if (MonoDepthRelativePose3ptVaryingFocal(
            x1h, x2h, depth1, depth2, &poses) == 0) {
      return false;
    }

    results->reserve(poses.size());
    for (const MonoDepthRelativePose& pose : poses) {
      if (pose.translation.norm() < 1e-12 || pose.focal_length1 <= 0.0 ||
          pose.focal_length2 <= 0.0) {
        continue;
      }
      MonoDepthRelativePoseResult result;
      result.rotation = pose.rotation;
      result.position =
          -pose.rotation.transpose() * pose.translation.normalized();
      result.scale = pose.scale;
      result.focal_length1 = pose.focal_length1;
      result.focal_length2 = pose.focal_length2;
      results->push_back(result);
    }
    return results->size() > 0;
  }

  bool RefineModel(const std::vector<FeatureCorrespondence>& correspondences,
                   const double error_thresh,
                   MonoDepthRelativePoseResult* result) const {
    std::vector<Eigen::Vector2d> x1, x2;
    std::vector<double> depth1, depth2;
    ExtractPointsAndDepths(correspondences, &x1, &x2, &depth1, &depth2);
    return RefineMonoDepthVaryingFocalRelativePose(x1,
                                                   x2,
                                                   depth1,
                                                   depth2,
                                                   error_thresh,
                                                   &result->rotation,
                                                   &result->position,
                                                   &result->scale,
                                                   &result->focal_length1,
                                                   &result->focal_length2);
  }

  double Error(const FeatureCorrespondence& correspondence,
              const MonoDepthRelativePoseResult& result) const {
    const Matrix3d fundamental_matrix = FundamentalMatrixFromResult(result);
    return SquaredSampsonDistance(fundamental_matrix,
                                  correspondence.feature1.point_,
                                  correspondence.feature2.point_);
  }

  std::vector<double> Residuals(
      const std::vector<FeatureCorrespondence>& correspondences,
      const MonoDepthRelativePoseResult& result) const override {
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
    const Matrix3d fundamental_matrix = FundamentalMatrixFromResult(result);
    return SquaredSampsonDistances(fundamental_matrix, cached_x1_, cached_x2_);
  }

 private:
  mutable Eigen::Matrix3Xd cached_x1_, cached_x2_;
  mutable const std::vector<FeatureCorrespondence>* cached_correspondences_ =
      nullptr;

  DISALLOW_COPY_AND_ASSIGN(MonoDepthRelativePoseVaryingFocalEstimator);
};

}  // namespace

bool EstimateMonoDepthRelativePose(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& normalized_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary) {
  MonoDepthRelativePoseEstimator estimator;
  std::unique_ptr<SampleConsensusEstimator<MonoDepthRelativePoseEstimator> >
      ransac = CreateAndInitializeRansacVariant(
          ransac_type, ransac_params, estimator);
  return ransac->Estimate(
      normalized_correspondences, result, ransac_summary);
}

bool EstimateMonoDepthRelativePoseSharedFocal(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary) {
  MonoDepthRelativePoseSharedFocalEstimator estimator;
  std::unique_ptr<
      SampleConsensusEstimator<MonoDepthRelativePoseSharedFocalEstimator> >
      ransac = CreateAndInitializeRansacVariant(
          ransac_type, ransac_params, estimator);
  return ransac->Estimate(
      centered_correspondences, result, ransac_summary);
}

bool EstimateMonoDepthRelativePoseVaryingFocal(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence>& centered_correspondences,
    MonoDepthRelativePoseResult* result,
    RansacSummary* ransac_summary) {
  MonoDepthRelativePoseVaryingFocalEstimator estimator;
  std::unique_ptr<
      SampleConsensusEstimator<MonoDepthRelativePoseVaryingFocalEstimator> >
      ransac = CreateAndInitializeRansacVariant(
          ransac_type, ransac_params, estimator);
  return ransac->Estimate(
      centered_correspondences, result, ransac_summary);
}

}  // namespace theia
