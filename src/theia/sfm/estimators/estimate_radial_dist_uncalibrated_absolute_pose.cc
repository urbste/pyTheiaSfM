// Copyright (C) 2019 The Regents of the University of California (Regents).
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

// This file was created by Steffen Urban (urbste@googlemail.com) October 2019

#include "theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h"

#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "theia/sfm/camera/projection_matrix_utils.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/pose/four_point_focal_length_radial_distortion.h"
#include "theia/sfm/types.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/util.h"

namespace theia {
namespace {

void DistortPoint(const Eigen::Vector2d& point2d, const double& distortion,
                  Eigen::Vector2d* distorted_point) {
  const double r_u_sq = point2d[0] * point2d[0] + point2d[1] * point2d[1];

  const double denom = 2.0 * distortion * r_u_sq;
  const double inner_sqrt = 1.0 - 4.0 * distortion * r_u_sq;

  // If the denominator is nearly zero then we can evaluate the distorted
  // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
  if (std::abs(denom) < 1e-15 || inner_sqrt < 0.0) {
    (*distorted_point)[0] = point2d[0];
    (*distorted_point)[1] = point2d[1];
  } else {
    const double scale = (1.0 - std::sqrt(inner_sqrt)) / denom;
    (*distorted_point)[0] = point2d[0] * scale;
    (*distorted_point)[1] = point2d[1] * scale;
  }
}

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
// An estimator for computing the uncalibrated absolute pose from 4 feature
// correspondences. The feature correspondences should be normalized such that
// the principal point is at (0, 0).
class RadialDistUncalibratedAbsolutePoseEstimator
    : public Estimator<FeatureCorrespondence2D3D,
                       RadialDistUncalibratedAbsolutePose> {
 public:
  RadialDistUncalibratedAbsolutePoseEstimator() {}

  // 4 correspondences are needed to determine the absolute pose.
  double SampleSize() const { return 4; }

  // Estimates candidate absolute poses from correspondences.
  bool EstimateModel(
      const std::vector<FeatureCorrespondence2D3D>& correspondences,
      std::vector<RadialDistUncalibratedAbsolutePose>* absolute_poses) const {
    const Vector2d features[4] = {
        correspondences[0].feature, correspondences[1].feature,
        correspondences[2].feature, correspondences[3].feature};
    const Vector3d world_points[4] = {
        correspondences[0].world_point, correspondences[1].world_point,
        correspondences[2].world_point, correspondences[3].world_point};

    std::vector<Matrix3d> rotations;
    std::vector<Vector3d> translations;
    std::vector<double> radial_distortions;
    std::vector<double> focal_lenghts;

    if (!FourPointsPoseFocalLengthRadialDistortion(
            features, world_points, meta_data_.max_focal_length,
            meta_data_.min_focal_length, meta_data_.max_radial_distortion,
            meta_data_.min_radial_distortion, &rotations, &translations,
            &radial_distortions, &focal_lenghts))
      return false;

    absolute_poses->resize(rotations.size());
    for (int i = 0; i < rotations.size(); ++i) {
      (*absolute_poses)[i].radial_distortion = radial_distortions[i];
      (*absolute_poses)[i].focal_length = focal_lenghts[i];
      (*absolute_poses)[i].rotation = rotations[i];
      (*absolute_poses)[i].translation = translations[i];
    }

    return rotations.size() > 0;
  }

  // The error for a correspondences given an absolute pose. This is the squared
  // reprojection error.
  double Error(const FeatureCorrespondence2D3D& correspondence,
               const RadialDistUncalibratedAbsolutePose& absolute_pose) const {
    // undistort the feature with the estimated radial distortion parameter
    // project der world point with the given focal length and
    // compare it to the undistorted image point
    Matrix3d K =
        Vector3d(absolute_pose.focal_length, absolute_pose.focal_length, 1.0)
            .asDiagonal();
    Vector3d reproj_pt = (absolute_pose.rotation * correspondence.world_point +
                          absolute_pose.translation);
    const Eigen::Vector2d reproj_pt_2d = (K * reproj_pt).hnormalized();
    Eigen::Vector2d distorted_point;
    DistortPoint(reproj_pt_2d, absolute_pose.radial_distortion,
                 &distorted_point);

    return (distorted_point - correspondence.feature).squaredNorm();
  }

  void SetMetadata(RadialDistUncalibratedAbsolutePoseMetaData meta_data) {
    meta_data_ = meta_data;
  }

 private:
  RadialDistUncalibratedAbsolutePoseMetaData meta_data_;

  DISALLOW_COPY_AND_ASSIGN(RadialDistUncalibratedAbsolutePoseEstimator);
};

}  // namespace

bool EstimateRadialDistUncalibratedAbsolutePose(
    const RansacParameters& ransac_params, const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences,
    const RadialDistUncalibratedAbsolutePoseMetaData& meta_data,
    RadialDistUncalibratedAbsolutePose* absolute_pose,
    RansacSummary* ransac_summary) {
  RadialDistUncalibratedAbsolutePoseEstimator absolute_pose_estimator;
  absolute_pose_estimator.SetMetadata(meta_data);

  std::unique_ptr<
      SampleConsensusEstimator<RadialDistUncalibratedAbsolutePoseEstimator> >
      ransac = CreateAndInitializeRansacVariant(ransac_type, ransac_params,
                                                absolute_pose_estimator);
  // Estimate the absolute pose.
  const bool success = ransac->Estimate(normalized_correspondences,
                                        absolute_pose, ransac_summary);

  return success;
}

}  // namespace theia
