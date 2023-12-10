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

#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"
#include "theia/sfm/pose/perspective_three_point.h"
#include "theia/sfm/pose/sqpnp.h"
#include "theia/sfm/pose/dls_pnp.h"
#include "theia/sfm/pose/mlpnp.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/util.h"

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// An estimator for computing the absolute pose from 3 feature
// correspondences. The feature correspondences should be normalized by the
// focal length with the principal point at (0, 0).
class CalibratedAbsolutePoseEstimator
    : public Estimator<FeatureCorrespondence2D3D, CalibratedAbsolutePose> {
 public:
  CalibratedAbsolutePoseEstimator() : pnp_type_(PnPType::KNEIP) {
    ba_opts_.max_num_iterations = 2;
    ba_opts_.use_homogeneous_point_parametrization = false;
    ba_opts_.intrinsics_to_optimize = theia::OptimizeIntrinsicsType::NONE;
  }

  CalibratedAbsolutePoseEstimator(const PnPType& pnp_type) : pnp_type_(pnp_type) {}
  // 3 correspondences are needed to determine the absolute pose.
  double SampleSize() const { 
    if (pnp_type_ == PnPType::MLPNP) {
      return 6;
    } 
    else {
      return 3;
    }
  }

  // Estimates candidate absolute poses from correspondences.
  bool EstimateModel(
      const std::vector<FeatureCorrespondence2D3D>& correspondences,
      std::vector<CalibratedAbsolutePose>* absolute_poses) const {
    std::vector<Eigen::Vector2d> features(3);
    std::vector<Eigen::Vector3d> world_points(3);
    for (int i = 0; i < 3; ++i) {
        features[i] = correspondences[i].feature;
        world_points[i] = correspondences[i].world_point;
    }

    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Quaterniond> quats;

    if (pnp_type_ == PnPType::KNEIP) {
        if (!PoseFromThreePoints(
                features, world_points, &rotations, &translations)) {
          return false;
        }
    } else if (pnp_type_ == PnPType::DLS) {
        if (!DlsPnp(features, world_points, &quats, &translations)) {
            return false;
        }
        for (const auto& q : quats) {
            rotations.push_back(q.matrix());
        }
    } else if (pnp_type_ == PnPType::SQPNP) {
      if (!SQPnP(features, world_points, &quats, &translations)) {
          return false;
      }
      for (const auto& q : quats) {
          rotations.push_back(q.matrix());
      }
    } else if (pnp_type_ == PnPType::MLPNP) {
      rotations.resize(1);
      translations.resize(1);
      if (!MLPnP(features, {}, world_points, &rotations[0], &translations[0])) {
          return false;
      }
    } else {
      LOG(FATAL) << "Unknown PnP type.";
    }

    for (size_t i = 0; i < rotations.size(); i++) {
      CalibratedAbsolutePose pose;
      pose.rotation = rotations[i];
      pose.position = -pose.rotation.transpose() * translations[i];
      absolute_poses->emplace_back(pose);
    }

    return absolute_poses->size() > 0;
  }

  bool RefineModel(const std::vector<FeatureCorrespondence2D3D>& correspondences,
    CalibratedAbsolutePose* absolute_pose) const {
    Reconstruction reconstruction;
    const auto v_id = reconstruction.AddView("0", 0, 0.0);
    auto m_view = reconstruction.MutableView(v_id);
    auto m_cam = m_view->MutableCamera();
    m_view->SetEstimated(true);
    m_cam->SetOrientationFromRotationMatrix(absolute_pose->rotation);
    m_cam->SetPosition(absolute_pose->position);
    for (int i=0; i < correspondences.size(); ++i) {
        const auto t_id = reconstruction.AddTrack();
        auto m_track = reconstruction.MutableTrack(t_id);
        m_track->SetEstimated(true);
        m_track->SetPoint(correspondences[i].world_point.homogeneous());
        reconstruction.AddObservation(v_id, t_id, theia::Feature(correspondences[i].feature));
    }
    
    theia::BundleAdjustmentSummary ba_summary = theia::BundleAdjustView(
        ba_opts_, v_id, &reconstruction);

    absolute_pose->position = m_cam->GetPosition();
    absolute_pose->rotation = m_cam->GetOrientationAsRotationMatrix();
    return ba_summary.final_cost < ba_summary.initial_cost && ba_summary.success;
  }


  // The error for a correspondences given an absolute pose. This is the squared
  // reprojection error.
  double Error(const FeatureCorrespondence2D3D& correspondence,
               const CalibratedAbsolutePose& absolute_pose) const {
    // The reprojected point is computed as R * (X - c) where R is the camera
    // rotation, c is the position, and X is the 3D point.
    const Eigen::Vector2d reprojected_feature =
        (absolute_pose.rotation *
         (correspondence.world_point - absolute_pose.position))
            .hnormalized();
    return (reprojected_feature - correspondence.feature).squaredNorm();
  }

 private:
  PnPType pnp_type_;
  theia::BundleAdjustmentOptions ba_opts_;
  DISALLOW_COPY_AND_ASSIGN(CalibratedAbsolutePoseEstimator);
};

}  // namespace

bool EstimateCalibratedAbsolutePose(
    const RansacParameters& ransac_params,
    const RansacType& ransac_type,
    const PnPType& pnp_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences,
    CalibratedAbsolutePose* absolute_pose,
    RansacSummary* ransac_summary) {
  CalibratedAbsolutePoseEstimator absolute_pose_estimator(pnp_type);
  std::unique_ptr<SampleConsensusEstimator<CalibratedAbsolutePoseEstimator> >
      ransac = CreateAndInitializeRansacVariant(
          ransac_type, ransac_params, absolute_pose_estimator);
  // Estimate the absolute pose.
  return ransac->Estimate(
      normalized_correspondences, absolute_pose, ransac_summary);
}

}  // namespace theia
