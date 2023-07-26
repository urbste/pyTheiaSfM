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

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "theia/math/util.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/pairwise_translation_error.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"
#include "theia/util/random.h"
#include "theia/util/stringprintf.h"
#include "gtest/gtest.h"

namespace theia {

using Eigen::Vector3d;

namespace {

RandomNumberGenerator rng(63);

Camera RandomCamera() {
  Camera camera;
  camera.SetPosition(10 * rng.RandVector3d());
  camera.SetOrientationFromAngleAxis(0.2 * rng.RandVector3d());
  camera.SetImageSize(1000, 1000);
  camera.SetFocalLength(800);
  camera.SetPrincipalPoint(500.0, 500.0);
  return camera;
}

Camera SequentialCamera(const Eigen::Vector3d dir, const double step_size) {
  Camera camera;
  const Eigen::Vector3d pos = dir * step_size + rng.RandVector3d(-0.1,0.1);
  camera.SetPosition(pos);
  camera.SetOrientationFromAngleAxis(0.01* rng.RandVector3d());
  camera.SetImageSize(1000, 1000);
  camera.SetFocalLength(800);
  camera.SetPrincipalPoint(500.0, 500.0);
  return camera;
}

Vector3d RelativeRotationFromTwoRotations(const Vector3d& rotation1,
                                          const Vector3d& rotation2,
                                          const double noise) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(noisy_rotation * rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

Vector3d RelativeTranslationFromTwoPositions(const Vector3d& position1,
                                             const Vector3d& position2,
                                             const Vector3d& rotation1,
                                             const double noise,
                                             double* scale_estimate) {
  const Eigen::AngleAxisd noisy_translation(DegToRad(noise),
                                            rng.RandVector3d().normalized());
  Eigen::Matrix3d rotation_matrix1;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  *scale_estimate = (position2 - position1).norm();
  const Vector3d relative_translation =
      rotation_matrix1 * (position2 - position1).normalized();
  return noisy_translation * relative_translation;
}

// Aligns positions to the ground truth positions via a similarity
// transformation.
void AlignPositions(const std::unordered_map<ViewId, Vector3d>& gt_positions,
                    std::unordered_map<ViewId, Vector3d>* positions) {
  // Collect all positions into a vector.
  std::vector<Vector3d> gt_pos, pos;
  for (const auto& gt_position : gt_positions) {
    gt_pos.push_back(gt_position.second);
    const Vector3d& position = FindOrDie(*positions, gt_position.first);
    pos.push_back(position);
  }

  Eigen::Matrix3d rotation;
  Vector3d translation;
  double scale;
  AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);
  std::cout<<"alignment scale: "<<scale<<std::endl;
  // Apply the similarity transformation.
  for (auto& position : *positions) {
    position.second = scale * (rotation * position.second) + translation;
  }
}

}  // namespace

class EstimatePositionsNonlinearTest : public ::testing::Test {
 public:
  void TestNonlinearPositionEstimator(const int num_views,
                                      const int num_tracks,
                                      const int num_view_pairs,
                                      const double pose_noise,
                                      const double position_tolerance,
                                      std::set<ViewId> fixed_views,
                                      const bool sequential_camera_trajectory=false,
                                      const int min_nr_pts_per_view = 0,
                                      const bool use_scale = false) {
    // Set up the camera.
    SetupReconstruction(num_views, num_tracks, sequential_camera_trajectory);
    GetTwoViewInfos(num_view_pairs, pose_noise, use_scale);

    // Estimate the positions.
    options_.rng = std::make_shared<RandomNumberGenerator>(rng);
    options_.min_num_points_per_view = min_nr_pts_per_view;
    NonlinearPositionEstimator position_estimator(options_, reconstruction_);
    std::unordered_map<ViewId, Vector3d> estimated_positions;

    if (!fixed_views.size()) {
        EXPECT_TRUE(position_estimator.EstimatePositions(
            view_pairs_, orientations_, &estimated_positions));
        EXPECT_EQ(estimated_positions.size(), positions_.size());
    } else {
        // use all view ids
        const auto v_ids = reconstruction_.ViewIds();
        std::unordered_set<ViewId> ids_in_subrecon(v_ids.begin(),v_ids.end());
        // set positions of fixed views
        for (auto& v_id : fixed_views) {
            estimated_positions[v_id] = reconstruction_.View(v_id)->Camera().GetPosition();
        }
        position_estimator.EstimateRemainingPositionsInRecon(
            fixed_views, ids_in_subrecon, view_pairs_, &estimated_positions);
    }
    // Align the positions and measure the error.
    AlignPositions(positions_, &estimated_positions);
    for (const auto& position : positions_) {
      const Vector3d& estimated_position =
          FindOrDie(estimated_positions, position.first);
      const double position_error =
          (position.second - estimated_position).squaredNorm();

      EXPECT_LT(position_error, position_tolerance)
          << "\nview id = " << position.first
          << "\ng.t. position = " << position.second.transpose()
          << "\nestimated position = " << estimated_position.transpose();
    }
  }

 protected:
  void SetUp() {}

  void SetupReconstruction(const int num_views, const int num_tracks, bool sequential_cam=false) {
    // Create random views.
    std::vector<ViewId> view_ids;
    for (int i = 0; i < num_views; i++) {
      const ViewId view_id = reconstruction_.AddView(StringPrintf("%d", i), i);
      view_ids.push_back(view_id);

      // Create a random pose.
      if (sequential_cam) {
          *reconstruction_.MutableView(view_id)->MutableCamera() = SequentialCamera(Eigen::Vector3d(0,i,0), 1.0);
      } else {
          *reconstruction_.MutableView(view_id)->MutableCamera() = RandomCamera();
      }
      orientations_[view_id] =
          reconstruction_.View(view_id)->Camera().GetOrientationAsAngleAxis();
      positions_[view_id] =
          reconstruction_.View(view_id)->Camera().GetPosition();
    }

    // Add random tracks.
    for (int i = 0; i < num_tracks; i++) {
      // Shuffle the view ids so that we can obtain tracks in random views.
      std::random_shuffle(view_ids.begin(), view_ids.end());

      // Create a track that is seen in several views.
      Eigen::Vector4d point = rng.RandVector4d();
      point[2] += 20.0;
      point[3] = 1.0;
      std::vector<std::pair<ViewId, Feature> > features;
      for (size_t j = 0; j < view_ids.size(); j++) {
        const View* view = reconstruction_.View(view_ids[j]);
        Eigen::Vector2d pixel;
        view->Camera().ProjectPoint(point, &pixel);
        features.emplace_back(view_ids[j], pixel);
      }
      Track* track =
          reconstruction_.MutableTrack(reconstruction_.AddTrack(features));
      track->SetEstimated(true);
      *track->MutablePoint() = point;
    }
  }

  void GetTwoViewInfos(const size_t num_view_pairs, const double pose_noise, const bool use_scale = false) {
    // Create a single connected component.
    std::vector<ViewId> view_ids;
    view_ids.push_back(0);
    for (size_t i = 1; i < positions_.size(); i++) {
      const ViewIdPair view_id_pair(i - 1, i);
      view_pairs_[view_id_pair] = CreateTwoViewInfo(view_id_pair, pose_noise, use_scale);
      view_ids.push_back(i);
    }

    while (view_pairs_.size() < num_view_pairs) {
      std::random_shuffle(view_ids.begin(), view_ids.end());
      const ViewIdPair view_id_pair =
          (view_ids[0] < view_ids[1]) ? ViewIdPair(view_ids[0], view_ids[1])
                                      : ViewIdPair(view_ids[1], view_ids[0]);
      if (ContainsKey(view_pairs_, view_id_pair)) {
        continue;
      }

      view_pairs_[view_id_pair] = CreateTwoViewInfo(view_id_pair, pose_noise, use_scale);
    }
  }

  TwoViewInfo CreateTwoViewInfo(const ViewIdPair& view_id_pair,
                                const double pose_noise,
                                const bool use_scale = false) {
    CHECK_LT(view_id_pair.first, view_id_pair.second);
    TwoViewInfo info;
    info.focal_length_1 = 800.0;
    info.focal_length_2 = 800.0;

    // These objects will add noise to the relative pose.
    const Eigen::Vector2d noise = pose_noise * rng.RandVector2d();

    // Determine the relative rotation and add noise.
    info.rotation_2 = RelativeRotationFromTwoRotations(
        FindOrDie(orientations_, view_id_pair.first),
        FindOrDie(orientations_, view_id_pair.second),
        noise(0));

    // Determine the relative position and add noise.
    double scale_estimate = -1.;
    info.position_2 = RelativeTranslationFromTwoPositions(
        FindOrDie(positions_, view_id_pair.first),
        FindOrDie(positions_, view_id_pair.second),
        FindOrDie(orientations_, view_id_pair.first),
        noise(1),
        &scale_estimate);
    
    if (use_scale) {
        info.scale_estimate = scale_estimate;
        if (pose_noise > 0.0) {
            info.scale_estimate += rng.RandDouble(-pose_noise/2, pose_noise/2);
        }
    } else {
        info.scale_estimate = -1.0;
    }


    return info;
  }

  NonlinearPositionEstimator::Options options_;
  std::unordered_map<ViewId, Vector3d> positions_;
  std::unordered_map<ViewId, Vector3d> orientations_;
  std::unordered_map<ViewIdPair, TwoViewInfo> view_pairs_;
  Reconstruction reconstruction_;
};

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoNoise) {
  static const double kTolerance = 1e-4;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  std::set<ViewId> fixed_views;
  TestNonlinearPositionEstimator(
      kNumViews, kNumTracksPerView, kNumViewPairs,
      0.0, kTolerance, fixed_views);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoNoiseWScale) {
  static const double kTolerance = 1e-4;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const bool kUseScale = true;
  std::set<ViewId> fixed_views;
  TestNonlinearPositionEstimator(
      kNumViews, kNumTracksPerView, kNumViewPairs,
      0.0, kTolerance, fixed_views, false, 0, kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestWithNoise) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 1.0;
  std::set<ViewId> fixed_views;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestWithNoiseWScale) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 1.0;
  static const bool kUseScale = true;
  std::set<ViewId> fixed_views;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 false, 0,
                                 kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoNoiseFixedCamsSequential) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 0.0;
  std::set<ViewId> fixed_views = {0};
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 true);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoiseCamsSequentialWScale) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 0.1;
  static const bool kUseScale = true;
  std::set<ViewId> fixed_views;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 true, 0, 
                                 kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoiseFixedCamsSequential) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 4;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 6;
  static const double kPoseNoiseDegrees = 1.0;
  std::set<ViewId> fixed_views = {0,1};
  static const int kNrPointsPerView = 5;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 true,
                                 kNrPointsPerView);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseFixedCamsSequential) {
  static const double kTolerance = 0.1;
  static const int kNumViews = 20;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 50;
  static const double kPoseNoiseDegrees = 0.5;
  std::set<ViewId> fixed_views = {0,1,2,3,4,5};
  static const int kNrPointsPerView = 10;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 true,
                                 kNrPointsPerView);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseWScale) {
  static const double kTolerance = 0.5;
  static const int kNumViews = 1000;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 3000;
  static const double kPoseNoiseDegrees = 0.5;
  static const bool kUseScale = true;
  std::set<ViewId> fixed_views;
  static const int kNrPointsPerView = 10;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 false,
                                 kNrPointsPerView,
                                 kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseFixedCamsSequentialWScale) {
  static const double kTolerance = 0.5;
  static const int kNumViews = 1000;
  static const int kNumTracksPerView = 10;
  static const int kNumViewPairs = 3000;
  static const double kPoseNoiseDegrees = 0.5;
  static const bool kUseScale = true;
  std::set<ViewId> fixed_views = {0,1,2,3,4,5};
  static const int kNrPointsPerView = 10;
  TestNonlinearPositionEstimator(kNumViews,
                                 kNumTracksPerView,
                                 kNumViewPairs,
                                 kPoseNoiseDegrees,
                                 kTolerance,
                                 fixed_views,
                                 true,
                                 kNrPointsPerView,
                                 kUseScale);
}

}  // namespace theia
