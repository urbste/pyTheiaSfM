// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/global_pose_estimation/glomap_position_estimator.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"
#include "theia/util/random.h"
#include "theia/util/stringprintf.h"

namespace theia {
namespace {

using Eigen::Vector3d;

void AddCamera(const ViewId view_id,
               const Vector3d& position,
               Reconstruction* reconstruction,
               std::unordered_map<ViewId, Vector3d>* orientations,
               std::unordered_map<ViewId, Vector3d>* positions) {
  Camera* camera = reconstruction->MutableView(view_id)->MutableCamera();
  camera->SetImageSize(1000, 1000);
  camera->SetFocalLength(800.0);
  camera->SetPrincipalPoint(500.0, 500.0);
  camera->SetOrientationFromAngleAxis(Vector3d::Zero());
  camera->SetPosition(position);
  (*orientations)[view_id] = Vector3d::Zero();
  (*positions)[view_id] = position;
}

TwoViewInfo MetricTwoViewInfo(const Vector3d& position1,
                              const Vector3d& position2) {
  TwoViewInfo info;
  info.position_2 = (position2 - position1).normalized();
  info.scale_estimate = (position2 - position1).norm();
  info.num_verified_matches = 100;
  return info;
}

}  // namespace

TEST(GlomapPositionEstimatorTest, RecoversMetricPositionsFromTracksAndScales) {
  Reconstruction reconstruction;
  std::unordered_map<ViewId, Vector3d> orientations;
  std::unordered_map<ViewId, Vector3d> positions;
  std::vector<ViewId> view_ids;
  const std::vector<Vector3d> camera_positions = {
      Vector3d(0.0, 0.0, 0.0),
      Vector3d(1.0, 0.0, 0.0),
      Vector3d(0.0, 1.0, 0.0),
      Vector3d(1.0, 1.0, 0.0)};

  for (int i = 0; i < static_cast<int>(camera_positions.size()); ++i) {
    const ViewId view_id = reconstruction.AddView(StringPrintf("%d", i), i);
    view_ids.push_back(view_id);
    AddCamera(view_id, camera_positions[i], &reconstruction, &orientations,
              &positions);
  }

  const std::vector<Vector3d> points = {
      Vector3d(0.2, 0.2, 4.0),
      Vector3d(0.8, 0.3, 4.5),
      Vector3d(0.4, 0.9, 5.0),
      Vector3d(1.1, 0.8, 4.2),
      Vector3d(-0.2, 0.5, 4.8)};
  for (const Vector3d& point3d : points) {
    std::vector<std::pair<ViewId, Feature> > features;
    const Eigen::Vector4d homogeneous_point(point3d.x(), point3d.y(),
                                            point3d.z(), 1.0);
    for (const ViewId view_id : view_ids) {
      Eigen::Vector2d pixel;
      reconstruction.View(view_id)->Camera().ProjectPoint(homogeneous_point,
                                                          &pixel);
      features.emplace_back(view_id, pixel);
    }
    reconstruction.AddTrack(features);
  }

  std::unordered_map<ViewIdPair, TwoViewInfo> view_pairs;
  for (int i = 0; i < static_cast<int>(view_ids.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(view_ids.size()); ++j) {
      view_pairs[ViewIdPair(view_ids[i], view_ids[j])] =
          MetricTwoViewInfo(camera_positions[i], camera_positions[j]);
    }
  }

  GlomapPositionEstimator::Options options;
  options.rng = std::make_shared<RandomNumberGenerator>(42);
  options.max_num_iterations = 100;
  options.min_track_length = 3;
  options.use_pairwise_scale_priors = true;
  GlomapPositionEstimator estimator(options, &reconstruction);
  std::unordered_map<ViewId, Vector3d> estimated_positions;

  EXPECT_TRUE(
      estimator.EstimatePositions(view_pairs, orientations, &estimated_positions));
  ASSERT_EQ(estimated_positions.size(), positions.size());
  for (const auto& position : positions) {
    const Vector3d& estimated_position =
        FindOrDie(estimated_positions, position.first);
    EXPECT_LT((estimated_position - position.second).norm(), 0.05)
        << "view id: " << position.first
        << "\nground truth: " << position.second.transpose()
        << "\nestimated: " << estimated_position.transpose();
  }
}

}  // namespace theia
