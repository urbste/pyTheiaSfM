// Tests of the ART position estimator (Li, Cai, Wu, arXiv:2611.11770, Sec. 5.3).

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/global_pose_estimation/art_position_estimator.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/util/random.h"

namespace theia {

namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

Matrix3d AngleAxisToMatrix(const Vector3d& angle_axis) {
  Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(angle_axis.data(), rotation.data());
  return rotation;
}

// Similarity alignment (scale, rotation, translation) of positions to the
// ground truth.
void AlignPositions(const std::unordered_map<ViewId, Vector3d>& gt_positions,
                    std::unordered_map<ViewId, Vector3d>* positions) {
  std::vector<Vector3d> gt_pos, pos;
  for (const auto& gt_position : gt_positions) {
    gt_pos.push_back(gt_position.second);
    pos.push_back(positions->at(gt_position.first));
  }
  Matrix3d rotation;
  Vector3d translation;
  double scale;
  AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);
  for (auto& position : *positions) {
    position.second = scale * (rotation * position.second) + translation;
  }
}

// A random synthetic scene: cameras with random poses (world-to-camera
// orientation, camera center), points seen in all views.
struct SyntheticScene {
  Reconstruction reconstruction;
  std::unordered_map<ViewId, Vector3d> positions;
  std::unordered_map<ViewId, Vector3d> orientations;
  std::unordered_map<ViewIdPair, TwoViewInfo> view_pairs;
};

// Builds the scene with all view pairs. The relative translation directions
// are exact (only their sign is used by the estimator).
void BuildScene(RandomNumberGenerator* rng,
                const int num_views,
                const int num_points,
                const double image_noise_px,
                SyntheticScene* scene) {
  std::vector<ViewId> view_ids;
  for (int i = 0; i < num_views; ++i) {
    const ViewId view_id =
        scene->reconstruction.AddView(std::to_string(i), 0, i);
    view_ids.push_back(view_id);
    Camera camera;
    camera.SetPosition(10.0 * rng->RandVector3d());
    camera.SetOrientationFromAngleAxis(0.2 * rng->RandVector3d());
    camera.SetImageSize(1000, 1000);
    camera.SetFocalLength(800);
    camera.SetPrincipalPoint(500.0, 500.0);
    *scene->reconstruction.MutableView(view_id)->MutableCamera() = camera;
    scene->orientations[view_id] = camera.GetOrientationAsAngleAxis();
    scene->positions[view_id] = camera.GetPosition();
  }

  for (int i = 0; i < num_points; ++i) {
    Vector4d point;
    point.head<3>() = rng->RandVector3d(-2.0, 2.0);
    point(2) += 20.0;
    point(3) = 1.0;
    std::vector<std::pair<ViewId, Feature>> features;
    for (const ViewId view_id : view_ids) {
      Eigen::Vector2d pixel;
      scene->reconstruction.View(view_id)->Camera().ProjectPoint(point, &pixel);
      AddNoiseToProjection(image_noise_px, rng, &pixel);
      features.emplace_back(view_id, pixel);
    }
    Track* track =
        scene->reconstruction.MutableTrack(scene->reconstruction.AddTrack(features));
    *track->MutablePoint() = point;
  }

  for (size_t i = 0; i < view_ids.size(); ++i) {
    for (size_t j = i + 1; j < view_ids.size(); ++j) {
      const ViewId a = view_ids[i];
      const ViewId b = view_ids[j];
      TwoViewInfo info;
      info.focal_length_1 = 800.0;
      info.focal_length_2 = 800.0;
      info.rotation_2 = Eigen::Vector3d::Zero();
      info.position_2 = (AngleAxisToMatrix(scene->orientations[a]) *
                         (scene->positions[b] - scene->positions[a]))
                            .normalized();
      scene->view_pairs[ViewIdPair(a, b)] = info;
    }
  }
}

// Extent of the scene: largest distance between two camera centers.
double SceneExtent(const std::unordered_map<ViewId, Vector3d>& positions) {
  double extent = 0.0;
  for (const auto& a : positions) {
    for (const auto& b : positions) {
      extent = std::max(extent, (a.second - b.second).norm());
    }
  }
  return extent;
}

// Normalized ray (x, y, 1) of a point seen from a camera (R, c).
Vector3d Ray(const Matrix3d& R, const Vector3d& c, const Vector3d& X) {
  const Vector3d Xc = R * (X - c);
  return Xc / Xc.z();
}

}  // namespace

// (1) Three cameras, noise-free: the triplet solution is parallel to the ground
// truth baselines, with the correct sign and length ratio.
TEST(ARTTripletTest, RecoversBaselinesNoNoise) {
  RandomNumberGenerator rng(7);
  Matrix3d R[3];
  Vector3d c[3];
  for (int i = 0; i < 3; ++i) {
    R[i] = AngleAxisToMatrix(0.3 * rng.RandVector3d());
    c[i] = 2.0 * rng.RandVector3d();
  }

  std::vector<Vector3d> x[3];
  while (x[0].size() < 30) {
    const Vector3d p(rng.RandDouble(-2.0, 2.0), rng.RandDouble(-2.0, 2.0),
                     rng.RandDouble(8.0, 12.0));
    const Vector3d X = c[0] + R[0].transpose() * p;
    bool in_front = true;
    for (int i = 0; i < 3; ++i) {
      if ((R[i] * (X - c[i])).z() <= 0.0) {
        in_front = false;
      }
    }
    if (!in_front) {
      continue;
    }
    for (int i = 0; i < 3; ++i) {
      x[i].push_back(Ray(R[i], c[i], X));
    }
  }

  const std::vector<double> weights(x[0].size(), 1.0);
  ARTTripletSolution sol;
  ASSERT_TRUE(SolveARTTriplet(x[0], x[1], x[2], R[0], R[1], R[2], weights,
                              &sol));

  const Vector3d gt_ez = c[0] - c[1];
  const Vector3d gt_en = c[0] - c[2];
  EXPECT_GT(sol.b_ez.normalized().dot(gt_ez.normalized()), 1.0 - 1e-9);
  EXPECT_GT(sol.b_en.normalized().dot(gt_en.normalized()), 1.0 - 1e-9);
  EXPECT_NEAR(sol.b_en.norm() / sol.b_ez.norm(),
              gt_en.norm() / gt_ez.norm(), 1e-6 * gt_en.norm() / gt_ez.norm());
  EXPECT_LT(sol.eigen_ratio, 1e-6);
}

// (2) Noise-free, 10 cameras, 200 points: the positions are exact up to a
// similarity transformation.
TEST(ARTPositionEstimatorTest, NoNoiseTenViews) {
  RandomNumberGenerator rng(11);
  SyntheticScene scene;
  BuildScene(&rng, 10, 200, 0.0, &scene);

  ARTPositionEstimator::Options options;
  ARTPositionEstimator estimator(options, scene.reconstruction);
  std::unordered_map<ViewId, Vector3d> estimated;
  ASSERT_TRUE(estimator.EstimatePositions(scene.view_pairs, scene.orientations,
                                          &estimated));
  ASSERT_EQ(estimated.size(), scene.positions.size());

  AlignPositions(scene.positions, &estimated);
  for (const auto& position : scene.positions) {
    const Vector3d& est = estimated.at(position.first);
    EXPECT_LT((position.second - est).norm(), 1e-6)
        << "view " << position.first;
  }
}

// (3) 0.5 px noise, 20 cameras: the median position error after alignment is
// below 1% of the scene extent.
TEST(ARTPositionEstimatorTest, NoisyTwentyViews) {
  RandomNumberGenerator rng(23);
  SyntheticScene scene;
  BuildScene(&rng, 20, 200, 0.5, &scene);

  ARTPositionEstimator::Options options;
  ARTPositionEstimator estimator(options, scene.reconstruction);
  std::unordered_map<ViewId, Vector3d> estimated;
  ASSERT_TRUE(estimator.EstimatePositions(scene.view_pairs, scene.orientations,
                                          &estimated));
  ASSERT_EQ(estimated.size(), scene.positions.size());

  AlignPositions(scene.positions, &estimated);
  std::vector<double> errors;
  for (const auto& position : scene.positions) {
    errors.push_back((position.second - estimated.at(position.first)).norm());
  }
  std::nth_element(errors.begin(), errors.begin() + errors.size() / 2,
                   errors.end());
  const double median_error = errors[errors.size() / 2];
  const double extent = SceneExtent(scene.positions);
  std::cout << "ART noisy test: median position error " << median_error
            << ", scene extent " << extent << ", relative "
            << median_error / extent << std::endl;
  EXPECT_LT(median_error, 0.01 * extent);
}

// (4) Degenerate triplet: all three camera centers identical. There is no
// baseline to recover, so the triplet must be rejected.
TEST(ARTTripletTest, RejectsDegenerateTriplet) {
  RandomNumberGenerator rng(5);
  Matrix3d R[3];
  const Vector3d center = 2.0 * rng.RandVector3d();
  for (int i = 0; i < 3; ++i) {
    R[i] = AngleAxisToMatrix(0.3 * rng.RandVector3d());
  }

  std::vector<Vector3d> x[3];
  while (x[0].size() < 30) {
    const Vector3d p(rng.RandDouble(-2.0, 2.0), rng.RandDouble(-2.0, 2.0),
                     rng.RandDouble(8.0, 12.0));
    const Vector3d X = center + R[0].transpose() * p;
    for (int i = 0; i < 3; ++i) {
      x[i].push_back(Ray(R[i], center, X));
    }
  }

  const std::vector<double> weights(x[0].size(), 1.0);
  ARTTripletSolution sol;
  EXPECT_FALSE(SolveARTTriplet(x[0], x[1], x[2], R[0], R[1], R[2], weights,
                               &sol));
}

}  // namespace theia
