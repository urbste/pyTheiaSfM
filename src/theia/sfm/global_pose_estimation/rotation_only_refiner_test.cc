// Tests of the rotation-only refiner (MORE, Li, Cai, Wu, arXiv:2611.11770,
// Sec. 5.2 Eq. 21-23 and Alg. 2).

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/global_pose_estimation/art_position_estimator.h"
#include "theia/sfm/global_pose_estimation/ligt_constraints.h"
#include "theia/sfm/global_pose_estimation/rotation_only_refiner.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
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

// Angle (degrees) of the relative rotation between two rotation matrices.
double AngleDeg(const Matrix3d& a, const Matrix3d& b) {
  const Matrix3d delta = a.transpose() * b;
  return Eigen::AngleAxisd(delta).angle() * 180.0 / M_PI;
}

// Global rotation H that best aligns the estimated rotations E_v to the ground
// truth G_v (E_v H ~ G_v), and the median of the per-view angular errors
// (degrees) after alignment.
double AlignedMedianErrorDeg(
    const std::unordered_map<ViewId, Matrix3d>& estimated,
    const std::unordered_map<ViewId, Matrix3d>& ground_truth) {
  Matrix3d M = Matrix3d::Zero();
  for (const auto& e : estimated) {
    M += e.second.transpose() * ground_truth.at(e.first);
  }
  Eigen::JacobiSVD<Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d V = svd.matrixV();
  Matrix3d H = svd.matrixU() * V.transpose();
  if (H.determinant() < 0.0) {
    V.col(2) *= -1.0;
    H = svd.matrixU() * V.transpose();
  }
  std::vector<double> errors;
  for (const auto& e : estimated) {
    errors.push_back(AngleDeg(e.second * H, ground_truth.at(e.first)));
  }
  std::sort(errors.begin(), errors.end());
  const size_t mid = errors.size() / 2;
  return errors.size() % 2 == 1 ? errors[mid]
                                : 0.5 * (errors[mid - 1] + errors[mid]);
}

struct SyntheticScene {
  Reconstruction reconstruction;
  std::unordered_map<ViewId, Vector3d> positions;
  std::unordered_map<ViewId, Vector3d> orientations;  // ground truth
  std::unordered_map<ViewIdPair, TwoViewInfo> view_pairs;
};

// Sampson distance (pixels) of the correspondence (pixel_a, pixel_b) under the
// ground-truth epipolar geometry of two views (world-to-camera rotations R and
// camera centers c). The test cameras have focal 800 and principal point 500.
double GroundTruthSampsonPx(const Eigen::Vector2d& pixel_a,
                            const Matrix3d& R_a, const Vector3d& c_a,
                            const Eigen::Vector2d& pixel_b,
                            const Matrix3d& R_b, const Vector3d& c_b) {
  const double f = 800.0;
  const double pp = 500.0;
  const Vector3d x_a((pixel_a.x() - pp) / f, (pixel_a.y() - pp) / f, 1.0);
  const Vector3d x_b((pixel_b.x() - pp) / f, (pixel_b.y() - pp) / f, 1.0);
  // X_b = R_ab X_a + t_ab with R_ab = R_b R_a^T and t_ab = R_b (c_a - c_b).
  const Matrix3d R_ab = R_b * R_a.transpose();
  const Vector3d t = R_b * (c_a - c_b);
  Matrix3d skew;
  skew << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;
  const Matrix3d E = skew * R_ab;
  const Vector3d Ex = E * x_a;
  const Vector3d Etx = E.transpose() * x_b;
  const double num = x_b.dot(Ex);
  const double den = Ex.x() * Ex.x() + Ex.y() * Ex.y() + Etx.x() * Etx.x() +
                     Etx.y() * Etx.y();
  return std::abs(num) / std::sqrt(den) * f;
}

// Random cameras looking at a cloud of points in front of them. Every point is
// observed in every view, with Gaussian-like pixel noise of `image_noise_px`
// and a fraction `outlier_fraction` of the observations replaced by random
// pixels.
void BuildScene(RandomNumberGenerator* rng,
                const int num_views,
                const int num_points,
                const double image_noise_px,
                const double outlier_fraction,
                SyntheticScene* scene,
                const bool verify_outliers = false) {
  std::vector<ViewId> view_ids;
  for (int i = 0; i < num_views; ++i) {
    const ViewId view_id =
        scene->reconstruction.AddView(std::to_string(i), 0, i);
    view_ids.push_back(view_id);
    Camera camera;
    camera.SetPosition(4.0 * rng->RandVector3d());
    camera.SetOrientationFromAngleAxis(0.3 * rng->RandVector3d());
    camera.SetImageSize(1000, 1000);
    camera.SetFocalLength(800);
    camera.SetPrincipalPoint(500.0, 500.0);
    *scene->reconstruction.MutableView(view_id)->MutableCamera() = camera;
    scene->orientations[view_id] = camera.GetOrientationAsAngleAxis();
    scene->positions[view_id] = camera.GetPosition();
  }

  for (int p = 0; p < num_points; ++p) {
    Vector4d point;
    point.head<3>() = rng->RandVector3d(-2.0, 2.0);
    point(2) += 20.0;
    point(3) = 1.0;
    std::vector<std::pair<ViewId, Feature>> features;
    std::vector<bool> is_outlier;
    for (const ViewId view_id : view_ids) {
      Eigen::Vector2d pixel;
      scene->reconstruction.View(view_id)->Camera().ProjectPoint(point, &pixel);
      AddNoiseToProjection(image_noise_px, rng, &pixel);
      is_outlier.push_back(false);
      if (rng->RandDouble(0.0, 1.0) < outlier_fraction) {
        pixel = Eigen::Vector2d(rng->RandDouble(0.0, 1000.0),
                                rng->RandDouble(0.0, 1000.0));
        is_outlier.back() = true;
      }
      features.emplace_back(view_id, pixel);
    }
    if (verify_outliers) {
      // Simulates two-view verification at 2 px: an outlier observation is
      // dropped if its Sampson error under the ground-truth epipolar geometry
      // exceeds 2 px for any other view of the track. The outliers that remain
      // are consistent with the other views.
      std::vector<std::pair<ViewId, Feature>> verified;
      for (size_t a = 0; a < features.size(); ++a) {
        if (is_outlier[a]) {
          bool consistent = true;
          for (size_t b = 0; b < features.size() && consistent; ++b) {
            if (b == a) {
              continue;
            }
            const ViewId va = features[a].first;
            const ViewId vb = features[b].first;
            const double err = GroundTruthSampsonPx(
                features[a].second.point_, AngleAxisToMatrix(scene->orientations.at(va)),
                scene->positions.at(va),
                features[b].second.point_, AngleAxisToMatrix(scene->orientations.at(vb)),
                scene->positions.at(vb));
            consistent = err <= 2.0;
          }
          if (!consistent) {
            continue;
          }
        }
        verified.push_back(features[a]);
      }
      features.swap(verified);
    }
    if (features.size() < 2) {
      continue;
    }
    Track* track =
        scene->reconstruction.MutableTrack(scene->reconstruction.AddTrack(features));
    *track->MutablePoint() = point;
  }

  for (size_t i = 0; i < view_ids.size(); ++i) {
    for (size_t j = i + 1; j < view_ids.size(); ++j) {
      scene->view_pairs[ViewIdPair(view_ids[i], view_ids[j])] = TwoViewInfo();
    }
  }
}

std::unordered_map<ViewId, Matrix3d> ToMatrices(
    const std::unordered_map<ViewId, Vector3d>& angle_axes) {
  std::unordered_map<ViewId, Matrix3d> matrices;
  for (const auto& entry : angle_axes) {
    matrices[entry.first] = AngleAxisToMatrix(entry.second);
  }
  return matrices;
}

// Perturbs every orientation by a random rotation of `angle_deg` degrees.
std::unordered_map<ViewId, Vector3d> PerturbOrientations(
    RandomNumberGenerator* rng,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    const double angle_deg) {
  std::unordered_map<ViewId, Vector3d> perturbed;
  for (const auto& entry : orientations) {
    const Vector3d axis = rng->RandVector3d().normalized();
    const Matrix3d delta =
        AngleAxisToMatrix(axis * angle_deg * M_PI / 180.0);
    Matrix3d R = delta * AngleAxisToMatrix(entry.second);
    Eigen::AngleAxisd aa(R);
    perturbed[entry.first] = aa.angle() * aa.axis();
  }
  return perturbed;
}

}  // namespace

// (1) Ground-truth rotations and noise-free features: the refinement keeps the
// rotations.
TEST(RotationOnlyRefinerTest, GroundTruthIsFixedPoint) {
  RandomNumberGenerator rng(3);
  SyntheticScene scene;
  BuildScene(&rng, 10, 100, 0.0, 0.0, &scene);

  std::unordered_map<ViewId, Vector3d> orientations = scene.orientations;
  RotationOnlyRefinerOptions options;
  ASSERT_TRUE(RefineRotationsWithRotationOnlyError(
      options, scene.reconstruction, scene.view_pairs, &orientations));

  const auto gt = ToMatrices(scene.orientations);
  const auto estimated = ToMatrices(orientations);
  const double error = AlignedMedianErrorDeg(estimated, gt);
  std::cout << "GT fixed point: median error after alignment = " << error
            << " deg\n";
  EXPECT_LT(error, 1e-6);
  for (const auto& entry : estimated) {
    EXPECT_LT(AngleDeg(entry.second, gt.at(entry.first)), 1e-6);
  }
}

// (2) Noisy features (0.5 px), 1 degree rotation noise: the refinement halves
// the median rotation error or better.
TEST(RotationOnlyRefinerTest, ReducesRotationErrorWithNoise) {
  RandomNumberGenerator rng(17);
  SyntheticScene scene;
  BuildScene(&rng, 20, 300, 0.5, 0.0, &scene);
  const auto perturbed = PerturbOrientations(&rng, scene.orientations, 1.0);

  const auto gt = ToMatrices(scene.orientations);
  const double before = AlignedMedianErrorDeg(ToMatrices(perturbed), gt);

  std::unordered_map<ViewId, Vector3d> orientations = perturbed;
  RotationOnlyRefinerOptions options;
  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(RefineRotationsWithRotationOnlyError(
      options, scene.reconstruction, scene.view_pairs, &orientations));
  const auto stop = std::chrono::steady_clock::now();
  const double seconds =
      std::chrono::duration<double>(stop - start).count();

  const double after = AlignedMedianErrorDeg(ToMatrices(orientations), gt);
  std::cout << "Noisy scene: median rotation error before = " << before
            << " deg, after = " << after << " deg, refinement runtime = "
            << seconds << " s\n";
  EXPECT_LT(after, 0.5 * before);
}

// (3) As (2), with 10% of the observations replaced by random pixels that
// pass a two-view verification at 2 px (see BuildScene): the refinement
// improves the median rotation error, by more than a factor two.
//
// Why verified outliers: the random-pixel model below (DISABLED_) is not what
// the pipeline feeds MORE; it receives RANSAC-verified tracks. With random
// pixels, the refinement converges to a wrong minimum (see the comment there).
TEST(RotationOnlyRefinerTest, ImprovesRotationErrorWithVerifiedOutliers) {
  RandomNumberGenerator rng(29);
  SyntheticScene scene;
  BuildScene(&rng, 20, 300, 0.5, 0.1, &scene, /*verify_outliers=*/true);
  const auto perturbed = PerturbOrientations(&rng, scene.orientations, 1.0);

  const auto gt = ToMatrices(scene.orientations);
  const double before = AlignedMedianErrorDeg(ToMatrices(perturbed), gt);

  std::unordered_map<ViewId, Vector3d> orientations = perturbed;
  RotationOnlyRefinerOptions options;
  ASSERT_TRUE(RefineRotationsWithRotationOnlyError(
      options, scene.reconstruction, scene.view_pairs, &orientations));

  const double after = AlignedMedianErrorDeg(ToMatrices(orientations), gt);
  std::cout << "Verified-outlier scene (10%): median rotation error before = "
            << before << " deg, after = " << after << " deg\n";
  EXPECT_LT(after, before);
  EXPECT_LT(after, 0.5 * before);
}

// Random-pixel outliers without two-view verification. The epipolar
// verification of the refiner (RotationOnlyRefinerOptions::
// verify_observations_px) removes them; without it the outliers enter the
// Theorem 1 solves and the error grows (measured: 1.00 deg -> 1.23 deg). With
// it: 1.00 deg -> 0.13 deg, the same as the verified-outlier scene.
TEST(RotationOnlyRefinerTest, ImprovesRotationErrorWithRandomOutliers) {
  RandomNumberGenerator rng(29);
  SyntheticScene scene;
  BuildScene(&rng, 20, 300, 0.5, 0.1, &scene);
  const auto perturbed = PerturbOrientations(&rng, scene.orientations, 1.0);

  const auto gt = ToMatrices(scene.orientations);
  const double before = AlignedMedianErrorDeg(ToMatrices(perturbed), gt);

  std::unordered_map<ViewId, Vector3d> orientations = perturbed;
  RotationOnlyRefinerOptions options;
  ASSERT_TRUE(RefineRotationsWithRotationOnlyError(
      options, scene.reconstruction, scene.view_pairs, &orientations));

  const double after = AlignedMedianErrorDeg(ToMatrices(orientations), gt);
  std::cout << "Random-outlier scene (10%): median rotation error before = "
            << before << " deg, after = " << after << " deg\n";
  EXPECT_LT(after, 0.5 * before);
}

// Rays of the tracks seen in the three views (sorted view ids) of a scene.
void TripletRays(const SyntheticScene& scene,
                 const std::vector<ViewId>& ids,
                 std::vector<Vector3d>* x_i,
                 std::vector<Vector3d>* x_l,
                 std::vector<Vector3d>* x_j) {
  const View* vi = scene.reconstruction.View(ids[0]);
  const View* vl = scene.reconstruction.View(ids[1]);
  const View* vj = scene.reconstruction.View(ids[2]);
  for (const TrackId track_id : scene.reconstruction.TrackIds()) {
    if (vi->GetFeature(track_id) == nullptr ||
        vl->GetFeature(track_id) == nullptr ||
        vj->GetFeature(track_id) == nullptr) {
      continue;
    }
    x_i->push_back(GetNormalizedFeatureRay(*vi, track_id));
    x_l->push_back(GetNormalizedFeatureRay(*vl, track_id));
    x_j->push_back(GetNormalizedFeatureRay(*vj, track_id));
  }
}

std::vector<ViewId> SortedViewIds(const SyntheticScene& scene) {
  std::vector<ViewId> ids;
  for (const auto& entry : scene.orientations) {
    ids.push_back(entry.first);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

// (4) Autodiff Jacobian of one triplet cost (Theorem 1 inside the residual)
// against central finite differences.
TEST(RotationOnlyRefinerTest, TripletCostGradientMatchesFiniteDifferences) {
  RandomNumberGenerator rng(41);
  SyntheticScene scene;
  BuildScene(&rng, 3, 60, 0.5, 0.0, &scene);
  const std::vector<ViewId> ids = SortedViewIds(scene);
  const auto perturbed = PerturbOrientations(&rng, scene.orientations, 1.0);

  std::vector<Vector3d> x_i, x_l, x_j;
  TripletRays(scene, ids, &x_i, &x_l, &x_j);
  ASSERT_GT(x_i.size(), 10u);

  const Vector3d a[3] = {perturbed.at(ids[0]), perturbed.at(ids[1]),
                         perturbed.at(ids[2])};
  Vector3d b_il, b_ij;
  ASSERT_TRUE(rotation_only_internal::ComputeTheorem1Baselines(
      x_i, x_l, x_j, a[0], a[1], a[2], Vector3d(1.0, 2.0, 3.0).normalized(),
      &b_il, &b_ij));
  std::unique_ptr<ceres::CostFunction> cost(
      rotation_only_internal::CreateTripletCostFunctionForTest(x_i, x_l, x_j,
                                                               b_il));
  const int num_residuals = cost->num_residuals();
  ASSERT_EQ(num_residuals, 4 * static_cast<int>(x_i.size()));

  const double* params[3] = {a[0].data(), a[1].data(), a[2].data()};
  std::vector<double> residual(num_residuals);
  std::vector<std::vector<double>> jacobian(
      3, std::vector<double>(num_residuals * 3));
  double* jacobians[3] = {jacobian[0].data(), jacobian[1].data(),
                          jacobian[2].data()};
  ASSERT_TRUE(cost->Evaluate(params, residual.data(), jacobians));

  const double h = 1e-6;
  double max_error = 0.0;
  for (int b = 0; b < 3; ++b) {
    for (int c = 0; c < 3; ++c) {
      Vector3d plus[3] = {a[0], a[1], a[2]};
      Vector3d minus[3] = {a[0], a[1], a[2]};
      plus[b](c) += h;
      minus[b](c) -= h;
      const double* pp[3] = {plus[0].data(), plus[1].data(), plus[2].data()};
      const double* pm[3] = {minus[0].data(), minus[1].data(),
                             minus[2].data()};
      std::vector<double> rp(num_residuals), rm(num_residuals);
      ASSERT_TRUE(cost->Evaluate(pp, rp.data(), nullptr));
      ASSERT_TRUE(cost->Evaluate(pm, rm.data(), nullptr));
      for (int r = 0; r < num_residuals; ++r) {
        const double fd = (rp[r] - rm[r]) / (2.0 * h);
        const double ad = jacobian[b][r * 3 + c];
        const double err = std::abs(ad - fd) / std::max(1.0, std::abs(fd));
        max_error = std::max(max_error, err);
      }
    }
  }
  std::cout << "Triplet cost gradient: max relative error vs finite differences = "
            << max_error << " (" << num_residuals << " residuals)\n";
  EXPECT_LT(max_error, 1e-5);
}

// (5) The templated Theorem 1 at double precision equals SolveARTTriplet.
TEST(RotationOnlyRefinerTest, Theorem1TemplatedMatchesSolveARTTriplet) {
  RandomNumberGenerator rng(43);
  SyntheticScene scene;
  BuildScene(&rng, 3, 60, 0.5, 0.0, &scene);
  const std::vector<ViewId> ids = SortedViewIds(scene);
  const auto perturbed = PerturbOrientations(&rng, scene.orientations, 1.0);

  std::vector<Vector3d> x_i, x_l, x_j;
  TripletRays(scene, ids, &x_i, &x_l, &x_j);
  ASSERT_GT(x_i.size(), 10u);

  const Vector3d a_i = perturbed.at(ids[0]);
  const Vector3d a_l = perturbed.at(ids[1]);
  const Vector3d a_j = perturbed.at(ids[2]);
  const std::vector<double> weights(x_i.size(), 1.0);
  ARTTripletSolution sol;
  ASSERT_TRUE(SolveARTTriplet(x_i, x_l, x_j, AngleAxisToMatrix(a_i),
                              AngleAxisToMatrix(a_l), AngleAxisToMatrix(a_j),
                              weights, &sol));

  for (const double sign : {1.0, -1.0}) {
    Vector3d b_il, b_ij;
    ASSERT_TRUE(rotation_only_internal::ComputeTheorem1Baselines(
        x_i, x_l, x_j, a_i, a_l, a_j, sign * sol.b_ez, &b_il, &b_ij));
    const double err_il = (b_il - sign * sol.b_ez).norm();
    const double err_ij = (b_ij - sign * sol.b_en).norm();
    std::cout << "Theorem 1 (sign " << sign << "): |b_il - ART| = " << err_il
              << ", |b_ij - ART| = " << err_ij << "\n";
    EXPECT_LT(err_il, 1e-9);
    EXPECT_LT(err_ij, 1e-9);
  }
}

}  // namespace theia
