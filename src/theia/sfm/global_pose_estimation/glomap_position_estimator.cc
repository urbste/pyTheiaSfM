// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/global_pose_estimation/glomap_position_estimator.h"

#include <Eigen/Core>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/global_pose_estimation/pairwise_translation_error.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"
#include "theia/util/random.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

struct GlomapRayError {
  GlomapRayError(const Vector3d& ray, const double weight)
      : ray_(ray), weight_(weight) {}

  template <typename T>
  bool operator()(const T* camera_position,
                  const T* point,
                  const T* inverse_depth,
                  T* residuals) const {
    residuals[0] = T(weight_) *
                   (T(ray_.x()) -
                    inverse_depth[0] * (point[0] - camera_position[0]));
    residuals[1] = T(weight_) *
                   (T(ray_.y()) -
                    inverse_depth[0] * (point[1] - camera_position[1]));
    residuals[2] = T(weight_) *
                   (T(ray_.z()) -
                    inverse_depth[0] * (point[2] - camera_position[2]));
    return true;
  }

  static ceres::CostFunction* Create(const Vector3d& ray,
                                     const double weight) {
    return new ceres::AutoDiffCostFunction<GlomapRayError, 3, 3, 3, 1>(
        new GlomapRayError(ray, weight));
  }

  const Vector3d ray_;
  const double weight_;
};

struct TrackObservation {
  ViewId view_id;
  TrackId track_id;
  Vector3d ray;
};

Matrix3d AngleAxisToRotationMatrix(const Vector3d& angle_axis) {
  Matrix3d rotation;
  ceres::AngleAxisToRotationMatrix(
      angle_axis.data(), ceres::ColumnMajorAdapter3x3(rotation.data()));
  return rotation;
}

Vector3d RotateRelativeTranslationIntoWorld(const Vector3d& orientation,
                                            const Vector3d& translation) {
  return AngleAxisToRotationMatrix(orientation).transpose() * translation;
}

Vector3d WorldRayFromFeature(const Camera& camera,
                             const Vector3d& orientation,
                             const Feature& feature) {
  Camera temp_camera = camera;
  temp_camera.SetOrientationFromAngleAxis(orientation);
  return temp_camera.PixelToUnitDepthRay(feature.point_).normalized();
}

ViewId SmallestViewId(
    const std::unordered_map<ViewId, Vector3d>& positions) {
  ViewId smallest_view_id = kInvalidViewId;
  for (const auto& position : positions) {
    if (smallest_view_id == kInvalidViewId ||
        position.first < smallest_view_id) {
      smallest_view_id = position.first;
    }
  }
  return smallest_view_id;
}

}  // namespace

GlomapPositionEstimator::GlomapPositionEstimator(
    const Options& options, Reconstruction* reconstruction)
    : options_(options), reconstruction_(CHECK_NOTNULL(reconstruction)) {
  CHECK_GT(options_.num_threads, 0);
  CHECK_GT(options_.max_num_iterations, 0);
  CHECK_GT(options_.robust_loss_width, 0.0);
  CHECK_GE(options_.min_track_length, 2);
  CHECK_GT(options_.pairwise_scale_prior_weight, 0.0);

  if (options_.rng.get() == nullptr) {
    rng_ = std::make_shared<RandomNumberGenerator>();
  } else {
    rng_ = options_.rng;
  }
}

bool GlomapPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    std::unordered_map<ViewId, Vector3d>* positions) {
  CHECK_NOTNULL(positions)->clear();
  if (orientations.empty()) {
    return false;
  }

  std::vector<TrackObservation> observations;
  std::unordered_map<TrackId, Vector3d> points;
  std::unordered_set<ViewId> constrained_views;
  const std::vector<TrackId> track_ids = reconstruction_->TrackIds();
  observations.reserve(track_ids.size() * options_.min_track_length);
  points.reserve(track_ids.size());

  for (const TrackId track_id : track_ids) {
    if (options_.max_num_tracks > 0 &&
        static_cast<int>(points.size()) >= options_.max_num_tracks) {
      break;
    }

    const Track* track = reconstruction_->Track(track_id);
    if (track == nullptr ||
        track->NumViews() < options_.min_track_length) {
      continue;
    }

    std::vector<TrackObservation> track_observations;
    track_observations.reserve(track->NumViews());
    for (const ViewId view_id : track->ViewIds()) {
      const View* view = reconstruction_->View(view_id);
      const Vector3d* orientation = FindOrNull(orientations, view_id);
      if (view == nullptr || orientation == nullptr) {
        continue;
      }
      const Feature* feature = view->GetFeature(track_id);
      if (feature == nullptr) {
        continue;
      }

      track_observations.push_back(
          {view_id,
           track_id,
           WorldRayFromFeature(view->Camera(), *orientation, *feature)});
    }

    if (track_observations.size() <
        static_cast<size_t>(options_.min_track_length)) {
      continue;
    }

    points[track_id] = rng_->RandVector3d(-1.0, 1.0);
    for (const TrackObservation& observation : track_observations) {
      constrained_views.insert(observation.view_id);
      observations.push_back(observation);
    }
  }

  if (points.empty() || constrained_views.size() < 2 || observations.empty()) {
    VLOG(2) << "Insufficient tracks or views for GLOMAP positioning.";
    return false;
  }

  positions->reserve(constrained_views.size());
  for (const ViewId view_id : constrained_views) {
    if (options_.initialize_from_reconstruction) {
      const View* view = reconstruction_->View(view_id);
      if (view != nullptr && view->IsEstimated()) {
        (*positions)[view_id] = view->Camera().GetPosition();
        continue;
      }
    }
    (*positions)[view_id] = rng_->RandVector3d(-1.0, 1.0);
  }

  const ViewId fixed_view_id = SmallestViewId(*positions);
  if (!options_.initialize_from_reconstruction) {
    FindOrDie(*positions, fixed_view_id).setZero();
  }

  std::vector<double> inverse_depths(observations.size(), 1.0);
  ceres::Problem problem;
  for (size_t i = 0; i < observations.size(); ++i) {
    const TrackObservation& observation = observations[i];
    ceres::CostFunction* cost_function =
        GlomapRayError::Create(observation.ray, 1.0);
    problem.AddResidualBlock(
        cost_function,
        new ceres::HuberLoss(options_.robust_loss_width),
        FindOrDie(*positions, observation.view_id).data(),
        FindOrDie(points, observation.track_id).data(),
        &inverse_depths[i]);
    problem.SetParameterLowerBound(&inverse_depths[i], 0, 0.0);
  }

  problem.SetParameterBlockConstant(
      FindOrDie(*positions, fixed_view_id).data());

  if (options_.use_pairwise_scale_priors) {
    for (const auto& view_pair : view_pairs) {
      if (view_pair.second.scale_estimate <= 0.0 ||
          !ContainsKey(*positions, view_pair.first.first) ||
          !ContainsKey(*positions, view_pair.first.second) ||
          !ContainsKey(orientations, view_pair.first.first)) {
        continue;
      }

      const Vector3d translation_direction =
          RotateRelativeTranslationIntoWorld(
              FindOrDie(orientations, view_pair.first.first),
              view_pair.second.position_2);
      ceres::CostFunction* cost_function = PairwiseTranslationError::Create(
          translation_direction,
          options_.pairwise_scale_prior_weight,
          view_pair.second.scale_estimate);
      problem.AddResidualBlock(
          cost_function,
          new ceres::HuberLoss(options_.robust_loss_width),
          FindOrDie(*positions, view_pair.first.first).data(),
          FindOrDie(*positions, view_pair.first.second).data());
    }
  }

  ceres::Solver::Options solver_options;
  solver_options.num_threads = options_.num_threads;
  solver_options.max_num_iterations = options_.max_num_iterations;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  VLOG(2) << summary.FullReport();
  if (!summary.IsSolutionUsable()) {
    return false;
  }

  if (options_.write_points_to_reconstruction) {
    for (const auto& point : points) {
      Track* track = reconstruction_->MutableTrack(point.first);
      if (track == nullptr) {
        continue;
      }
      *track->MutablePoint() = point.second.homogeneous();
      track->SetEstimated(true);
    }
  }

  return true;
}

std::unordered_map<ViewId, Vector3d>
GlomapPositionEstimator::EstimatePositionsWrapper(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations) {
  std::unordered_map<ViewId, Vector3d> positions;
  EstimatePositions(view_pairs, orientations, &positions);
  return positions;
}

}  // namespace theia
