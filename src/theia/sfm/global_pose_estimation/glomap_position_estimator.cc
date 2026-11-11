// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/global_pose_estimation/glomap_position_estimator.h"

#include <Eigen/Core>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <limits>
#include <queue>
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

// Residual enforcing c_view - c_anchor = delta for a rig constraint.
struct RigOffsetError {
  RigOffsetError(const Vector3d& delta, const double weight)
      : delta_(delta), weight_(weight) {}

  template <typename T>
  bool operator()(const T* view_position,
                  const T* anchor_position,
                  T* residuals) const {
    residuals[0] = T(weight_) *
                   (view_position[0] - anchor_position[0] - T(delta_.x()));
    residuals[1] = T(weight_) *
                   (view_position[1] - anchor_position[1] - T(delta_.y()));
    residuals[2] = T(weight_) *
                   (view_position[2] - anchor_position[2] - T(delta_.z()));
    return true;
  }

  static ceres::CostFunction* Create(const Vector3d& delta,
                                     const double weight) {
    return new ceres::AutoDiffCostFunction<RigOffsetError, 3, 3, 3>(
        new RigOffsetError(delta, weight));
  }

  const Vector3d delta_;
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

// Initial position for the scale-prior problem: chain the metric displacements
// (scale * translation direction for scale-prior edges, rig offsets for rig
// constraints) along a breadth-first tree rooted at |root|. Starting from the
// metric chain instead of random positions avoids converging to a shrunken
// solution. Views that are not reached keep their current position.
void InitializePositionsFromMetricChain(
    const std::unordered_map<ViewIdPair, TwoViewInfo>& view_pairs,
    const std::unordered_map<ViewId, Vector3d>& orientations,
    const std::vector<std::pair<ViewId, RigOffsetConstraint>>& rig_constraints,
    const ViewId root,
    std::unordered_map<ViewId, Vector3d>* positions) {
  // Edge (other, displacement): position(other) = position(self) + displacement.
  std::unordered_map<ViewId, std::vector<std::pair<ViewId, Vector3d>>> edges;
  for (const auto& view_pair : view_pairs) {
    const ViewId first = view_pair.first.first;
    const ViewId second = view_pair.first.second;
    if (view_pair.second.scale_estimate <= 0.0 ||
        !ContainsKey(*positions, first) || !ContainsKey(*positions, second) ||
        !ContainsKey(orientations, first)) {
      continue;
    }
    const Vector3d displacement =
        view_pair.second.scale_estimate *
        RotateRelativeTranslationIntoWorld(FindOrDie(orientations, first),
                                           view_pair.second.position_2);
    edges[first].emplace_back(second, displacement);
    edges[second].emplace_back(first, -displacement);
  }
  for (const auto& constraint : rig_constraints) {
    const ViewId view_id = constraint.first;
    const RigOffsetConstraint& rig = constraint.second;
    edges[rig.anchor].emplace_back(view_id, rig.delta);
    edges[view_id].emplace_back(rig.anchor, -rig.delta);
  }

  std::unordered_set<ViewId> visited;
  std::queue<ViewId> frontier;
  visited.insert(root);
  frontier.push(root);
  while (!frontier.empty()) {
    const ViewId current = frontier.front();
    frontier.pop();
    const auto it = edges.find(current);
    if (it == edges.end()) {
      continue;
    }
    const Vector3d current_position = FindOrDie(*positions, current);
    for (const auto& edge : it->second) {
      if (visited.count(edge.first) > 0) {
        continue;
      }
      visited.insert(edge.first);
      (*positions)[edge.first] = current_position + edge.second;
      frontier.push(edge.first);
    }
  }
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
  CHECK_GT(options_.rig_constraint_weight, 0.0);

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
    (*positions)[view_id] = rng_->RandVector3d(-1.0, 1.0);
  }

  // Rig constraints: both the constrained view and its anchor must be
  // optimized, even if they have few (or no) feature observations.
  std::vector<std::pair<ViewId, RigOffsetConstraint>> valid_rig_constraints;
  for (const auto& constraint : rig_constraints_) {
    const ViewId view_id = constraint.first;
    const RigOffsetConstraint& rig = constraint.second;
    if (view_id == rig.anchor || reconstruction_->View(view_id) == nullptr ||
        reconstruction_->View(rig.anchor) == nullptr) {
      continue;
    }
    valid_rig_constraints.push_back(constraint);
  }
  for (const auto& constraint : valid_rig_constraints) {
    if (!ContainsKey(*positions, constraint.first) &&
        !ContainsKey(*positions, constraint.second.anchor)) {
      (*positions)[constraint.second.anchor] = rng_->RandVector3d(-1.0, 1.0);
    }
  }
  for (const auto& constraint : valid_rig_constraints) {
    const ViewId view_id = constraint.first;
    const RigOffsetConstraint& rig = constraint.second;
    if (!ContainsKey(*positions, view_id)) {
      (*positions)[view_id] =
          FindOrDie(*positions, rig.anchor) + rig.delta;
    } else if (!ContainsKey(*positions, rig.anchor)) {
      (*positions)[rig.anchor] = FindOrDie(*positions, view_id) - rig.delta;
    }
  }

  const ViewId fixed_view_id = SmallestViewId(*positions);
  FindOrDie(*positions, fixed_view_id).setZero();
  if (options_.use_pairwise_scale_priors) {
    InitializePositionsFromMetricChain(
        view_pairs, orientations, valid_rig_constraints, fixed_view_id,
        positions);
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

  // Hard (high-weight, no robust loss) rig offset constraints. No gauge
  // normalization of the output is performed in this estimator, so the metric
  // scale fixed by these constraints is preserved.
  for (const auto& constraint : valid_rig_constraints) {
    ceres::CostFunction* cost_function = RigOffsetError::Create(
        constraint.second.delta, options_.rig_constraint_weight);
    problem.AddResidualBlock(
        cost_function,
        nullptr,
        FindOrDie(*positions, constraint.first).data(),
        FindOrDie(*positions, constraint.second.anchor).data());
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
