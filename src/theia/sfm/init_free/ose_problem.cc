// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/ose_problem.h"

#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace theia {

namespace {

// Rays this close to the image plane (z ~ 0) are dropped: their normalized
// coordinates blow up and they carry no useful OSE information.
constexpr double kMinRayDepth = 1e-6;

bool NormalizeObservation(const Camera& camera,
                          const Eigen::Matrix3d& normalization,
                          const OseObservationNormalization type,
                          const Eigen::Vector2d& pixel,
                          Eigen::Vector2d* m) {
  if (type == OseObservationNormalization::CALIBRATED_RAYS) {
    const Eigen::Vector3d ray = camera.PixelToNormalizedCoordinates(pixel);
    if (!ray.allFinite() || std::abs(ray.z()) < kMinRayDepth) {
      return false;
    }
    *m = ray.hnormalized();
  } else {
    *m = (normalization * pixel.homogeneous()).hnormalized();
  }
  return m->allFinite();
}

Eigen::Matrix3d IsotropicNormalization(const Camera& camera) {
  const double w = camera.ImageWidth();
  const double h = camera.ImageHeight();
  const double scale = 2.0 / std::max(w, h);
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  T(0, 0) = scale;
  T(1, 1) = scale;
  T(0, 2) = -scale * 0.5 * w;
  T(1, 2) = -scale * 0.5 * h;
  return T;
}

}  // namespace

bool BuildOseProblem(const Reconstruction& reconstruction,
                     const ViewGraph* view_graph,
                     const InitFreeOptions& options,
                     OseProblem* problem,
                     std::string* error) {
  CHECK_NOTNULL(problem);
  CHECK_NOTNULL(error);
  *problem = OseProblem();
  problem->normalization = options.observation_normalization;

  const bool needs_rotations =
      options.formulation == OseFormulationType::POSE_ROT;
  if (needs_rotations && options.observation_normalization !=
                             OseObservationNormalization::CALIBRATED_RAYS) {
    *error = "POSE_ROT needs CALIBRATED_RAYS observations.";
    return false;
  }
  if (needs_rotations && view_graph == nullptr) {
    *error = "POSE_ROT needs a view graph with relative rotations.";
    return false;
  }
  if (options.min_track_length < 2) {
    *error = "min_track_length must be at least 2.";
    return false;
  }
  if (options.observation_normalization ==
      OseObservationNormalization::PIXEL_ISOTROPIC) {
    for (const ViewId view_id : reconstruction.ViewIds()) {
      const Camera& camera = reconstruction.View(view_id)->Camera();
      if (camera.ImageWidth() <= 0 || camera.ImageHeight() <= 0) {
        *error = "PIXEL_ISOTROPIC needs the image size of every view.";
        return false;
      }
    }
  }

  // Candidate views: all views, or the views of the view graph.
  std::unordered_set<ViewId> active_views;
  if (view_graph != nullptr && view_graph->NumViews() > 0) {
    active_views = view_graph->ViewIds();
  } else {
    for (const ViewId view_id : reconstruction.ViewIds()) {
      active_views.insert(view_id);
    }
  }

  // Alternate between dropping short tracks and weakly observed views until
  // nothing changes. Usually converges in one or two passes.
  std::vector<TrackId> all_track_ids = reconstruction.TrackIds();
  std::sort(all_track_ids.begin(), all_track_ids.end());
  std::vector<TrackId> active_tracks;
  for (int pass = 0; pass < 10; ++pass) {
    active_tracks.clear();
    std::unordered_map<ViewId, int> view_counts;
    for (const TrackId track_id : all_track_ids) {
      const Track* track = reconstruction.Track(track_id);
      int count = 0;
      for (const ViewId view_id : track->ViewIds()) {
        count += active_views.count(view_id);
      }
      if (count < options.min_track_length) {
        continue;
      }
      active_tracks.push_back(track_id);
      for (const ViewId view_id : track->ViewIds()) {
        if (active_views.count(view_id)) {
          ++view_counts[view_id];
        }
      }
    }
    std::unordered_set<ViewId> next_views;
    for (const auto& [view_id, count] : view_counts) {
      if (count >= options.min_observations_per_view) {
        next_views.insert(view_id);
      }
    }
    if (next_views.size() == active_views.size()) {
      break;
    }
    active_views = std::move(next_views);
  }

  if (active_views.size() < 2 || active_tracks.empty()) {
    *error = "Not enough views or tracks after filtering.";
    return false;
  }

  // Camera numbering in ascending ViewId order for reproducibility.
  problem->view_ids.assign(active_views.begin(), active_views.end());
  std::sort(problem->view_ids.begin(), problem->view_ids.end());
  std::unordered_map<ViewId, int> camera_index;
  for (int i = 0; i < problem->NumCameras(); ++i) {
    const ViewId view_id = problem->view_ids[i];
    camera_index[view_id] = i;
    problem->normalizations.push_back(
        options.observation_normalization ==
                OseObservationNormalization::PIXEL_ISOTROPIC
            ? IsotropicNormalization(reconstruction.View(view_id)->Camera())
            : Eigen::Matrix3d::Identity());
  }

  // Observations, grouped by point. A track is kept only if enough of its
  // observations survive normalization.
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      measurements;
  problem->point_offsets.push_back(0);
  std::vector<std::pair<int, Eigen::Vector2d>,
              Eigen::aligned_allocator<std::pair<int, Eigen::Vector2d>>>
      track_observations;
  for (const TrackId track_id : active_tracks) {
    const Track* track = reconstruction.Track(track_id);
    track_observations.clear();
    for (const ViewId view_id : track->ViewIds()) {
      const auto it = camera_index.find(view_id);
      if (it == camera_index.end()) {
        continue;
      }
      const View* view = reconstruction.View(view_id);
      const Feature* feature = view->GetFeature(track_id);
      if (feature == nullptr) {
        continue;
      }
      Eigen::Vector2d m;
      if (!NormalizeObservation(view->Camera(),
                                problem->normalizations[it->second],
                                options.observation_normalization,
                                feature->point_,
                                &m)) {
        continue;
      }
      track_observations.emplace_back(it->second, m);
    }
    if (static_cast<int>(track_observations.size()) <
        options.min_track_length) {
      continue;
    }
    std::sort(track_observations.begin(),
              track_observations.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    const int point = problem->NumPoints();
    problem->track_ids.push_back(track_id);
    for (const auto& [camera, m] : track_observations) {
      problem->observation_camera.push_back(camera);
      problem->observation_point.push_back(point);
      measurements.push_back(m);
    }
    problem->point_offsets.push_back(
        static_cast<int>(problem->observation_camera.size()));
  }

  if (problem->NumPoints() == 0) {
    *error = "No track has enough valid observations.";
    return false;
  }
  problem->observations.resize(2, measurements.size());
  for (size_t k = 0; k < measurements.size(); ++k) {
    problem->observations.col(k) = measurements[k];
  }

  if (needs_rotations) {
    // TwoViewInfo convention: R_second = R_rel * R_first, so the term for the
    // pair is A_second * A_first^T ~ R_rel.
    std::vector<ViewIdPair> pairs;
    for (const auto& [pair, info] : view_graph->GetAllEdges()) {
      pairs.push_back(pair);
    }
    std::sort(pairs.begin(), pairs.end());
    for (const ViewIdPair& pair : pairs) {
      const auto first = camera_index.find(pair.first);
      const auto second = camera_index.find(pair.second);
      if (first == camera_index.end() || second == camera_index.end()) {
        continue;
      }
      const TwoViewInfo* info = view_graph->GetEdge(pair.first, pair.second);
      OseRotationTerm term;
      term.camera_a = second->second;
      term.camera_b = first->second;
      ceres::AngleAxisToRotationMatrix(
          info->rotation_2.data(),
          ceres::ColumnMajorAdapter3x3(term.relative_rotation.data()));
      // TODO(init-free): Olsson & Nilsson weight each term with a 9x9 matrix
      // W_kl derived from the two-view reprojection Hessian. A scalar weight
      // is used for now.
      term.weight = options.rotation_weight;
      problem->rotation_terms.push_back(term);
    }
    if (problem->rotation_terms.empty()) {
      *error = "POSE_ROT: no view graph edge connects two problem cameras.";
      return false;
    }
    if (options.include_self_rotation_terms) {
      for (int i = 0; i < problem->NumCameras(); ++i) {
        OseRotationTerm term;
        term.camera_a = i;
        term.camera_b = i;
        term.weight = options.rotation_weight;
        problem->rotation_terms.push_back(term);
      }
    }
  }

  VLOG(1) << "OseProblem: " << problem->NumCameras() << " cameras, "
          << problem->NumPoints() << " points, " << problem->NumObservations()
          << " observations, " << problem->rotation_terms.size()
          << " rotation terms.";
  return true;
}

}  // namespace theia
