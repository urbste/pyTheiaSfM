// Copyright (C) 2026 The pyTheiaSfM Authors.

#include "theia/sfm/rig/capture_view_graph.h"

#include <algorithm>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/estimators/estimate_relative_rig_info.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/pose/generalized_ray_correspondence.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/map_util.h"

namespace theia {
namespace {

Eigen::Matrix3d AngleAxisToMatrix(const Eigen::Vector3d& aa) {
  Eigen::Matrix3d R;
  ceres::AngleAxisToRotationMatrix(aa.data(),
                                   ceres::ColumnMajorAdapter3x3(R.data()));
  return R;
}

Eigen::Vector3d MatrixToAngleAxis(const Eigen::Matrix3d& R) {
  Eigen::Vector3d aa;
  ceres::RotationMatrixToAngleAxis(ceres::ColumnMajorAdapter3x3(R.data()),
                                   aa.data());
  return aa;
}

bool ViewEdgeToCaptureEdge(const RigSensor& sensor1,
                           const RigSensor& sensor2,
                           const TwoViewInfo& view_info,
                           TwoViewInfo* capture_info) {
  CHECK_NOTNULL(capture_info);
  const Eigen::Matrix3d R_s1 = sensor1.GetOrientationAsRotationMatrix();
  const Eigen::Matrix3d R_s2 = sensor2.GetOrientationAsRotationMatrix();

  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d c1 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R2 = AngleAxisToMatrix(view_info.rotation_2);
  const Eigen::Vector3d c2 = view_info.position_2;

  const Eigen::Matrix3d R_g1 = R_s1.transpose() * R1;
  const Eigen::Vector3d c_g1 = c1 - R_g1.transpose() * sensor1.position;
  const Eigen::Matrix3d R_g2 = R_s2.transpose() * R2;
  const Eigen::Vector3d c_g2 = c2 - R_g2.transpose() * sensor2.position;

  const Eigen::Matrix3d R_g2_rel = R_g2 * R_g1.transpose();
  const Eigen::Vector3d c_g2_rel = R_g1 * (c_g2 - c_g1);

  *capture_info = view_info;
  capture_info->rotation_2 = MatrixToAngleAxis(R_g2_rel);
  capture_info->position_2 = c_g2_rel;
  return true;
}

bool LiftFeatureToRigRay(const View& view,
                         const RigSensor& sensor,
                         const Feature& feature,
                         Eigen::Vector3d* origin,
                         Eigen::Vector3d* direction) {
  CHECK_NOTNULL(origin);
  CHECK_NOTNULL(direction);
  *origin = sensor.position;
  const Eigen::Vector3d bearing_cam =
      view.Camera().PixelToNormalizedCoordinates(feature.point_).normalized();
  if (bearing_cam.squaredNorm() < 1e-16) {
    return false;
  }
  *direction =
      sensor.GetOrientationAsRotationMatrix().transpose() * bearing_cam;
  return direction->squaredNorm() > 1e-16;
}

bool AddEdgeToCaptureGraph(const CaptureId c1,
                           const CaptureId c2,
                           const TwoViewInfo& capture_info,
                           ViewGraph* capture_view_graph,
                           int* num_added,
                           const bool only_if_missing) {
  const CaptureId a = std::min(c1, c2);
  const CaptureId b = std::max(c1, c2);
  TwoViewInfo stored = capture_info;
  if (c1 != a) {
    SwapCameras(&stored);
  }
  const TwoViewInfo* existing = capture_view_graph->GetEdge(a, b);
  if (existing != nullptr) {
    if (only_if_missing) {
      return false;
    }
    if (stored.num_verified_matches <= existing->num_verified_matches) {
      return false;
    }
  }
  capture_view_graph->AddEdge(a, b, stored);
  if (existing == nullptr) {
    ++(*num_added);
  }
  return true;
}

bool BuildCaptureViewGraphFromViewEdges(
    const Reconstruction& reconstruction,
    const ViewGraph& view_graph,
    ViewGraph* capture_view_graph,
    int* num_added,
    const bool only_if_missing) {
  const auto& edges = view_graph.GetAllEdges();
  for (const auto& edge : edges) {
    const ViewId view_id1 = edge.first.first;
    const ViewId view_id2 = edge.first.second;
    const TwoViewInfo& view_info = edge.second;

    const ViewRigMembership* m1 = reconstruction.GetViewRigMembership(view_id1);
    const ViewRigMembership* m2 = reconstruction.GetViewRigMembership(view_id2);
    if (m1 == nullptr || m2 == nullptr) {
      continue;
    }
    if (m1->capture_id == m2->capture_id) {
      continue;
    }
    if (m1->rig_id != m2->rig_id) {
      LOG(WARNING) << "Skipping edge between views on different rigs.";
      continue;
    }

    const CameraRig* rig = reconstruction.GetCameraRig(m1->rig_id);
    if (rig == nullptr) {
      continue;
    }
    const RigSensor* s1 = rig->GetSensor(m1->rig_camera_id);
    const RigSensor* s2 = rig->GetSensor(m2->rig_camera_id);
    if (s1 == nullptr || s2 == nullptr) {
      continue;
    }

    TwoViewInfo capture_info;
    if (!ViewEdgeToCaptureEdge(*s1, *s2, view_info, &capture_info)) {
      continue;
    }
    AddEdgeToCaptureGraph(m1->capture_id,
                          m2->capture_id,
                          capture_info,
                          capture_view_graph,
                          num_added,
                          only_if_missing);
  }
  return *num_added > 0;
}

bool EstimateMetricCaptureEdge(
    const Reconstruction& reconstruction,
    const CaptureId capture_id1,
    const CaptureId capture_id2,
    const BuildCaptureViewGraphOptions& options,
    TwoViewInfo* capture_info) {
  CHECK_NOTNULL(capture_info);
  const RigCapture* capture1 = reconstruction.GetRigCapture(capture_id1);
  const RigCapture* capture2 = reconstruction.GetRigCapture(capture_id2);
  if (capture1 == nullptr || capture2 == nullptr) {
    return false;
  }
  if (capture1->GetRigId() != capture2->GetRigId()) {
    return false;
  }
  const CameraRig* rig = reconstruction.GetCameraRig(capture1->GetRigId());
  if (rig == nullptr) {
    return false;
  }

  // sensor_id -> central matches (same sensor on both captures).
  std::unordered_map<RigCameraId, std::vector<GeneralizedRayCorrespondence>>
      central_by_sensor;
  std::vector<GeneralizedRayCorrespondence> generalized;

  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track* track = reconstruction.Track(track_id);
    if (track == nullptr) {
      continue;
    }

    struct Obs {
      RigCameraId sensor_id;
      const View* view;
      Feature feature;
    };
    std::vector<Obs> obs1, obs2;

    for (const auto& sensor_and_view : capture1->ViewIds()) {
      const ViewId view_id = sensor_and_view.second;
      if (!ContainsKey(track->ViewIds(), view_id)) {
        continue;
      }
      const View* view = reconstruction.View(view_id);
      const Feature* feature =
          view != nullptr ? view->GetFeature(track_id) : nullptr;
      const RigSensor* sensor = rig->GetSensor(sensor_and_view.first);
      if (view == nullptr || feature == nullptr || sensor == nullptr) {
        continue;
      }
      obs1.push_back({sensor_and_view.first, view, *feature});
    }
    for (const auto& sensor_and_view : capture2->ViewIds()) {
      const ViewId view_id = sensor_and_view.second;
      if (!ContainsKey(track->ViewIds(), view_id)) {
        continue;
      }
      const View* view = reconstruction.View(view_id);
      const Feature* feature =
          view != nullptr ? view->GetFeature(track_id) : nullptr;
      const RigSensor* sensor = rig->GetSensor(sensor_and_view.first);
      if (view == nullptr || feature == nullptr || sensor == nullptr) {
        continue;
      }
      obs2.push_back({sensor_and_view.first, view, *feature});
    }
    if (obs1.empty() || obs2.empty()) {
      continue;
    }

    for (const Obs& o1 : obs1) {
      const RigSensor* s1 = rig->GetSensor(o1.sensor_id);
      Eigen::Vector3d origin1, dir1;
      if (s1 == nullptr ||
          !LiftFeatureToRigRay(*o1.view, *s1, o1.feature, &origin1, &dir1)) {
        continue;
      }
      for (const Obs& o2 : obs2) {
        const RigSensor* s2 = rig->GetSensor(o2.sensor_id);
        Eigen::Vector3d origin2, dir2;
        if (s2 == nullptr ||
            !LiftFeatureToRigRay(*o2.view, *s2, o2.feature, &origin2, &dir2)) {
          continue;
        }
        GeneralizedRayCorrespondence corr;
        corr.origin1 = origin1;
        corr.direction1 = dir1;
        corr.origin2 = origin2;
        corr.direction2 = dir2;
        if (o1.sensor_id == o2.sensor_id) {
          central_by_sensor[o1.sensor_id].push_back(corr);
        } else {
          generalized.push_back(corr);
        }
      }
    }
  }

  // Prefer the largest same-sensor pool as the central 5-pt set.
  RigCameraId best_sensor = kInvalidRigCameraId;
  size_t best_count = 0;
  for (const auto& entry : central_by_sensor) {
    if (entry.second.size() > best_count) {
      best_count = entry.second.size();
      best_sensor = entry.first;
    }
  }
  if (best_count < 5 || generalized.empty()) {
    return false;
  }

  RelativeRigInfo info;
  RansacSummary summary;
  if (!EstimateRelativeRigInfo(options.relative_rig_ransac,
                               central_by_sensor[best_sensor],
                               generalized,
                               &info,
                               &summary)) {
    return false;
  }

  info.ToTwoViewInfo(capture_info);
  capture_info->num_verified_matches = static_cast<int>(summary.inliers.size());
  capture_info->visibility_score = capture_info->num_verified_matches;
  return true;
}

void CollectCapturePairsFromTracks(
    const Reconstruction& reconstruction,
    std::vector<ViewIdPair>* pairs) {
  CHECK_NOTNULL(pairs)->clear();
  std::map<ViewIdPair, int> counts;
  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track* track = reconstruction.Track(track_id);
    if (track == nullptr) {
      continue;
    }
    std::vector<CaptureId> captures;
    for (const ViewId view_id : track->ViewIds()) {
      const ViewRigMembership* m = reconstruction.GetViewRigMembership(view_id);
      if (m == nullptr) {
        continue;
      }
      captures.push_back(m->capture_id);
    }
    std::sort(captures.begin(), captures.end());
    captures.erase(std::unique(captures.begin(), captures.end()),
                   captures.end());
    for (size_t i = 0; i < captures.size(); ++i) {
      for (size_t j = i + 1; j < captures.size(); ++j) {
        counts[ViewIdPair(captures[i], captures[j])] += 1;
      }
    }
  }
  pairs->reserve(counts.size());
  for (const auto& entry : counts) {
    // Need enough shared tracks to hope for 5 central + 1 generalized.
    if (entry.second >= 6) {
      pairs->push_back(entry.first);
    }
  }
}

}  // namespace

bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph) {
  return BuildCaptureViewGraph(
      reconstruction, view_graph, capture_view_graph,
      BuildCaptureViewGraphOptions());
}

bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph,
                           const BuildCaptureViewGraphOptions& options) {
  CHECK_NOTNULL(capture_view_graph);
  *capture_view_graph = ViewGraph();
  int num_added = 0;
  int num_metric = 0;

  if (options.use_metric_relative_rig_pose) {
    std::vector<ViewIdPair> pairs;
    CollectCapturePairsFromTracks(reconstruction, &pairs);
    for (const ViewIdPair& pair : pairs) {
      TwoViewInfo capture_info;
      if (EstimateMetricCaptureEdge(
              reconstruction, pair.first, pair.second, options, &capture_info)) {
        if (AddEdgeToCaptureGraph(pair.first,
                                  pair.second,
                                  capture_info,
                                  capture_view_graph,
                                  &num_added,
                                  /*only_if_missing=*/false)) {
          ++num_metric;
        }
      }
    }
    VLOG(1) << "Metric capture edges estimated: " << num_metric << " / "
            << pairs.size() << " candidate pairs.";
  }

  if (options.fallback_to_twoview_strip) {
    // Fill missing pairs only — do not overwrite metric edges.
    BuildCaptureViewGraphFromViewEdges(reconstruction,
                                       view_graph,
                                       capture_view_graph,
                                       &num_added,
                                       /*only_if_missing=*/true);
  } else if (!options.use_metric_relative_rig_pose) {
    BuildCaptureViewGraphFromViewEdges(reconstruction,
                                       view_graph,
                                       capture_view_graph,
                                       &num_added,
                                       /*only_if_missing=*/false);
  }

  VLOG(1) << "Built capture view graph with " << capture_view_graph->NumEdges()
          << " edges (" << num_metric << " metric).";
  return capture_view_graph->NumEdges() > 0;
}

}  // namespace theia
