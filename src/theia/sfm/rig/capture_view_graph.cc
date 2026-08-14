// Copyright (C) 2026 The pyTheiaSfM Authors.

#include "theia/sfm/rig/capture_view_graph.h"

#include <algorithm>
#include <cmath>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
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
#include "theia/sfm/triangulation/triangulation.h"
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
  // Stripped essentials are unit-scale directions, not metric baselines.
  capture_info->scale_estimate = -1.0;
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

struct IndexedObs {
  RigCameraId sensor_id;
  const View* view;
  Feature feature;
};

using TrackCaptureIndex = std::unordered_map<
    TrackId,
    std::unordered_map<CaptureId, std::vector<IndexedObs>>>;

void BuildTrackCaptureIndex(const Reconstruction& reconstruction,
                            TrackCaptureIndex* index) {
  CHECK_NOTNULL(index)->clear();
  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track* track = reconstruction.Track(track_id);
    if (track == nullptr) {
      continue;
    }
    for (const ViewId view_id : track->ViewIds()) {
      const ViewRigMembership* m = reconstruction.GetViewRigMembership(view_id);
      const View* view = reconstruction.View(view_id);
      const Feature* feature =
          view != nullptr ? view->GetFeature(track_id) : nullptr;
      if (m == nullptr || view == nullptr || feature == nullptr) {
        continue;
      }
      (*index)[track_id][m->capture_id].push_back(
          {m->rig_camera_id, view, *feature});
    }
  }
}

void CollectCapturePairsFromViewGraph(
    const Reconstruction& reconstruction,
    const ViewGraph& view_graph,
    std::vector<ViewIdPair>* pairs,
    std::map<ViewIdPair, TwoViewInfo>* stripped_edges) {
  CHECK_NOTNULL(pairs)->clear();
  CHECK_NOTNULL(stripped_edges)->clear();
  for (const auto& edge : view_graph.GetAllEdges()) {
    const ViewRigMembership* m1 =
        reconstruction.GetViewRigMembership(edge.first.first);
    const ViewRigMembership* m2 =
        reconstruction.GetViewRigMembership(edge.first.second);
    if (m1 == nullptr || m2 == nullptr || m1->capture_id == m2->capture_id ||
        m1->rig_id != m2->rig_id) {
      continue;
    }
    const CameraRig* rig = reconstruction.GetCameraRig(m1->rig_id);
    const RigSensor* s1 =
        rig != nullptr ? rig->GetSensor(m1->rig_camera_id) : nullptr;
    const RigSensor* s2 =
        rig != nullptr ? rig->GetSensor(m2->rig_camera_id) : nullptr;
    if (s1 == nullptr || s2 == nullptr) {
      continue;
    }
    TwoViewInfo stripped;
    if (!ViewEdgeToCaptureEdge(*s1, *s2, edge.second, &stripped)) {
      continue;
    }
    const CaptureId a = std::min(m1->capture_id, m2->capture_id);
    const CaptureId b = std::max(m1->capture_id, m2->capture_id);
    if (m1->capture_id != a) {
      SwapCameras(&stripped);
    }
    const ViewIdPair pair(a, b);
    const auto existing = stripped_edges->find(pair);
    if (existing != stripped_edges->end() &&
        stripped.num_verified_matches <= existing->second.num_verified_matches) {
      continue;
    }
    (*stripped_edges)[pair] = stripped;
  }
  pairs->reserve(stripped_edges->size());
  for (const auto& entry : *stripped_edges) {
    pairs->push_back(entry.first);
  }
}

void CollectCapturePairsFromTrackIndex(const TrackCaptureIndex& index,
                                       std::vector<ViewIdPair>* pairs) {
  CHECK_NOTNULL(pairs)->clear();
  std::set<ViewIdPair> unique;
  for (const auto& track_entry : index) {
    if (track_entry.second.size() < 2) {
      continue;
    }
    std::vector<CaptureId> caps;
    caps.reserve(track_entry.second.size());
    for (const auto& cap_obs : track_entry.second) {
      caps.push_back(cap_obs.first);
    }
    for (size_t i = 0; i < caps.size(); ++i) {
      for (size_t j = i + 1; j < caps.size(); ++j) {
        unique.emplace(std::min(caps[i], caps[j]), std::max(caps[i], caps[j]));
      }
    }
  }
  pairs->assign(unique.begin(), unique.end());
}

bool MotionParallelToBaseline(const TwoViewInfo& capture_edge,
                              const CameraRig& rig,
                              const double max_alignment) {
  const double t_norm = capture_edge.position_2.norm();
  if (t_norm < 1e-8) {
    return false;
  }
  const Eigen::Vector3d t = capture_edge.position_2 / t_norm;
  const std::vector<RigCameraId> ids = rig.SensorIds();
  for (size_t i = 0; i < ids.size(); ++i) {
    const RigSensor* s1 = rig.GetSensor(ids[i]);
    if (s1 == nullptr) {
      continue;
    }
    for (size_t j = i + 1; j < ids.size(); ++j) {
      const RigSensor* s2 = rig.GetSensor(ids[j]);
      if (s2 == nullptr) {
        continue;
      }
      const Eigen::Vector3d baseline = s2->position - s1->position;
      const double b_norm = baseline.norm();
      if (b_norm < 1e-8) {
        continue;
      }
      if (std::abs(t.dot(baseline / b_norm)) > max_alignment) {
        return true;
      }
    }
  }
  return false;
}

// TwoViewInfo stores camera-2's center in camera 1 (C2) and R mapping cam1→cam2:
//   X2 = R * (X1 - s * t_unit),  t_unit = C2 / ||C2||
// PoseLib / generalized epipolar uses X2 = R * X1 + t with t = -R * C2.

bool MedianPositiveScale(std::vector<double> scales,
                         const int min_count,
                         double* scale) {
  CHECK_NOTNULL(scale);
  if (static_cast<int>(scales.size()) < min_count) {
    return false;
  }
  const auto median_of = [](std::vector<double>* v) {
    std::nth_element(v->begin(), v->begin() + v->size() / 2, v->end());
    return (*v)[v->size() / 2];
  };
  double median = median_of(&scales);
  std::vector<double> absdev;
  absdev.reserve(scales.size());
  for (const double s : scales) {
    absdev.push_back(std::abs(s - median));
  }
  const double mad = median_of(&absdev);
  const double thresh = std::max(3.0 * 1.4826 * mad, 0.05 * std::abs(median));
  std::vector<double> inliers;
  inliers.reserve(scales.size());
  for (const double s : scales) {
    if (std::abs(s - median) <= thresh) {
      inliers.push_back(s);
    }
  }
  if (static_cast<int>(inliers.size()) < min_count) {
    return false;
  }
  *scale = median_of(&inliers);
  return std::isfinite(*scale) && *scale > 1e-4;
}

bool TriangulateIndexedCapture(const std::vector<IndexedObs>& observations,
                               const CameraRig& rig,
                               Eigen::Vector3d* X) {
  CHECK_NOTNULL(X);
  std::vector<Eigen::Vector3d> origins;
  std::vector<Eigen::Vector3d> directions;
  std::unordered_set<RigCameraId> sensors;
  for (const IndexedObs& obs : observations) {
    if (!sensors.insert(obs.sensor_id).second) {
      continue;
    }
    const RigSensor* sensor = rig.GetSensor(obs.sensor_id);
    Eigen::Vector3d origin, direction;
    if (sensor == nullptr ||
        !LiftFeatureToRigRay(
            *obs.view, *sensor, obs.feature, &origin, &direction)) {
      continue;
    }
    origins.push_back(origin);
    directions.push_back(direction.normalized());
  }
  if (origins.size() < 2) {
    return false;
  }
  Eigen::Vector4d point;
  if (!TriangulateMidpoint(origins, directions, &point) ||
      std::abs(point[3]) < 1e-12) {
    return false;
  }
  *X = point.hnormalized();
  if (!X->allFinite()) {
    return false;
  }
  // Reject near-infinity stereo (KITTI highway far structure). 0.2° at a
  // 0.47 m baseline is ~135 m; closer points pin scale much more tightly.
  const Eigen::Vector3d v0 = (*X - origins[0]).normalized();
  const Eigen::Vector3d v1 = (*X - origins[1]).normalized();
  constexpr double kMinParallaxCos = 0.9999939;  // cos(0.2 deg)
  if (std::abs(v0.dot(v1)) > kMinParallaxCos) {
    return false;
  }
  if ((*X - origins[0]).dot(directions[0]) <= 0.0) {
    return false;
  }
  return true;
}

bool RecoverMetricScaleFromStereo(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& t_unit,
    const TrackCaptureIndex& index,
    const CaptureId capture_id1,
    const CaptureId capture_id2,
    const CameraRig& rig,
    double* scale) {
  CHECK_NOTNULL(scale);
  const Eigen::Vector3d R_t = rotation * t_unit;
  std::vector<double> scales_3d3d;
  std::vector<double> scales_bearing;
  for (const auto& track_entry : index) {
    const auto it1 = track_entry.second.find(capture_id1);
    const auto it2 = track_entry.second.find(capture_id2);
    if (it1 == track_entry.second.end() || it2 == track_entry.second.end()) {
      continue;
    }
    Eigen::Vector3d X1;
    if (it1->second.size() < 2 ||
        !TriangulateIndexedCapture(it1->second, rig, &X1)) {
      continue;
    }
    Eigen::Vector3d X2;
    if (it2->second.size() >= 2 &&
        TriangulateIndexedCapture(it2->second, rig, &X2)) {
      // X2 = R * (X1 - s * t_unit)  =>  s = -(X2 - R X1) · (R t_unit)
      const double s = -(X2 - rotation * X1).dot(R_t);
      if (std::isfinite(s) && s > 1e-4) {
        scales_3d3d.push_back(s);
      }
      continue;
    }
    if (it2->second.empty()) {
      continue;
    }
    const IndexedObs& obs2 = it2->second.front();
    const RigSensor* sensor2 = rig.GetSensor(obs2.sensor_id);
    Eigen::Vector3d origin2, direction2;
    if (sensor2 == nullptr ||
        !LiftFeatureToRigRay(
            *obs2.view, *sensor2, obs2.feature, &origin2, &direction2)) {
      continue;
    }
    direction2.normalize();
    // origin2 + λ d2 = R (X1 - s t_unit)
    // s (d2 × R t) = d2 × (R X1 - origin2)
    const Eigen::Vector3d d_cross_Rt = direction2.cross(R_t);
    const double denom = d_cross_Rt.squaredNorm();
    if (denom < 1e-12) {
      continue;
    }
    const Eigen::Vector3d RX = rotation * X1;
    const double s = direction2.cross(RX - origin2).dot(d_cross_Rt) / denom;
    const Eigen::Vector3d X2_pred = RX - s * R_t;
    if (std::isfinite(s) && s > 1e-4 &&
        (X2_pred - origin2).dot(direction2) > 0.0) {
      scales_bearing.push_back(s);
    }
  }
  if (MedianPositiveScale(std::move(scales_3d3d), /*min_count=*/8, scale)) {
    return true;
  }
  return MedianPositiveScale(std::move(scales_bearing), /*min_count=*/10, scale);
}

bool RecoverMetricScaleFromGeneralizedRays(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& t_unit,
    const std::vector<GeneralizedRayCorrespondence>& generalized,
    double* scale) {
  CHECK_NOTNULL(scale);
  // X2 = R X1 + γ t_epipolar with t_epipolar = -R * C2_unit.
  const Eigen::Vector3d t_epipolar = -rotation * t_unit;
  std::vector<double> gammas;
  gammas.reserve(generalized.size());
  for (const auto& corr : generalized) {
    const Eigen::Vector3d d1 = corr.direction1.normalized();
    const Eigen::Vector3d d2 = corr.direction2.normalized();
    const Eigen::Vector3d w = d2.cross(rotation * d1);
    const double denom = w.dot(t_epipolar);
    if (std::abs(denom) < 1e-8) {
      continue;
    }
    const double gamma =
        w.dot(corr.origin2 - rotation * corr.origin1) / denom;
    if (std::isfinite(gamma) && gamma > 1e-4) {
      gammas.push_back(gamma);
    }
  }
  return MedianPositiveScale(std::move(gammas), /*min_count=*/10, scale);
}

bool EstimateMetricCaptureEdgeFromIndex(
    const Reconstruction& reconstruction,
    const CaptureId capture_id1,
    const CaptureId capture_id2,
    const TrackCaptureIndex& index,
    const BuildCaptureViewGraphOptions& options,
    const TwoViewInfo* stripped_prior,
    TwoViewInfo* capture_info) {
  CHECK_NOTNULL(capture_info);
  const RigCapture* capture1 = reconstruction.GetRigCapture(capture_id1);
  const RigCapture* capture2 = reconstruction.GetRigCapture(capture_id2);
  if (capture1 == nullptr || capture2 == nullptr ||
      capture1->GetRigId() != capture2->GetRigId()) {
    return false;
  }
  const CameraRig* rig = reconstruction.GetCameraRig(capture1->GetRigId());
  if (rig == nullptr) {
    return false;
  }

  auto fill_stripped_metric = [&](const Eigen::Vector3d& t_unit,
                                  double scale,
                                  const char* method) {
    *capture_info = *stripped_prior;
    capture_info->position_2 = t_unit;
    capture_info->scale_estimate = scale;
    if (std::abs(static_cast<int>(capture_id1) -
                 static_cast<int>(capture_id2)) <= 1) {
      LOG(INFO) << "Metric consecutive edge " << capture_id1 << "–"
                << capture_id2 << " (" << method << ") scale=" << scale
                << " t=[" << t_unit.transpose() << "]"
                << " |rot|=" << capture_info->rotation_2.norm();
    }
  };

  if (stripped_prior != nullptr) {
    const Eigen::Matrix3d R = AngleAxisToMatrix(stripped_prior->rotation_2);
    Eigen::Vector3d t_unit = stripped_prior->position_2;
    const double t_norm = t_unit.norm();
    if (t_norm > 1e-8) {
      t_unit /= t_norm;
      double scale = 0.0;
      if (RecoverMetricScaleFromStereo(R,
                                       t_unit,
                                       index,
                                       capture_id1,
                                       capture_id2,
                                       *rig,
                                       &scale)) {
        fill_stripped_metric(t_unit, scale, "stereo 3D");
        return true;
      }
    }
  }

  std::unordered_map<RigCameraId, std::vector<GeneralizedRayCorrespondence>>
      central_by_sensor;
  std::vector<GeneralizedRayCorrespondence> generalized;

  for (const auto& track_entry : index) {
    const auto it1 = track_entry.second.find(capture_id1);
    const auto it2 = track_entry.second.find(capture_id2);
    if (it1 == track_entry.second.end() || it2 == track_entry.second.end()) {
      continue;
    }
    for (const IndexedObs& o1 : it1->second) {
      const RigSensor* s1 = rig->GetSensor(o1.sensor_id);
      Eigen::Vector3d origin1, dir1;
      if (s1 == nullptr ||
          !LiftFeatureToRigRay(*o1.view, *s1, o1.feature, &origin1, &dir1)) {
        continue;
      }
      for (const IndexedObs& o2 : it2->second) {
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

  if (stripped_prior != nullptr && !generalized.empty()) {
    const Eigen::Matrix3d R = AngleAxisToMatrix(stripped_prior->rotation_2);
    Eigen::Vector3d t_unit = stripped_prior->position_2;
    const double t_norm = t_unit.norm();
    if (t_norm > 1e-8) {
      t_unit /= t_norm;
      double scale = 0.0;
      if (RecoverMetricScaleFromGeneralizedRays(
              R, t_unit, generalized, &scale)) {
        fill_stripped_metric(t_unit, scale, "generalized rays");
        return true;
      }
    }
  }

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
  if (MotionParallelToBaseline(*capture_info, *rig,
                               options.max_baseline_translation_alignment)) {
    return false;
  }
  if (std::abs(static_cast<int>(capture_id1) - static_cast<int>(capture_id2)) <=
      1) {
    LOG(INFO) << "Metric consecutive edge " << capture_id1 << "–" << capture_id2
              << " (5+1) sensor=" << best_sensor
              << " scale=" << capture_info->scale_estimate
              << " t=[" << capture_info->position_2.transpose() << "]"
              << " |rot|=" << capture_info->rotation_2.norm();
  }
  return true;
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

  TrackCaptureIndex index;
  if (options.use_metric_relative_rig_pose) {
    BuildTrackCaptureIndex(reconstruction, &index);
    std::vector<ViewIdPair> pairs;
    std::map<ViewIdPair, TwoViewInfo> stripped_edges;
    if (options.metric_only_for_viewgraph_pairs) {
      CollectCapturePairsFromViewGraph(
          reconstruction, view_graph, &pairs, &stripped_edges);
    } else {
      CollectCapturePairsFromTrackIndex(index, &pairs);
    }
    for (const ViewIdPair& pair : pairs) {
      const auto stripped_it = stripped_edges.find(pair);
      const RigCapture* capture =
          reconstruction.GetRigCapture(static_cast<CaptureId>(pair.first));
      const CameraRig* rig =
          capture != nullptr ? reconstruction.GetCameraRig(capture->GetRigId())
                             : nullptr;
      if (options.skip_metric_if_baseline_degenerate && rig != nullptr &&
          stripped_it != stripped_edges.end() &&
          MotionParallelToBaseline(stripped_it->second,
                                   *rig,
                                   options.max_baseline_translation_alignment)) {
        AddEdgeToCaptureGraph(pair.first,
                              pair.second,
                              stripped_it->second,
                              capture_view_graph,
                              &num_added,
                              /*only_if_missing=*/false);
        continue;
      }
      if (stripped_it != stripped_edges.end() &&
          std::abs(static_cast<int>(pair.first) -
                   static_cast<int>(pair.second)) <= 1) {
        LOG(INFO) << "Stripped consecutive edge " << pair.first << "–"
                  << pair.second << " t=["
                  << stripped_it->second.position_2.transpose() << "]"
                  << " |rot|=" << stripped_it->second.rotation_2.norm()
                  << " n=" << stripped_it->second.num_verified_matches;
      }
      TwoViewInfo capture_info;
      const TwoViewInfo* stripped_prior =
          stripped_it != stripped_edges.end() ? &stripped_it->second : nullptr;
      if (EstimateMetricCaptureEdgeFromIndex(reconstruction,
                                             pair.first,
                                             pair.second,
                                             index,
                                             options,
                                             stripped_prior,
                                             &capture_info)) {
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
    LOG(INFO) << "Metric capture edges estimated: " << num_metric << " / "
              << pairs.size() << " candidate pairs.";
  }

  if (options.fallback_to_twoview_strip) {
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

  LOG(INFO) << "Built capture view graph with " << capture_view_graph->NumEdges()
            << " edges (" << num_metric << " metric).";
  return capture_view_graph->NumEdges() > 0;
}

}  // namespace theia
