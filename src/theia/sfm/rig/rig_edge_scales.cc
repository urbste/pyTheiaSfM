// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/rig/rig_edge_scales.h"

#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/Core>
#include <utility>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace theia {
namespace {

// Minimum angle (degrees) between the two rays used for triangulation.
constexpr double kMinTriangulationAngleDeg = 1.0;
// Maximum depth of a triangulated point, in units of the rig baseline.
constexpr double kMaxDepthInBaselines = 1000.0;
// (feature in v1, feature in u, feature in v2) for one shared track.
struct TrackMatch {
  const Feature* f1;
  const Feature* fu;
  const Feature* f2;
};

// Fraction of estimates that must agree within a factor of two of the median.
constexpr double kMinConsistentFraction = 0.5;

// World-frame unit bearing of |feature| as seen by |view|.
bool WorldBearing(const View& view,
                  const Feature& feature,
                  Eigen::Vector3d* bearing) {
  const Eigen::Matrix3d R_wc = view.Camera().GetOrientationAsRotationMatrix();
  const Eigen::Vector3d bearing_cam =
      view.Camera().PixelToNormalizedCoordinates(feature.point_);
  if (bearing_cam.squaredNorm() < 1e-16) {
    return false;
  }
  *bearing = R_wc.transpose() * bearing_cam.normalized();
  return true;
}

// Midpoint triangulation of two rays. Returns false when the rays are nearly
// parallel or the point lies behind either camera.
bool TriangulateMidpointRays(const Eigen::Vector3d& o1,
                             const Eigen::Vector3d& d1,
                             const Eigen::Vector3d& o2,
                             const Eigen::Vector3d& d2,
                             Eigen::Vector3d* X) {
  const double b = d1.dot(d2);
  const double denom = 1.0 - b * b;  // sin^2 of the ray angle
  const double min_sin =
      std::sin(kMinTriangulationAngleDeg * 3.14159265358979323846 / 180.0);
  if (denom < min_sin * min_sin) {
    return false;
  }
  const Eigen::Vector3d w0 = o1 - o2;
  const double d = d1.dot(w0);
  const double e = d2.dot(w0);
  const double t1 = (b * e - d) / denom;
  const double t2 = (e - b * d) / denom;
  if (t1 <= 0.0 || t2 <= 0.0) {
    return false;
  }
  const Eigen::Vector3d P1 = o1 + t1 * d1;
  const Eigen::Vector3d P2 = o2 + t2 * d2;
  *X = 0.5 * (P1 + P2);
  return true;
}

// Estimates the metric distance between the centres of v1 and v2 for one
// edge. Returns true and sets |scale| on success.
bool EstimateEdgeScale(const Reconstruction& reconstruction,
                       const ViewId v1_id,
                       const ViewId v2_id,
                       const TwoViewInfo& info,
                       int min_points,
                       double* scale) {
  const ViewRigMembership* m1 = reconstruction.GetViewRigMembership(v1_id);
  const ViewRigMembership* m2 = reconstruction.GetViewRigMembership(v2_id);
  if (m1 == nullptr || m2 == nullptr) {
    return false;
  }
  if (m1->rig_id != m2->rig_id || m1->capture_id == m2->capture_id ||
      m1->rig_camera_id != m2->rig_camera_id) {
    return false;
  }

  const View* view1 = reconstruction.View(v1_id);
  const View* view2 = reconstruction.View(v2_id);
  if (view1 == nullptr || view2 == nullptr || !view1->IsEstimated() ||
      !view2->IsEstimated()) {
    return false;
  }

  // Choose the other sensor view of v1's capture that shares the most tracks
  // with both v1 and v2.
  const RigCapture* capture1 = reconstruction.GetRigCapture(m1->capture_id);
  if (capture1 == nullptr) {
    return false;
  }
  const std::vector<TrackId> tracks1 = view1->TrackIds();
  const View* best_u = nullptr;
  std::vector<TrackMatch> best_matches;
  for (const auto& rig_view : capture1->ViewIds()) {
    if (rig_view.first == m1->rig_camera_id) {
      continue;
    }
    const View* u = reconstruction.View(rig_view.second);
    if (u == nullptr || !u->IsEstimated()) {
      continue;
    }
    std::vector<TrackMatch> matches;
    for (const TrackId track_id : tracks1) {
      const Feature* f1 = view1->GetFeature(track_id);
      const Feature* fu = u->GetFeature(track_id);
      const Feature* f2 = view2->GetFeature(track_id);
      if (f1 == nullptr || fu == nullptr || f2 == nullptr) {
        continue;
      }
      matches.push_back(TrackMatch{f1, fu, f2});
    }
    if (matches.size() > best_matches.size()) {
      best_matches = matches;
      best_u = u;
    }
  }
  if (best_u == nullptr ||
      static_cast<int>(best_matches.size()) < min_points) {
    return false;
  }

  // Posed centres and world bearings for v1 and v2.
  const Eigen::Vector3d c1 = view1->Camera().GetPosition();
  const Eigen::Vector3d cu = best_u->Camera().GetPosition();
  const double baseline = (cu - c1).norm();
  if (baseline < 1e-9) {
    return false;
  }
  const Eigen::Matrix3d R1 = view1->Camera().GetOrientationAsRotationMatrix();
  const Eigen::Vector3d d_w = R1.transpose() * info.position_2;
  if (d_w.squaredNorm() < 1e-16) {
    return false;
  }
  const Eigen::Vector3d dir_w = d_w.normalized();

  std::vector<double> estimates;
  estimates.reserve(best_matches.size());
  for (const TrackMatch& match : best_matches) {
    const Feature& f1 = *match.f1;
    const Feature& fu = *match.fu;
    const Feature& f2 = *match.f2;

    Eigen::Vector3d b1, bu, b2;
    if (!WorldBearing(*view1, f1, &b1) || !WorldBearing(*best_u, fu, &bu) ||
        !WorldBearing(*view2, f2, &b2)) {
      continue;
    }

    Eigen::Vector3d X;
    if (!TriangulateMidpointRays(c1, b1, cu, bu, &X)) {
      continue;
    }
    if ((X - c1).norm() > kMaxDepthInBaselines * baseline) {
      continue;
    }

    // Points on the ray of v2: cross(b2, X - c1 - s * dir_w) = 0.
    const Eigen::Vector3d a = b2.cross(X - c1);
    const Eigen::Vector3d b = b2.cross(dir_w);
    const double bb = b.dot(b);
    if (bb < 1e-8) {
      continue;
    }
    const double s_i = a.dot(b) / bb;
    if (!std::isfinite(s_i)) {
      continue;
    }
    estimates.push_back(s_i);
  }

  if (static_cast<int>(estimates.size()) < min_points) {
    return false;
  }
  std::sort(estimates.begin(), estimates.end());
  const size_t n = estimates.size();
  const double median = (n % 2 == 1)
                            ? estimates[n / 2]
                            : 0.5 * (estimates[n / 2 - 1] + estimates[n / 2]);
  if (!(median > 0.0)) {
    return false;
  }

  size_t consistent = 0;
  for (const double s : estimates) {
    if (s >= 0.5 * median && s <= 2.0 * median) {
      ++consistent;
    }
  }
  if (static_cast<double>(consistent) <
      kMinConsistentFraction * static_cast<double>(n)) {
    return false;
  }

  *scale = median;
  return true;
}

}  // namespace

int EstimateMetricEdgeScales(const Reconstruction& reconstruction,
                             ViewGraph* view_graph,
                             const int min_points) {
  CHECK_NOTNULL(view_graph);
  std::vector<std::pair<ViewIdPair, double>> updates;
  for (const auto& edge : view_graph->GetAllEdges()) {
    double scale = -1.0;
    if (EstimateEdgeScale(reconstruction,
                          edge.first.first,
                          edge.first.second,
                          edge.second,
                          min_points,
                          &scale)) {
      updates.push_back(std::make_pair(edge.first, scale));
    }
  }

  for (const auto& update : updates) {
    TwoViewInfo* info =
        view_graph->GetMutableEdge(update.first.first, update.first.second);
    if (info != nullptr) {
      info->scale_estimate = update.second;
    }
  }
  return static_cast<int>(updates.size());
}

}  // namespace theia
