// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/incremental_rig_reconstructor.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <glog/logging.h>
#include <unordered_set>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/estimators/camera_and_feature_correspondence_2d_3d.h"
#include "theia/sfm/estimators/estimate_rigid_transformation_2d_3d.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/localize_view_to_reconstruction.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/rig/rig_utils.h"
#include "theia/sfm/rigid_transformation.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/map_util.h"
#include "theia/util/timer.h"

namespace theia {
namespace {

Camera CameraInRigFrame(const Camera& source_camera, const RigSensor& sensor) {
  Camera camera;
  camera.DeepCopy(source_camera);
  camera.SetPosition(sensor.position);
  camera.SetOrientationFromAngleAxis(sensor.orientation);
  return camera;
}

// Triangulation angle (degrees) at the midpoint of two rays o1 + t d1 and
// o2 + t d2 (unit d1, d2). Returns 0 for parallel rays or points behind a
// camera, so such pairs never satisfy a positive minimum angle.
double MidpointTriangulationAngleDegrees(const Eigen::Vector3d& o1,
                                         const Eigen::Vector3d& d1,
                                         const Eigen::Vector3d& o2,
                                         const Eigen::Vector3d& d2) {
  const double b = d1.dot(d2);
  const double denom = 1.0 - b * b;
  if (denom < 1e-12) {
    return 0.0;
  }
  const Eigen::Vector3d w0 = o1 - o2;
  const double d = d1.dot(w0);
  const double e = d2.dot(w0);
  const double t1 = (b * e - d) / denom;
  const double t2 = (e - b * d) / denom;
  if (t1 <= 0.0 || t2 <= 0.0) {
    return 0.0;
  }
  const Eigen::Vector3d X = 0.5 * ((o1 + t1 * d1) + (o2 + t2 * d2));
  const Eigen::Vector3d v1 = o1 - X;
  const Eigen::Vector3d v2 = o2 - X;
  if (v1.squaredNorm() < 1e-18 || v2.squaredNorm() < 1e-18) {
    return 0.0;
  }
  const double cos_angle =
      std::max(-1.0, std::min(1.0, v1.normalized().dot(v2.normalized())));
  return std::acos(cos_angle) * 180.0 / 3.14159265358979323846;
}

}  // namespace

IncrementalRigReconstructor::IncrementalRigReconstructor(
    const IncrementalRigReconstructorOptions& options)
    : options_(options) {}

ReconstructionEstimatorSummary IncrementalRigReconstructor::Estimate(
    ViewGraph* view_graph, Reconstruction* reconstruction) {
  CHECK_NOTNULL(view_graph);
  CHECK_NOTNULL(reconstruction);

  ReconstructionEstimatorSummary summary;
  Timer total_timer;
  Timer timer;

  SetCameraIntrinsicsFromPriors(reconstruction);

  timer.Reset();
  if (!SeedInitialCapture(reconstruction)) {
    summary.success = false;
    summary.message = "Failed to seed an initial rig capture.";
    summary.total_time = total_timer.ElapsedTimeInSeconds();
    return summary;
  }
  summary.pose_estimation_time += timer.ElapsedTimeInSeconds();

  timer.Reset();
  EstimateStructure(reconstruction);
  summary.triangulation_time += timer.ElapsedTimeInSeconds();

  if (options_.bundle_adjust_after_localize) {
    timer.Reset();
    BundleAdjustAndPropagate(reconstruction);
    summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();
  }

  // The full bundle adjustment runs only every N localized captures; the
  // captures localized since the last one are tracked here.
  const int ba_every_n = std::max(1, options_.bundle_adjust_every_n_captures);
  int num_localized_total = 0;
  int num_localized_since_ba = 0;

  std::vector<CaptureId> remaining = OrderedUnestimatedCaptures(*reconstruction);
  while (!remaining.empty()) {
    CaptureId best_capture = kInvalidCaptureId;
    int best_score = -1;
    for (const CaptureId capture_id : remaining) {
      const int score =
          NumEstimatedTracksInCapture(capture_id, *reconstruction);
      if (score > best_score) {
        best_score = score;
        best_capture = capture_id;
      }
    }

    if (best_capture == kInvalidCaptureId ||
        best_score < options_.min_num_abs_pose_features) {
      break;
    }

    timer.Reset();
    const bool localized = LocalizeCapture(best_capture, reconstruction);
    summary.pose_estimation_time += timer.ElapsedTimeInSeconds();
    if (!localized) {
      // Try the next-best capture next iteration by treating this as failed:
      // mark nothing, but remove from consideration by skipping via a local set.
      // Simpler: erase and continue.
      remaining.erase(
          std::remove(remaining.begin(), remaining.end(), best_capture),
          remaining.end());
      continue;
    }

    // Triangulation runs after every localization, so tracks that failed the
    // triangulation angle test at the (small-baseline) stereo seed are retried
    // as soon as a larger baseline is available.
    timer.Reset();
    EstimateStructure(reconstruction);
    summary.triangulation_time += timer.ElapsedTimeInSeconds();
    ++num_localized_total;
    ++num_localized_since_ba;

    if (options_.bundle_adjust_after_localize &&
        num_localized_total % ba_every_n == 0) {
      timer.Reset();
      BundleAdjustAndPropagate(reconstruction);
      summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();
      num_localized_since_ba = 0;
    }

    remaining = OrderedUnestimatedCaptures(*reconstruction);
  }

  if (options_.bundle_adjust_after_localize && num_localized_since_ba > 0) {
    timer.Reset();
    BundleAdjustAndPropagate(reconstruction);
    summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();
  }

  for (const ViewId view_id : reconstruction->ViewIds()) {
    const View* view = reconstruction->View(view_id);
    if (view != nullptr && view->IsEstimated()) {
      summary.estimated_views.insert(view_id);
    }
  }
  for (const TrackId track_id : reconstruction->TrackIds()) {
    const Track* track = reconstruction->Track(track_id);
    if (track != nullptr && track->IsEstimated()) {
      summary.estimated_tracks.insert(track_id);
    }
  }

  summary.success = !summary.estimated_views.empty();
  summary.total_time = total_timer.ElapsedTimeInSeconds();
  if (summary.success) {
    summary.message = "Incremental rig reconstruction succeeded.";
  } else {
    summary.message = "Incremental rig reconstruction failed.";
  }
  return summary;
}

bool IncrementalRigReconstructor::SeedInitialCapture(
    Reconstruction* reconstruction) {
  std::vector<CaptureId> captures = reconstruction->CaptureIds();
  if (captures.empty()) {
    LOG(WARNING) << "No rig captures in the reconstruction.";
    return false;
  }

  std::sort(captures.begin(),
            captures.end(),
            [&](const CaptureId a, const CaptureId b) {
              return reconstruction->GetRigCapture(a)->GetTimestamp() <
                     reconstruction->GetRigCapture(b)->GetTimestamp();
            });

  // Stereo seed: among the first few captures pick the one with the most
  // intra-capture tracks whose triangulation angle is large enough.
  CaptureId seed = kInvalidCaptureId;
  int best_count = 0;
  const int num_candidates =
      std::min(static_cast<int>(captures.size()),
               std::max(1, options_.num_seed_candidates));
  for (int i = 0; i < num_candidates; ++i) {
    const int count = CountStereoTracksInCapture(
        captures[i], *reconstruction, options_.min_triangulation_angle_degrees);
    if (count > best_count) {
      best_count = count;
      seed = captures[i];
    }
  }

  // Fallback: the first capture that has any intra-capture track.
  if (seed == kInvalidCaptureId) {
    seed = captures.front();
    for (const CaptureId capture_id : captures) {
      if (CountStereoTracksInCapture(capture_id, *reconstruction, 0.0) > 0) {
        seed = capture_id;
        break;
      }
    }
  }
  LOG(INFO) << "Seeding rig reconstruction with capture " << seed << " ("
            << best_count << " stereo tracks with sufficient angle).";

  RigCapture* capture = reconstruction->MutableRigCapture(seed);
  capture->SetPosition(Eigen::Vector3d::Zero());
  capture->SetOrientationFromAngleAxis(Eigen::Vector3d::Zero());
  capture->SetEstimated(true);
  return PropagateCameraPosesForCapture(seed, reconstruction);
}

int IncrementalRigReconstructor::CountStereoTracksInCapture(
    const CaptureId capture_id,
    const Reconstruction& reconstruction,
    const double min_angle_degrees) const {
  const RigCapture* capture = reconstruction.GetRigCapture(capture_id);
  if (capture == nullptr) {
    return 0;
  }
  const CameraRig* rig = reconstruction.GetCameraRig(capture->GetRigId());
  if (rig == nullptr) {
    return 0;
  }
  // Rig-frame rays (centre, unit direction) of every observation in the
  // capture, grouped by track. The capture pose itself is not needed.
  struct Ray {
    Eigen::Vector3d center;
    Eigen::Vector3d direction;
  };
  std::unordered_map<TrackId, std::vector<Ray>> rays_by_track;
  for (const auto& sensor_and_view : capture->ViewIds()) {
    const RigSensor* sensor = rig->GetSensor(sensor_and_view.first);
    const View* view = reconstruction.View(sensor_and_view.second);
    if (sensor == nullptr || view == nullptr) {
      continue;
    }
    const Camera camera = CameraInRigFrame(view->Camera(), *sensor);
    for (const TrackId track_id : view->TrackIds()) {
      const Feature* feature = view->GetFeature(track_id);
      if (feature == nullptr) {
        continue;
      }
      rays_by_track[track_id].push_back(
          Ray{camera.GetPosition(),
              camera.PixelToUnitDepthRay(feature->point_).normalized()});
    }
  }

  int count = 0;
  for (const auto& track_rays : rays_by_track) {
    const std::vector<Ray>& rays = track_rays.second;
    bool sufficient = false;
    for (size_t i = 0; i < rays.size() && !sufficient; ++i) {
      for (size_t j = i + 1; j < rays.size(); ++j) {
        const double angle = MidpointTriangulationAngleDegrees(
            rays[i].center, rays[i].direction, rays[j].center,
            rays[j].direction);
        if (angle >= min_angle_degrees) {
          sufficient = true;
          break;
        }
      }
    }
    if (sufficient) {
      ++count;
    }
  }
  return count;
}

bool IncrementalRigReconstructor::LocalizeCapture(
    const CaptureId capture_id, Reconstruction* reconstruction) {
  if (options_.use_generalized_localization &&
      LocalizeCaptureGeneralized(capture_id, reconstruction)) {
    return true;
  }
  return LocalizeCaptureFromSingleView(capture_id, reconstruction);
}

bool IncrementalRigReconstructor::LocalizeCaptureGeneralized(
    const CaptureId capture_id, Reconstruction* reconstruction) {
  RigCapture* capture = reconstruction->MutableRigCapture(capture_id);
  if (capture == nullptr) {
    return false;
  }
  const CameraRig* rig = reconstruction->GetCameraRig(capture->GetRigId());
  if (rig == nullptr) {
    return false;
  }

  std::vector<CameraAndFeatureCorrespondence2D3D> correspondences;
  for (const auto& sensor_and_view : capture->ViewIds()) {
    const RigCameraId rig_camera_id = sensor_and_view.first;
    const ViewId view_id = sensor_and_view.second;
    const RigSensor* sensor = rig->GetSensor(rig_camera_id);
    const View* view = reconstruction->View(view_id);
    if (sensor == nullptr || view == nullptr) {
      continue;
    }
    const Camera query_camera = CameraInRigFrame(view->Camera(), *sensor);

    for (const TrackId track_id : view->TrackIds()) {
      const Track* track = reconstruction->Track(track_id);
      const Feature* feature = view->GetFeature(track_id);
      if (track == nullptr || feature == nullptr || !track->IsEstimated()) {
        continue;
      }
      CameraAndFeatureCorrespondence2D3D correspondence;
      correspondence.camera = query_camera;
      correspondence.observation = *feature;
      correspondence.point3d = track->Point();
      correspondences.emplace_back(correspondence);
    }
  }

  if (static_cast<int>(correspondences.size()) <
      options_.min_num_abs_pose_features) {
    return false;
  }

  RigidTransformation transform;
  RansacSummary ransac_summary;
  RansacParameters ransac_params = options_.ransac_params;
  ransac_params.error_thresh = options_.max_reprojection_error_in_pixels *
                               options_.max_reprojection_error_in_pixels;
  if (!EstimateRigidTransformation2D3D(ransac_params,
                                       RansacType::RANSAC,
                                       correspondences,
                                       &transform,
                                       &ransac_summary)) {
    return false;
  }
  if (static_cast<int>(ransac_summary.inliers.size()) <
      options_.min_num_inliers_for_localization) {
    return false;
  }

  // Estimated transform maps world points into the rig frame:
  //   X_rig = R * X_world + t
  // RigCapture stores world-to-rig orientation and rig origin in world.
  capture->SetOrientationFromRotationMatrix(transform.rotation);
  capture->SetPosition(-transform.rotation.transpose() * transform.translation);
  capture->SetEstimated(true);
  return PropagateCameraPosesForCapture(capture_id, reconstruction);
}

bool IncrementalRigReconstructor::LocalizeCaptureFromSingleView(
    const CaptureId capture_id, Reconstruction* reconstruction) {
  RigCapture* capture = reconstruction->MutableRigCapture(capture_id);
  if (capture == nullptr) {
    return false;
  }
  const CameraRig* rig = reconstruction->GetCameraRig(capture->GetRigId());
  if (rig == nullptr) {
    return false;
  }

  ViewId best_view = kInvalidViewId;
  int best_count = -1;
  RigCameraId best_sensor = kInvalidRigCameraId;
  for (const auto& sensor_and_view : capture->ViewIds()) {
    const View* view = reconstruction->View(sensor_and_view.second);
    if (view == nullptr) {
      continue;
    }
    int count = 0;
    for (const TrackId track_id : view->TrackIds()) {
      const Track* track = reconstruction->Track(track_id);
      if (track != nullptr && track->IsEstimated()) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_view = sensor_and_view.second;
      best_sensor = sensor_and_view.first;
    }
  }

  if (best_view == kInvalidViewId ||
      best_count < options_.min_num_abs_pose_features) {
    return false;
  }

  LocalizeViewToReconstructionOptions localize_options =
      options_.localize_options;
  localize_options.reprojection_error_threshold_pixels =
      options_.max_reprojection_error_in_pixels;
  localize_options.min_num_inliers = options_.min_num_inliers_for_localization;
  localize_options.bundle_adjust_view = false;

  RansacSummary localize_summary;
  if (!LocalizeViewToReconstruction(
          best_view, localize_options, reconstruction, &localize_summary)) {
    return false;
  }

  const View* view = reconstruction->View(best_view);
  const RigSensor* sensor = rig->GetSensor(best_sensor);
  Eigen::Vector3d rig_position;
  Eigen::Matrix3d rig_orientation;
  ComposeRigPoseFromCamera(view->Camera().GetPosition(),
                           view->Camera().GetOrientationAsRotationMatrix(),
                           *sensor,
                           &rig_position,
                           &rig_orientation);
  capture->SetPosition(rig_position);
  capture->SetOrientationFromRotationMatrix(rig_orientation);
  capture->SetEstimated(true);
  return PropagateCameraPosesForCapture(capture_id, reconstruction);
}

void IncrementalRigReconstructor::EstimateStructure(
    Reconstruction* reconstruction) {
  TrackEstimator::Options track_options = options_.track_estimator_options;
  track_options.max_acceptable_reprojection_error_pixels =
      options_.max_reprojection_error_in_pixels;
  track_options.min_triangulation_angle_degrees =
      options_.min_triangulation_angle_degrees;
  TrackEstimator track_estimator(track_options, reconstruction);
  track_estimator.EstimateAllTracks();
}

void IncrementalRigReconstructor::BundleAdjustAndPropagate(
    Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options_.ba_options;
  if (options_.use_rig_constrained_ba) {
    BundleAdjustRigReconstruction(ba_options, reconstruction);
    return;
  }
  BundleAdjustReconstruction(ba_options, reconstruction);
  // Independent view BA can drift sensors apart; snap back to shared body
  // poses by re-estimating each capture from its first sensor, then propagate.
  for (const CaptureId capture_id : reconstruction->CaptureIds()) {
    RigCapture* capture = reconstruction->MutableRigCapture(capture_id);
    if (capture == nullptr || !capture->IsEstimated()) {
      continue;
    }
    const CameraRig* rig = reconstruction->GetCameraRig(capture->GetRigId());
    if (rig == nullptr || capture->ViewIds().empty()) {
      continue;
    }
    const auto first = *capture->ViewIds().begin();
    const RigCameraId rig_camera_id = first.first;
    const ViewId view_id = first.second;
    const View* view = reconstruction->View(view_id);
    const RigSensor* sensor = rig->GetSensor(rig_camera_id);
    if (view == nullptr || sensor == nullptr || !view->IsEstimated()) {
      continue;
    }
    Eigen::Vector3d rig_position;
    Eigen::Matrix3d rig_orientation;
    ComposeRigPoseFromCamera(view->Camera().GetPosition(),
                             view->Camera().GetOrientationAsRotationMatrix(),
                             *sensor,
                             &rig_position,
                             &rig_orientation);
    capture->SetPosition(rig_position);
    capture->SetOrientationFromRotationMatrix(rig_orientation);
    PropagateCameraPosesForCapture(capture_id, reconstruction);
  }
}

std::vector<CaptureId> IncrementalRigReconstructor::OrderedUnestimatedCaptures(
    const Reconstruction& reconstruction) const {
  std::vector<CaptureId> captures;
  for (const CaptureId capture_id : reconstruction.CaptureIds()) {
    const RigCapture* capture = reconstruction.GetRigCapture(capture_id);
    if (capture != nullptr && !capture->IsEstimated()) {
      captures.push_back(capture_id);
    }
  }
  std::sort(captures.begin(),
            captures.end(),
            [&](const CaptureId a, const CaptureId b) {
              return reconstruction.GetRigCapture(a)->GetTimestamp() <
                     reconstruction.GetRigCapture(b)->GetTimestamp();
            });
  return captures;
}

int IncrementalRigReconstructor::NumEstimatedTracksInCapture(
    const CaptureId capture_id, const Reconstruction& reconstruction) const {
  const RigCapture* capture = reconstruction.GetRigCapture(capture_id);
  if (capture == nullptr) {
    return 0;
  }
  std::unordered_set<TrackId> tracks;
  for (const ViewId view_id : capture->GetViewIds()) {
    const View* view = reconstruction.View(view_id);
    if (view == nullptr) {
      continue;
    }
    for (const TrackId track_id : view->TrackIds()) {
      const Track* track = reconstruction.Track(track_id);
      if (track != nullptr && track->IsEstimated()) {
        tracks.insert(track_id);
      }
    }
  }
  return static_cast<int>(tracks.size());
}

}  // namespace theia
