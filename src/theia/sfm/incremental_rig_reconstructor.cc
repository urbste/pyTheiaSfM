// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.

#include "theia/sfm/incremental_rig_reconstructor.h"

#include <Eigen/Core>
#include <algorithm>
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
#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
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

  estimated_captures_.clear();
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
      remaining.erase(
          std::remove(remaining.begin(), remaining.end(), best_capture),
          remaining.end());
      continue;
    }
    estimated_captures_.push_back(best_capture);

    timer.Reset();
    EstimateStructure(reconstruction);
    summary.triangulation_time += timer.ElapsedTimeInSeconds();

    if (options_.bundle_adjust_after_localize) {
      timer.Reset();
      const bool run_full =
          options_.bundle_adjust_every_n_captures > 0 &&
          (static_cast<int>(estimated_captures_.size()) %
           options_.bundle_adjust_every_n_captures) == 0;
      if (run_full) {
        BundleAdjustAndPropagate(reconstruction);
      } else {
        PartialBundleAdjustAndPropagate(reconstruction);
      }
      summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();
    }

    remaining = OrderedUnestimatedCaptures(*reconstruction);
  }

  timer.Reset();
  BundleAdjustAndPropagate(reconstruction);
  summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();

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

  CaptureId seed = captures.front();
  for (const CaptureId capture_id : captures) {
    const RigCapture* capture = reconstruction->GetRigCapture(capture_id);
    if (capture == nullptr) {
      continue;
    }
    // GetViewIds() returns by value; do not pass begin()/end() of two
    // temporaries into a range constructor (dangling iterators / SIGSEGV).
    std::unordered_set<ViewId> capture_views;
    for (const auto& sensor_and_view : capture->ViewIds()) {
      capture_views.insert(sensor_and_view.second);
    }
    if (capture_views.empty()) {
      continue;
    }
    bool has_intra_rig_track = false;
    for (const TrackId track_id : reconstruction->TrackIds()) {
      const Track* track = reconstruction->Track(track_id);
      if (track == nullptr) {
        continue;
      }
      int observations_in_capture = 0;
      for (const ViewId view_id : track->ViewIds()) {
        if (ContainsKey(capture_views, view_id)) {
          ++observations_in_capture;
        }
      }
      if (observations_in_capture >= 2) {
        has_intra_rig_track = true;
        break;
      }
    }
    if (has_intra_rig_track) {
      seed = capture_id;
      break;
    }
  }

  RigCapture* capture = reconstruction->MutableRigCapture(seed);
  capture->SetPosition(Eigen::Vector3d::Zero());
  capture->SetOrientationFromAngleAxis(Eigen::Vector3d::Zero());
  capture->SetEstimated(true);
  if (!PropagateCameraPosesForCapture(seed, reconstruction)) {
    return false;
  }
  estimated_captures_.push_back(seed);
  return true;
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

  // Only triangulate tracks visible in the capture we just estimated. Retrying
  // every unestimated track after each localization is the dominant cost on
  // dense stereo sequences.
  if (estimated_captures_.empty()) {
    track_estimator.EstimateAllTracks();
    return;
  }
  const RigCapture* capture =
      reconstruction->GetRigCapture(estimated_captures_.back());
  if (capture == nullptr) {
    track_estimator.EstimateAllTracks();
    return;
  }
  std::unordered_set<TrackId> tracks;
  for (const ViewId view_id : capture->GetViewIds()) {
    const View* view = reconstruction->View(view_id);
    if (view == nullptr) {
      continue;
    }
    for (const TrackId track_id : view->TrackIds()) {
      const Track* track = reconstruction->Track(track_id);
      if (track != nullptr && !track->IsEstimated()) {
        tracks.insert(track_id);
      }
    }
  }
  if (!tracks.empty()) {
    track_estimator.EstimateTracks(tracks);
  }
}

void IncrementalRigReconstructor::BundleAdjustAndPropagate(
    Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options_.ba_options;
  ba_options.use_rig_constraints = true;
  BundleAdjustReconstruction(ba_options, reconstruction);
  PropagateAllEstimatedCapturePoses(reconstruction);
}

void IncrementalRigReconstructor::PartialBundleAdjustAndPropagate(
    Reconstruction* reconstruction) {
  const int partial_ba_size = std::min(
      static_cast<int>(estimated_captures_.size()),
      options_.partial_bundle_adjustment_num_captures);
  if (partial_ba_size <= 0) {
    return;
  }

  std::unordered_set<ViewId> views_to_optimize;
  std::unordered_set<TrackId> tracks_to_optimize;
  for (int i = static_cast<int>(estimated_captures_.size()) - partial_ba_size;
       i < static_cast<int>(estimated_captures_.size());
       ++i) {
    const RigCapture* capture =
        reconstruction->GetRigCapture(estimated_captures_[i]);
    if (capture == nullptr) {
      continue;
    }
    for (const ViewId view_id : capture->GetViewIds()) {
      const View* view = reconstruction->View(view_id);
      if (view == nullptr || !view->IsEstimated()) {
        continue;
      }
      views_to_optimize.insert(view_id);
      for (const TrackId track_id : view->TrackIds()) {
        const Track* track = reconstruction->Track(track_id);
        if (track != nullptr && track->IsEstimated()) {
          tracks_to_optimize.insert(track_id);
        }
      }
    }
  }
  if (views_to_optimize.empty() || tracks_to_optimize.empty()) {
    return;
  }

  BundleAdjustmentOptions ba_options = options_.ba_options;
  ba_options.use_rig_constraints = true;
  ba_options.use_inner_iterations = false;
  BundleAdjustPartialReconstruction(
      ba_options, views_to_optimize, tracks_to_optimize, reconstruction);
  PropagateAllEstimatedCapturePoses(reconstruction);
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
