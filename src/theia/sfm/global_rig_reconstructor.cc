// Copyright (C) 2026 The pyTheiaSfM Authors.

#include "theia/sfm/global_rig_reconstructor.h"

#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include <memory>
#include <unordered_set>
#include <vector>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/estimate_track.h"
#include "theia/sfm/filter_view_pairs_from_orientation.h"
#include "theia/sfm/filter_view_pairs_from_relative_translation.h"
#include "theia/sfm/global_pose_estimation/hybrid_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/lagrange_dual_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/global_pose_estimation/robust_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"
#include "theia/sfm/global_pose_estimation/glomap_position_estimator.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/rig/capture_view_graph.h"
#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/rig/rig_utils.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/orientations_from_maximum_spanning_tree.h"
#include "theia/sfm/view_graph/remove_disconnected_view_pairs.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/map_util.h"
#include "theia/util/timer.h"

namespace theia {

GlobalRigReconstructor::GlobalRigReconstructor(
    const GlobalRigReconstructorOptions& options)
    : options_(options) {}

ReconstructionEstimatorSummary GlobalRigReconstructor::Estimate(
    ViewGraph* view_graph, Reconstruction* reconstruction) {
  CHECK_NOTNULL(view_graph);
  CHECK_NOTNULL(reconstruction);
  reconstruction_ = reconstruction;
  orientations_.clear();
  positions_.clear();

  ReconstructionEstimatorSummary summary;
  Timer total_timer;
  Timer timer;
  const ReconstructionEstimatorOptions& opt = options_.sfm_options;

  LOG(INFO) << "Calibrating any uncalibrated cameras.";
  timer.Reset();
  SetCameraIntrinsicsFromPriors(reconstruction_);

  LOG(INFO) << "Building the capture view graph.";
  timer.Reset();
  ViewGraph capture_graph;
  if (!BuildCaptureViewGraph(*reconstruction_,
                             *view_graph,
                             &capture_graph,
                             options_.capture_graph_options)) {
    LOG(WARNING) << "Failed to build capture view graph from view matches.";
    summary.success = false;
    summary.message =
        "Failed to build capture view graph from view matches. Ensure Views "
        "have rig membership and inter-capture matches exist.";
    summary.total_time = total_timer.ElapsedTimeInSeconds();
    return summary;
  }
  capture_view_graph_ = &capture_graph;
  LOG(INFO) << "Built capture view graph with "
            << capture_view_graph_->NumEdges() << " edges.";

  LOG(INFO) << "Filtering the initial capture view graph.";
  if (!FilterCaptureGraph()) {
    LOG(INFO) << "Insufficient capture pairs to perform estimation.";
    summary.success = false;
    summary.message = "Capture graph too sparse after filtering.";
    summary.total_time = total_timer.ElapsedTimeInSeconds();
    return summary;
  }
  LOG(INFO) << "Capture view graph has " << capture_view_graph_->NumEdges()
            << " edges after filtering.";

  LOG(INFO) << "Estimating the global rotations of all captures.";
  timer.Reset();
  if (!EstimateCaptureRotations()) {
    LOG(WARNING) << "Rotation estimation failed!";
    summary.success = false;
    summary.message = "Capture rotation averaging failed.";
    summary.total_time = total_timer.ElapsedTimeInSeconds();
    return summary;
  }
  summary.pose_estimation_time += timer.ElapsedTimeInSeconds();
  LOG(INFO) << orientations_.size()
            << " capture rotations were estimated successfully.";

  LOG(INFO) << "Filtering any bad rotation estimations.";
  timer.Reset();
  FilterCaptureRotations();
  summary.pose_estimation_time += timer.ElapsedTimeInSeconds();

  LOG(INFO) << "Estimating the positions of all captures.";
  timer.Reset();
  bool positions_ok = false;
  if (opt.global_position_estimator_type ==
      GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION) {
    positions_ok = EstimateCapturePositions();
  }
  if (!positions_ok) {
    LOG(INFO) << "Capture-graph position estimator failed or was skipped; "
                 "falling back to view-graph position estimation.";
    positions_ok = EstimateCapturePositionsFromViewGraph(view_graph);
  }
  summary.pose_estimation_time += timer.ElapsedTimeInSeconds();
  if (!positions_ok) {
    LOG(WARNING) << "Position estimation failed!";
    summary.success = false;
    summary.message = "Capture position averaging failed.";
    summary.total_time = total_timer.ElapsedTimeInSeconds();
    return summary;
  }
  LOG(INFO) << positions_.size()
            << " capture positions were estimated successfully.";

  if (options_.rescale_positions_to_metric_edges) {
    LOG(INFO) << "Rescaling capture positions to metric edge lengths.";
    RescaleCapturePositionsToMetricEdges();
  }

  LOG(INFO) << "Setting capture poses and propagating to views.";
  SetCapturePosesAndPropagate();

  for (int i = 0; i < opt.num_retriangulation_iterations + 1; ++i) {
    LOG(INFO) << "Triangulating all features.";
    timer.Reset();
    EstimateStructure();
    summary.triangulation_time += timer.ElapsedTimeInSeconds();

    LOG(INFO) << "Performing bundle adjustment.";
    timer.Reset();
    BundleAdjustAndPropagate();
    summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();
  }

  for (const ViewId view_id : reconstruction_->ViewIds()) {
    const View* view = reconstruction_->View(view_id);
    if (view != nullptr && view->IsEstimated()) {
      summary.estimated_views.insert(view_id);
    }
  }
  for (const TrackId track_id : reconstruction_->TrackIds()) {
    const Track* track = reconstruction_->Track(track_id);
    if (track != nullptr && track->IsEstimated()) {
      summary.estimated_tracks.insert(track_id);
    }
  }

  summary.success = !summary.estimated_views.empty();
  summary.message = summary.success ? "Global rig reconstruction succeeded."
                                    : "Global rig reconstruction failed.";
  summary.total_time = total_timer.ElapsedTimeInSeconds();
  return summary;
}

bool GlobalRigReconstructor::FilterCaptureGraph() {
  const auto& opt = options_.sfm_options;
  std::unordered_set<ViewIdPair> edges_to_remove;
  for (const auto& edge : capture_view_graph_->GetAllEdges()) {
    if (edge.second.num_verified_matches < opt.min_num_two_view_inliers) {
      edges_to_remove.insert(edge.first);
    }
  }
  for (const ViewIdPair& pair : edges_to_remove) {
    capture_view_graph_->RemoveEdge(pair.first, pair.second);
  }
  RemoveDisconnectedViewPairs(capture_view_graph_);
  return capture_view_graph_->NumEdges() >= 1;
}

bool GlobalRigReconstructor::EstimateCaptureRotations() {
  const auto& opt = options_.sfm_options;
  const auto& view_pairs = capture_view_graph_->GetAllEdges();
  std::unique_ptr<RotationEstimator> rotation_estimator;

  switch (opt.global_rotation_estimator_type) {
    case GlobalRotationEstimatorType::ROBUST_L1L2: {
      OrientationsFromMaximumSpanningTree(*capture_view_graph_, &orientations_);
      rotation_estimator.reset(new RobustRotationEstimator(
          RobustRotationEstimator::Options()));
      break;
    }
    case GlobalRotationEstimatorType::NONLINEAR: {
      OrientationsFromMaximumSpanningTree(*capture_view_graph_, &orientations_);
      rotation_estimator.reset(new NonlinearRotationEstimator());
      break;
    }
    case GlobalRotationEstimatorType::LINEAR: {
      rotation_estimator.reset(new LinearRotationEstimator());
      break;
    }
    case GlobalRotationEstimatorType::LAGRANGE_DUAL: {
      OrientationsFromMaximumSpanningTree(*capture_view_graph_, &orientations_);
      rotation_estimator.reset(new LagrangeDualRotationEstimator());
      break;
    }
    case GlobalRotationEstimatorType::HYBRID: {
      OrientationsFromMaximumSpanningTree(*capture_view_graph_, &orientations_);
      rotation_estimator.reset(new HybridRotationEstimator());
      break;
    }
    default:
      LOG(FATAL) << "Unknown GlobalRotationEstimatorType.";
  }

  return rotation_estimator->EstimateRotations(view_pairs, &orientations_);
}

void GlobalRigReconstructor::FilterCaptureRotations() {
  const auto& opt = options_.sfm_options;
  FilterViewPairsFromOrientation(
      orientations_,
      opt.rotation_filtering_max_difference_degrees,
      capture_view_graph_);
  const std::unordered_set<ViewId> removed =
      RemoveDisconnectedViewPairs(capture_view_graph_);
  for (const ViewId id : removed) {
    orientations_.erase(id);
  }
}

bool GlobalRigReconstructor::EstimateCapturePositions() {
  const auto& opt = options_.sfm_options;
  if (opt.filter_relative_translations_with_1dsfm) {
    FilterViewPairsFromRelativeTranslationOptions filter_opts;
    FilterViewPairsFromRelativeTranslation(
        filter_opts, orientations_, capture_view_graph_);
    const std::unordered_set<ViewId> removed =
        RemoveDisconnectedViewPairs(capture_view_graph_);
    for (const ViewId id : removed) {
      orientations_.erase(id);
    }
  }

  LeastUnsquaredDeviationPositionEstimator estimator(
      opt.least_unsquared_deviation_position_estimator_options);
  return estimator.EstimatePositions(
      capture_view_graph_->GetAllEdges(), orientations_, &positions_);
}

void GlobalRigReconstructor::RescaleCapturePositionsToMetricEdges() {
  if (capture_view_graph_ == nullptr || positions_.empty()) {
    return;
  }
  std::vector<double> scales;
  scales.reserve(capture_view_graph_->NumEdges());
  for (const auto& edge : capture_view_graph_->GetAllEdges()) {
    const ViewId a = edge.first.first;
    const ViewId b = edge.first.second;
    if (!ContainsKey(positions_, a) || !ContainsKey(positions_, b)) {
      continue;
    }
    const double metric = edge.second.position_2.norm();
    if (metric < 1e-8) {
      continue;
    }
    // TwoViewInfo.position_2 is b's center in a's identity frame. With known
    // orientations, predicted baseline in world is ||Cb - Ca||.
    const double predicted = (positions_[b] - positions_[a]).norm();
    if (predicted < 1e-8) {
      continue;
    }
    scales.push_back(metric / predicted);
  }
  if (scales.empty()) {
    return;
  }
  std::nth_element(
      scales.begin(), scales.begin() + scales.size() / 2, scales.end());
  const double scale = scales[scales.size() / 2];
  if (!std::isfinite(scale) || scale <= 0.0) {
    return;
  }
  LOG(INFO) << "Rescaling capture positions by metric edge factor " << scale;
  for (auto& pos : positions_) {
    pos.second *= scale;
  }
}

bool GlobalRigReconstructor::EstimateCapturePositionsFromViewGraph(
    ViewGraph* view_graph) {
  const auto& opt = options_.sfm_options;

  for (const auto& ori : orientations_) {
    const CaptureId capture_id = static_cast<CaptureId>(ori.first);
    RigCapture* capture = reconstruction_->MutableRigCapture(capture_id);
    if (capture == nullptr) {
      continue;
    }
    capture->SetOrientationFromAngleAxis(ori.second);
    capture->SetPosition(Eigen::Vector3d::Zero());
    capture->SetEstimated(true);
    PropagateCameraPosesForCapture(capture_id, reconstruction_);
  }

  std::unordered_map<ViewId, Eigen::Vector3d> view_orientations;
  for (const ViewId view_id : reconstruction_->ViewIds()) {
    const View* view = reconstruction_->View(view_id);
    if (view == nullptr || !view->IsEstimated()) {
      continue;
    }
    view_orientations[view_id] = view->Camera().GetOrientationAsAngleAxis();
  }

  std::unique_ptr<PositionEstimator> position_estimator;
  switch (opt.global_position_estimator_type) {
    case GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION: {
      position_estimator.reset(new LeastUnsquaredDeviationPositionEstimator(
          opt.least_unsquared_deviation_position_estimator_options));
      break;
    }
    case GlobalPositionEstimatorType::NONLINEAR: {
      position_estimator.reset(new NonlinearPositionEstimator(
          opt.nonlinear_position_estimator_options, *reconstruction_));
      break;
    }
    case GlobalPositionEstimatorType::LINEAR_TRIPLET: {
      position_estimator.reset(new LinearPositionEstimator(
          opt.linear_triplet_position_estimator_options, *reconstruction_));
      break;
    }
    case GlobalPositionEstimatorType::LIGT: {
      position_estimator.reset(new LiGTPositionEstimator(
          opt.ligt_position_estimator_options, *reconstruction_));
      break;
    }
    case GlobalPositionEstimatorType::GLOMAP: {
      position_estimator.reset(new GlomapPositionEstimator(
          opt.glomap_position_estimator_options, reconstruction_));
      break;
    }
    default:
      LOG(FATAL) << "Unknown GlobalPositionEstimatorType.";
  }

  std::unordered_map<ViewId, Eigen::Vector3d> view_positions;
  if (!position_estimator->EstimatePositions(
          view_graph->GetAllEdges(), view_orientations, &view_positions)) {
    return false;
  }

  positions_.clear();
  for (const CaptureId capture_id : reconstruction_->CaptureIds()) {
    RigCapture* capture = reconstruction_->MutableRigCapture(capture_id);
    if (capture == nullptr || capture->ViewIds().empty()) {
      continue;
    }
    if (!ContainsKey(orientations_, static_cast<ViewId>(capture_id))) {
      continue;
    }
    const CameraRig* rig = reconstruction_->GetCameraRig(capture->GetRigId());
    if (rig == nullptr) {
      continue;
    }

    const auto first = *capture->ViewIds().begin();
    const ViewId view_id = first.second;
    if (!ContainsKey(view_positions, view_id)) {
      continue;
    }
    View* view = reconstruction_->MutableView(view_id);
    const RigSensor* sensor = rig->GetSensor(first.first);
    if (view == nullptr || sensor == nullptr) {
      continue;
    }
    view->MutableCamera()->SetPosition(view_positions[view_id]);
    view->MutableCamera()->SetOrientationFromAngleAxis(
        view_orientations[view_id]);

    Eigen::Vector3d rig_position;
    Eigen::Matrix3d rig_orientation;
    ComposeRigPoseFromCamera(view->Camera().GetPosition(),
                             view->Camera().GetOrientationAsRotationMatrix(),
                             *sensor,
                             &rig_position,
                             &rig_orientation);
    positions_[static_cast<ViewId>(capture_id)] = rig_position;
  }
  return !positions_.empty();
}

void GlobalRigReconstructor::SetCapturePosesAndPropagate() {
  for (const auto& ori : orientations_) {
    const CaptureId capture_id = static_cast<CaptureId>(ori.first);
    RigCapture* capture = reconstruction_->MutableRigCapture(capture_id);
    if (capture == nullptr) {
      continue;
    }
    capture->SetOrientationFromAngleAxis(ori.second);
    if (ContainsKey(positions_, ori.first)) {
      capture->SetPosition(positions_[ori.first]);
    } else {
      capture->SetPosition(Eigen::Vector3d::Zero());
    }
    capture->SetEstimated(true);
    PropagateCameraPosesForCapture(capture_id, reconstruction_);
  }
}

void GlobalRigReconstructor::EstimateStructure() {
  const auto& opt = options_.sfm_options;
  TrackEstimator::Options opts;
  opts.max_acceptable_reprojection_error_pixels =
      opt.triangulation_max_reprojection_error_in_pixels;
  opts.min_triangulation_angle_degrees = opt.min_triangulation_angle_degrees;
  opts.bundle_adjustment = opt.bundle_adjust_tracks;
  opts.ba_options = SetBundleAdjustmentOptions(opt, 0);
  opts.num_threads = opt.num_threads;
  TrackEstimator estimator(opts, reconstruction_);
  estimator.EstimateAllTracks();
}

void GlobalRigReconstructor::BundleAdjustAndPropagate() {
  const auto& opt = options_.sfm_options;
  BundleAdjustmentOptions ba_options = SetBundleAdjustmentOptions(opt, 0);
  BundleAdjustReconstruction(ba_options, reconstruction_);

  for (const CaptureId capture_id : reconstruction_->CaptureIds()) {
    RigCapture* capture = reconstruction_->MutableRigCapture(capture_id);
    if (capture == nullptr || !capture->IsEstimated()) {
      continue;
    }
    const CameraRig* rig = reconstruction_->GetCameraRig(capture->GetRigId());
    if (rig == nullptr || capture->ViewIds().empty()) {
      continue;
    }
    const auto first = *capture->ViewIds().begin();
    const View* view = reconstruction_->View(first.second);
    const RigSensor* sensor = rig->GetSensor(first.first);
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
    PropagateCameraPosesForCapture(capture_id, reconstruction_);
  }

  SetOutlierTracksToUnestimated(opt.max_reprojection_error_in_pixels,
                                opt.min_triangulation_angle_degrees,
                                reconstruction_);
}

}  // namespace theia
