// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/init_free_reconstruction_estimator.h"

#include <glog/logging.h>

#include <limits>
#include <sstream>
#include <unordered_set>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/filter_view_graph_by_relative_inliers.h"
#include "theia/sfm/init_free/camera_sampler.h"
#include "theia/sfm/init_free/metric_upgrade.h"
#include "theia/sfm/init_free/ose_formulation.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/view_graph/remove_disconnected_view_pairs.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/timer.h"

namespace theia {

bool RunOseRestarts(const OseProblem& problem,
                    const InitFreeOptions& options,
                    OseSolution* best,
                    InitFreeDiagnostics* diagnostics,
                    std::string* error) {
  std::unique_ptr<OseFormulation> formulation =
      CreateOseFormulation(options, error);
  if (formulation == nullptr) {
    return false;
  }
  if (formulation->UsesRotationTerms() && problem.rotation_terms.empty()) {
    *error = "The formulation needs rotation terms but the problem has none.";
    return false;
  }
  const int num_restarts = std::max(1, options.num_random_restarts);
  double best_cost = std::numeric_limits<double>::infinity();
  diagnostics->restarts.clear();
  diagnostics->best_restart = -1;
  for (int k = 0; k < num_restarts; ++k) {
    const unsigned int seed = options.seed + k;
    const OseCameraVector initial = SampleInitialCameras(
        problem.NumCameras(), options.camera_init, seed);
    OseSolution solution;
    if (!SolveOseVarPro(
            problem, *formulation, options.varpro, initial, &solution)) {
      *error = solution.message;
      return false;
    }
    InitFreeRestartDiagnostics restart;
    restart.seed = seed;
    restart.initial_cost = solution.initial_cost;
    restart.final_cost = solution.final_cost;
    restart.num_iterations = solution.num_iterations;
    restart.converged = solution.converged;
    restart.solve_time_seconds = solution.solve_time_seconds;
    restart.message = solution.message;
    diagnostics->restarts.push_back(restart);
    VLOG(1) << "OSE restart " << k << " (seed " << seed
            << "): cost " << solution.initial_cost << " -> "
            << solution.final_cost << " in " << solution.num_iterations
            << " iterations. " << solution.message;
    if (solution.final_cost < best_cost) {
      best_cost = solution.final_cost;
      diagnostics->best_restart = k;
      *best = std::move(solution);
    }
  }
  return diagnostics->best_restart >= 0;
}

InitFreeReconstructionEstimator::InitFreeReconstructionEstimator(
    const ReconstructionEstimatorOptions& options)
    : options_(options) {}

ReconstructionEstimatorSummary InitFreeReconstructionEstimator::Estimate(
    ViewGraph* view_graph, Reconstruction* reconstruction) {
  CHECK_NOTNULL(view_graph);
  CHECK_NOTNULL(reconstruction);
  const InitFreeOptions& opts = options_.init_free_options;
  diagnostics_ = InitFreeDiagnostics();
  ReconstructionEstimatorSummary summary;
  Timer total_timer;
  Timer timer;

  // 1. View graph filtering, as in the global estimator.
  FilterViewGraphForEstimation(options_.view_graph_min_relative_inliers,
                               options_.view_graph_max_cycle_rotation_error_degrees,
                               view_graph,
                               reconstruction);
  std::unordered_set<ViewIdPair> weak_pairs;
  for (const auto& [pair, info] : view_graph->GetAllEdges()) {
    if (info.num_verified_matches < options_.min_num_two_view_inliers) {
      weak_pairs.insert(pair);
    }
  }
  for (const ViewIdPair& pair : weak_pairs) {
    view_graph->RemoveEdge(pair.first, pair.second);
  }
  RemoveDisconnectedViewPairs(view_graph);
  if (view_graph->NumEdges() < 1) {
    summary.message = "Init-free: view graph is empty after filtering.";
    return summary;
  }

  // 2. OSE problem.
  OseProblem problem;
  std::string error;
  if (!BuildOseProblem(*reconstruction, view_graph, opts, &problem, &error)) {
    summary.message = "Init-free: " + error;
    return summary;
  }
  diagnostics_.num_cameras = problem.NumCameras();
  diagnostics_.num_points = problem.NumPoints();
  diagnostics_.num_observations = problem.NumObservations();
  diagnostics_.num_rotation_terms =
      static_cast<int>(problem.rotation_terms.size());
  diagnostics_.problem_build_time = timer.ElapsedTimeInSeconds();

  // 3. OSE stage from random starts.
  timer.Reset();
  OseSolution solution;
  if (!RunOseRestarts(problem, opts, &solution, &diagnostics_, &error)) {
    summary.message = "Init-free: " + error;
    return summary;
  }
  diagnostics_.ose_time = timer.ElapsedTimeInSeconds();

  // 4. Projective refinement (stage 2 of the study).
  if (opts.projective_refinement_max_iterations > 0) {
    LOG(WARNING) << "Init-free: projective refinement is not implemented yet "
                    "and is skipped.";
  }

  // 5. Metric upgrade.
  timer.Reset();
  if (opts.metric_upgrade == MetricUpgradeType::NONE) {
    summary.message =
        "Init-free: metric_upgrade is NONE, so no metric reconstruction can "
        "be written. Use the stage-wise API (BuildOseProblem, SolveOse) for "
        "projective results.";
    return summary;
  }
  if (problem.normalization != OseObservationNormalization::CALIBRATED_RAYS) {
    summary.message =
        "Init-free: the uncalibrated metric upgrade is not implemented yet; "
        "use CALIBRATED_RAYS.";
    return summary;
  }
  if (opts.metric_upgrade == MetricUpgradeType::LINEAR_AND_NONLINEAR) {
    LOG(WARNING) << "Init-free: nonlinear metric refinement is not "
                    "implemented yet; using the linear upgrade only.";
  }
  MetricUpgradeResult metric;
  if (!UpgradeToMetricCalibrated(
          problem, solution.cameras, solution.points, &metric, &error)) {
    summary.message = "Init-free: " + error;
    return summary;
  }
  diagnostics_.metric_upgrade_success = true;
  diagnostics_.fraction_in_front = metric.fraction_in_front;
  diagnostics_.mean_orthogonality_error = metric.mean_orthogonality_error;
  diagnostics_.metric_upgrade_time = timer.ElapsedTimeInSeconds();

  // 6. Write the metric result into the reconstruction.
  for (const ViewId view_id : reconstruction->ViewIds()) {
    reconstruction->MutableView(view_id)->SetEstimated(false);
  }
  for (const TrackId track_id : reconstruction->TrackIds()) {
    reconstruction->MutableTrack(track_id)->SetEstimated(false);
  }
  for (int i = 0; i < problem.NumCameras(); ++i) {
    View* view = reconstruction->MutableView(problem.view_ids[i]);
    Camera* camera = view->MutableCamera();
    const Eigen::Matrix3d& R = metric.rotations[i];
    camera->SetOrientationFromRotationMatrix(R);
    camera->SetPosition(-R.transpose() * metric.translations[i]);
    view->SetEstimated(true);
  }
  for (int j = 0; j < problem.NumPoints(); ++j) {
    if (!metric.point_valid[j]) continue;
    Track* track = reconstruction->MutableTrack(problem.track_ids[j]);
    track->SetPoint(metric.points.col(j).homogeneous());
    track->SetEstimated(true);
  }
  summary.pose_estimation_time = diagnostics_.problem_build_time +
                                 diagnostics_.ose_time +
                                 diagnostics_.metric_upgrade_time;

  // 7. Metric bundle adjustment.
  if (opts.run_metric_bundle_adjustment) {
    timer.Reset();
    if (options_.track_parametrization_type ==
        TrackParametrizationType::INVERSE_DEPTH) {
      reconstruction->InitializeInverseDepth();
    }
    const BundleAdjustmentOptions ba_options =
        SetBundleAdjustmentOptions(options_, problem.NumCameras());
    const BundleAdjustmentSummary ba_summary =
        BundleAdjustReconstruction(ba_options, reconstruction);
    if (!ba_summary.success) {
      summary.message = "Init-free: metric bundle adjustment failed.";
      return summary;
    }
    const int removed = SetOutlierTracksToUnestimated(
        options_.max_reprojection_error_in_pixels,
        options_.min_triangulation_angle_degrees,
        reconstruction);
    VLOG(1) << "Init-free: removed " << removed << " outlier tracks.";
    diagnostics_.bundle_adjustment_time = timer.ElapsedTimeInSeconds();
    summary.bundle_adjustment_time = diagnostics_.bundle_adjustment_time;
  }

  GetEstimatedViewsFromReconstruction(*reconstruction,
                                      &summary.estimated_views);
  GetEstimatedTracksFromReconstruction(*reconstruction,
                                       &summary.estimated_tracks);
  summary.success = true;
  summary.total_time = total_timer.ElapsedTimeInSeconds();

  const InitFreeRestartDiagnostics& best =
      diagnostics_.restarts[diagnostics_.best_restart];
  std::ostringstream message;
  message << "Init-free reconstruction: " << problem.NumCameras()
          << " cameras, " << problem.NumPoints() << " points, "
          << diagnostics_.restarts.size() << " restart(s), best cost "
          << best.final_cost << " after " << best.num_iterations
          << " iterations (" << best.message << "), "
          << 100.0 * diagnostics_.fraction_in_front
          << "% observations in front, OSE time " << diagnostics_.ose_time
          << " s.";
  summary.message = message.str();
  return summary;
}

}  // namespace theia
