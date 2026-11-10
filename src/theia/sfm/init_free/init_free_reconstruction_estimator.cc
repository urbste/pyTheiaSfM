// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/init_free_reconstruction_estimator.h"

#include <glog/logging.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/filter_view_graph_by_relative_inliers.h"
#include "theia/sfm/init_free/camera_sampler.h"
#include "theia/sfm/init_free/metric_upgrade.h"
#include "theia/sfm/init_free/ose_formulation.h"
#include "theia/sfm/init_free/projective_refinement.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/view_graph/remove_disconnected_view_pairs.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/timer.h"

namespace theia {

bool RunProjectiveStages(const OseProblem& problem,
                         const InitFreeOptions& options,
                         ProjectiveReconstruction* best,
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
  const bool refine = options.projective_refinement.max_num_iterations > 0;
  if (refine && formulation->IsRadial()) {
    *error =
        "Projective refinement needs full 3x4 cameras; the radial RPOSE "
        "formulation estimates only two rows.";
    return false;
  }
  const VarProOptions varpro = ResolvedVarProOptions(options);
  const int num_threads = std::max(1, options.num_threads);
  const int num_restarts = std::max(1, options.num_random_restarts);
  double best_score = std::numeric_limits<double>::infinity();
  diagnostics->restarts.clear();
  diagnostics->best_restart = -1;
  for (int k = 0; k < num_restarts; ++k) {
    const unsigned int seed = options.seed + k;
    const OseCameraVector initial = SampleInitialCameras(
        problem.NumCameras(), options.camera_init, seed);
    OseSolution solution;
    if (!SolveOseVarPro(
            problem, *formulation, varpro, initial, &solution)) {
      *error = solution.message;
      return false;
    }
    ProjectiveReconstruction candidate;
    candidate.cameras = std::move(solution.cameras);
    candidate.points = solution.points.colwise().homogeneous();

    InitFreeRestartDiagnostics restart;
    restart.seed = seed;
    restart.initial_cost = solution.initial_cost;
    restart.final_cost = solution.final_cost;
    restart.num_iterations = solution.num_iterations;
    restart.converged = solution.converged;
    restart.solve_time_seconds = solution.solve_time_seconds;
    restart.message = solution.message;
    // Radial cameras have no meaningful third row, so no projective RMS.
    restart.ose_rms_error =
        formulation->IsRadial()
            ? -1.0
            : ProjectiveRmsError(problem, candidate.cameras, candidate.points);
    double rms = restart.ose_rms_error;

    if (refine) {
      ProjectiveRefinementResult refined;
      if (RefineProjectiveReconstruction(problem,
                                         candidate.cameras,
                                         candidate.points,
                                         options.projective_refinement,
                                         num_threads,
                                         &refined)) {
        candidate.cameras = std::move(refined.cameras);
        candidate.points = std::move(refined.points);
        restart.refined_rms_error = refined.final_rms_error;
        rms = refined.final_rms_error;
      } else {
        LOG(WARNING) << "Projective refinement of restart " << k
                     << " failed: " << refined.message;
      }
      restart.refinement_iterations = refined.num_iterations;
      restart.refinement_time_seconds = refined.solve_time_seconds;
    }
    diagnostics->restarts.push_back(restart);
    VLOG(1) << "Restart " << k << " (seed " << seed << "): OSE cost "
            << restart.initial_cost << " -> " << restart.final_cost << " in "
            << restart.num_iterations << " iterations, RMS "
            << restart.ose_rms_error << " -> " << rms;

    const double score =
        options.restart_selection == InitFreeRestartSelection::OSE_COST ||
                formulation->IsRadial()
            ? restart.final_cost
            : rms;
    if (std::isfinite(score) && score < best_score) {
      best_score = score;
      diagnostics->best_restart = k;
      *best = std::move(candidate);
    }
  }
  if (diagnostics->best_restart < 0) {
    *error = "No restart produced a finite result.";
    return false;
  }
  return true;
}

InitFreeReconstructionEstimator::InitFreeReconstructionEstimator(
    const ReconstructionEstimatorOptions& options)
    : options_(options) {}

ReconstructionEstimatorSummary InitFreeReconstructionEstimator::Estimate(
    ViewGraph* view_graph, Reconstruction* reconstruction) {
  CHECK_NOTNULL(view_graph);
  CHECK_NOTNULL(reconstruction);
  InitFreeOptions opts = options_.init_free_options;
  if (opts.num_threads <= 0) {
    opts.num_threads = options_.num_threads;
  }
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

  if (opts.formulation == OseFormulationType::RPOSE &&
      opts.metric_upgrade != MetricUpgradeType::NONE) {
    summary.message =
        "Init-free: RPOSE estimates radial (2x4) cameras; the metric upgrade "
        "for radial cameras is not implemented. Use the stage-wise API.";
    return summary;
  }

  // 3. + 4. OSE stage and projective refinement for every random start.
  timer.Reset();
  ProjectiveReconstruction projective;
  if (!RunProjectiveStages(problem, opts, &projective, &diagnostics_, &error)) {
    summary.message = "Init-free: " + error;
    return summary;
  }
  diagnostics_.ose_time = timer.ElapsedTimeInSeconds();

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
          problem, projective.cameras, projective.points, &metric, &error)) {
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
          << diagnostics_.restarts.size() << " restart(s), best OSE cost "
          << best.final_cost << " after " << best.num_iterations
          << " iterations (" << best.message << "), projective RMS "
          << best.ose_rms_error;
  if (best.refined_rms_error >= 0.0) {
    message << " -> " << best.refined_rms_error << " after refinement";
  }
  message << ", "
          << 100.0 * diagnostics_.fraction_in_front
          << "% observations in front, OSE time " << diagnostics_.ose_time
          << " s.";
  summary.message = message.str();
  return summary;
}

}  // namespace theia
