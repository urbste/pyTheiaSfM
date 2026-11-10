// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/init_free_reconstruction_estimator.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
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

namespace {

// Empty if the metric upgrade passes the validity checks of the options.
// The comparisons are negated so that NaN fails.
std::string MetricCheckFailure(const MetricUpgradeResult& metric,
                               const InitFreeOptions& options) {
  std::string failure;
  if (!(metric.fraction_in_front >= options.min_fraction_in_front)) {
    std::ostringstream out;
    out << "fraction in front " << metric.fraction_in_front << " < "
        << options.min_fraction_in_front;
    failure += out.str();
  }
  if (!(metric.mean_orthogonality_error <=
        options.max_mean_orthogonality_error)) {
    std::ostringstream out;
    if (!failure.empty()) failure += "; ";
    out << "mean orthogonality error " << metric.mean_orthogonality_error
        << " > " << options.max_mean_orthogonality_error;
    failure += out.str();
  }
  return failure;
}

}  // namespace

bool RunProjectiveStagesRanked(
    const OseProblem& problem,
    const InitFreeOptions& options,
    std::vector<RankedProjectiveReconstruction>* ranked,
    InitFreeDiagnostics* diagnostics,
    std::string* error) {
  CHECK_NOTNULL(ranked);
  CHECK_NOTNULL(diagnostics);
  CHECK_NOTNULL(error);
  ranked->clear();
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
  diagnostics->restarts.clear();
  diagnostics->best_restart = -1;
  // Per restart: the reconstruction and its score (+inf if not finite).
  std::vector<ProjectiveReconstruction> candidates;
  std::vector<double> scores;
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
    candidates.push_back(std::move(candidate));
    scores.push_back(std::isfinite(score)
                         ? score
                         : std::numeric_limits<double>::infinity());
  }

  // Stable sort: ties keep the restart order.
  std::vector<int> order(num_restarts);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&scores](int a, int b) {
    return scores[a] < scores[b];
  });
  for (const int k : order) {
    if (!std::isfinite(scores[k])) break;
    RankedProjectiveReconstruction entry;
    entry.restart = k;
    entry.reconstruction = std::move(candidates[k]);
    ranked->push_back(std::move(entry));
  }
  if (ranked->empty()) {
    *error = "No restart produced a finite result.";
    return false;
  }
  diagnostics->best_restart = ranked->front().restart;
  return true;
}

bool RunProjectiveStages(const OseProblem& problem,
                         const InitFreeOptions& options,
                         ProjectiveReconstruction* best,
                         InitFreeDiagnostics* diagnostics,
                         std::string* error) {
  std::vector<RankedProjectiveReconstruction> ranked;
  if (!RunProjectiveStagesRanked(problem, options, &ranked, diagnostics,
                                 error)) {
    return false;
  }
  *best = std::move(ranked.front().reconstruction);
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

  if (opts.metric_upgrade == MetricUpgradeType::NONE) {
    summary.message =
        "Init-free: metric_upgrade is NONE, so no metric reconstruction can "
        "be written. Use the stage-wise API (BuildOseProblem, SolveOse) for "
        "projective results.";
    return summary;
  }

  // 3. + 4. OSE stage and projective refinement for every random start, ranked
  // by restart_selection. RPOSE cameras are radial (two rows), so projective
  // refinement is skipped; the metric bundle adjustment refines the full
  // cameras instead.
  const bool radial = opts.formulation == OseFormulationType::RPOSE;
  InitFreeOptions stage_opts = opts;
  if (radial && stage_opts.projective_refinement.max_num_iterations > 0) {
    LOG(INFO) << "Init-free: projective refinement is skipped for RPOSE "
                 "(radial cameras).";
    stage_opts.projective_refinement.max_num_iterations = 0;
  }
  timer.Reset();
  std::vector<RankedProjectiveReconstruction> ranked;
  if (!RunProjectiveStagesRanked(problem, stage_opts, &ranked, &diagnostics_,
                                 &error)) {
    summary.message = "Init-free: " + error;
    return summary;
  }
  diagnostics_.ose_time = timer.ElapsedTimeInSeconds();

  // 5. Metric upgrade. The restarts are upgraded in ranked order, and the first
  // one that succeeds and passes the validity checks is used.
  timer.Reset();
  MetricUpgradeOptions upgrade_options;
  // The linear radial estimate is unreliable (the RPOSE affine term biases the
  // OSE solution), so the nonlinear refinement always runs for radial cameras.
  upgrade_options.nonlinear_refinement =
      opts.metric_upgrade == MetricUpgradeType::LINEAR_AND_NONLINEAR || radial;
  if (radial && opts.metric_upgrade == MetricUpgradeType::LINEAR) {
    LOG(INFO) << "Init-free: RPOSE always runs the nonlinear metric upgrade "
                 "(metric_upgrade LINEAR is overridden).";
  }
  upgrade_options.shared_focal_length = opts.shared_focal_length;
  upgrade_options.radial_cameras = radial;
  int used = -1;  // position in ranked
  MetricUpgradeResult metric;
  for (int r = 0; r < static_cast<int>(ranked.size()); ++r) {
    const int k = ranked[r].restart;
    InitFreeRestartDiagnostics& restart = diagnostics_.restarts[k];
    restart.upgrade_attempted = true;
    MetricUpgradeResult candidate;
    std::string upgrade_error;
    if (!UpgradeToMetric(problem,
                         ranked[r].reconstruction.cameras,
                         ranked[r].reconstruction.points,
                         upgrade_options,
                         &candidate,
                         &upgrade_error)) {
      restart.upgrade_message = upgrade_error;
      VLOG(1) << "Init-free: metric upgrade of restart " << k
              << " failed: " << upgrade_error;
      continue;
    }
    restart.upgrade_fraction_in_front = candidate.fraction_in_front;
    restart.upgrade_mean_orthogonality_error =
        candidate.mean_orthogonality_error;
    const std::string failure = MetricCheckFailure(candidate, opts);
    if (!failure.empty()) {
      restart.upgrade_message = "rejected: " + failure;
      VLOG(1) << "Init-free: restart " << k << " rejected: " << failure;
      continue;
    }
    restart.upgrade_success = true;
    restart.upgrade_message = "ok";
    used = r;
    metric = std::move(candidate);
    break;
  }
  if (used < 0) {
    diagnostics_.best_restart = -1;  // no restart was used
    std::ostringstream message;
    message << "Init-free: no restart passed the metric upgrade checks "
               "(min_fraction_in_front "
            << opts.min_fraction_in_front << ", max_mean_orthogonality_error "
            << opts.max_mean_orthogonality_error << "). Restarts in ranked order:";
    for (const RankedProjectiveReconstruction& entry : ranked) {
      const InitFreeRestartDiagnostics& restart =
          diagnostics_.restarts[entry.restart];
      message << " restart " << entry.restart << " (seed " << restart.seed
              << "): " << restart.upgrade_message << ";";
    }
    summary.message = message.str();
    return summary;
  }
  diagnostics_.best_restart = ranked[used].restart;
  diagnostics_.metric_upgrade_success = true;
  diagnostics_.fraction_in_front = metric.fraction_in_front;
  diagnostics_.mean_orthogonality_error = metric.mean_orthogonality_error;
  diagnostics_.linear_mean_orthogonality_error =
      metric.linear_mean_orthogonality_error;
  if (problem.normalization == OseObservationNormalization::PIXEL_ISOTROPIC) {
    for (int i = 0; i < problem.NumCameras(); ++i) {
      diagnostics_.focal_lengths.push_back(metric.focal_lengths[i] /
                                           problem.normalizations[i](0, 0));
    }
  }
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
    if (problem.normalization ==
        OseObservationNormalization::PIXEL_ISOTROPIC) {
      // Observations were m = T x with T = [s 0 -s cx; 0 s -s cy; 0 0 1], so
      // K_pixel = T^-1 diag(f, f, 1): focal f / s, principal point (cx, cy).
      const Eigen::Matrix3d& T = problem.normalizations[i];
      camera->SetFocalLength(metric.focal_lengths[i] / T(0, 0));
      camera->SetPrincipalPoint(-T(0, 2) / T(0, 0), -T(1, 2) / T(1, 1));
    }
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
          << diagnostics_.restarts.size() << " restart(s), restart "
          << diagnostics_.best_restart << " used (" << used + 1 << " of "
          << ranked.size() << " ranked restart(s) tried), OSE cost "
          << best.final_cost << " after " << best.num_iterations
          << " iterations (" << best.message << ")";
  if (best.ose_rms_error >= 0.0) {
    message << ", projective RMS " << best.ose_rms_error;
  }
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
