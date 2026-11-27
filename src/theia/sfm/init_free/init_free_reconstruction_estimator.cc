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

// Drops cameras with a large own orthogonality error (see
// InitFreeOptions::max_camera_orthogonality_error) and recomputes the mean
// orthogonality error and the fraction in front over the remaining cameras.
// Returns a failure message if too many cameras would be dropped.
std::string DropBrokenCameras(const OseProblem& problem,
                              const InitFreeOptions& options,
                              MetricUpgradeResult* metric,
                              std::vector<char>* camera_used) {
  const int n = problem.NumCameras();
  camera_used->assign(n, 1);
  const std::vector<double>& errors = metric->camera_orthogonality_errors;
  if (options.max_camera_orthogonality_error <= 0.0 ||
      static_cast<int>(errors.size()) != n) {
    return "";
  }
  int dropped = 0;
  for (int i = 0; i < n; ++i) {
    if (!(errors[i] <= options.max_camera_orthogonality_error)) {
      (*camera_used)[i] = 0;
      ++dropped;
    }
  }
  if (dropped == 0) return "";
  if (dropped > options.max_fraction_dropped_cameras * n) {
    std::ostringstream out;
    out << dropped << " of " << n
        << " cameras with orthogonality error above "
        << options.max_camera_orthogonality_error << " (at most "
        << options.max_fraction_dropped_cameras * n << " may be dropped)";
    return out.str();
  }
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    if ((*camera_used)[i]) sum += errors[i];
  }
  metric->mean_orthogonality_error = sum / (n - dropped);
  int in_front = 0, total = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int c = problem.observation_camera[k];
    const int j = problem.observation_point[k];
    if (!(*camera_used)[c] || !metric->point_valid[j]) continue;
    const double depth =
        (metric->rotations[c] * metric->points.col(j) + metric->translations[c])
            .z();
    in_front += depth > 0.0;
    ++total;
  }
  metric->fraction_in_front =
      total > 0 ? static_cast<double>(in_front) / total : 0.0;
  return "";
}

// Median angle (degrees) between the relative rotations of the upgraded
// cameras and the two-view relative rotations of the problem's rotation terms
// (R_a R_b^T ~ relative_rotation), over the terms whose cameras are both
// used. -1 if there is no such term.
double MedianRelativeRotationError(const OseProblem& problem,
                                   const MetricUpgradeResult& metric,
                                   const std::vector<char>& camera_used) {
  std::vector<double> errors;
  errors.reserve(problem.rotation_terms.size());
  for (const OseRotationTerm& term : problem.rotation_terms) {
    if (term.camera_a == term.camera_b) continue;  // self terms
    if (!camera_used[term.camera_a] || !camera_used[term.camera_b]) continue;
    const Eigen::Matrix3d D = metric.rotations[term.camera_a] *
                              metric.rotations[term.camera_b].transpose() *
                              term.relative_rotation.transpose();
    const double c = std::clamp(0.5 * (D.trace() - 1.0), -1.0, 1.0);
    errors.push_back(std::acos(c) * 180.0 / M_PI);
  }
  if (errors.empty()) return -1.0;
  std::nth_element(errors.begin(), errors.begin() + errors.size() / 2,
                   errors.end());
  return errors[errors.size() / 2];
}

// Median angle (degrees) between the direction of camera a's center in the
// frame of camera b after the upgrade and the two-view estimate
// (OseRotationTerm::center_a_in_b), over the terms with a known direction and
// both cameras used. A wrong upgrade can keep the rotations nearly right while
// the camera centers are wrong (ETH3D office 4-15 with 20 % outliers: 3.6 deg
// median rotation error but 141 deg after alignment). -1 if no such term.
double MedianTranslationDirectionError(const OseProblem& problem,
                                       const MetricUpgradeResult& metric,
                                       const std::vector<char>& camera_used) {
  std::vector<double> errors;
  for (const OseRotationTerm& term : problem.rotation_terms) {
    const int a = term.camera_a, b = term.camera_b;
    if (a == b || !camera_used[a] || !camera_used[b]) continue;
    if (!(term.center_a_in_b.norm() > 0.0)) continue;
    const Eigen::Vector3d ca =
        -metric.rotations[a].transpose() * metric.translations[a];
    const Eigen::Vector3d cb =
        -metric.rotations[b].transpose() * metric.translations[b];
    const Eigen::Vector3d predicted = metric.rotations[b] * (ca - cb);
    if (!(predicted.norm() > 0.0)) continue;
    const double c = std::clamp(
        predicted.normalized().dot(term.center_a_in_b.normalized()), -1.0,
        1.0);
    errors.push_back(std::acos(c) * 180.0 / M_PI);
  }
  if (errors.empty()) return -1.0;
  std::nth_element(errors.begin(), errors.begin() + errors.size() / 2,
                   errors.end());
  return errors[errors.size() / 2];
}

// Empty if the metric upgrade passes the validity checks of the options.
// The comparisons are negated so that NaN fails.
std::string MetricCheckFailure(const MetricUpgradeResult& metric,
                               const double relative_rotation_error,
                               const double translation_direction_error,
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
  if (options.max_relative_rotation_error_deg > 0.0 &&
      relative_rotation_error >= 0.0 &&
      !(relative_rotation_error <= options.max_relative_rotation_error_deg)) {
    std::ostringstream out;
    if (!failure.empty()) failure += "; ";
    out << "median relative rotation error " << relative_rotation_error
        << " deg > " << options.max_relative_rotation_error_deg
        << " deg (disagrees with the view graph)";
    failure += out.str();
  }
  if (options.max_translation_direction_error_deg > 0.0 &&
      translation_direction_error >= 0.0 &&
      !(translation_direction_error <=
        options.max_translation_direction_error_deg)) {
    std::ostringstream out;
    if (!failure.empty()) failure += "; ";
    out << "median translation direction error "
        << translation_direction_error << " deg > "
        << options.max_translation_direction_error_deg
        << " deg (disagrees with the view graph)";
    failure += out.str();
  }
  // 0 disables the check. Otherwise NaN fails.
  if (options.min_upgrade_conditioning > 0.0 &&
      !(metric.conditioning >= options.min_upgrade_conditioning)) {
    std::ostringstream out;
    if (!failure.empty()) failure += "; ";
    out << "ill-conditioned self-calibration (critical motion?): conditioning "
        << metric.conditioning << " < " << options.min_upgrade_conditioning;
    failure += out.str();
  }
  return failure;
}

// Metric result of a Euclidean (DOSE) solution: the cameras are already metric,
// so H is the identity and no self-calibration is run. Points are dehomogenized
// and marked invalid if their w is (numerically) zero or they are not finite.
bool EuclideanMetricResult(const OseProblem& problem,
                           const ProjectiveReconstruction& rec,
                           MetricUpgradeResult* result,
                           std::string* error) {
  const int n = problem.NumCameras();
  if (n == 0) {
    *error = "No cameras in the problem.";
    return false;
  }
  if (static_cast<int>(rec.cameras.size()) != n) {
    *error = "Euclidean result: camera count does not match the problem.";
    return false;
  }
  if (rec.points.cols() != problem.NumPoints()) {
    *error = "Euclidean result: point count does not match the problem.";
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (!rec.cameras[i].allFinite()) {
      *error = "Euclidean result: camera " + std::to_string(i) +
               " is not finite.";
      return false;
    }
  }

  result->H = Eigen::Matrix4d::Identity();
  result->rotations.resize(n);
  result->translations.resize(n);
  result->focal_lengths.assign(n, 1.0);
  result->camera_orthogonality_errors.resize(n);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    result->rotations[i] = rec.cameras[i].leftCols<3>();
    result->translations[i] = rec.cameras[i].col(3);
    const Eigen::Matrix3d& R = result->rotations[i];
    result->camera_orthogonality_errors[i] =
        (R * R.transpose() - Eigen::Matrix3d::Identity()).norm();
    sum += result->camera_orthogonality_errors[i];
  }
  result->mean_orthogonality_error = sum / n;
  result->linear_mean_orthogonality_error = result->mean_orthogonality_error;
  result->conditioning = 1.0;

  const int num_points = problem.NumPoints();
  result->points.resize(3, num_points);
  result->point_valid.assign(num_points, 0);
  for (int j = 0; j < num_points; ++j) {
    const double w = rec.points(3, j);
    Eigen::Vector3d cartesian = Eigen::Vector3d::Zero();
    bool valid = false;
    if (std::abs(w) > 1e-12) {
      cartesian = rec.points.col(j).head<3>() / w;
      valid = cartesian.allFinite();
    }
    if (!valid) cartesian.setZero();
    result->point_valid[j] = valid;
    result->points.col(j) = cartesian;
  }

  int in_front = 0, total = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int c = problem.observation_camera[k];
    const int j = problem.observation_point[k];
    if (!result->point_valid[j]) continue;
    const double depth =
        (result->rotations[c] * result->points.col(j) + result->translations[c])
            .z();
    in_front += depth > 0.0;
    ++total;
  }
  result->fraction_in_front =
      total > 0 ? static_cast<double>(in_front) / total : 0.0;
  return true;
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
  const bool euclidean = formulation->IsEuclidean();
  if (euclidean && options.projective_refinement.max_num_iterations > 0) {
    *error =
        "Projective refinement would destroy the Euclidean DOSE cameras; set "
        "projective_refinement.max_num_iterations = 0.";
    return false;
  }
  const VarProOptions varpro = ResolvedVarProOptions(options);
  const int num_threads = std::max(1, options.num_threads);
  const int num_restarts = std::max(1, options.num_random_restarts);
  diagnostics->restarts.clear();
  diagnostics->best_restart = -1;
  // Per restart: the reconstruction and its score (+inf if not finite).
  std::vector<ProjectiveReconstruction> candidates;
  std::vector<ProjectiveReconstruction> unrefined_candidates;
  std::vector<std::vector<bool>> candidate_inliers;
  std::vector<double> scores;
  for (int k = 0; k < num_restarts; ++k) {
    const unsigned int seed = options.seed + k;
    const OseCameraVector initial = SampleInitialCameras(
        problem.NumCameras(), options.camera_init, seed);
    OseSolution solution;
    const bool solved =
        euclidean
            ? SolveDoseVarPro(problem, options, varpro, initial, &solution)
            : SolveOseVarPro(problem, *formulation, varpro, initial, &solution);
    if (!solved) {
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

    ProjectiveReconstruction unrefined;
    std::vector<bool> inliers;
    if (refine) {
      unrefined = candidate;
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
        restart.refined_num_outlier_observations =
            refined.num_outlier_observations;
        restart.refined_truncated_rms_error =
            refined.final_truncated_rms_error;
        // With outlier rejection the truncated RMS ranks the restarts (the
        // plain RMS is dominated by the outliers, the inlier RMS rewards
        // rejecting more observations).
        rms = refined.final_truncated_rms_error;
        if (refined.num_outlier_observations > 0) {
          inliers = std::move(refined.observation_inlier);
        }
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
    unrefined_candidates.push_back(std::move(unrefined));
    candidate_inliers.push_back(std::move(inliers));
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
    entry.unrefined = std::move(unrefined_candidates[k]);
    entry.observation_inlier = std::move(candidate_inliers[k]);
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

  // DOSE cameras are metric already, so it needs no upgrade.
  const bool dose = opts.formulation == OseFormulationType::DOSE;
  if (opts.metric_upgrade == MetricUpgradeType::NONE && !dose) {
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
  if (dose && stage_opts.projective_refinement.max_num_iterations > 0) {
    LOG(INFO) << "Init-free: projective refinement is skipped for DOSE "
                 "(Euclidean cameras).";
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
  std::vector<char> camera_used;  // cameras kept after DropBrokenCameras
  // Upgrades ranked[r].reconstruction (or .unrefined) and applies the checks.
  const auto try_upgrade = [&](const int r, const bool use_unrefined) {
    const int k = ranked[r].restart;
    InitFreeRestartDiagnostics& restart = diagnostics_.restarts[k];
    const ProjectiveReconstruction& projective =
        use_unrefined ? ranked[r].unrefined : ranked[r].reconstruction;
    bool& attempted = use_unrefined ? restart.unrefined_upgrade_attempted
                                    : restart.upgrade_attempted;
    std::string& upgrade_message = use_unrefined
                                       ? restart.unrefined_upgrade_message
                                       : restart.upgrade_message;
    attempted = true;
    MetricUpgradeResult candidate;
    std::string upgrade_error;
    const bool upgraded =
        dose ? EuclideanMetricResult(problem, projective, &candidate,
                                     &upgrade_error)
             : UpgradeToMetric(problem,
                               projective.cameras,
                               projective.points,
                               upgrade_options,
                               &candidate,
                               &upgrade_error);
    if (!upgraded) {
      upgrade_message = upgrade_error;
      VLOG(1) << "Init-free: metric upgrade of restart " << k
              << (use_unrefined ? " (unrefined)" : "")
              << " failed: " << upgrade_error;
      return false;
    }
    std::vector<char> candidate_cameras;
    std::string failure =
        DropBrokenCameras(problem, opts, &candidate, &candidate_cameras);
    if (failure.empty() && !dose &&
        std::count(candidate_cameras.begin(), candidate_cameras.end(), 0) >
            0) {
      // The broken cameras also bias the quadric of the others, so the
      // upgrade is estimated again without them and the check repeated.
      MetricUpgradeOptions masked_options = upgrade_options;
      masked_options.camera_mask.assign(candidate_cameras.begin(),
                                        candidate_cameras.end());
      MetricUpgradeResult masked;
      std::string masked_error;
      if (UpgradeToMetric(problem,
                          projective.cameras,
                          projective.points,
                          masked_options,
                          &masked,
                          &masked_error)) {
        candidate = std::move(masked);
        failure =
            DropBrokenCameras(problem, opts, &candidate, &candidate_cameras);
      } else {
        VLOG(1) << "Init-free: upgrade without the broken cameras of restart "
                << k << " failed (" << masked_error
                << "); keeping the first upgrade.";
      }
    }
    const int dropped = static_cast<int>(
        std::count(candidate_cameras.begin(), candidate_cameras.end(), 0));
    if (!use_unrefined) {
      restart.upgrade_fraction_in_front = candidate.fraction_in_front;
      restart.upgrade_mean_orthogonality_error =
          candidate.mean_orthogonality_error;
      restart.upgrade_conditioning = candidate.conditioning;
      restart.upgrade_num_dropped_cameras = dropped;
    }
    const double rotation_error =
        candidate_cameras.empty()
            ? -1.0
            : MedianRelativeRotationError(problem, candidate,
                                          candidate_cameras);
    const double direction_error =
        candidate_cameras.empty()
            ? -1.0
            : MedianTranslationDirectionError(problem, candidate,
                                              candidate_cameras);
    if (!use_unrefined) {
      restart.upgrade_relative_rotation_error_deg = rotation_error;
      restart.upgrade_translation_direction_error_deg = direction_error;
    }
    if (failure.empty()) {
      failure = MetricCheckFailure(candidate, rotation_error,
                                   direction_error, opts);
    }
    if (!failure.empty()) {
      upgrade_message = "rejected: " + failure;
      VLOG(1) << "Init-free: restart " << k
              << (use_unrefined ? " (unrefined)" : "")
              << " rejected: " << failure;
      return false;
    }
    if (use_unrefined) {
      restart.unrefined_upgrade_success = true;
    } else {
      restart.upgrade_success = true;
    }
    upgrade_message = "ok";
    metric = std::move(candidate);
    camera_used = std::move(candidate_cameras);
    return true;
  };
  for (int r = 0; r < static_cast<int>(ranked.size()) && used < 0; ++r) {
    if (try_upgrade(r, false)) used = r;
  }
  const bool has_unrefined =
      !ranked.empty() && !ranked.front().unrefined.cameras.empty();
  if (used < 0 && opts.upgrade_unrefined_fallback && has_unrefined) {
    for (int r = 0; r < static_cast<int>(ranked.size()) && used < 0; ++r) {
      if (try_upgrade(r, true)) {
        used = r;
        diagnostics_.used_unrefined_fallback = true;
      }
    }
  }
  if (used < 0) {
    diagnostics_.best_restart = -1;  // no restart was used
    std::ostringstream message;
    message << "Init-free: no restart passed the metric upgrade checks "
               "(min_fraction_in_front "
            << opts.min_fraction_in_front << ", max_mean_orthogonality_error "
            << opts.max_mean_orthogonality_error << ", min_upgrade_conditioning "
            << opts.min_upgrade_conditioning
            << "). Restarts in ranked order:";
    for (const RankedProjectiveReconstruction& entry : ranked) {
      const InitFreeRestartDiagnostics& restart =
          diagnostics_.restarts[entry.restart];
      message << " restart " << entry.restart << " (seed " << restart.seed
              << "): " << restart.upgrade_message;
      if (restart.unrefined_upgrade_attempted) {
        message << ", unrefined: " << restart.unrefined_upgrade_message;
      }
      message << ";";
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
    if (!camera_used[i]) {
      diagnostics_.dropped_view_ids.push_back(problem.view_ids[i]);
      continue;
    }
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
  // Points need two observations in kept cameras (dropped cameras are not
  // estimated).
  std::vector<int> kept_observations(problem.NumPoints(), 0);
  for (int k = 0; k < problem.NumObservations(); ++k) {
    if (camera_used[problem.observation_camera[k]]) {
      ++kept_observations[problem.observation_point[k]];
    }
  }
  for (int j = 0; j < problem.NumPoints(); ++j) {
    if (!metric.point_valid[j] || kept_observations[j] < 2) continue;
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
    // Real tracks contain wrongly merged observations, which pull the metric
    // bundle adjustment away from the upgrade (ETH3D relief_2 with SIFT:
    // median rotation error 0.11 deg after the upgrade, 0.66 deg after BA).
    // Tracks with an observation rejected by the projective refinement are
    // therefore left out. A reprojection filter at this point would also
    // remove good tracks, because the upgraded cameras are not yet accurate
    // (office: 37 % of the tracks), so without the projective outlier
    // rejection nothing is removed here.
    const std::vector<bool>& inlier =
        diagnostics_.used_unrefined_fallback
            ? std::vector<bool>()
            : ranked[used].observation_inlier;
    int removed_before_ba = 0;
    if (!inlier.empty()) {
      std::unordered_set<TrackId> outlier_tracks;
      for (int k = 0; k < problem.NumObservations(); ++k) {
        if (!inlier[k]) {
          outlier_tracks.insert(
              problem.track_ids[problem.observation_point[k]]);
        }
      }
      for (const TrackId track_id : outlier_tracks) {
        Track* track = reconstruction->MutableTrack(track_id);
        if (track != nullptr && track->IsEstimated()) {
          track->SetEstimated(false);
          ++removed_before_ba;
        }
      }
    }
    VLOG(1) << "Init-free: " << removed_before_ba
            << " outlier tracks left out of the metric bundle adjustment.";
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
    // As in the global pipeline: if the filter removed tracks, adjust again.
    if (removed > 0) {
      const BundleAdjustmentSummary second =
          BundleAdjustReconstruction(ba_options, reconstruction);
      if (!second.success) {
        summary.message = "Init-free: metric bundle adjustment failed.";
        return summary;
      }
      SetOutlierTracksToUnestimated(options_.max_reprojection_error_in_pixels,
                                    options_.min_triangulation_angle_degrees,
                                    reconstruction);
    }
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
    if (best.refined_num_outlier_observations > 0) {
      message << " (" << best.refined_num_outlier_observations
              << " outlier observations removed)";
    }
  }
  if (diagnostics_.used_unrefined_fallback) {
    message << ", metric upgrade of the unrefined OSE solution (fallback)";
  }
  if (!diagnostics_.dropped_view_ids.empty()) {
    message << ", " << diagnostics_.dropped_view_ids.size()
            << " camera(s) dropped after the upgrade";
  }
  message << ", "
          << 100.0 * diagnostics_.fraction_in_front
          << "% observations in front, OSE time " << diagnostics_.ose_time
          << " s.";
  summary.message = message.str();
  return summary;
}

}  // namespace theia
