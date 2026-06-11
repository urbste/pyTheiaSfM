// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#include "theia/sfm/transformation/cross_reconstruction_sim3_pose_graph_optimizer.h"

#include <algorithm>
#include <cmath>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "theia/sfm/track.h"
#include "theia/sfm/transformation/cross_reconstruction_pose_graph_errors.h"
#include "theia/sfm/transformation/sim3_pose_from_view.h"

namespace theia {
namespace {

bool HasVariablePose(const Sim3LieMap& poses, const ViewId view_id) {
  return poses.find(view_id) != poses.end();
}

constexpr double kSim3SigmaLowerBound = -20.0;
constexpr double kSim3SigmaUpperBound = 20.0;

void AddSim3LieParameterBlock(ceres::Problem* problem,
                              double* data,
                              bool constant) {
  problem->AddParameterBlock(data, 7);
  problem->SetParameterLowerBound(data, 6, kSim3SigmaLowerBound);
  problem->SetParameterUpperBound(data, 6, kSim3SigmaUpperBound);
  if (constant) {
    problem->SetParameterBlockConstant(data);
  }
}

double MaxAbsInJacobianRows(const ceres::CRSMatrix& jacobian,
                            int row_begin,
                            int row_end) {
  double max_abs = 0.0;
  for (int row = row_begin; row < row_end; ++row) {
    const int start = jacobian.rows[row];
    const int end = jacobian.rows[row + 1];
    for (int idx = start; idx < end; ++idx) {
      max_abs = std::max(max_abs, std::abs(jacobian.values[idx]));
    }
  }
  return max_abs;
}

}  // namespace

CrossReconstructionSim3PoseGraphOptimizer::
    CrossReconstructionSim3PoseGraphOptimizer(
        const CrossReconstructionPoseGraphOptions& options)
    : options_(options) {}

void CrossReconstructionSim3PoseGraphOptimizer::ClearProblem() {
  problem_.reset();
  anchor_losses_.clear();
  num_sequential_residuals_ = 0;
  num_anchor_residuals_ = 0;
  num_scale_smooth_residuals_ = 0;
}

void CrossReconstructionSim3PoseGraphOptimizer::SetFixedReconstruction(
    const Reconstruction& fixed_reconstruction,
    const std::vector<ViewId>& anchor_view_ids) {
  fixed_lies_.clear();
  GetSim3LiesFromReconstruction(
      fixed_reconstruction, anchor_view_ids, &fixed_lies_);
}

void CrossReconstructionSim3PoseGraphOptimizer::SetVariableReconstruction(
    const Reconstruction& variable_reconstruction,
    const std::vector<ViewId>& keyframe_view_ids) {
  variable_lies_.clear();
  variable_keyframe_order_ = keyframe_view_ids;
  GetSim3LiesFromReconstruction(
      variable_reconstruction, keyframe_view_ids, &variable_lies_);
}

void CrossReconstructionSim3PoseGraphOptimizer::SetInitialVariablePoses(
    const Sim3LieMap& initial_poses) {
  for (const auto& entry : initial_poses) {
    auto it = variable_lies_.find(entry.first);
    if (it != variable_lies_.end()) {
      it->second = entry.second;
    }
  }
}

void CrossReconstructionSim3PoseGraphOptimizer::AddSequentialEdge(
    const SequentialSim3Edge& edge) {
  sequential_edges_.push_back(edge);
}

void CrossReconstructionSim3PoseGraphOptimizer::AddCrossViewEdge(
    const CrossViewAnchorEdge& edge) {
  cross_view_edges_.push_back(edge);
}

void CrossReconstructionSim3PoseGraphOptimizer::AddScaleSmoothnessEdge(
    ViewId view_i, ViewId view_j, double weight) {
  scale_smooth_edges_.emplace_back(view_i, view_j, weight);
}

void CrossReconstructionSim3PoseGraphOptimizer::SetConstraints(
    const CrossReconstructionConstraints& constraints) {
  sequential_edges_ = constraints.sequential_edges;
  cross_view_edges_ = constraints.cross_view_edges;
  scale_smooth_edges_.clear();
}

void CrossReconstructionSim3PoseGraphOptimizer::AddAutoScaleSmoothnessEdges() {
  if (!options_.auto_scale_smoothness || variable_keyframe_order_.size() < 2) {
    return;
  }
  for (size_t i = 1; i < variable_keyframe_order_.size(); ++i) {
    AddScaleSmoothnessEdge(variable_keyframe_order_[i - 1],
                           variable_keyframe_order_[i],
                           options_.scale_smooth_weight);
  }
}

bool CrossReconstructionSim3PoseGraphOptimizer::PosesAreFinite(
    const Sim3LieMap& poses, const char* label) const {
  for (const auto& entry : poses) {
    const Eigen::Matrix<double, 7, 1>& lie = entry.second;
    for (int i = 0; i < 7; ++i) {
      if (!std::isfinite(lie(i))) {
        LOG(ERROR) << label << ": non-finite pose for view " << entry.first
                   << " component " << i << " = " << lie(i);
        return false;
      }
    }
  }
  return true;
}

void CrossReconstructionSim3PoseGraphOptimizer::LogCostBreakdown(
    const char* label) const {
  if (!problem_) {
    return;
  }
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = true;
  double total_cost = 0.0;
  std::vector<double> residuals;
  if (!problem_->Evaluate(eval_options, &total_cost, &residuals, nullptr,
                          nullptr)) {
    LOG(WARNING) << label << ": Problem::Evaluate failed.";
    return;
  }
  LOG(INFO) << label << ": total cost=" << total_cost
            << " num_residuals=" << residuals.size()
            << " sequential_blocks=" << num_sequential_residuals_
            << " anchor_blocks=" << num_anchor_residuals_
            << " scale_smooth_blocks=" << num_scale_smooth_residuals_;
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (!std::isfinite(residuals[i])) {
      LOG(ERROR) << label << ": non-finite residual[" << i << "]="
                 << residuals[i];
    }
  }
}

void CrossReconstructionSim3PoseGraphOptimizer::LogJacobianBreakdown(
    const char* label) const {
  if (!problem_) {
    return;
  }
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = false;
  std::vector<double> residuals;
  std::vector<double> gradient;
  ceres::CRSMatrix jacobian;
  if (!problem_->Evaluate(eval_options, nullptr, &residuals, &gradient,
                          &jacobian)) {
    LOG(WARNING) << label << ": Jacobian evaluate failed.";
    return;
  }

  double max_grad = 0.0;
  for (const double g : gradient) {
    max_grad = std::max(max_grad, std::abs(g));
  }

  const int seq_rows = num_sequential_residuals_ * 7;
  const int anchor_rows = num_anchor_residuals_ * 7;
  const int scale_rows = num_scale_smooth_residuals_;
  const double max_j_seq =
      seq_rows > 0 ? MaxAbsInJacobianRows(jacobian, 0, seq_rows) : 0.0;
  const double max_j_anchor = anchor_rows > 0
                                  ? MaxAbsInJacobianRows(jacobian, seq_rows,
                                                         seq_rows + anchor_rows)
                                  : 0.0;
  const double max_j_scale =
      scale_rows > 0 ? MaxAbsInJacobianRows(jacobian, seq_rows + anchor_rows,
                                            seq_rows + anchor_rows + scale_rows)
                     : 0.0;

  LOG(INFO) << label << ": max |gradient|=" << max_grad
            << " max|J| sequential=" << max_j_seq
            << " anchor=" << max_j_anchor << " scale_smooth=" << max_j_scale;
}

void CrossReconstructionSim3PoseGraphOptimizer::FillResidualCostSummary(
    CrossReconstructionPoseGraphSummary* summary) const {
  if (!problem_ || summary == nullptr) {
    return;
  }
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.apply_loss_function = true;
  double total_cost = 0.0;
  std::vector<double> residuals;
  if (!problem_->Evaluate(eval_options, &total_cost, &residuals, nullptr,
                          nullptr)) {
    return;
  }
  size_t offset = 0;
  auto accumulate = [&](int num_blocks, int residual_dim, double* out_cost) {
    *out_cost = 0.0;
    for (int b = 0; b < num_blocks; ++b) {
      for (int r = 0; r < residual_dim; ++r) {
        if (offset < residuals.size()) {
          const double v = residuals[offset++];
          *out_cost += v * v;
        }
      }
    }
  };
  accumulate(num_sequential_residuals_, 7,
             &summary->sequential_residual_cost);
  accumulate(num_anchor_residuals_, 7, &summary->anchor_residual_cost);
  accumulate(num_scale_smooth_residuals_, 1,
             &summary->scale_smooth_residual_cost);
}

bool CrossReconstructionSim3PoseGraphOptimizer::BuildProblem() {
  if (variable_lies_.empty()) {
    LOG(ERROR) << "No variable keyframe poses set.";
    return false;
  }

  ClearProblem();
  problem_.reset(new ceres::Problem());
  scale_smooth_edges_.clear();
  AddAutoScaleSmoothnessEdges();

  for (auto& entry : variable_lies_) {
    AddSim3LieParameterBlock(problem_.get(), entry.second.data(), false);
  }
  for (auto& entry : fixed_lies_) {
    AddSim3LieParameterBlock(problem_.get(), entry.second.data(), true);
  }

  for (const SequentialSim3Edge& edge : sequential_edges_) {
    if (!HasVariablePose(variable_lies_, edge.view_id_i) ||
        !HasVariablePose(variable_lies_, edge.view_id_j)) {
      LOG(WARNING) << "Skipping sequential edge with unknown variable views.";
      continue;
    }
    Eigen::Matrix<double, 7, 7> sqrt_info =
        options_.sequential_weight * edge.sqrt_information;
    ceres::CostFunction* cost = ScaleFreeSequentialSim3ErrorTerm::Create(
        edge.measured_S_ji,
        sqrt_info,
        options_.sequential_translation_magnitude_weight);
    problem_->AddResidualBlock(
        cost,
        nullptr,
        variable_lies_.at(edge.view_id_i).data(),
        variable_lies_.at(edge.view_id_j).data());
    ++num_sequential_residuals_;
  }

  for (const CrossViewAnchorEdge& edge : cross_view_edges_) {
    if (!HasVariablePose(variable_lies_, edge.variable_view_id)) {
      LOG(WARNING) << "Skipping cross-view edge with unknown variable pose.";
      continue;
    }
    const double w = options_.anchor_weight * edge.weight;
    ceres::CostFunction* cost = Sim3AbsoluteAnchorPoseErrorTerm::Create(
        edge.measured_S_run_in_seg, w);
    ceres::LossFunction* loss = nullptr;
    if (options_.huber_delta_anchor > 0.0) {
      loss = new ceres::HuberLoss(options_.huber_delta_anchor);
      anchor_losses_.emplace_back(loss);
    }
    problem_->AddResidualBlock(
        cost,
        loss,
        variable_lies_.at(edge.variable_view_id).data());
    ++num_anchor_residuals_;
  }

  for (const auto& smooth_edge : scale_smooth_edges_) {
    const ViewId view_i = std::get<0>(smooth_edge);
    const ViewId view_j = std::get<1>(smooth_edge);
    const double weight = std::get<2>(smooth_edge);
    if (!HasVariablePose(variable_lies_, view_i) ||
        !HasVariablePose(variable_lies_, view_j)) {
      continue;
    }
    ceres::CostFunction* cost =
        Sim3ScaleSmoothnessErrorTerm::Create(weight);
    problem_->AddResidualBlock(cost,
                               nullptr,
                               variable_lies_.at(view_i).data(),
                               variable_lies_.at(view_j).data());
    ++num_scale_smooth_residuals_;
  }

  return problem_->NumResidualBlocks() > 0;
}

bool CrossReconstructionSim3PoseGraphOptimizer::Optimize(
    CrossReconstructionPoseGraphSummary* summary) {
  CHECK(summary != nullptr);
  summary->success = false;
  summary->initial_cost = 0.0;
  summary->final_cost = 0.0;
  summary->num_iterations = 0;

  if (!BuildProblem()) {
    LOG(ERROR) << "Failed to build pose graph problem.";
    return false;
  }

  summary->poses_finite_before =
      PosesAreFinite(variable_lies_, "variable poses before solve") &&
      PosesAreFinite(fixed_lies_, "fixed poses before solve");
  if (!summary->poses_finite_before) {
    LOG(ERROR) << "Aborting optimization due to non-finite initial poses.";
    ClearProblem();
    return false;
  }

  if (options_.debug_cost_breakdown) {
    LogCostBreakdown("PGO before solve");
    LogJacobianBreakdown("PGO before solve");
    FillResidualCostSummary(summary);
    LOG(INFO) << "PGO residual sq-norms before: sequential="
              << summary->sequential_residual_cost
              << " anchor=" << summary->anchor_residual_cost
              << " scale_smooth=" << summary->scale_smooth_residual_cost;
  }

  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = options_.max_num_iterations;
  solver_options.minimizer_progress_to_stdout = options_.verbose;
  solver_options.num_threads = 1;
  solver_options.use_nonmonotonic_steps = false;
  solver_options.initial_trust_region_radius = 10.0;
  solver_options.function_tolerance = 1e-6;
  solver_options.gradient_tolerance = 1e-4;
  if (variable_lies_.size() > 80) {
    solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  } else {
    solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  }

  ceres::Solver::Summary ceres_summary;
  ceres::Solve(solver_options, problem_.get(), &ceres_summary);

  summary->success = ceres_summary.IsSolutionUsable();
  summary->initial_cost = ceres_summary.initial_cost;
  summary->final_cost = ceres_summary.final_cost;
  summary->num_iterations =
      static_cast<int>(ceres_summary.iterations.size());

  summary->poses_finite_after =
      PosesAreFinite(variable_lies_, "variable poses after solve") &&
      PosesAreFinite(fixed_lies_, "fixed poses after solve");
  if (!summary->poses_finite_after) {
    LOG(ERROR) << "Optimization produced non-finite poses.";
    summary->success = false;
  }

  if (options_.debug_cost_breakdown) {
    LogCostBreakdown("PGO after solve");
    LogJacobianBreakdown("PGO after solve");
    FillResidualCostSummary(summary);
    LOG(INFO) << "PGO residual sq-norms after: sequential="
              << summary->sequential_residual_cost
              << " anchor=" << summary->anchor_residual_cost
              << " scale_smooth=" << summary->scale_smooth_residual_cost;
  }

  if (options_.verbose) {
    LOG(INFO) << ceres_summary.BriefReport();
  }

  ClearProblem();
  return summary->success;
}

void CrossReconstructionSim3PoseGraphOptimizer::ApplyToVariableReconstruction(
    Reconstruction* variable_reconstruction,
    bool transform_tracks) {
  CHECK(variable_reconstruction != nullptr);

  std::map<ViewId, Sophus::Sim3d> old_sim3;
  for (const auto& entry : variable_lies_) {
    const ViewId view_id = entry.first;
    View* view = variable_reconstruction->MutableView(view_id);
    if (view == nullptr || !view->IsEstimated()) {
      continue;
    }
    old_sim3[view_id] = GetSim3PoseFromView(*view);
    SetViewCameraFromSim3Lie(view, entry.second);
  }

  if (!transform_tracks) {
    return;
  }

  const auto track_ids = variable_reconstruction->TrackIds();
  for (const TrackId track_id : track_ids) {
    Track* track = variable_reconstruction->MutableTrack(track_id);
    if (track == nullptr || !track->IsEstimated()) {
      continue;
    }
    Eigen::Vector3d point = track->Point().hnormalized();
    const auto& view_ids = track->ViewIds();
    Eigen::Vector3d transformed = Eigen::Vector3d::Zero();
    int count = 0;
    for (const ViewId view_id : view_ids) {
      const auto old_it = old_sim3.find(view_id);
      const auto new_it = variable_lies_.find(view_id);
      if (old_it == old_sim3.end() || new_it == variable_lies_.end()) {
        continue;
      }
      const Sophus::Sim3d S_old = old_it->second;
      const Sophus::Sim3d S_new = Sophus::Sim3d::exp(new_it->second);
      const Sophus::Sim3d delta = S_new * S_old.inverse();
      transformed += delta * point;
      ++count;
    }
    if (count > 0) {
      point = transformed / static_cast<double>(count);
      *track->MutablePoint() = point.homogeneous();
    }
  }
}

bool AlignReconstructionsWithPoseGraph(
    const Reconstruction& fixed_reconstruction,
    Reconstruction* variable_reconstruction,
    const CrossReconstructionConstraints& constraints,
    const CrossReconstructionPoseGraphOptions& options,
    CrossReconstructionPoseGraphSummary* summary,
    const bool apply_to_variable_reconstruction) {
  CHECK(variable_reconstruction != nullptr);
  CHECK(summary != nullptr);

  CrossReconstructionSim3PoseGraphOptimizer optimizer(options);
  optimizer.SetFixedReconstruction(fixed_reconstruction,
                                   constraints.fixed_anchor_view_ids);
  optimizer.SetVariableReconstruction(
      *variable_reconstruction, constraints.variable_keyframe_view_ids);
  optimizer.SetConstraints(constraints);

  if (!optimizer.Optimize(summary)) {
    return false;
  }
  if (apply_to_variable_reconstruction) {
    optimizer.ApplyToVariableReconstruction(variable_reconstruction, true);
  }
  return summary->success;
}

}  // namespace theia
