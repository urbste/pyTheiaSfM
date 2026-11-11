// Copyright (C) 2017 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (sweeney.chris.m@gmail.com)

#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"

#include <algorithm>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <memory>
#include <unordered_set>
#include <vector>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/bundle_adjustment/position_error.h"
#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/create_reprojection_error_cost_function.h"
#include "theia/sfm/bundle_adjustment/position_error.h"
#include "theia/sfm/bundle_adjustment/gravity_direction_error.h"
#include "theia/sfm/bundle_adjustment/gravity_error.h"
#include "theia/sfm/bundle_adjustment/depth_prior_error.h"
#include "theia/sfm/bundle_adjustment/point_prior_error.h"
#include "theia/sfm/bundle_adjustment/orientation_error.h"
#include "theia/sfm/bundle_adjustment/relative_pose_error.h"
#include "theia/sfm/bundle_adjustment/scaled_relative_pose_error.h"

#include "theia/sfm/bundle_adjustment/analytic_reprojection_error.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/types.h"
#include "theia/util/map_util.h"
#include "theia/util/timer.h"

namespace theia {
namespace {
// Set the solver options to defaults.
void SetSolverOptions(const BundleAdjustmentOptions& options,
                      ceres::Solver::Options* solver_options) {
  solver_options->linear_solver_type = options.linear_solver_type;
  solver_options->preconditioner_type = options.preconditioner_type;
  solver_options->visibility_clustering_type =
      options.visibility_clustering_type;
  solver_options->logging_type =
      options.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;
  solver_options->num_threads = options.num_threads;
  solver_options->max_num_iterations = options.max_num_iterations;
  solver_options->max_solver_time_in_seconds =
      options.max_solver_time_in_seconds;
  solver_options->use_inner_iterations = options.use_inner_iterations;
  solver_options->function_tolerance = options.function_tolerance;
  solver_options->gradient_tolerance = options.gradient_tolerance;
  solver_options->parameter_tolerance = options.parameter_tolerance;
  solver_options->max_trust_region_radius = options.max_trust_region_radius;
  solver_options->use_mixed_precision_solves = options.use_mixed_precision_solves;
  solver_options->max_num_refinement_iterations = options.max_num_refinement_iterations;
  solver_options->dense_linear_algebra_library_type = options.dense_linear_algebra_library_type;
  solver_options->sparse_linear_algebra_library_type = options.sparse_linear_algebra_library_type;

  // Solver options takes ownership of the ordering so that we can order the BA
  // problem by points and cameras.
  solver_options->linear_solver_ordering.reset(
      new ceres::ParameterBlockOrdering);
}
}  // namespace

BundleAdjuster::BundleAdjuster(const BundleAdjustmentOptions& options,
                               Reconstruction* reconstruction)
    : options_(options), reconstruction_(reconstruction) {
  CHECK_NOTNULL(reconstruction);

  // Start setup timer.
  timer_.Reset();

  // Get the loss function that will be used for BA.
  loss_function_ =
      CreateLossFunction(options.loss_function_type, options.robust_loss_width);
  // Get the loss function that will be used for BA.
  depth_prior_loss_function_ =
      CreateLossFunction(options.loss_function_type, options.robust_loss_width_depth_prior);
  // Point priors use a plain quadratic loss unless a robust width is given.
  if (options.robust_loss_width_point_prior > 0.0) {
    point_prior_loss_function_ = CreateLossFunction(
        options.loss_function_type, options.robust_loss_width_point_prior);
  }

  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_.reset(new ceres::Problem(problem_options));

  // Set solver options.
  SetSolverOptions(options, &solver_options_);
  parameter_ordering_ = solver_options_.linear_solver_ordering.get();
}

void BundleAdjuster::AddView(const ViewId view_id) {
  View* view = CHECK_NOTNULL(reconstruction_->MutableView(view_id));

  // Only optimize estimated views.
  if (!view->IsEstimated() || ContainsKey(optimized_views_, view_id)) {
    return;
  }

  // Mark the view as optimized.
  optimized_views_.emplace(view_id);

  // Set the grouping for schur elimination.
  SetCameraSchurGroups(view_id);

  // Mark the camera intrinsics as optimized.
  const CameraIntrinsicsGroupId intrinsics_group_id =
      reconstruction_->CameraIntrinsicsGroupIdFromViewId(view_id);
  optimized_camera_intrinsics_groups_.emplace(intrinsics_group_id);

  // Fetch the camera that will be optimized.
  Camera* camera = view->MutableCamera();
  // Add residuals for all tracks in the view.
  for (const TrackId track_id : view->TrackIds()) {
    const Feature* feature = CHECK_NOTNULL(view->GetFeature(track_id));
    Track* track = CHECK_NOTNULL(reconstruction_->MutableTrack(track_id));
    // Only consider tracks with an estimated 3d point.
    if (!track->IsEstimated()) {
      continue;
    }

    // Add the reprojection error to the optimization.
    AddReprojectionErrorResidual(*feature, camera, track);

    // Add the point to group 0.
    SetTrackConstant(track_id);

    // Add depth priors
    // a depth of zero does not make much sense for a camera
    if (options_.use_depth_priors && feature->depth_prior() != 0.0) {
      AddDepthPriorErrorResidual(*feature, camera, track);
    }
  }

  // add a position prior if available
  if (options_.use_position_priors && view->HasPositionPrior()) {
    AddPositionPriorErrorResidual(view, camera);
  }

  // add a gravity prior if available
  if (options_.use_gravity_priors && view->HasGravityPrior()) {
    AddGravityPriorErrorResidual(view, camera);
  }

  // add a orientation prior if available
  if (options_.use_orientation_priors && view->HasOrientationPrior()) {
    AddOrientationPriorErrorResidual(view, camera);
  }
}

void BundleAdjuster::AddTrack(const TrackId track_id) {
  Track* track = CHECK_NOTNULL(reconstruction_->MutableTrack(track_id));
  // Only optimize estimated tracks.
  if (!track->IsEstimated() || ContainsKey(optimized_tracks_, track_id)) {
    return;
  }

  // A track without an estimated observing view has no residual, so its
  // parameter block would not be part of the problem.
  const auto& observed_view_ids = track->ViewIds();
  bool has_estimated_view = false;
  for (const ViewId view_id : observed_view_ids) {
    const View* view = reconstruction_->View(view_id);
    if (view != nullptr && view->IsEstimated()) {
      has_estimated_view = true;
      break;
    }
  }
  if (!has_estimated_view) {
    return;
  }

  // Mark the track as optimized.
  optimized_tracks_.emplace(track_id);

  // Add all observations of the track to the problem.
  for (const ViewId view_id : observed_view_ids) {

    View* view = CHECK_NOTNULL(reconstruction_->MutableView(view_id));

    // Only optimize estimated views that have not already been added.
    if (ContainsKey(optimized_views_, view_id) || !view->IsEstimated()) {
      continue;
    }

    const Feature* feature = CHECK_NOTNULL(view->GetFeature(track_id));
    Camera* camera = view->MutableCamera();

    // Add the reprojection error to the optimization.
    AddReprojectionErrorResidual(*feature, camera, track);
  
    // Any camera that reaches this point was not added by AddView() and so we
    // want to mark it as constant.
    SetCameraExtrinsicsConstant(view_id);

    // Mark the camera intrinsics as "potentially constant." We only set the
    // parameter block to constant if the shared intrinsics are not shared
    // with cameras that are being optimized.
    const CameraIntrinsicsGroupId intrinsics_group_id =
        reconstruction_->CameraIntrinsicsGroupIdFromViewId(view_id);
    potentially_constant_camera_intrinsics_groups_.emplace(intrinsics_group_id);
  }

  SetTrackVariable(track_id);
  SetTrackSchurGroup(track_id);

  if (options_.use_homogeneous_point_parametrization) {
    SetHomogeneousPointParametrization(track_id);
  }

  // Point priors are only added for optimized tracks (this function) and only
  // for the homogeneous (or Euclidean 4-vector) point parametrization.
  if (options_.use_point_priors && track->HasPositionPrior()) {
    if (options_.use_inverse_depth_parametrization) {
      if (!warned_point_prior_inverse_depth_) {
        LOG(WARNING) << "Point priors (use_point_priors) are not supported with "
                        "the inverse depth parametrization and are ignored.";
        warned_point_prior_inverse_depth_ = true;
      }
    } else if (!options_.use_homogeneous_point_parametrization) {
      if (!warned_point_prior_inverse_depth_) {
        LOG(WARNING) << "Point priors (use_point_priors) require "
                        "use_homogeneous_point_parametrization and are ignored.";
        warned_point_prior_inverse_depth_ = true;
      }
    } else {
      AddPointPriorErrorResidual(track);
    }
  }
}

bool BundleAdjuster::AddInvTrack(
    const TrackId track_id,
    const bool views_constant,
    const std::unordered_set<ViewId>* variable_views) {
  Track* track = CHECK_NOTNULL(reconstruction_->MutableTrack(track_id));
  // Only optimize estimated tracks.
  if (!track->IsEstimated() || ContainsKey(optimized_tracks_, track_id)) {
    return false;
  }

  // Derive the reference view, the unit bearing vector and the inverse depth
  // from the current 3D point so that the anchor is consistent with the poses.
  if (!reconstruction_->InitializeTrackInverseDepth(track_id)) {
    VLOG(1) << "Track " << track_id
            << " can not be parametrized with an inverse depth. Skipping it.";
    return false;
  }
  const ViewId reference_view_id = track->ReferenceViewId();

  // Collect the observations that can enter the problem.
  struct Observation {
    ViewId view_id;
    const Feature* feature;
  };
  std::vector<Observation> observations;
  int num_non_reference_observations = 0;
  for (const ViewId view_id : track->ViewIds()) {
    View* view = CHECK_NOTNULL(reconstruction_->MutableView(view_id));
    if (!view->IsEstimated()) {
      continue;
    }
    const Feature* feature = view->GetFeature(track_id);
    if (feature == nullptr) {
      continue;
    }
    // The inverse depth cost functions exist for central cameras only.
    if (view->Camera().GetCameraIntrinsicsModelType() ==
        CameraIntrinsicsModelType::ORTHOGRAPHIC) {
      return false;
    }
    observations.push_back({view_id, feature});
    if (view_id != reference_view_id) {
      ++num_non_reference_observations;
    }
  }
  // The inverse depth is only constrained by observations in other views.
  if (num_non_reference_observations == 0) {
    return false;
  }

  // Mark the track as optimized.
  optimized_tracks_.emplace(track_id);

  View* ref_view = reconstruction_->MutableView(reference_view_id);
  for (const Observation& observation : observations) {
    const ViewId view_id = observation.view_id;
    View* view = reconstruction_->MutableView(view_id);
    Camera* camera = view->MutableCamera();

    AddInvReprojectionErrorResidual(*observation.feature,
                                    track->ReferenceBearingVector(),
                                    ref_view->MutableCamera(),
                                    camera,
                                    track);

    const CameraIntrinsicsGroupId intrinsics_group_id =
        reconstruction_->CameraIntrinsicsGroupIdFromViewId(view_id);
    const bool view_is_variable =
        !views_constant &&
        (variable_views == nullptr || ContainsKey(*variable_views, view_id));
    if (view_is_variable) {
      optimized_camera_intrinsics_groups_.emplace(intrinsics_group_id);
      SetCameraSchurGroups(view_id);
      optimized_views_.emplace(view_id);
    } else {
      // Mark the camera intrinsics as "potentially constant." We only set the
      // parameter block to constant if the shared intrinsics are not shared
      // with cameras that are being optimized.
      potentially_constant_camera_intrinsics_groups_.emplace(
          intrinsics_group_id);
      SetCameraExtrinsicsConstant(view_id);
    }
  }

  if (options_.use_point_priors && track->HasPositionPrior() &&
      !warned_point_prior_inverse_depth_) {
    LOG(WARNING) << "Point priors (use_point_priors) are not supported with "
                    "the inverse depth parametrization and are ignored.";
    warned_point_prior_inverse_depth_ = true;
  }

  SetTrackVariable(track_id);
  SetTrackSchurGroup(track_id);
  // The inverse depth has to stay non-negative: a negative value would place
  // the point behind the reference camera. Zero is a point at infinity.
  problem_->SetParameterLowerBound(track->MutableInverseDepth(), 0, 0.0);
  return true;
}

void BundleAdjuster::AddViewPriors() {

  for (const auto vid : optimized_views_) {
    View* view = CHECK_NOTNULL(reconstruction_->MutableView(vid));
    Camera* camera = view->MutableCamera();

    // add a position prior if available
    if (options_.use_position_priors && view->HasPositionPrior()) {
      AddPositionPriorErrorResidual(view, camera);
    }

    // add a gravity prior if available
    if (options_.use_gravity_priors && view->HasGravityPrior()) {
      AddGravityPriorErrorResidual(view, camera);
    }

    // add a orientation prior if available
    if (options_.use_orientation_priors && view->HasOrientationPrior()) {
      AddOrientationPriorErrorResidual(view, camera);
    }
  }

}

BundleAdjustmentSummary BundleAdjuster::Optimize() {
  // Set extrinsics parameterization of the camera poses. This will set
  // orientation and/or positions as constant if desired.
  SetCameraExtrinsicsParameterization();

  // Set intrinsics group parameterization. This will control which of the
  // intrinsics parameters or optimized or held constant. Note that each camera
  // intrinsics group may be a different camera and/or a different camera
  // intrinsics model.
  SetCameraIntrinsicsParameterization();

  // Views (or tracks) that were added to the ordering but never received a
  // residual block are not part of the problem. Drop them from the ordering.
  RemoveBlocksNotInProblemFromOrdering();

  // NOTE: csweeney found a thread on the Ceres Solver email group that
  // indicated using the reverse BA order (i.e., using cameras then points) is a
  // good idea for inner iterations.
  if (solver_options_.use_inner_iterations && 
      !options_.use_inverse_depth_parametrization) {
    solver_options_.inner_iteration_ordering.reset(
        new ceres::ParameterBlockOrdering(*parameter_ordering_));
    solver_options_.inner_iteration_ordering->Reverse();
  }

  // Solve the problem.
  const double internal_setup_time = timer_.ElapsedTimeInSeconds();
  ceres::Solver::Summary solver_summary;
  ceres::Solve(solver_options_, problem_.get(), &solver_summary);
  LOG_IF(INFO, options_.verbose) << solver_summary.FullReport();

  // Set the BundleAdjustmentSummary.
  BundleAdjustmentSummary summary;
  summary.setup_time_in_seconds =
      internal_setup_time + solver_summary.preprocessor_time_in_seconds;
  summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
  summary.initial_cost = solver_summary.initial_cost;
  summary.final_cost = solver_summary.final_cost;

  // This only indicates whether the optimization was successfully run and makes
  // no guarantees on the quality or convergence.
  summary.success = solver_summary.IsSolutionUsable();

  return summary;
}

void BundleAdjuster::RemoveBlocksNotInProblemFromOrdering() {
  // Every parameter block that is ever added to the ordering belongs to an
  // optimized view or an optimized track.
  for (const ViewId view_id : optimized_views_) {
    Camera* camera = reconstruction_->MutableView(view_id)->MutableCamera();
    for (double* block : {camera->mutable_extrinsics(),
                          camera->mutable_intrinsics()}) {
      if (!problem_->HasParameterBlock(block)) {
        parameter_ordering_->Remove(block);
      }
    }
  }
  for (const TrackId track_id : optimized_tracks_) {
    Track* track = reconstruction_->MutableTrack(track_id);
    double* block = options_.use_inverse_depth_parametrization
                        ? track->MutableInverseDepth()
                        : track->MutablePoint()->data();
    if (!problem_->HasParameterBlock(block)) {
      parameter_ordering_->Remove(block);
    }
  }
}

void BundleAdjuster::SetCameraExtrinsicsParameterization() {
  // Collect all constant extrinsics parameters in a single subset. Ceres only
  // keeps one manifold per parameter block, so setting the orientation,
  // position and tz subsets one after another would overwrite each other.
  std::vector<int> constant_parameters;
  if (options_.constant_camera_orientation) {
    for (int i = 0; i < 3; ++i) {
      constant_parameters.emplace_back(Camera::ORIENTATION + i);
    }
  }
  if (options_.constant_camera_position) {
    for (int i = 0; i < 3; ++i) {
      constant_parameters.emplace_back(Camera::POSITION + i);
    }
  } else if (options_.orthographic_camera) {
    // for orthographic cameras we set tz constant = 0
    constant_parameters.emplace_back(Camera::POSITION + 2);
  }
  if (constant_parameters.empty()) {
    return;
  }

  for (const ViewId view_id : optimized_views_) {
    double* extrinsics =
        reconstruction_->MutableView(view_id)->MutableCamera()
            ->mutable_extrinsics();
    // Views without any residual (e.g. no estimated tracks) were never added
    // to the problem; Ceres aborts if we set a manifold on them.
    if (!problem_->HasParameterBlock(extrinsics)) {
      continue;
    }
    if (static_cast<int>(constant_parameters.size()) ==
        Camera::kExtrinsicsSize) {
      // If all extrinsics are constant then mark the entire parameter block
      // as constant.
      problem_->SetParameterBlockConstant(extrinsics);
    } else {
      problem_->SetManifold(
          extrinsics,
          new ceres::SubsetManifold(Camera::kExtrinsicsSize,
                                    constant_parameters));
    }
  }
}

void BundleAdjuster::SetCameraIntrinsicsParameterization() {
  // Loop through all optimized camera intrinsics groups to set the intrinsics
  // parameterization.
  for (const CameraIntrinsicsGroupId camera_intrinsics_group :
       optimized_camera_intrinsics_groups_) {
    // Get the shared camera intrinsics parameters.
    std::shared_ptr<CameraIntrinsicsModel> camera_intrinsics =
        GetIntrinsicsForCameraIntrinsicsGroup(camera_intrinsics_group);

    // The shared intrinsics block only enters the problem through a residual.
    // If no view of this group has one (e.g. views without observations), there
    // is nothing to parameterize.
    if (!problem_->HasParameterBlock(camera_intrinsics->mutable_parameters())) {
      continue;
    }

    // Get the subset parameterization of the intrinsics to keep constant.
    const std::vector<int> constant_intrinsics =
        camera_intrinsics->GetSubsetFromOptimizeIntrinsicsType(
            options_.intrinsics_to_optimize);

    // set lower bound for focal length optimization ( > 0)
    const std::vector<int> focal_length_id =
        camera_intrinsics->GetSubsetFromOptimizeIntrinsicsType(
            OptimizeIntrinsicsType::ASPECT_RATIO |
            OptimizeIntrinsicsType::PRINCIPAL_POINTS |
            OptimizeIntrinsicsType::RADIAL_DISTORTION |
            OptimizeIntrinsicsType::SKEW |
            OptimizeIntrinsicsType::TANGENTIAL_DISTORTION);

    // set lower bound for focal length (>1)
    problem_->SetParameterLowerBound(camera_intrinsics->mutable_parameters(),
                                     focal_length_id[0], 1.0);

    if (camera_intrinsics->Type() ==
        theia::CameraIntrinsicsModelType::DOUBLE_SPHERE) {
      problem_->SetParameterLowerBound(
          camera_intrinsics->mutable_parameters(), 5, -1.0);
      problem_->SetParameterUpperBound(
          camera_intrinsics->mutable_parameters(), 5, 1.0);
      problem_->SetParameterLowerBound(
          camera_intrinsics->mutable_parameters(), 6, 0.0);
      problem_->SetParameterUpperBound(
          camera_intrinsics->mutable_parameters(), 6, 1.0);
    } else if (camera_intrinsics->Type() ==
               theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED) {
      problem_->SetParameterLowerBound(
          camera_intrinsics->mutable_parameters(), 5, 0.0);
      problem_->SetParameterUpperBound(
          camera_intrinsics->mutable_parameters(), 5, 1.0);
      problem_->SetParameterLowerBound(
          camera_intrinsics->mutable_parameters(), 6, 0.1);
    }

    // Set the constant parameters if any are requested.
    if (constant_intrinsics.size() == camera_intrinsics->NumParameters()) {
      if (problem_->HasParameterBlock(camera_intrinsics->mutable_parameters())) {
        problem_->SetParameterBlockConstant(
            camera_intrinsics->mutable_parameters());
      }
    } else if (constant_intrinsics.size() > 0) {
      ceres::SubsetManifold* subset_parameterization =
          new ceres::SubsetManifold(camera_intrinsics->NumParameters(),
                                            constant_intrinsics);
      problem_->SetManifold(camera_intrinsics->mutable_parameters(),
                                    subset_parameterization);
    }
  }

  // Set camera intrinsics to be constant if no cameras in the intrinsics group
  // are being optimized.
  for (const CameraIntrinsicsGroupId camera_intrinsics_group :
       potentially_constant_camera_intrinsics_groups_) {
    // If the intrinsics group is being optimized, do not mark it as constant.
    if (ContainsKey(optimized_camera_intrinsics_groups_,
                    camera_intrinsics_group)) {
      continue;
    }

    // Get the shared camera intrinsics parameters.
    std::shared_ptr<CameraIntrinsicsModel> camera_intrinsics =
        GetIntrinsicsForCameraIntrinsicsGroup(camera_intrinsics_group);

    // Set the intrinsics to be constant (only if a residual already registered
    // this shared parameter block with the problem).
    if (problem_->HasParameterBlock(camera_intrinsics->mutable_parameters())) {
      problem_->SetParameterBlockConstant(
          camera_intrinsics->mutable_parameters());
    }
  }
}

std::shared_ptr<CameraIntrinsicsModel>
BundleAdjuster::GetIntrinsicsForCameraIntrinsicsGroup(
    const CameraIntrinsicsGroupId camera_intrinsics_group) {
  // Get a representative view for the intrinsics group.
  const auto& views_in_intrinsics_groups =
      reconstruction_->GetViewsInCameraIntrinsicGroup(camera_intrinsics_group);
  CHECK(!views_in_intrinsics_groups.empty());
  const ViewId representative_view_id = *views_in_intrinsics_groups.begin();

  // Get the shared camera intrinsics parameters.
  return reconstruction_->MutableView(representative_view_id)
      ->MutableCamera()
      ->MutableCameraIntrinsics();
}

void BundleAdjuster::SetCameraExtrinsicsConstant(const ViewId view_id) {
  View* view = reconstruction_->MutableView(view_id);
  Camera* camera = view->MutableCamera();
  double* extrinsics = camera->mutable_extrinsics();
  // A constant view whose observations were all removed (and that has no gravity
  // prior) has no residual block yet; Ceres aborts on SetParameterBlockConstant
  // for an unknown block. An isolated constant block is harmless.
  if (!problem_->HasParameterBlock(extrinsics)) {
    problem_->AddParameterBlock(extrinsics, Camera::kExtrinsicsSize);
  }
  problem_->SetParameterBlockConstant(extrinsics);
}

bool BundleAdjuster::IsViewInProblem(const ViewId view_id) const {
  return ContainsKey(optimized_views_, view_id);
}

void BundleAdjuster::EnsureViewExtrinsicsInProblem(const ViewId view_id) {
  View* view = reconstruction_->MutableView(view_id);
  if (view == nullptr || !view->IsEstimated() ||
      ContainsKey(optimized_views_, view_id)) {
    return;
  }

  optimized_views_.emplace(view_id);

  Camera* camera = view->MutableCamera();
  double* extrinsics = camera->mutable_extrinsics();
  if (!problem_->HasParameterBlock(extrinsics)) {
    problem_->AddParameterBlock(extrinsics, Camera::kExtrinsicsSize);
  }

  double* intrinsics = camera->mutable_intrinsics();
  const int num_intrinsics = camera->CameraIntrinsics()->NumParameters();
  if (!problem_->HasParameterBlock(intrinsics)) {
    problem_->AddParameterBlock(intrinsics, num_intrinsics);
  }

  SetCameraSchurGroups(view_id);

  const CameraIntrinsicsGroupId intrinsics_group_id =
      reconstruction_->CameraIntrinsicsGroupIdFromViewId(view_id);
  potentially_constant_camera_intrinsics_groups_.emplace(intrinsics_group_id);
  optimized_camera_intrinsics_groups_.emplace(intrinsics_group_id);
}

void BundleAdjuster::SetCameraPositionConstant(const ViewId view_id) {
  static const std::vector<int> position_parameters = {
      Camera::POSITION + 0, Camera::POSITION + 1, Camera::POSITION + 2};
  ceres::SubsetManifold* subset_parameterization =
      new ceres::SubsetManifold(Camera::kExtrinsicsSize,
                                        position_parameters);
  View* view = reconstruction_->MutableView(view_id);
  Camera* camera = view->MutableCamera();
  problem_->SetManifold(camera->mutable_extrinsics(),
                        subset_parameterization);
}

void BundleAdjuster::SetCameraOrientationConstant(const ViewId view_id) {
  static const std::vector<int> orientation_parameters = {
      Camera::ORIENTATION + 0,
      Camera::ORIENTATION + 1,
      Camera::ORIENTATION + 2};
  ceres::SubsetManifold* subset_parameterization =
      new ceres::SubsetManifold(Camera::kExtrinsicsSize,
                                        orientation_parameters);
  View* view = reconstruction_->MutableView(view_id);
  Camera* camera = view->MutableCamera();
  problem_->SetManifold(camera->mutable_extrinsics(),
                        subset_parameterization);
}

void BundleAdjuster::SetTzConstant(const ViewId view_id) {
    static const std::vector<int> position_parameters = {Camera::POSITION + 2};
    ceres::SubsetManifold* subset_parameterization =
        new ceres::SubsetManifold(Camera::kExtrinsicsSize,
                                          position_parameters);
    View* view = reconstruction_->MutableView(view_id);
    Camera* camera = view->MutableCamera();
    problem_->SetManifold(camera->mutable_extrinsics(),
                          subset_parameterization);
}

void BundleAdjuster::SetTrackConstant(const TrackId track_id) {
  Track* track = reconstruction_->MutableTrack(track_id);
  if (options_.use_inverse_depth_parametrization) {
    problem_->SetParameterBlockConstant(track->MutableInverseDepth());
  } else {
    problem_->SetParameterBlockConstant(track->MutablePoint()->data());
  }
}

void BundleAdjuster::SetTrackVariable(const TrackId track_id) {
  Track* track = reconstruction_->MutableTrack(track_id);
  if (options_.use_inverse_depth_parametrization) {
    problem_->SetParameterBlockVariable(track->MutableInverseDepth());
  } else {
    problem_->SetParameterBlockVariable(track->MutablePoint()->data());
  }
}

void BundleAdjuster::SetHomogeneousPointParametrization(
    const TrackId track_id) {
  ceres::Manifold* point_parametrization =
      new ceres::SphereManifold<4>();
  Track* track = reconstruction_->MutableTrack(track_id);
  problem_->SetManifold(track->MutablePoint()->data(),
                        point_parametrization);
}

void BundleAdjuster::SetCameraSchurGroups(const ViewId view_id) {
  View* view = reconstruction_->MutableView(view_id);
  Camera* camera = view->MutableCamera();

  if (options_.optimize_for_forward_facing_trajectory) {
    // COLLAPSED GROUPS: Put everything except points into Group 1
    static const int kCameraGroup = 1;
    parameter_ordering_->AddElementToGroup(camera->mutable_extrinsics(), kCameraGroup);
    parameter_ordering_->AddElementToGroup(camera->mutable_intrinsics(), kCameraGroup);
  } else {
    // Three-group elimination
    static const int kIntrinsicsParameterGroup = 1;
    static const int kExtrinsicsParameterGroup = 2;
    parameter_ordering_->AddElementToGroup(camera->mutable_intrinsics(), kIntrinsicsParameterGroup);
    parameter_ordering_->AddElementToGroup(camera->mutable_extrinsics(), kExtrinsicsParameterGroup);
  }
}

void BundleAdjuster::SetTrackSchurGroup(const TrackId track_id) {
  static const int kTrackParameterGroup = 0;
  Track* track = reconstruction_->MutableTrack(track_id);
  // Set the parameter ordering for Schur elimination. We do this after the loop
  // above so that the track is already added to the problem.
  if (options_.use_inverse_depth_parametrization) {
    parameter_ordering_->AddElementToGroup(track->MutableInverseDepth(),
                                           kTrackParameterGroup);
  } else {
    parameter_ordering_->AddElementToGroup(track->MutablePoint()->data(),
                                           kTrackParameterGroup);
  }
}

void BundleAdjuster::AddReprojectionErrorResidual(const Feature& feature,
                                                  Camera* camera,
                                                  Track* track) {
  // Add the residual for the track to the problem. The shared intrinsics
  // parameter block will be set to constant after the loop if no optimized
  // cameras share the same camera intrinsics.
  ceres::CostFunction* cost_function = nullptr;
  if (options_.analytic_jacobians && !options_.orthographic_camera) {
    cost_function = CreateAnalyticReprojectionErrorCostFunction(
        camera->GetCameraIntrinsicsModelType(), feature);
  }
  if (cost_function == nullptr) {
    cost_function = CreateReprojectionErrorCostFunction(
        camera->GetCameraIntrinsicsModelType(), feature);
  }
  problem_->AddResidualBlock(
      cost_function,
      loss_function_.get(),
      camera->mutable_extrinsics(),
      camera->mutable_intrinsics(),
      track->MutablePoint()->data());
}

void BundleAdjuster::AddInvReprojectionErrorResidual(const Feature& feature,
                                                     const Eigen::Vector3d& ref_bearing,
                                                     Camera* camera_ref,
                                                     Camera* camera_other,
                                                     Track* track) {
  // Add the residual for the track to the problem. The shared intrinsics
  // parameter block will be set to constant after the loop if no optimized
  // cameras share the same camera intrinsics.
  if (camera_ref == camera_other) {
      ceres::CostFunction* pose_cf = nullptr;
      if (options_.analytic_jacobians) {
        pose_cf = CreateAnalyticInvReprojectionPoseErrorCostFunction(
            camera_ref->GetCameraIntrinsicsModelType(), feature, ref_bearing);
      }
      if (pose_cf == nullptr) {
        pose_cf = CreateInvReprojectionPoseErrorCostFunction(
            camera_ref->GetCameraIntrinsicsModelType(), feature, ref_bearing);
      }
      problem_->AddResidualBlock(
        pose_cf,
        loss_function_.get(),
        camera_ref->mutable_extrinsics(),
        camera_ref->mutable_intrinsics(),
        track->MutableInverseDepth());
  } else {
    ceres::CostFunction* inv_cf = nullptr;
    if (options_.analytic_jacobians) {
      inv_cf = CreateAnalyticInvReprojectionErrorCostFunction(
          camera_other->GetCameraIntrinsicsModelType(), feature, ref_bearing);
    }
    if (inv_cf == nullptr) {
      inv_cf = CreateInvReprojectionErrorCostFunction(
          camera_other->GetCameraIntrinsicsModelType(), feature, ref_bearing);
    }
    problem_->AddResidualBlock(
        inv_cf,
        loss_function_.get(),
        camera_ref->mutable_extrinsics(),
        camera_other->mutable_extrinsics(),
        camera_other->mutable_intrinsics(),
        track->MutableInverseDepth());
  }
}

void BundleAdjuster::AddPositionPriorErrorResidual(View* view, Camera* camera) {
  // Adds a position priors to the camera poses. This can for example be a GPS
  // position.
  problem_->AddResidualBlock(
      PositionError::Create(view->GetPositionPrior(),
                            view->GetPositionPriorSqrtInformation()),
      NULL,
      camera->mutable_extrinsics());
}

void BundleAdjuster::AddGravityPriorErrorResidual(View* view, Camera* camera) {
  const Eigen::Vector3d& gravity_prior = view->GetGravityPrior();
  const Eigen::Matrix3d& gravity_sqrt_information =
      view->GetGravityPriorSqrtInformation();
  ceres::CostFunction* cost_function = NULL;
  if (options_.gravity_prior_error_type ==
      GravityPriorErrorType::DIRECTION_CROSS) {
    cost_function = GravityDirectionError::Create(
        options_.gravity_world_direction,
        gravity_prior,
        gravity_sqrt_information);
  } else {
    cost_function = GravityError::Create(
        options_.gravity_world_direction,
        gravity_prior,
        gravity_sqrt_information);
  }
  problem_->AddResidualBlock(cost_function, NULL, camera->mutable_extrinsics());
}

void BundleAdjuster::AddDepthPriorErrorResidual(const Feature& feature,
                                          Camera* camera,
                                          Track* track) {

  problem_->AddResidualBlock(
    DepthPriorError::Create(feature),
    depth_prior_loss_function_.get(),
    camera->mutable_extrinsics(), track->MutablePoint()->data());
}

void BundleAdjuster::AddPointPriorErrorResidual(Track* track) {
  // Whitened residual sqrt_information * (X - prior) on the homogeneous point.
  problem_->AddResidualBlock(
      PointPriorError::Create(track->PositionPrior(),
                              track->PositionPriorSqrtInformation()),
      point_prior_loss_function_.get(),
      track->MutablePoint()->data());
}

void BundleAdjuster::AddOrientationPriorErrorResidual(View* view, Camera* camera) {
  problem_->AddResidualBlock(
      OrientationPriorError::Create(view->GetOrientationPrior(),
                            view->GetOrientationPriorSqrtInformation()),
      NULL, camera->mutable_extrinsics());
}

void BundleAdjuster::AddRelativePoseConstraint(
    const ViewId view_id_i,
    const ViewId view_id_j,
    const Matrix6d& sqrt_information) {
  View* view_i = CHECK_NOTNULL(reconstruction_->MutableView(view_id_i));
  View* view_j = CHECK_NOTNULL(reconstruction_->MutableView(view_id_j));
  Camera* camera_i = view_i->MutableCamera();
  Camera* camera_j = view_j->MutableCamera();

  // Snapshot the measured relative pose (cam_i -> cam_j) from the current world
  // -> cam poses so the edge preserves the present relative geometry.
  const auto world_to_cam = [](const Camera& camera) {
    const Eigen::Matrix3d R = camera.GetOrientationAsRotationMatrix();
    return Sophus::SE3d(Sophus::SO3d(R), -(R * camera.GetPosition()));
  };
  const Sophus::SE3d g_i = world_to_cam(*camera_i);
  const Sophus::SE3d g_j = world_to_cam(*camera_j);
  const Sophus::SE3d measured_i_to_j = g_j * g_i.inverse();

  problem_->AddResidualBlock(
      RelativePoseError::Create(measured_i_to_j, sqrt_information),
      NULL,
      camera_i->mutable_extrinsics(),
      camera_j->mutable_extrinsics());
}

void BundleAdjuster::AddScaledRelativePoseConstraint(
    const ViewId view_id_i,
    const ViewId view_id_j,
    const double rotation_sqrt_weight,
    const double translation_direction_sqrt_weight,
    const double translation_magnitude_sqrt_weight) {
  View* view_i = CHECK_NOTNULL(reconstruction_->MutableView(view_id_i));
  View* view_j = CHECK_NOTNULL(reconstruction_->MutableView(view_id_j));
  Camera* camera_i = view_i->MutableCamera();
  Camera* camera_j = view_j->MutableCamera();

  const auto world_to_cam = [](const Camera& camera) {
    const Eigen::Matrix3d R = camera.GetOrientationAsRotationMatrix();
    return Sophus::SE3d(Sophus::SO3d(R), -(R * camera.GetPosition()));
  };
  const Sophus::SE3d g_i = world_to_cam(*camera_i);
  const Sophus::SE3d g_j = world_to_cam(*camera_j);
  const Sophus::SE3d measured_i_to_j = g_j * g_i.inverse();

  problem_->AddResidualBlock(
      ScaledRelativePoseError::Create(measured_i_to_j,
                                      rotation_sqrt_weight,
                                      translation_direction_sqrt_weight,
                                      translation_magnitude_sqrt_weight),
      NULL,
      camera_i->mutable_extrinsics(),
      camera_j->mutable_extrinsics());
}

bool BundleAdjuster::GetCovarianceForTrack(const TrackId track_id,
                                           Matrix3d* covariance_matrix) {
  const Track* track = reconstruction_->Track(track_id);
  *covariance_matrix = Matrix3d::Identity();
  ceres::Covariance covariance_estimator(covariance_options_);

  std::vector<std::pair<const double*, const double*>> covariance_blocks = {
      std::make_pair(track->Point().data(), track->Point().data())};

  if (problem_->HasParameterBlock(track->Point().data()) &&
      !problem_->IsParameterBlockConstant(track->Point().data())) {
    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
      return false;
    }
    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {track->Point().data()}, (*covariance_matrix).data());

    return true;
  } else {
    return false;
  }
}

bool BundleAdjuster::GetCovarianceForTracks(
    const std::vector<TrackId>& track_ids,
    std::map<TrackId, Eigen::Matrix3d>* covariance_matrices) {
  ceres::Covariance covariance_estimator(covariance_options_);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  std::vector<TrackId> est_track_ids;
  for (const auto& t_id : track_ids) {
    const Track* track = reconstruction_->Track(t_id);
    if (problem_->HasParameterBlock(track->Point().data()) &&
        !problem_->IsParameterBlockConstant(track->Point().data())) {
      est_track_ids.push_back(t_id);
      covariance_blocks.push_back(
          std::make_pair(track->Point().data(), track->Point().data()));
    } else {
      LOG(ERROR)
          << "There was a track that could not be found in the reconstruction "
             "or is set to fixed! No covariance estimation possible.\n";
      return false;
    }
  }

  if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
    return false;
  }

  for (size_t i = 0; i < est_track_ids.size(); ++i) {
    Eigen::Matrix3d cov_mat;
    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {covariance_blocks[i].first}, cov_mat.data());
    covariance_matrices->insert(std::make_pair(est_track_ids[i], cov_mat));
  }

  return true;
}

bool BundleAdjuster::GetCovarianceForView(const ViewId view_id,
                                          Matrix6d* covariance_matrix) {
  const auto camera = reconstruction_->View(view_id)->Camera();
  *covariance_matrix = Matrix6d::Identity();

  ceres::Covariance covariance_estimator(covariance_options_);

  std::vector<std::pair<const double*, const double*>> covariance_blocks = {
      std::make_pair(camera.extrinsics(), camera.extrinsics())};

  if (problem_->HasParameterBlock(camera.extrinsics()) &&
      !problem_->IsParameterBlockConstant(camera.extrinsics())) {
    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
      return false;
    }
    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {camera.extrinsics()}, (*covariance_matrix).data());
    return true;
  } else {
    return false;
  }
}

bool BundleAdjuster::GetCovarianceForViews(
    const std::vector<ViewId>& view_ids,
    std::map<ViewId, Matrix6d>* covariance_matrices) {
  ceres::Covariance covariance_estimator(covariance_options_);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  std::vector<ViewId> est_view_ids;
  for (const auto& v_id : view_ids) {
    const auto extr_ptr = reconstruction_->View(v_id)->Camera().extrinsics();
    if (problem_->HasParameterBlock(extr_ptr) &&
        !problem_->IsParameterBlockConstant(extr_ptr)) {
      est_view_ids.push_back(v_id);
      covariance_blocks.push_back(std::make_pair(extr_ptr, extr_ptr));
    } else {
      LOG(ERROR)
          << "There was a view that could not be found in the reconstruction "
             "or is set to fixed! No covariance estimation possible.\n";
      return false;
    }
  }

  if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
    return false;
  }

  for (size_t i = 0; i < est_view_ids.size(); ++i) {
    Matrix6d cov_mat;
    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {covariance_blocks[i].first}, cov_mat.data());
    covariance_matrices->insert(std::make_pair(est_view_ids[i], cov_mat));
  }

  return true;
}

}  // namespace theia
