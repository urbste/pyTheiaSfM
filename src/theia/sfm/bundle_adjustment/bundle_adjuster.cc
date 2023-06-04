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
#include "theia/sfm/bundle_adjustment/gravity_error.h"
#include "theia/sfm/bundle_adjustment/depth_prior_error.h"

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
}

void BundleAdjuster::AddTrack(const TrackId track_id) {
  Track* track = CHECK_NOTNULL(reconstruction_->MutableTrack(track_id));
  // Only optimize estimated tracks.
  if (!track->IsEstimated() || ContainsKey(optimized_tracks_, track_id)) {
    return;
  }

  // Mark the track as optimized.
  optimized_tracks_.emplace(track_id);

  // Add all observations of the track to the problem.
  const auto& reference_view_id = track->ReferenceViewId();
  const auto& observed_view_ids = track->ViewIds();
  for (const ViewId view_id : observed_view_ids) {
    if (reference_view_id == view_id && options_.use_inverse_depth_parametrization) {
      continue;
    }
    
    View* view = CHECK_NOTNULL(reconstruction_->MutableView(view_id));
    View* ref_view = CHECK_NOTNULL(reconstruction_->MutableView(reference_view_id));

    // Only optimize estimated views that have not already been added.
    if (ContainsKey(optimized_views_, view_id) || !view->IsEstimated()) {
      continue;
    }
    if (ContainsKey(optimized_views_, reference_view_id) || !ref_view->IsEstimated()) {
      continue;
    }

    const Feature* feature = CHECK_NOTNULL(view->GetFeature(track_id));
    Camera* camera = view->MutableCamera();

    // Add the reprojection error to the optimization.
    if (options_.use_inverse_depth_parametrization) {
      AddInvReprojectionErrorResidual(*feature, track->ReferenceBearingVector(),
        ref_view->MutableCamera(), camera, track);
    } else {
      AddReprojectionErrorResidual(*feature, camera, track);
    }

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

  if (options_.use_inverse_depth_parametrization) {
    problem_->SetParameterLowerBound(track->MutableInverseDepth(), 0, 0.0);
  } else if (options_.use_homogeneous_point_parametrization) {
    SetHomogeneousPointParametrization(track_id);
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

  // NOTE: csweeney found a thread on the Ceres Solver email group that
  // indicated using the reverse BA order (i.e., using cameras then points) is a
  // good idea for inner iterations.
  if (solver_options_.use_inner_iterations) {
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

void BundleAdjuster::SetCameraExtrinsicsParameterization() {
  if (options_.constant_camera_orientation &&
      options_.constant_camera_position) {
    // If all extrinsics are constant then mark the entire parameter block
    // as constant.
    for (const ViewId view_id : optimized_views_) {
      SetCameraExtrinsicsConstant(view_id);
    }
  } else if (options_.constant_camera_orientation) {
    for (const ViewId view_id : optimized_views_) {
      SetCameraOrientationConstant(view_id);
    }
  } else if (options_.constant_camera_position) {
    for (const ViewId view_id : optimized_views_) {
      SetCameraPositionConstant(view_id);
    }
  }
  // for orthographic cameras we set tz constant = 0
  if (options_.orthographic_camera) {
    for (const ViewId view_id : optimized_views_) {
      SetTzConstant(view_id);
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
      problem_->SetParameterBlockConstant(
          camera_intrinsics->mutable_parameters());
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

    // Set the intrinsics to be constant.
    problem_->SetParameterBlockConstant(
        camera_intrinsics->mutable_parameters());
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
  problem_->SetParameterBlockConstant(camera->mutable_extrinsics());
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
  static const int kIntrinsicsParameterGroup = 1;
  static const int kExtrinsicsParameterGroup = 2;
  View* view = reconstruction_->MutableView(view_id);
  Camera* camera = view->MutableCamera();

  // Add camera parameters to groups 1 and 2. The extrinsics *must* belong to
  // group 2. This is because inner iterations uses a reverse ordering of
  // elimination and the Schur-based solvers require the first group to be an
  // independent set. Since the intrinsics may be shared, they are not
  // guaranteed to form an independent set and so we must use the extrinsics
  // in group 2.
  parameter_ordering_->AddElementToGroup(camera->mutable_extrinsics(),
                                         kExtrinsicsParameterGroup);
  parameter_ordering_->AddElementToGroup(camera->mutable_intrinsics(),
                                         kIntrinsicsParameterGroup);
}

void BundleAdjuster::SetTrackSchurGroup(const TrackId track_id) {
  static const int kTrackParameterGroup = 0;
  Track* track = reconstruction_->MutableTrack(track_id);
  // Set the parameter ordering for Schur elimination. We do this after the loop
  // above so that the track is already added to the problem.
  parameter_ordering_->AddElementToGroup(track->MutablePoint()->data(),
                                         kTrackParameterGroup);
}

void BundleAdjuster::AddReprojectionErrorResidual(const Feature& feature,
                                                  Camera* camera,
                                                  Track* track) {
  // Add the residual for the track to the problem. The shared intrinsics
  // parameter block will be set to constant after the loop if no optimized
  // cameras share the same camera intrinsics.
  problem_->AddResidualBlock(
      CreateReprojectionErrorCostFunction(
          camera->GetCameraIntrinsicsModelType(), feature),
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
  problem_->AddResidualBlock(
      CreateInvReprojectionErrorCostFunction(
          camera_other->GetCameraIntrinsicsModelType(), 
          feature, ref_bearing),
      loss_function_.get(),
      camera_ref->mutable_extrinsics(),
      camera_other->mutable_extrinsics(),
      camera_other->mutable_intrinsics(),
      track->MutableInverseDepth());
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
  // Adds a gravity priors to the camera orientation
  problem_->AddResidualBlock(
      GravityError::Create(view->GetGravityPrior(),
                           view->GetGravityPriorSqrtInformation()),
      NULL,
      camera->mutable_extrinsics());
}

void BundleAdjuster::AddDepthPriorErrorResidual(const Feature& feature,
                                          Camera* camera,
                                          Track* track) {

  problem_->AddResidualBlock(
    DepthPriorError::Create(feature),
    depth_prior_loss_function_.get(),
    camera->mutable_extrinsics(), track->MutablePoint()->data());
}

bool BundleAdjuster::GetCovarianceForTrack(const TrackId track_id,
                                           Matrix3d* covariance_matrix) {
  const Track* track = reconstruction_->Track(track_id);
  *covariance_matrix = Matrix3d::Identity();
  ceres::Covariance covariance_estimator(covariance_options_);

  std::vector<std::pair<const double*, const double*>> covariance_blocks = {
      std::make_pair(track->Point().data(), track->Point().data())};

  if (!problem_->IsParameterBlockConstant(track->Point().data()) &&
      problem_->HasParameterBlock(track->Point().data())) {
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
    if (!problem_->IsParameterBlockConstant(track->Point().data()) &&
        problem_->HasParameterBlock(track->Point().data())) {
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

  if (!problem_->IsParameterBlockConstant(camera.extrinsics()) &&
      problem_->HasParameterBlock(camera.extrinsics())) {
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
    if (!problem_->IsParameterBlockConstant(extr_ptr) &&
        problem_->HasParameterBlock(extr_ptr)) {
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
