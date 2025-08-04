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

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTER_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTER_H_

#include <ceres/ceres.h>
#include <ceres/types.h>
#include <unordered_set>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/util/timer.h"

namespace theia {
class Camera;
class CameraIntrinsicsModel;
class Reconstruction;
class Track;
class View;

// This class sets up nonlinear optimization problems for bundle adjustment.
// Bundle adjustment problems are set up by adding views and tracks to be
// optimized. Only the views and tracks supplied with AddView and AddTrack will
// be optimized. All other parameters are held constant.
//
// NOTE: It is required that AddViews is called before AddTracks if any views
// are being optimized.
class BundleAdjuster {
 public:
  // Set up the bundle adjuster. The reconstruction will be modified during
  // bundle adjustment.
  BundleAdjuster(const BundleAdjustmentOptions& options,
                 Reconstruction* reconstruction);

  // Add a view to be optimized with bundle adjustment. A residual is created
  // for each estimated track that the view observes.
  void AddView(const ViewId view_id);

  // Add a track to be optimized with bundle adjustment. A residual is created
  // for each estimated view that observes the track.
  void AddTrack(const TrackId track_id);
  
  // Add a inverse depth parametrization track to be optimized with bundle adjustment. 
  // A residual is created for each estimated view that observes the track.
  void AddInvTrack(const TrackId track_id, const bool views_constant);

  void AddViewPriors();

  // After AddView and AddTrack have been called, optimize the provided views
  // and tracks with bundle adjustment.
  BundleAdjustmentSummary Optimize();

  // Get covariance for a single track
  bool GetCovarianceForTrack(const TrackId track_id,
                             Eigen::Matrix3d* covariance_matrix);

  // Get covariance for all tracks
  bool GetCovarianceForTracks(
      const std::vector<TrackId>& track_ids,
      std::map<TrackId, Eigen::Matrix3d>* covariance_matrices);

  // Get covariance for a single view
  bool GetCovarianceForView(const ViewId view_id, Matrix6d* covariance_matrix);

  // Get covariance for a single view
  bool GetCovarianceForViews(const std::vector<ViewId> &track_ids, 
    std::map<ViewId, Matrix6d>* covariance_matrices);
    
  void SetCameraExtrinsicsConstant(const ViewId view_id);


 protected:
  // Add all camera extrinsics and intrinsics to the optimization problem.
  void SetCameraExtrinsicsParameterization();
  void SetCameraIntrinsicsParameterization();

  // Get the camera intrinsics model for the intrinsics group.
  std::shared_ptr<CameraIntrinsicsModel> GetIntrinsicsForCameraIntrinsicsGroup(
      const CameraIntrinsicsGroupId camera_intrinsics_group);

  // Methods for setting camera extrinsics to be (partially) constant.
  virtual void SetCameraPositionConstant(const ViewId view_id);
  virtual void SetCameraOrientationConstant(const ViewId view_id);
  virtual void SetTzConstant(const ViewId view_id);
  virtual void SetTrackConstant(const TrackId track_id);
  virtual void SetTrackVariable(const TrackId track_id);
  virtual void SetHomogeneousPointParametrization(const TrackId track_id);

  // Set the schur ordering for the parameters.
  virtual void SetCameraSchurGroups(const ViewId view_id);
  virtual void SetTrackSchurGroup(const TrackId track_id);

  // Add the reprojection error residual to the problem.
  virtual void AddReprojectionErrorResidual(const Feature& feature,
                                            Camera* camera,
                                            Track* track);
                                        
  // Add the inverse depth reprojection error residual to the problem.
  virtual void AddInvReprojectionErrorResidual(const Feature& feature,
                                               const Eigen::Vector3d& ref_bearing,
                                               Camera* camera_anch,
                                               Camera* camera_other,
                                               Track* track);

  // Add a position prior residual. This can for example be a GPS position.
  virtual void AddPositionPriorErrorResidual(View* view, Camera* camera);

  // Add a orientation prior residual world to camera.
  virtual void AddOrientationPriorErrorResidual(View* view, Camera* camera);

  // Add a gravity prior residual. Gravity is supposed to be measured in image coordinates.
  virtual void AddGravityPriorErrorResidual(View* view, Camera* camera);

  // Add a depth prior residual. Could be used e.g. for RGB-D cameras
  virtual void AddDepthPriorErrorResidual(const Feature& feature,
                                          Camera* camera,
                                          Track* track);


  const BundleAdjustmentOptions options_;
  Reconstruction* reconstruction_;
  Timer timer_;

  // Ceres problem for optimization.
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Solver::Options solver_options_;

  // The potentially robust loss function to use for reprojection error
  // minimization.
  std::unique_ptr<ceres::LossFunction> loss_function_;

  // The potentially robust loss function to use for reprojection error
  // minimization.
  std::unique_ptr<ceres::LossFunction> depth_prior_loss_function_;

  // The parameter group ordering for bundle adjustment.
  ceres::ParameterBlockOrdering* parameter_ordering_;

  // The optimized views.
  std::unordered_set<ViewId> optimized_views_;
  // The optimized tracks.
  std::unordered_set<TrackId> optimized_tracks_;

  // The intrinsics groups that are optimized.
  std::unordered_set<CameraIntrinsicsGroupId>
      optimized_camera_intrinsics_groups_;

  // Intrinsics groups that have at least 1 camera marked as "const" during
  // optimization. Only the intrinsics that have no optimized cameras are kept
  // as constant during optimization.
  std::unordered_set<CameraIntrinsicsGroupId>
      potentially_constant_camera_intrinsics_groups_;

  // Covariance estimator
  ceres::Covariance::Options covariance_options_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTER_H_
