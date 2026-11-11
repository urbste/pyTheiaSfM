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
#include <array>
#include <unordered_map>
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
  
  // Add a track with inverse depth parametrization to be optimized with bundle
  // adjustment. A residual is created for each estimated view that observes the
  // track. The reference view, reference bearing vector and inverse depth of
  // the track are (re-)initialized from the current 3D point of the track, so
  // they never go stale when poses changed since the last call to
  // Reconstruction::InitializeInverseDepth().
  //
  // views_constant: all observing views (including the reference view) are held
  // constant. variable_views: if not null, only the listed views are optimized
  // and all other observing views are held constant (like AddTrack() does for
  // views that were not added with AddView()).
  //
  // Returns false (and adds nothing) if the track can not be represented with
  // an inverse depth: not estimated, no estimated reference view, point behind
  // the reference ray, orthographic camera, or no observation besides the
  // reference view.
  bool AddInvTrack(const TrackId track_id,
                   const bool views_constant,
                   const std::unordered_set<ViewId>* variable_views = nullptr);

  // Tracks that were actually added to the problem.
  const std::unordered_set<TrackId>& OptimizedTrackIds() const {
    return optimized_tracks_;
  }

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

  // Register camera extrinsics (and intrinsics for Schur ordering) in the Ceres
  // problem when inverse-depth BA has not yet added residuals for this view.
  void EnsureViewExtrinsicsInProblem(const ViewId view_id);

  bool IsViewInProblem(const ViewId view_id) const;

  // Add an SE3 relative pose-to-pose constraint between two views. The measured
  // relative pose (cam_i -> cam_j) is snapshotted from the views' current poses,
  // so the constraint preserves the present relative geometry (odometry). The
  // 6x6 sqrt-information uses Sophus tangent order [translation(3), rotation(3)].
  // Both views must already have been added via AddView().
  void AddRelativePoseConstraint(const ViewId view_id_i,
                                 const ViewId view_id_j,
                                 const Matrix6d& sqrt_information);

  void AddScaledRelativePoseConstraint(
      const ViewId view_id_i,
      const ViewId view_id_j,
      double rotation_sqrt_weight,
      double translation_direction_sqrt_weight,
      double translation_magnitude_sqrt_weight);

  // Rig-constrained BA. Registers the estimated views of |capture_id| (an
  // estimated RigCapture whose CameraRig is known) as members of one shared
  // 6-DoF capture pose. Their reprojection residuals are RigReprojectionError
  // terms with the capture pose and the constant sensor extrinsics of the rig,
  // so the view poses are never free parameters. Only the intrinsics of the
  // member views are optimized here.
  //
  // Must be called before AddTrack() for the tracks observed by these views,
  // and the member views must not be passed to AddView(). Returns false (and
  // adds nothing) if the capture is not estimated, has no usable member view,
  // or contains an orthographic view.
  bool AddRigCapture(const CaptureId capture_id);

  // True if the view is observed through a capture added with AddRigCapture().
  bool IsViewInRigCapture(const ViewId view_id) const;

  // Writes the optimized capture poses back into their RigCaptures and
  // propagates them to the member views (PropagateCameraPosesForCapture). Only
  // captures that were part of the problem are updated.
  void UpdateRigCapturePoses();

 protected:
  // Constant rig data of a view that belongs to a capture added with
  // AddRigCapture(): the sensor extrinsics (Camera convention) of its RigSensor.
  struct RigViewInfo {
    CaptureId capture_id = kInvalidCaptureId;
    Eigen::Matrix3d rotation_rig_to_cam = Eigen::Matrix3d::Identity();
    Eigen::Vector3d position_in_rig = Eigen::Vector3d::Zero();
  };

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

  // Removes the parameter blocks of optimized views and tracks that never
  // received a residual block (and are therefore not in the problem) from the
  // linear solver ordering. Ceres requires the ordering to match the blocks of
  // the problem exactly.
  void RemoveBlocksNotInProblemFromOrdering();

  // Add the reprojection error residual to the problem.
  virtual void AddReprojectionErrorResidual(const Feature& feature,
                                            Camera* camera,
                                            Track* track);
                                        
  // Add the rig reprojection error residual of a view that belongs to a capture
  // added with AddRigCapture(). The residual uses the capture pose block.
  void AddRigReprojectionErrorResidual(const Feature& feature,
                                       const RigViewInfo& rig_view,
                                       Camera* camera,
                                       Track* track);

  // Position / gravity / orientation priors of a rig member view, expressed on
  // the capture pose (the camera pose is composed with the sensor extrinsics).
  void AddRigViewPriors(View* view, const RigViewInfo& rig_view);

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

  // Add a 3D point prior residual (e.g. ground control point) for an optimized
  // track that has a position prior. Only called for homogeneous point
  // parametrization; see BundleAdjustmentOptions::use_point_priors.
  virtual void AddPointPriorErrorResidual(Track* track);


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

  // Loss for point (track position) priors. Null (plain quadratic) when
  // robust_loss_width_point_prior is 0.
  std::unique_ptr<ceres::LossFunction> point_prior_loss_function_;

  // Set once the warning about unsupported point priors with inverse depth has
  // been logged.
  bool warned_point_prior_inverse_depth_ = false;

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

  // Rig captures added with AddRigCapture(). Each capture owns one shared
  // parameter block in the Camera extrinsics layout: [position(3),
  // angle-axis(3)] of the rig body (see Camera::POSITION / Camera::ORIENTATION).
  std::unordered_map<CaptureId, std::array<double, 6>> capture_poses_;

  // Member views of the captures in |capture_poses_|, with their sensor data.
  std::unordered_map<ViewId, RigViewInfo> rig_views_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTER_H_
