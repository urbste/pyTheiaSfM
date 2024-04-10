// Copyright (C) 2014 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_SFM_RECONSTRUCTION_ESTIMATOR_OPTIONS_H_
#define THEIA_SFM_RECONSTRUCTION_ESTIMATOR_OPTIONS_H_

#include <memory>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/estimate_track.h"
#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"
#include "theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/util/random.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "src/theia/solvers/sample_consensus_estimator.h"

namespace theia {

// Global SfM methods are considered to be more scalable while incremental SfM
// is less scalable but often more robust.
enum class ReconstructionEstimatorType {
  GLOBAL = 0,
  INCREMENTAL = 1,
  HYBRID = 2
};

// The recommended type of rotations solver is the Robust L1-L2 method. This
// method is scalable, extremely accurate, and very efficient. See the
// global_pose_estimation directory for more details.
enum class GlobalRotationEstimatorType {
  ROBUST_L1L2 = 0,
  NONLINEAR = 1,
  LINEAR = 2,
  LAGRANGE_DUAL = 3,
  HYBRID = 4
};

// Global position estimation methods.
//   NONLINEAR: This method minimizes the nonlinear pairwise translation
//     constraint to solve for positions.
//   LINEAR_TRIPLET: This linear method computes camera positions by
//     minimizing an error for image triplets. Essentially, it tries to
//     enforce a loop/triangle constraint for triplets.
//   LEAST_UNSQUARED_DEVIATION: This robust method uses the least unsquared
//     deviation instead of least squares. It is essentially an L1 solver.
enum class GlobalPositionEstimatorType {
  NONLINEAR = 0,
  LINEAR_TRIPLET = 1,
  LEAST_UNSQUARED_DEVIATION = 2,
  LIGT = 3
};

// Track parametrization type:
//   XYZW: 4D point --> optimization over homogeneous 4 vector
//   XYZ_MANIFOLD: 4D point --> optimization over ceres::SphereManifold<4>() 
//   INVERSE_DEPTH: optimization of track using inverse depth
//   parametrization
enum class TrackParametrizationType {
  XYZW = 0,
  XYZW_MANIFOLD = 1,
  INVERSE_DEPTH = 2
};

// Options for the reconstruction estimation.
struct ReconstructionEstimatorOptions {
  // Type of reconstruction estimation to use.
  ReconstructionEstimatorType reconstruction_estimator_type =
      ReconstructionEstimatorType::GLOBAL;

  // If Global SfM is desired, which type of rotation and position estimation
  // methods are used.
  GlobalRotationEstimatorType global_rotation_estimator_type =
      GlobalRotationEstimatorType::ROBUST_L1L2;

  GlobalPositionEstimatorType global_position_estimator_type =
      GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION;

  // The random number generator used to generate random numbers through the
  // reconstruction estimation process. If this is a nullptr then the random
  // generator will be initialized based on the current time.
  std::shared_ptr<RandomNumberGenerator> rng;

  // Number of threads to use.
  int num_threads = 1;

  // Maximum reprojection error. This is the threshold used for filtering
  // outliers after bundle adjustment.
  double max_reprojection_error_in_pixels = 5.0;

  // Any edges in the view graph with fewer than min_num_two_view_inliers will
  // be removed as an initial filtering step.
  int min_num_two_view_inliers = 30;

  // --------------- RANSAC Options --------------- //
  double ransac_confidence = 0.9999;
  int ransac_min_iterations = 50;
  int ransac_max_iterations = 1000;
  bool ransac_use_mle = true;
  bool ransac_use_lo = true;
  int ransac_lo_start_iterations = 50;

  // --------------- Rotation Filtering Options --------------- //

  // After orientations are estimated, view pairs may be filtered/removed if the
  // relative rotation of the view pair differs from the relative rotation
  // formed by the global orientation estimations. Adjust this threshold to
  // control the threshold at which rotations are filtered. See
  // theia/sfm/filter_view_pairs_from_orientation.h
  double rotation_filtering_max_difference_degrees = 5.0;

  // --------------- Position Filtering Options --------------- //

  // Refine the relative translations based on the epipolar error and known
  // rotation estimations. This improve the quality of the translation
  // estimation.
  bool refine_relative_translations_after_rotation_estimation = true;

  // If true, the maximal rigid component of the viewing graph will be
  // extracted. This means that only the cameras that are well-constrained for
  // position estimation will be used. This method is somewhat slow, so enabling
  // it will cause a performance hit in terms of efficiency.
  //
  // NOTE: This method does not attempt to remove outlier 2-view geometries, it
  // only determines which cameras are well-conditioned for position estimation.
  bool extract_maximal_rigid_subgraph = false;

  // If true, filter the pairwise translation estimates to remove potentially
  // bad relative poses. Removing potential outliers can increase the
  // performance of position estimation.
  bool filter_relative_translations_with_1dsfm = true;

  // Before the camera positions are estimated, it is wise to remove any
  // relative translations estimates that are low quality. See
  // theia/sfm/filter_view_pairs_from_relative_translation.h
  int translation_filtering_num_iterations = 48;
  double translation_filtering_projection_tolerance = 0.1;

  // --------------- Global Rotation Estimation Options --------------- //

  // Robust loss function scales for nonlinear estimation.
  double rotation_estimation_robust_loss_scale = 0.1;

  // --------------- Global Position Estimation Options --------------- //
  NonlinearPositionEstimator::Options nonlinear_position_estimator_options;
  LinearPositionEstimator::Options linear_triplet_position_estimator_options;
  LiGTPositionEstimator::Options ligt_position_estimator_options;
  LeastUnsquaredDeviationPositionEstimator::Options
      least_unsquared_deviation_position_estimator_options;

  // For global SfM it may be advantageous to run a partial bundle adjustment
  // optimizing only the camera positions and 3d points while holding camera
  // orientation and intrinsics constant.
  bool refine_camera_positions_and_points_after_position_estimation = true;

  // --------------------- Incremental SfM Options --------------------- //

  // If M is the maximum number of 3D points observed by any view, we want to
  // localize all views that observe > M * multiple_view_localization_ratio 3D
  // points. This allows for multiple well-conditioned views to be added to the
  // reconstruction before needing bundle adjustment.
  double multiple_view_localization_ratio = 0.8;

  // When adding a new view to the current reconstruction, this is the
  // reprojection error that determines whether a 2D-3D correspondence is an
  // inlier during localization.
  //
  // NOTE: This threshold is with respect to an image that is 1024 pixels
  // wide. If the image dimensions are larger or smaller than this value then
  // the threshold will be appropriately scaled. This allows us to use a single
  // threshold for images that have varying resolutions.
  double absolute_pose_reprojection_error_threshold = 4.0;

  // Minimum number of inliers for absolute pose estimation to be considered
  // successful.
  int min_num_absolute_pose_inliers = 30;

  // Bundle adjustment of the entire reconstruction is triggered when the
  // reconstruction has grown by more than this percent. That is, if we last ran
  // BA when there were K views in the reconstruction and there are now N views,
  // then G = (N - K) / K is the percent that the model has grown. We run bundle
  // adjustment only if G is greater than this variable. This variable is
  // indicated in percent so e.g., 5.0 = 5%.
  double full_bundle_adjustment_growth_percent = 5.0;

  // During incremental SfM we run "partial" bundle adjustment on the most
  // recent views that have been added to the 3D reconstruction. This parameter
  // controls how many views should be part of the partial BA.
  int partial_bundle_adjustment_num_views = 20;

  // --------------------- Hybrid SfM Options --------------------- //

  // The relative position of the initial pair used for the incremental portion
  // of hybrid sfm is re-estimated using a simplified relative translations
  // solver (assuming known rotation). The relative position is re-estimated
  // using a RANSAC procedure with the inlier threshold defined by this
  // parameter.
  double relative_position_estimation_max_sampson_error_pixels = 4.0;

  // --------------- Triangulation Options --------------- //

  // Minimum angle required between a 3D point and 2 viewing rays in order to
  // consider triangulation a success.
  double min_triangulation_angle_degrees = 3.0;

  // The reprojection error to use for determining valid triangulation.
  double triangulation_max_reprojection_error_in_pixels = 10.0;

  // Bundle adjust a track immediately after estimating it.
  bool bundle_adjust_tracks = true;

  // Bundle adjust a track immediately after estimating it.
  TriangulationMethodType triangulation_method =
      TriangulationMethodType::MIDPOINT;

  // --------------- Bundle Adjustment Options --------------- //

  // After computing a model and performing an initial BA, the reconstruction
  // can be further improved (and even densified) if we attempt (again) to
  // retriangulate any tracks that are currently unestimated. For each
  // retriangulation iteration we do the following:
  //   1. Remove features that are above max_reprojection_error_in_pixels.
  //   2. Triangulate all unestimated tracks.
  //   3. Perform full bundle adjustment.
  //
  // NOTE: This is only utilized in the Global SfM module.
  int num_retriangulation_iterations = 1;

  // For bundle adjustment, we may want to use a robust loss function to improve
  // robustness to outliers. The various types of robust loss functions used can
  // be found at //theia/sfm/bundle_adjustment/create_loss_function.h
  LossFunctionType bundle_adjustment_loss_function_type =
      LossFunctionType::TRIVIAL;

  // For robust loss functions, the robustness will begin for values that have
  // an error greater than this value. For example, Tukey loss will have a
  // constant loss when the error values are greater than this.
  double bundle_adjustment_robust_loss_width = 10.0;

  // Use SPARSE_SCHUR for problems smaller than this size and ITERATIVE_SCHUR
  // for problems larger than this size.
  int min_cameras_for_iterative_solver = 1000;

  // If accurate calibration is known ahead of time then it is recommended to
  // set the camera intrinsics constant during bundle adjustment. Othewise, you
  // can choose which intrinsics to optimize. See
  // //theia/sfm/bundle_adjustment_options.h for full details.
  OptimizeIntrinsicsType intrinsics_to_optimize =
      OptimizeIntrinsicsType::FOCAL_LENGTH |
      OptimizeIntrinsicsType::RADIAL_DISTORTION;

  // The track parametrizeation type to use for bundle adjustment.
  // Standard is XYZW which will optimize a homogeneous vector, but on a
  // euclidean manifold
  TrackParametrizationType track_parametrization_type =
      TrackParametrizationType::XYZW_MANIFOLD;

  // --------------- Track Subsampling Options --------------- //

  // Bundle adjustment performs joint nonlinear optimization of point positions
  // and camera poses by minimizing reprojection error. For many scenes, the 3d
  // points can be highly redundant such that adding more points only marginally
  // improves the reconstruction quality (if at all) despite a large increase in
  // runtime. As such, we can reduce the number of 3d points used in bundle
  // adjustment and still achieve similar or even better quality reconstructions
  // by carefully choosing the points such that they properly constrain the
  // optimization.
  //
  // If subsampling the tracks is set to true, then the 3d points are chosen
  // such that they fit the following criteria:
  //
  //    a) High confidence (i.e. low reprojection error).
  //    b) Long tracks are preferred.
  //    c) The tracks used for optimization provide a good spatial coverage in
  //       each image.
  //    d) Each view observes at least K optimized tracks.
  //
  // Tracks are selected to optimize for these criteria using the thresholds
  // below.
  bool subsample_tracks_for_bundle_adjustment = false;

  // Long tracks are preferred during the track subsampling, but csweeney has
  // observed that long tracks often are more likely to contain outlier. Thus,
  // we cap the track length for track selection at 10 then sort tracks first by
  // the truncated track length, then secondarily by their mean reprojection
  // error. This allows us to choose the high quality tracks among all the long
  // tracks.
  int track_subset_selection_long_track_length_threshold = 10;

  // To satisfy c) above, we divide each image into an image grid with grid cell
  // widths specified by this threshold. The top ranked track in each grid cell
  // is chosen to be optimized so that each image has a good spatial coverage.
  int track_selection_image_grid_cell_size_pixels = 100;

  // The minimum number of optimized tracks required for each view when using
  // track subsampling. If the view does not observe this many tracks, then all
  // tracks in the view are optimized.
  int min_num_optimized_tracks_per_view = 200;

  // --------------- Reconstruction Localization Options --------------- //

  // The PnP type that is used in the calibrated camera case
  PnPType localization_pnp_type = PnPType::DLS;

};

}  // namespace theia

#endif  // THEIA_SFM_RECONSTRUCTION_ESTIMATOR_OPTIONS_H_
