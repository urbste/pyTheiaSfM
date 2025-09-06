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

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H_

#include <ceres/types.h>
#include <map>
#include <unordered_set>
#include <vector>
#include <thread>

#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/types.h"
#include "theia/util/enable_enum_bitmask_operators.h"

namespace theia {

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class Reconstruction;

// The camera intrinsics parameters are defined by:
//   - Focal length
//   - Aspect ratio
//   - Skew
//   - Principal points (x and y)
//   - Radial distortion
//   - Tangential distortion
// These intrinsic parameters may or may not be present for a given camera model
// and only the relevant intrinsics will be optimized per camera. It is often
// known for instance that skew is 0 and aspect ratio is 1, and so we do not
// always desire to optimize all camera intrinsics. In many cases, the focal
// length is the only parameter we care to optimize.
//
// Users can specify which intrinsics to optimize by using a bitmask. For
// instance FOCAL_LENGTH|PRINCIPAL_POINTS will optimize the focal length and
// principal points. The options NONE and ALL are also given for convenience.
enum class OptimizeIntrinsicsType {
  NONE = 0x00,
  FOCAL_LENGTH = 0x01,
  ASPECT_RATIO = 0x02,
  SKEW = 0x04,
  PRINCIPAL_POINTS = 0x08,
  RADIAL_DISTORTION = 0x10,
  TANGENTIAL_DISTORTION = 0x20,
  DISTORTION = RADIAL_DISTORTION | TANGENTIAL_DISTORTION,
  FOCAL_LENGTH_DISTORTION = RADIAL_DISTORTION | TANGENTIAL_DISTORTION | FOCAL_LENGTH,
  FOCAL_LENGTH_RADIAL_DISTORTION = RADIAL_DISTORTION | FOCAL_LENGTH,
  ALL = FOCAL_LENGTH | ASPECT_RATIO | SKEW | PRINCIPAL_POINTS |
        RADIAL_DISTORTION | TANGENTIAL_DISTORTION,
};
ENABLE_ENUM_BITMASK_OPERATORS(OptimizeIntrinsicsType)

struct BundleAdjustmentOptions {
  // The type of loss function used for BA. By default, we use a standard L2
  // loss function, but robust cost functions could be used.
  LossFunctionType loss_function_type = LossFunctionType::TRIVIAL;
  double robust_loss_width = 2.0;
  // e.g. 1cm for downweighting depth priors as outliers
  double robust_loss_width_depth_prior = 0.01;

  // For larger problems (> 1000 cameras) it is recommended to use the
  // ITERATIVE_SCHUR solver.
  ceres::LinearSolverType linear_solver_type = ceres::SPARSE_SCHUR;
  ceres::PreconditionerType preconditioner_type = ceres::SCHUR_JACOBI;
  ceres::VisibilityClusteringType visibility_clustering_type =
      ceres::CANONICAL_VIEWS;

  ceres::DenseLinearAlgebraLibraryType dense_linear_algebra_library_type = ceres::EIGEN;
  ceres::SparseLinearAlgebraLibraryType sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;

  // Use mixed precision solves
  bool use_mixed_precision_solves = false;

  // Max number of refinement iterations
  int max_num_refinement_iterations = 2;


  // If true, ceres will log verbosely.
  bool verbose = false;

  // If true, the camera orientations and/or positions will be set to
  // constant. This may be desirable if, for instance, you have accurate
  // positions from GPS but do not know camera orientations.
  bool constant_camera_orientation = false;
  bool constant_camera_position = false;

  // use local parametrization for points. Apply increments in local tangent
  // space. Reduce from dim 4 -> 3
  bool use_homogeneous_point_parametrization = true;

  // if inverse depth parametrization should be used
  // if this is set to true use_homogeneous_point_parametrization is ignored
  bool use_inverse_depth_parametrization = false;

  // Indicates which intrinsics should be optimized as part of bundle
  // adjustment. Default to NONE!
  OptimizeIntrinsicsType intrinsics_to_optimize = OptimizeIntrinsicsType::NONE;

  int num_threads = std::thread::hardware_concurrency();
  int max_num_iterations = 100;

  // Max BA time is 1 hour.
  double max_solver_time_in_seconds = 3600.0;

  // Inner iterations can improve the quality according to the Ceres email list.
  bool use_inner_iterations = true;

  // These variables may be useful to change if the optimization is converging
  // to a bad result.
  double function_tolerance = 1e-6;
  double gradient_tolerance = 1e-10;
  double parameter_tolerance = 1e-8;
  double max_trust_region_radius = 1e12;

  // Use position priors
  bool use_position_priors = false;

  // Use orientation priors
  bool use_orientation_priors = false;

  // Add depth priors
  bool use_depth_priors = false;

  // Adjusting orthographic camera
  bool orthographic_camera = false;

  // Use gravity priors
  bool use_gravity_priors = false;
};

// Some important metrics for analyzing bundle adjustment results.
struct BundleAdjustmentSummary {
  // This only indicates whether the optimization was successfully run and makes
  // no guarantees on the quality or convergence.
  bool success = false;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double setup_time_in_seconds = 0.0;
  double solve_time_in_seconds = 0.0;
};

// Bundle adjust all views and tracks in the reconstruction.
BundleAdjustmentSummary BundleAdjustReconstruction(
    const BundleAdjustmentOptions& options, Reconstruction* reconstruction);

// Bundle adjust the specified views and all tracks observed by those views.
BundleAdjustmentSummary BundleAdjustPartialReconstruction(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& views_to_optimize,
    const std::unordered_set<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction);

BundleAdjustmentSummary
BundleAdjustPartialViewsConstant(
    const BundleAdjustmentOptions &options,
    const std::vector<ViewId> &var_view_ids,
    const std::vector<ViewId> &const_view_ids,
    Reconstruction *reconstruction);

// Bundle adjust a single view.
BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         const ViewId view_id,
                                         Reconstruction* reconstruction);

// Bundle adjust a single view.
BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& view_ids_to_optimize,
    Reconstruction* reconstruction);

// Bundle adjust a single track.
BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options,
    const TrackId track_id,
    Reconstruction* reconstruction);

// Bundle adjust tracks.
BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction);

///////// With covariance estimation
// Bundle adjust a single track.
BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options,
    const TrackId track_id,
    Reconstruction* reconstruction,
    Matrix3d* empirical_covariance_matrix,
    double* empirical_variance_factor);

// Bundle adjust tracks.
BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction,
    std::map<TrackId, Eigen::Matrix3d>* empirical_covariance_matrices,
    double* emprical_variance_factor);

// Bundle adjust a single track.
BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         const ViewId view_id,
                                         Reconstruction* reconstruction,
                                         Matrix6d* empirical_covariance_matrix,
                                         double* empirical_variance_factor);

BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& view_ids,
    Reconstruction* reconstruction,
    std::map<ViewId, Matrix6d>* empirical_covariance_matrix,
    double* empirical_variance_factor);
}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_BUNDLE_ADJUSTMENT_H_
