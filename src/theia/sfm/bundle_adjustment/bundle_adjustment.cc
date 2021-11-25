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

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"

#include <glog/logging.h>
#include <unordered_set>

#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"

namespace theia {

// Bundle adjust the specified views and tracks.
BundleAdjustmentSummary BundleAdjustPartialReconstruction(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& view_ids,
    const std::unordered_set<TrackId>& track_ids,
    Reconstruction* reconstruction) {
  CHECK_NOTNULL(reconstruction);

  BundleAdjuster bundle_adjuster(options, reconstruction);
  for (const ViewId view_id : view_ids) {
    bundle_adjuster.AddView(view_id);
  }
  for (const TrackId track_id : track_ids) {
    bundle_adjuster.AddTrack(
        track_id, options.use_homogeneous_local_point_parametrization);
  }

  return bundle_adjuster.Optimize();
}

// Bundle adjust the specified views.
BundleAdjustmentSummary
BundleAdjustPartialViewsConstant(const BundleAdjustmentOptions &options,
                                 const std::vector<ViewId> &var_view_ids,
                                 const std::vector<ViewId> &const_view_ids,
                                 Reconstruction *reconstruction) {
  CHECK_NOTNULL(reconstruction);

  BundleAdjuster bundle_adjuster(options, reconstruction);

  for (const ViewId view_id : var_view_ids) {
    bundle_adjuster.AddView(view_id);
  }
  for (const ViewId view_id : const_view_ids) {
    bundle_adjuster.AddView(view_id);
    bundle_adjuster.SetCameraExtrinsicsConstant(view_id);
  }

  for (const TrackId track_id : reconstruction->TrackIds()) {
    bundle_adjuster.AddTrack(
        track_id, options.use_homogeneous_local_point_parametrization);
  }

  return bundle_adjuster.Optimize();
}

// Bundle adjust the entire reconstruction.
BundleAdjustmentSummary BundleAdjustReconstruction(
    const BundleAdjustmentOptions& options, Reconstruction* reconstruction) {
  const auto& view_ids = reconstruction->ViewIds();
  const auto& track_ids = reconstruction->TrackIds();

  BundleAdjuster bundle_adjuster(options, reconstruction);
  for (const ViewId view_id : view_ids) {
    bundle_adjuster.AddView(view_id);
  }
  for (const TrackId track_id : track_ids) {
    bundle_adjuster.AddTrack(
        track_id, options.use_homogeneous_local_point_parametrization);
  }

  return bundle_adjuster.Optimize();
}

// Bundle adjust a single view.
BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         const ViewId view_id,
                                         Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  bundle_adjuster.AddView(view_id);
  return bundle_adjuster.Optimize();
}

// Bundle adjust views.
BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& view_ids_to_optimize,
    Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  for (const auto& view_id : view_ids_to_optimize) {
    bundle_adjuster.AddView(view_id);
  }
  return bundle_adjuster.Optimize();
}

// Bundle adjust a single track.
BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options,
    const TrackId track_id,
    Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  bundle_adjuster.AddTrack(track_id,
                           options.use_homogeneous_local_point_parametrization);
  return bundle_adjuster.Optimize();
}

// Bundle adjust a single track.
BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options,
    const TrackId track_id,
    Reconstruction* reconstruction,
    Matrix3d* empirical_covariance_matrix,
    double* empirical_variance_factor) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  bundle_adjuster.AddTrack(track_id, true);

  BundleAdjustmentSummary summary = bundle_adjuster.Optimize();
  if (!summary.success) {
    *empirical_covariance_matrix = Matrix3d::Identity();
  } else {
    if (!bundle_adjuster.GetCovarianceForTrack(track_id,
                                               empirical_covariance_matrix)) {
      summary.success = false;
      *empirical_variance_factor = 1.0;
    } else {
      // now get redundancy
      const double redundancy =
          reconstruction->Track(track_id)->NumViews() * 2 - 3;
      *empirical_variance_factor = (2.0 * summary.final_cost) / redundancy;
      *empirical_covariance_matrix *= *empirical_variance_factor;
    }
  }
  return summary;
}

// Bundle adjust tracks.
BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction,
    std::map<TrackId, Eigen::Matrix3d>* empirical_covariance_matrices,
    double* empirical_variance_factor) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  for (const auto& track_id : tracks_to_optimize) {
    // set homogeneous representation to true. otherwise covariance matrix will
    // be singular
    bundle_adjuster.AddTrack(track_id, true);
  }
  BundleAdjustmentSummary summary = bundle_adjuster.Optimize();

  if (!summary.success) {
    return summary;
  } else {
    if (!bundle_adjuster.GetCovarianceForTracks(
            tracks_to_optimize, empirical_covariance_matrices)) {
      summary.success = false;
      *empirical_variance_factor = 1.0;
    } else {
      // now get redundancy to calculate empirical covariance matrix
      int nr_obs = 0;
      double total_nr_vars = tracks_to_optimize.size() * 3.0;
      for (const auto& t : tracks_to_optimize) {
        nr_obs += reconstruction->Track(t)->NumViews();
      }
      const double redundancy = nr_obs * 2 - total_nr_vars;

      *empirical_variance_factor = (2.0 * summary.final_cost) / redundancy;
      for (auto& cov : *empirical_covariance_matrices) {
        cov.second *= *empirical_variance_factor;
      }
      LOG(INFO) << "Redundancy in BundleAdjustTracks: " << redundancy << "\n"
                << ", final cost: " << summary.final_cost
                << ", root mean square reprojection error: "
                << std::sqrt(summary.final_cost * 2.0 / nr_obs)
                << ", empirical variance factor: " << *empirical_variance_factor
                << "\n";
    }
  }

  return summary;
}

// Bundle adjust tracks.
BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& tracks_to_optimize,
    Reconstruction* reconstruction) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  for (const auto& track_id : tracks_to_optimize) {
    bundle_adjuster.AddTrack(
        track_id, options.use_homogeneous_local_point_parametrization);
  }
  return bundle_adjuster.Optimize();
}

BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         const ViewId view_id,
                                         Reconstruction* reconstruction,
                                         Matrix6d* empirical_covariance_matrix,
                                         double* empirical_variance_factor) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  bundle_adjuster.AddView(view_id);

  BundleAdjustmentSummary summary = bundle_adjuster.Optimize();
  if (!summary.success) {
    *empirical_covariance_matrix = Matrix6d::Identity();
    return summary;
  } else {
    if (!bundle_adjuster.GetCovarianceForView(view_id,
                                              empirical_covariance_matrix)) {
      summary.success = false;
      *empirical_variance_factor = 1.0;
    } else {
      // now get redundancy
      const double redundancy =
          reconstruction->View(view_id)->NumFeatures() * 2 - 6;
      *empirical_variance_factor = (2.0 * summary.final_cost) / redundancy;
      *empirical_covariance_matrix *= *empirical_variance_factor;
    }
  }
  return summary;
}

BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& view_ids,
    Reconstruction* reconstruction,
    std::map<ViewId, Matrix6d>* empirical_covariance_matrices,
    double* empirical_variance_factor) {
  BundleAdjustmentOptions ba_options = options;
  ba_options.linear_solver_type = ceres::DENSE_QR;
  ba_options.use_inner_iterations = false;

  BundleAdjuster bundle_adjuster(ba_options, reconstruction);
  for (const auto& vid : view_ids) {
    bundle_adjuster.AddView(vid);
  }

  BundleAdjustmentSummary summary = bundle_adjuster.Optimize();
  if (!summary.success) {
    return summary;
  } else {
    if (!bundle_adjuster.GetCovarianceForViews(view_ids,
                                               empirical_covariance_matrices)) {
      summary.success = false;
      *empirical_variance_factor = 1.0;
    } else {
      // now get redundancy to calculate empirical covariance matrix
      int nr_obs = 0;
      double total_nr_vars = view_ids.size() * 6.0;
      for (const auto& v : view_ids) {
        nr_obs += reconstruction->View(v)->NumFeatures();
      }
      const double redundancy = nr_obs * 2 - total_nr_vars;

      *empirical_variance_factor = (2.0 * summary.final_cost) / redundancy;
      for (auto& cov : *empirical_covariance_matrices) {
        cov.second *= *empirical_variance_factor;
      }
      LOG(INFO) << "Redundancy in BundleAdjustViews: " << redundancy << "\n"
                << ", final cost: " << summary.final_cost
                << ", root mean square reprojection error: "
                << std::sqrt(summary.final_cost * 2.0 / nr_obs)
                << ", empirical variance factor: " << *empirical_variance_factor
                << "\n";
    }
  }
  return summary;
}

}  // namespace theia
