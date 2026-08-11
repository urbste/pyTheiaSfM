// Copyright (C) 2026 The pyTheiaSfM Authors.
// All rights reserved.
//
// Incremental reconstruction for multi-camera / stereo rigs. Seeds the first
// RigCapture at the abstract-body identity, triangulates intra-rig tracks
// (metric from known extrinsics), then localizes subsequent captures against
// the growing map.

#ifndef THEIA_SFM_INCREMENTAL_RIG_RECONSTRUCTOR_H_
#define THEIA_SFM_INCREMENTAL_RIG_RECONSTRUCTOR_H_

#include <unordered_set>
#include <vector>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/estimate_track.h"
#include "theia/sfm/localize_view_to_reconstruction.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/types.h"
#include "theia/solvers/sample_consensus_estimator.h"

namespace theia {

class Reconstruction;
class ViewGraph;

struct IncrementalRigReconstructorOptions {
  // Minimum number of estimated tracks visible in a capture to attempt
  // localization.
  int min_num_abs_pose_features = 30;

  // Minimum number of RANSAC inliers for a successful capture localization.
  int min_num_inliers_for_localization = 16;

  // Reprojection error threshold (pixels) for localization / track estimation.
  double max_reprojection_error_in_pixels = 4.0;

  // Minimum triangulation angle between views.
  double min_triangulation_angle_degrees = 2.0;

  // Run bundle adjustment after each newly localized capture.
  bool bundle_adjust_after_localize = true;

  // If true, try EstimateRigidTransformation2D3D using all sensors in the
  // capture (cameras posed in the abstract rig frame). Falls back to
  // localizing a single view and composing the body pose.
  bool use_generalized_localization = true;

  RansacParameters ransac_params;
  BundleAdjustmentOptions ba_options;
  LocalizeViewToReconstructionOptions localize_options;
  TrackEstimator::Options track_estimator_options;

  IncrementalRigReconstructorOptions() {
    ransac_params.error_thresh = max_reprojection_error_in_pixels *
                                 max_reprojection_error_in_pixels;
    ransac_params.failure_probability = 0.01;
    localize_options.reprojection_error_threshold_pixels =
        max_reprojection_error_in_pixels;
    localize_options.min_num_inliers = min_num_inliers_for_localization;
    localize_options.bundle_adjust_view = false;
    track_estimator_options.max_acceptable_reprojection_error_pixels =
        max_reprojection_error_in_pixels;
    track_estimator_options.min_triangulation_angle_degrees =
        min_triangulation_angle_degrees;
    track_estimator_options.bundle_adjustment = true;
  }
};

class IncrementalRigReconstructor {
 public:
  explicit IncrementalRigReconstructor(
      const IncrementalRigReconstructorOptions& options);

  // Estimates rig-capture poses and tracks. |view_graph| is reserved for
  // future capture-graph seeding; tracks are read from |reconstruction|.
  ReconstructionEstimatorSummary Estimate(ViewGraph* view_graph,
                                          Reconstruction* reconstruction);

 private:
  bool SeedInitialCapture(Reconstruction* reconstruction);
  bool LocalizeCapture(const CaptureId capture_id,
                       Reconstruction* reconstruction);
  bool LocalizeCaptureGeneralized(const CaptureId capture_id,
                                  Reconstruction* reconstruction);
  bool LocalizeCaptureFromSingleView(const CaptureId capture_id,
                                     Reconstruction* reconstruction);
  void EstimateStructure(Reconstruction* reconstruction);
  void BundleAdjustAndPropagate(Reconstruction* reconstruction);

  std::vector<CaptureId> OrderedUnestimatedCaptures(
      const Reconstruction& reconstruction) const;
  int NumEstimatedTracksInCapture(const CaptureId capture_id,
                                  const Reconstruction& reconstruction) const;

  const IncrementalRigReconstructorOptions options_;
};

}  // namespace theia

#endif  // THEIA_SFM_INCREMENTAL_RIG_RECONSTRUCTOR_H_
