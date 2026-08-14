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

#ifndef THEIA_SFM_ESTIMATE_TWOVIEW_INFO_H_
#define THEIA_SFM_ESTIMATE_TWOVIEW_INFO_H_

#include <memory>
#include <vector>

#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/pose/fast_iterative_five_point.h"
#include "theia/sfm/two_view_estimation_method.h"

namespace theia {

class RandomNumberGenerator;
class TwoViewInfo;
struct CameraIntrinsicsPrior;
struct FeatureCorrespondence;

// Options for estimating two view infos.
struct EstimateTwoViewInfoOptions {
  // The random number generator used to generate random numbers through the
  // two-view estimation process. If this is a nullptr then the random generator
  // will be initialized based on the current time.
  std::shared_ptr<RandomNumberGenerator> rng;

  // Type of Ransac variant to use.
  RansacType ransac_type = RansacType::RANSAC;

  // Maximum sampson error in pixels for correspondences to be inliers.
  //
  // NOTE: This threshold is with respect to an image that is 1024 pixels
  // wide. If the image dimensions are larger or smaller than this value then
  // the threshold will be appropriately scaled. This allows us to use a single
  // threshold for images of varying resolutions.
  double max_sampson_error_pixels = 6.0;

  // Ransac parameters.
  double expected_ransac_confidence = 0.9999;
  int min_ransac_iterations = 10;
  int max_ransac_iterations = 1000;
  bool use_mle = true;
  bool use_lo = false;
  int lo_start_iterations = 10;

  // Two-view estimation method. Defaults to Sturm 5-point.
  // Set to FAST_ITERATIVE_FIVE_POINT for fast Dogleg solver on forward-facing trajectories.
  TwoViewEstimationMethod estimation_method = TwoViewEstimationMethod::FIVE_POINT_STURM;

  // Options for the FastIterativeFivePoint solver (when estimation_method == FAST_ITERATIVE_FIVE_POINT).
  // Constrained to forward-facing trajectories with prior [0,0,1] / identity rotation.
  FastIterativeFivePointOptions fast_iterative_5pt_options;

  // Use the Sturm-sequence-based 5-point solver (ported from PoseLib) instead
  // of theia's Stewenius-style eigendecomposition solver inside the RANSAC
  // inner loop. See RansacParameters::use_sturm_5pt for details.
  bool use_sturm_5pt = true;

  // Use the monodepth 3-point minimal solvers (ported from PoseLib's
  // RePoseD) instead of the standard 5-/8-point solvers, when correspondences
  // carry a monocular-depth prior (Feature::depth_prior_ > 0 on both sides
  // for effectively all matches). Requires far fewer RANSAC iterations since
  // the minimal sample size drops from 5 (or 8, uncalibrated) to 3. Silently
  // (with a one-time warning) falls back to the standard path if depth
  // priors are missing.
  bool use_monodepth = false;

  // Only relevant when use_monodepth is true and the views are uncalibrated:
  // whether to assume both cameras share one unknown focal length (true) or
  // solve for two independent focal lengths (false).
  bool monodepth_shared_focal = true;

  // min and max focal length
  // this is very useful for the ransac loop, if we already
  // have a guess of the focal length of our cameras
  double min_focal_length = 1.;
  double max_focal_length = std::numeric_limits<double>::max();
};

// Estimates two view info for the given view pair from the correspondences. The
// correspondences should be in pixel coordinates (the method will perform
// normalization w.r.t focal length as necessary).
//
// There are three cases for estimating two view infos:
//   1) Both views are calibrated. In this case, the essential matrix is
//      estimated then decomposed to compute the two view info.
//   2) Both views are uncalibrated. The fundamental matrix is estimated and
//      decomposed to compute the two view info. NOTE: The quality of the focal
//      length is not always great with fundamental matrix decomposition.
//   3) One view is calibrated and one view is uncalibrated. NOTE: This case is
//      currently unsupported, and case 2) will be used instead.
//
// Returns true if a two view info could be successfully estimated and false if
// not.
bool EstimateTwoViewInfo(
    const EstimateTwoViewInfoOptions& options,
    const CameraIntrinsicsPrior& intrinsics1,
    const CameraIntrinsicsPrior& intrinsics2,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* twoview_info,
    std::vector<int>* inlier_indices);

}  // namespace theia

#endif  // THEIA_SFM_ESTIMATE_TWOVIEW_INFO_H_
