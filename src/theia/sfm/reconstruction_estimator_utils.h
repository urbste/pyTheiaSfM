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

#ifndef THEIA_SFM_RECONSTRUCTION_ESTIMATOR_UTILS_H_
#define THEIA_SFM_RECONSTRUCTION_ESTIMATOR_UTILS_H_

#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>

#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/util/map_util.h"

namespace theia {
class Reconstruction;
class ViewGraph;
struct ReconstructionEstimatorOptions;

// By default, Theia uses pixel error thresholds based on an image that is 1024
// pixels wide. For images of different resolutions this function will scale the
// threshold based on the maximum image dimensions. If the image dimensions are
// not set, then the input value is returned.
double ComputeResolutionScaledThreshold(const double threshold_pixels,
                                        const int image_width,
                                        const int image_height);

// Sets the bundle adjustment optiosn from the reconstruction estimator options.
BundleAdjustmentOptions SetBundleAdjustmentOptions(
    const ReconstructionEstimatorOptions& options, const int num_views);

// Sets the ransac parameters from the reconstruction estimator options.
// NOTE: This does not set the error threshold since that is application
// specific. The caller must set this threshold.
RansacParameters SetRansacParameters(
    const ReconstructionEstimatorOptions& options);

// Collects the relative rotations for each view pair into a simple map.
std::unordered_map<ViewIdPair, Eigen::Vector3d> RelativeRotationsFromViewGraph(
    const ViewGraph& view_graph);

// Each view that has a rotation and position estimated has the Camera pose set.
void SetReconstructionFromEstimatedPoses(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& positions,
    Reconstruction* reconstruction);

// Given a reconstruction, return a new reconstruction that contains only the
// estimated views and tracks.
void CreateEstimatedSubreconstruction(
    const Reconstruction& input_reconstruction,
    Reconstruction* estimated_reconstruction);

// Outputs the ViewId of all estimated views in the reconstruction.
void GetEstimatedViewsFromReconstruction(const Reconstruction& reconstruction,
                                         std::unordered_set<ViewId>* views);

// Outputs the TrackId of all estimated tracks in the reconstruction.
void GetEstimatedTracksFromReconstruction(const Reconstruction& reconstruction,
                                          std::unordered_set<TrackId>* tracks);

// Refine the relative translation estimates between view pairs by optimizing
// the epipolar constraint given the known rotation estimation.
void RefineRelativeTranslationsWithKnownRotations(
    const Reconstruction& reconstruction,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    const int num_threads,
    ViewGraph* view_graph);

// Sets all tracks that are not seen by enough estimated views to unestimated.
// Returns the number of tracks set to unestimated.
int SetUnderconstrainedTracksToUnestimated(Reconstruction* reconstruction);

// Sets all vies that are not seen by enough estimated tracks to unestimated.
// Returns the number of views set to unestimated.
int SetUnderconstrainedViewsToUnestimated(Reconstruction* reconstruction);

// Return the number of estimated views or tracks in the reconstruction.
int NumEstimatedViews(const Reconstruction& reconstruction);
int NumEstimatedTracks(const Reconstruction& reconstruction);

// A convenience method for setting a selection of tracks in the specified views
// to be unestimated. The specified set of input tracks will remain as
// "estimated", but all others will be set to unestimated.
template <typename Container>
void SetTracksInViewsToUnestimated(
    const Container& views,
    const std::unordered_set<TrackId>& tracks_to_stay_estimated,
    Reconstruction* reconstruction) {
  // For each estimated view in the provided view list, set the tracks in the
  // view to be unestimated.
  for (const ViewId view_id : views) {
    const View* view = reconstruction->View(view_id);
    if (view == nullptr || !view->IsEstimated()) {
      continue;
    }

    // Set all tracks in the view to unestimated, unless they are part of the
    // subset that remains estimated.
    const auto& tracks_in_view = view->TrackIds();
    for (const TrackId track_id : tracks_in_view) {
      Track* track = reconstruction->MutableTrack(track_id);
      // Skip this track if it is invalid or it was requested to stay
      // estimated. This assumes that tracks that "stay" estimated are already
      // set to be estimated.
      if (track == nullptr || ContainsKey(tracks_to_stay_estimated, track_id)) {
        continue;
      }
      track->SetEstimated(false);
    }
  }
}

}  // namespace theia

#endif  // THEIA_SFM_RECONSTRUCTION_ESTIMATOR_UTILS_H_
