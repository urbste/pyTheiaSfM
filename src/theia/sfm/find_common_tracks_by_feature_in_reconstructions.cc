// Copyright (C) 2023 The Regents of the University of California (Regents).
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
// Author: Steffen Urban (urbste@googlemail.com)

#include "theia/sfm/find_common_tracks_by_feature_in_reconstructions.h"

#include <unordered_map>
#include <unordered_set>
#include <glog/logging.h>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view.h"
#include "theia/sfm/track.h"
#include "theia/sfm/feature.h"

namespace theia {

void
FindCommonTracksByFeatureInReconstructions(
    const Reconstruction& reconstruction_ref,
    const Reconstruction& reconstruction_qry,
    const std::vector<std::pair<ViewId, ViewId>>& view_graph_matches_ref_qry,
    std::vector<Eigen::Vector3d>* points_ref,
    std::vector<Eigen::Vector3d>* points_qry,
    std::vector<std::pair<TrackId, TrackId>>* track_id_pairs) {
  

  // Create a map to store track correspondences
  std::unordered_map<TrackId, TrackId> track_correspondences;
  
  // Iterate through all view pairs to find common tracks
  for (const auto& view_pair : view_graph_matches_ref_qry) {
    const ViewId view_id_ref = view_pair.first;
    const ViewId view_id_qry = view_pair.second;
    
    const View* view_ref = reconstruction_ref.View(view_id_ref);
    const View* view_qry = reconstruction_qry.View(view_id_qry);
    
    if (!view_ref || !view_qry) {
      LOG(WARNING) << "View not found in reconstruction";
      continue;
    }
    
    const auto& tracks_ref = view_ref->TrackIds();
    
    for (const auto& track_id_ref : tracks_ref) {
      const Track* track_ref = reconstruction_ref.Track(track_id_ref);
      if (!track_ref || !track_ref->IsEstimated()) {
        continue;
      }
      
      const Feature* ref_feat = view_ref->GetFeature(track_id_ref);
      if (!ref_feat) {
        continue;
      }
      
      // Find corresponding track in query reconstruction
      const TrackId track_id_qry = view_qry->GetTrack(*ref_feat);
      if (track_id_qry == kInvalidTrackId) {
        continue;
      }
      
      const Track* track_qry = reconstruction_qry.Track(track_id_qry);
      if (!track_qry || !track_qry->IsEstimated()) {
        continue;
      }
      
      // Check if we already found this correspondence
      auto it = track_correspondences.find(track_id_ref);
      if (it != track_correspondences.end()) {
        // Verify it's the same track
        if (it->second != track_id_qry) {
          LOG(WARNING) << "Inconsistent track correspondence found";
          continue;
        }
      } else {
        track_correspondences[track_id_ref] = track_id_qry;
      }
      
      // Add to pairs
      points_ref->push_back(track_ref->Point().hnormalized());
      points_qry->push_back(track_qry->Point().hnormalized());
      track_id_pairs->push_back(std::make_pair(track_id_ref, track_id_qry));
    }
  }

}

}  // namespace theia