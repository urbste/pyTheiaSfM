// Copyright (C) 2023 Steffen Urban
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

#include "theia/sfm/view_graph/view_graph_from_reconstruction.h"
#include <Eigen/Core>


namespace theia {

TwoViewInfo CreateTwoViewInfo(
    const View* view_i,
    const View* view_j,
    const size_t& num_shared_tracks) {
  TwoViewInfo info;

  Eigen::Matrix3d orientation_i = view_i->Camera().GetOrientationAsRotationMatrix();
  Eigen::Matrix3d orientation_j = view_j->Camera().GetOrientationAsRotationMatrix();

  const Eigen::Matrix3d relative_rotation_mat =
      orientation_j * orientation_i.transpose();
  ceres::RotationMatrixToAngleAxis(relative_rotation_mat.data(),
                                   info.rotation_2.data());

  const Eigen::Vector3d position = (view_j->Camera().GetPosition() -
                             view_i->Camera().GetPosition()).normalized();
  info.position_2 = orientation_i * position;
  info.num_verified_matches = num_shared_tracks;
  return info;
}


std::set<TrackId> GetEstimatedTracks(const Reconstruction& reconstruction, const View* view) {
   std::set<TrackId> estimated_track_ids;
   for (const auto t_id : view->TrackIds()) {
     if (reconstruction.Track(t_id)->IsEstimated()) {
        estimated_track_ids.insert(t_id);
     }
   } 
   return estimated_track_ids;
}

void ViewGraphFromReconstruction(
    const theia::Reconstruction& reconstruction,
    const size_t min_shared_tracks,
    ViewGraph* view_graph) {

  const auto view_ids = reconstruction.ViewIds();

  for (size_t i = 0; i < view_ids.size(); ++i) {
    const auto view_id_i = view_ids[i];
    const View* view_i = reconstruction.View(view_id_i);
    if (!view_i->IsEstimated()) {
      continue;
    }
    const auto estimated_tracks_i = GetEstimatedTracks(reconstruction, view_i);

    for (size_t j = i + 1; j < view_ids.size(); ++j) {
      const auto view_id_j = view_ids[j];
      const View* view_j = reconstruction.View(view_ids[j]);

      const auto estimated_tracks_j = GetEstimatedTracks(reconstruction, view_j);
        
      std::set<int> shared_tracks;
      std::set_intersection(estimated_tracks_i.begin(), estimated_tracks_i.end(), 
          estimated_tracks_j.begin(), estimated_tracks_j.end(),
          std::inserter(shared_tracks, shared_tracks.begin())); 
        
      if (shared_tracks.size() > min_shared_tracks) {
        TwoViewInfo info = CreateTwoViewInfo(view_i, view_j, shared_tracks.size());
        view_graph->AddEdge(view_id_i, view_id_j, info);
      }
    }
  }
}

} // namespace theia