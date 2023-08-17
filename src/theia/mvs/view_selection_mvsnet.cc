// Copyright (C) 2023, Steffen Urban
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

#include "theia/mvs/view_selection_mvsnet.h"
#include "theia/math/util.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view_graph/view_graph_from_reconstruction.h"

namespace theia {

double ComputeAngleBetweenRays(const Eigen::Vector3d& ci,
                               const Eigen::Vector3d& cj,
                               const Eigen::Vector3d& p) {
  const Eigen::Vector3d ray1 = (p - ci).normalized();
  const Eigen::Vector3d ray2 = (p - cj).normalized();
  const double cos_angle = ray1.dot(ray2);
  return 180. / M_PI * std::acos(cos_angle);
}

double ScoreFun(const double theta, const double theta0, const double sigma) {
  return std::exp(-std::pow(theta - theta0, 2) / (2 * std::pow(sigma, 2)));
}

std::unordered_map<ViewId, std::map<double, ViewId, std::greater<double>>>
ViewSelectionMVSNet(const Reconstruction& reconstruction,
                    const int num_neighbors,
                    const double theta0,
                    const double sigma1,
                    const double sigma2) {
  std::unordered_map<ViewId, std::map<double, ViewId, std::greater<double>>>
      view_selection;
  // get all view ids
  const std::vector<ViewId> view_ids = reconstruction.ViewIds();

  ViewGraph view_graph;
  ViewGraphFromReconstruction(reconstruction, 10, &view_graph);

  for (const auto& view_i_id : view_ids) {
    const auto view_i = reconstruction.View(view_i_id);
    if (!view_i->IsEstimated()) {
      continue;
    }
    // get neighbors of view i and track_ids
    const auto neighbors = *view_graph.GetNeighborIdsForView(view_i_id);
    const auto& track_ids_i = reconstruction.View(view_i_id)->TrackIds();
    // position of view_i
    const Eigen::Vector3d ci = view_i->Camera().GetPosition();
    std::map<double, ViewId, std::greater<double>> all_view_scores;

    for (const auto& view_j_id : neighbors) {
      const auto view_j = reconstruction.View(view_j_id);
      if (!view_j->IsEstimated()) {
        continue;
      }
      // position of view_j
      const Eigen::Vector3d cj = view_j->Camera().GetPosition();

      // get track_ids of view j
      const auto& track_ids_j = reconstruction.View(view_j_id)->TrackIds();
      // get intersection of track_ids (same tracks in both views)
      std::vector<TrackId> intersection;
      std::set_intersection(track_ids_i.begin(),
                            track_ids_i.end(),
                            track_ids_j.begin(),
                            track_ids_j.end(),
                            std::back_inserter(intersection));
      // now
      double score = 0.0;
      for (const auto& t_id : intersection) {
        const auto track = reconstruction.Track(t_id);
        const Eigen::Vector3d p = track->Point().hnormalized();
        const double theta = ComputeAngleBetweenRays(ci, cj, p);

        if (theta <= theta0) {
          score += ScoreFun(theta, theta0, sigma1);
        } else {
          score += ScoreFun(theta, theta0, sigma2);
        }
      }
      all_view_scores[score] = view_j_id;
    }  // end loop over neighbors

    // get only the top view scores from the map
    std::map<double, ViewId, std::greater<double>> best_view_scores;

    int num_neighbors_max = num_neighbors;
    if (all_view_scores.size() < num_neighbors) {
      num_neighbors_max = all_view_scores.size();
    }
    for (int i = 0; i < num_neighbors_max; ++i) {
      best_view_scores[all_view_scores.begin()->first] =
          all_view_scores.begin()->second;
      all_view_scores.erase(all_view_scores.begin());
    }

    view_selection[view_i_id] = best_view_scores;
  }  // end loop over views

  return view_selection;
}

}  // namespace theia