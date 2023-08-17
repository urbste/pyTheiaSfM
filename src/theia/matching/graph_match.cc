// Copyright (C) 2023 Steffen Urban
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

#include "theia/matching/graph_match.h"

#include <Eigen/Core>
#include <algorithm>
#include <glog/logging.h>
#include <limits>
#include <stdint.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace theia {

struct MatchedImages {
  std::unordered_set<int> ranked_matches;
  std::unordered_set<int> expanded_matches;
};

std::vector<std::pair<std::string, std::string>> GraphMatch(
    const std::vector<std::string>& image_names,
    const std::vector<Eigen::VectorXf>& global_descriptors,
    const int num_nearest_neighbors_for_global_descriptor_matching) {
  VLOG(2) << "Computing image-to-image similarity scores with global "
             "descriptors...";

  // For each image, find the kNN and add those to our selection for matching.
  const int num_nearest_neighbors =
      std::min(static_cast<int>(image_names.size() - 1),
               num_nearest_neighbors_for_global_descriptor_matching);

  std::unordered_map<int, MatchedImages> pairs_to_match;

  // Match all pairs of global descriptors. For each image, the K most similar
  // image (i.e. the ones with the lowest distance between global descriptors)
  // are set for matching.
  std::vector<std::vector<std::pair<float, int>>> global_matching_scores(
      global_descriptors.size());
  for (int i = 0; i < global_descriptors.size(); i++) {
    // Compute the matching scores between all (i, j) pairs.
    for (int j = i + 1; j < global_descriptors.size(); j++) {
      const float global_feature_match_score =
          (global_descriptors[i] - global_descriptors[j]).squaredNorm();
      // Add the global feature matching score to both images.
      global_matching_scores[i].emplace_back(global_feature_match_score, j);
      global_matching_scores[j].emplace_back(global_feature_match_score, i);
    }

    // Find the top K matching results for image i.
    std::partial_sort(global_matching_scores[i].begin(),
                      global_matching_scores[i].begin() + num_nearest_neighbors,
                      global_matching_scores[i].end());

    // Add each of the kNN to the output indices.
    for (int j = 0; j < num_nearest_neighbors; j++) {
      const int second_id = global_matching_scores[i][j].second;

      // Perform query expansion by adding image i as a candidate match to all
      // of its matches neighbors.
      const auto& neighbors_of_second_id =
          pairs_to_match[second_id].ranked_matches;
      for (const int neighbor_of_second_id : neighbors_of_second_id) {
        pairs_to_match[neighbor_of_second_id].expanded_matches.insert(i);
      }

      // Add the match to both images so that edges are properly utilized for
      // query expansion.
      pairs_to_match[i].ranked_matches.insert(second_id);
      pairs_to_match[second_id].ranked_matches.insert(i);
    }

    // Remove the matching scores for image i to free up memory.
    global_matching_scores[i].clear();
  }

  std::vector<std::pair<std::string, std::string>> image_names_to_match;
  // Collect all matches into one container.
  image_names_to_match.reserve(num_nearest_neighbors *
                               global_descriptors.size());
  for (const auto& matches : pairs_to_match) {
    for (const int match : matches.second.ranked_matches) {
      if (matches.first < match) {
        image_names_to_match.emplace_back(image_names[matches.first],
                                          image_names[match]);
      }
    }

    for (const int match : matches.second.expanded_matches) {
      if (matches.first < match) {
        image_names_to_match.emplace_back(image_names[matches.first],
                                          image_names[match]);
      }
    }
  }

  // Uniquify the matches.
  std::sort(image_names_to_match.begin(), image_names_to_match.end());
  image_names_to_match.erase(
      std::unique(image_names_to_match.begin(), image_names_to_match.end()),
      image_names_to_match.end());
  return image_names_to_match;
}

}  // namespace theia