// Copyright (C) 2025 Steffen Urban
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

#ifndef THEIA_SFM_FIND_COMMON_TRACKS_BY_FEATURE_IN_RECONSTRUCTIONS_H_
#define THEIA_SFM_FIND_COMMON_TRACKS_BY_FEATURE_IN_RECONSTRUCTIONS_H_

#include <vector>
#include <unordered_map>
#include <utility>
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"

namespace theia {

// Find common tracks between two reconstructions based on feature matching
// Returns a tuple of (point_pairs, track_id_pairs) where each pair contains (reference, query)
void FindCommonTracksByFeatureInReconstructions(
    const Reconstruction& reconstruction_ref,
    const Reconstruction& reconstruction_qry,
    const std::vector<std::pair<ViewId, ViewId>>& view_graph_matches_ref_qry,
    std::vector<Eigen::Vector3d>* points_ref,
    std::vector<Eigen::Vector3d>* points_qry,
    std::vector<std::pair<TrackId, TrackId>>* track_id_pairs);

}  // namespace theia

#endif  // THEIA_SFM_FIND_COMMON_TRACKS_BY_FEATURE_IN_RECONSTRUCTIONS_H_
