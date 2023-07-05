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

#ifndef THEIA_SFM_VIEW_GRAPH_VIEW_GRAPH_FROM_RECONSTRUCTION_H_
#define THEIA_SFM_VIEW_GRAPH_VIEW_GRAPH_FROM_RECONSTRUCTION_H_

#include <unordered_map>
#include <unordered_set>
#include <set>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"

namespace theia {

TwoViewInfo CreateTwoViewInfo(
    const View& view_i,
    const View& view_j,
    const size_t& num_shared_tracks);

std::set<TrackId> GetEstimatedTracks(const Reconstruction& reconstruction, const View& view);

void ViewGraphFromReconstruction(
    const theia::Reconstruction& reconstruction,
    const size_t min_shared_tracks,
    ViewGraph* view_graph);


} // namespace theia

#endif  // THEIA_SFM_VIEW_GRAPH_VIEW_GRAPH_FROM_RECONSTRUCTION_H_