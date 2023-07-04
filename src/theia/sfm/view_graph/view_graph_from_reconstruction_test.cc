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

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/sfm/view_graph/view_graph_from_reconstruction.h"
#include "theia/util/hash.h"
#include "theia/util/map_util.h"
#include "theia/util/random.h"

namespace theia {

// This is needed for EXPECT_EQ(TwoViewInfo, TwoViewInfo);
bool operator==(const TwoViewInfo& lhs, const TwoViewInfo& rhs) {
  return lhs.position_2 == rhs.position_2 && lhs.rotation_2 == rhs.rotation_2 &&
         lhs.num_verified_matches == rhs.num_verified_matches;
}

TEST(ViewGraph, Constructor) {
  ViewGraph view_graph;

  Reconstruction reconstruction;
  theia::ReadReconstruction("/home/steffen/Dokumente/Muehltal/MilowsClaw/run1_recs_s/theia_recon_0.recon", &reconstruction);


  ViewGraphFromReconstruction(reconstruction, 5, &view_graph);

  for (const auto vid : reconstruction.ViewIds()) {
    const std::unordered_set<ViewId>* neighbors = view_graph.GetNeighborIdsForView(vid);

    std::cout<<"View "<<vid<<" has " << neighbors->size() <<" neighbors.\n";
  }

}


}  // namespace theia
