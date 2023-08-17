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

#ifndef THEIA_MVS_VIEW_SELECTION_MVSNET_H_
#define THEIA_MVS_VIEW_SELECTION_MVSNET_H_

#include <algorithm>
#include <map>
#include <unordered_map>
#include <vector>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/types.h"

namespace theia {

// Selects the views that are most likely to be useful for MVSNet.
// View selection is performed according to the slection criteria described in
//      Yao, Yao, et al. "Mvsnet: Depth inference for unstructured multi-view
//      stereo." Proceedings of the European conference on computer vision
//      (ECCV). 2018.
// A score is calculated for each view pair (i, j) based on the covisibilty and
// angles between observations of scene points
std::unordered_map<ViewId, std::map<double, ViewId, std::greater<double>>>
ViewSelectionMVSNet(const Reconstruction& reconstruction,
                    const int num_neighbors,
                    const double theta0 = 5.0,
                    const double sigma1 = 1.0,
                    const double sigma2 = 10.0);

}  // namespace theia

#endif  // THEIA_MVS_VIEW_SELECTION_MVSNET_H_
