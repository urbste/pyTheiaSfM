// Copyright (C) 2017 The Regents of the University of California (Regents).
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

#ifndef THEIA_SFM_SET_OUTLIER_TRACKS_TO_UNESTIMATED_H_
#define THEIA_SFM_SET_OUTLIER_TRACKS_TO_UNESTIMATED_H_

#include "theia/sfm/types.h"
#include <unordered_set>

namespace theia {
class Reconstruction;

// Removes features that have a reprojection error larger than the
// reprojection error threshold. Additionally, any features that are poorly
// constrained because of a small viewing angle are removed. Returns the number
// of features removed. Only the input tracks are checked.
int SetOutlierTracksToUnestimated(const std::unordered_set<TrackId>& tracks,
                                  const double max_inlier_reprojection_error,
                                  const double min_triangulation_angle_degrees,
                                  Reconstruction* reconstruction);
// Same as above, but checks all tracks.
int SetOutlierTracksToUnestimated(const double max_inlier_reprojection_error,
                                  const double min_triangulation_angle_degrees,
                                  Reconstruction* reconstruction);

}  // namespace theia

#endif  // THEIA_SFM_SET_OUTLIER_TRACKS_TO_UNESTIMATED_H_
