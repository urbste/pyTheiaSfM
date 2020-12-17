// Copyright (C) 2019 The Regents of the University of California (Regents).
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

// This file was created by Steffen Urban (urbste@googlemail.com) October 2019

#ifndef THEIA_SFM_ESTIMATORS_ESTIMATE_RADIAL_DIST_UNCALIBRATED_ABSOLUTE_POSE_H_
#define THEIA_SFM_ESTIMATORS_ESTIMATE_RADIAL_DIST_UNCALIBRATED_ABSOLUTE_POSE_H_

#include <Eigen/Core>
#include <vector>

#include "theia/sfm/create_and_initialize_ransac_variant.h"

namespace theia {
struct FeatureCorrespondence2D3D;
struct RansacParameters;
struct RansacSummary;

struct RadialDistUncalibratedAbsolutePose {
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double focal_length;
  double radial_distortion;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

// use this to sort out solutions that do not make sense
struct RadialDistUncalibratedAbsolutePoseMetaData {
  double max_focal_length = 10000;
  double min_focal_length = 200;
  double min_radial_distortion = -1e-9;
  double max_radial_distortion = -1e-5;
};

// Estimates the uncalibrated absolute pose using the ransac variant of choice
// (e.g. Ransac, Prosac, etc.). Apart from the focal length a radial distortion
// is estimated that can be used with the division undistortion camera model
// Returns true if a pose could be succesfully estimated, and false
// otherwise. The quality of the result depends on the quality of the input
// data. The feature correspondences should be normalized such that
// the principal point is at (0, 0).
bool EstimateRadialDistUncalibratedAbsolutePose(
    const RansacParameters& ransac_params, const RansacType& ransac_type,
    const std::vector<FeatureCorrespondence2D3D>& normalized_correspondences,
    const RadialDistUncalibratedAbsolutePoseMetaData& meta_data,
    RadialDistUncalibratedAbsolutePose* absolute_pose,
    RansacSummary* ransac_summary);

}  // namespace theia

#endif  // THEIA_SFM_ESTIMATORS_ESTIMATE_UNCALIBRATED_ABSOLUTE_POSE_H_
