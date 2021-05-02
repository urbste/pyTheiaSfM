// Copyright (C) 2021 Steffen Urban
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
//     * Neither the name of The Regents or University of California, Google,
//       nor the names of its contributors may be used to endorse or promote
//       products derived from this software without specific prior written
//       permission.
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

#ifndef SEVEN_POINT_RS_FOCAL_LENGTH_RADIAL_DISTORTION_H
#define SEVEN_POINT_RS_FOCAL_LENGTH_RADIAL_DISTORTION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "rolling_shutter_pose_utils.h"

namespace theia {

bool RSPoseFocalLengthRadialDistFromSevenPointsImpl(
    const std::vector<Eigen::Vector2d> &image_points,
    const std::vector<Eigen::Vector3d> &world_points,
    const double start_row,
    const Eigen::Vector3d &initial_rotational_velocity,
    std::vector<RSLinearizedCameraPose> *results);

// Description

// Input

// Output

bool RSPoseFocalLengthRadialDistFromSevenPoints(
    const std::vector<Eigen::Vector2d> &image_points,
    const std::vector<Eigen::Vector3d> &world_points, const int row_col_0,
    const RSDirection rs_direction, const int max_iter,
    RSLinearizedCameraPose &result);

} // namespace theia

#endif // SEVEN_POINT_RS_FOCAL_LENGTH_RADIAL_DISTORTION_H
