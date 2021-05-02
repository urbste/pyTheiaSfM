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

#ifndef ROLLING_SHUTTER_PROJECTION_UTILS_H
#define ROLLING_SHUTTER_PROJECTION_UTILS_H

#include <Eigen/Eigen>

#include "rolling_shutter_pose_utils.h"

namespace theia {

// Performs (single or double) linearized rolling shutter projection
// First a global shutter projection is performed
// x = K * (Rv * X + C)
// This gives us information about the current row (or column) of the image
// point x. Given a start row we can calculate the interpolation magnitude as:
// r = x(rs_direction) - row_col_0
// Subsequently the rolling shutter projection is as follows:
// x = K * ( (I + [r*rot_vel]) * Rv * X + C + r*trans_vel).
// The rs_direction indicates if the rolling shutter runs horizotally or
// vertically. In the case of single linearized projection the rotation Rv is
// parametrized using the Cayley rotation parametrization and can be converted
// to a rotation matrix In the case of double linearized projection you will
// first need to transform the world_points using an approximation of the
// rotation matrix R_gs (using global shutter PnP): X' = R_gs*X. The rotation Rv
// = (I + [init_rotation]_x), i.e. we essentially initialize Rv aroung R_gs.
bool RSLinearizedProjection(const Eigen::Vector3d &world_point,
                            const RSLinearizedCameraPose &rs_camera_pose,
                            const RSProjectionType &proj_type,
                            const RSDirection &rs_direction,
                            const int row_col_0, Eigen::Vector2d &image_point);

// For testing purposes. Calculated the projection error and hence can be used
// to evaluate rolling shutter projection.
double
RSLinearizedProjectionError(const std::vector<Eigen::Vector2d> &image_points,
                            const std::vector<Eigen::Vector3d> &world_points,
                            const RSLinearizedCameraPose &rs_camera_pose,
                            const RSProjectionType &proj_type,
                            const RSDirection &rs_direction,
                            const int row_col_0);

} // namespace theia

#endif // ROLLING_SHUTTER_PROJECTION_UTILS_H
