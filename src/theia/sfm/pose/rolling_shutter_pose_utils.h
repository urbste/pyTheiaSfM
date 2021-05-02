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

#ifndef ROLLING_SHUTTER_POSE_UTILS_H
#define ROLLING_SHUTTER_POSE_UTILS_H

#include <Eigen/Eigen>
#include <vector>

namespace theia {

enum RSProjectionType { SingleLinearized = 0, DoubleLinearized = 1 };

enum RSDirection { RowWise = 0, ColWise = 1 };

// These structures save the result of rolling shutter absolute pose algorithms
// See https://github.com/CenekAlbl/RnP/blob/master/c%2B%2B/rnp.h
struct RSCameraPose {
  Eigen::Matrix3d R; // Camera orientation as rotation matrix
  Eigen::Vector3d C; // Camera center
  Eigen::Vector3d w; // Camera rotational velocity (rotation axis where ||w|| is
                     // the angular velocity)
  Eigen::Vector3d
      t; // Camera translational velocity
         // focal length (only R7Pf and R7Pfr compute focal length, otherwise 1)
  double f;
  // radial distortion coefficient (only R7Pfr computes it, otherwise 0)
  double rd;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

// single linearized projection model
struct RSLinearizedCameraPose {
  // Camera orientation in Cayley parameterization (single linearized case) or
  // I + [v]_x in the double linearized case
  Eigen::Vector3d v;
  Eigen::Vector3d C; // Camera center
  Eigen::Vector3d w; // Camera rotational velocity (rotation axis where ||w|| is
                     // the angular velocity)
  Eigen::Vector3d t; // Camera translational velocity
  // focal length (only R7Pf and R7Pfr compute focal length, otherwise 1)
  double f;
  // radial distortion coefficient (only R7Pfr computes it, otherwise 0)
  double rd;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

inline void SwitchInputXY(std::vector<Eigen::Vector3d> &world_points,
                          std::vector<Eigen::Vector2d> &image_points) {
  assert(world_points.size() == image_points.size());
  for (auto i = 0; i < world_points.size(); ++i) {
    world_points[i] << world_points[i][1], world_points[i][0],
        world_points[i][2];
    image_points[i] << image_points[i][1], image_points[i][0];
  }
}

template <typename T> T SwitchOutputXY(const T &in) {
  T out;
  out.v(0) = -in.v(1);
  out.v(1) = -in.v(0);
  out.v(2) = -in.v(2);
  out.w(0) = -in.w(1);
  out.w(1) = -in.w(0);
  out.w(2) = -in.w(2);
  out.C(0) = in.C(1);
  out.C(1) = in.C(0);
  out.C(2) = in.C(2);
  out.t(0) = in.t(1);
  out.t(1) = in.t(0);
  out.t(2) = in.t(2);
  out.f = in.f;
  out.rd = in.rd;

  return out;
}

} // namespace theia

#endif // ROLLING_SHUTTER_POSE_UTILS_H
