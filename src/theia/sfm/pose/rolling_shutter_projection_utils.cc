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

#include "rolling_shutter_pose_utils.h"
#include "theia/util/util.h"

#include <Eigen/Geometry>

namespace theia {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vector2d = Eigen::Vector2d;
using AngleAxisd = Eigen::AngleAxisd;

bool RSLinearizedProjection(const Eigen::Vector3d &world_point,
                            const RSLinearizedCameraPose &rs_camera_pose,
                            const RSProjectionType &proj_type,
                            const RSDirection &rs_direction,
                            const int row_col_0, Eigen::Vector2d &image_point) {

  const Matrix3d I3 = Eigen::Matrix3d::Identity();
  // First initiate u with a global shutter projection
  Matrix3d K = Matrix3d::Zero();
  K.diagonal() << rs_camera_pose.f, rs_camera_pose.f, 1;
  Matrix3d Rv = I3;
  // how we construct the rotation depends on the linearization case
  if (proj_type == RSProjectionType::SingleLinearized) {
    double norm = rs_camera_pose.v.norm();
    if (norm > 1e-15) {
      Rv = AngleAxisd(norm, rs_camera_pose.v / norm).toRotationMatrix();
    }
  } else if (proj_type == RSProjectionType::DoubleLinearized) {
    Rv += GetSkew(rs_camera_pose.v);
  }

  Eigen::Vector3d image_point_global_shutter =
      K * (Rv * world_point + rs_camera_pose.C);
  image_point = image_point_global_shutter.hnormalized();

  const double d = image_point(rs_direction) - row_col_0;
  double diff = 1e15;
  int niter = 0;
  Vector3d temp_rs_point;
  while (diff > 1e-10) {
    const Matrix3d R_rs = (I3 + d * GetSkew(rs_camera_pose.w)) * Rv;
    const Vector3d t_rs = rs_camera_pose.C + d * rs_camera_pose.t;

    temp_rs_point = K * (R_rs * world_point + t_rs);
    temp_rs_point = temp_rs_point / temp_rs_point(2);
    if (rs_camera_pose.rd != 0.0) {
      const double rc2 =
          image_point(0) * image_point(0) + image_point(1) * image_point(1);
      temp_rs_point.head(2) *= (1 + rs_camera_pose.rd * rc2);
    }
    diff = (image_point - temp_rs_point.head(2)).norm();
    image_point = temp_rs_point.head(2);
    if (niter > 100) {
      return false;
    }
    ++niter;
  }
  return true;
}

double
RSLinearizedProjectionError(const std::vector<Eigen::Vector2d> &image_points,
                            const std::vector<Eigen::Vector3d> &world_points,

                            const RSLinearizedCameraPose &rs_camera_pose,
                            const RSProjectionType &proj_type,
                            const RSDirection &rs_direction,
                            const int row_col_0) {
  const Matrix3d I3 = Eigen::Matrix3d::Identity();
  double err = 0;
  for (size_t i = 0; i < world_points.size(); ++i) {
    double z = 1.0;
    if (rs_camera_pose.rd != 0.0) {
      z += rs_camera_pose.rd * (image_points[i](0) * image_points[i](0) +
                                image_points[i](1) * image_points[i](1));
    }
    Vector3d uh;
    uh << image_points[i], z;
    Matrix3d K = I3;
    K(2, 2) = 1 / rs_camera_pose.f;
    double d = (image_points[i](rs_direction) - row_col_0);
    Matrix3d Rv = I3;
    if (proj_type == RSProjectionType::SingleLinearized) {
      double norm = rs_camera_pose.v.norm();
      if (norm > 1e-15) {
        Rv = AngleAxisd(norm, rs_camera_pose.v / norm).toRotationMatrix();
      }
    } else if (proj_type == RSProjectionType::DoubleLinearized) {
      Rv += GetSkew(rs_camera_pose.v);
    }
    const Matrix3d R_rs = (I3 + d * GetSkew(rs_camera_pose.w)) * Rv;
    const Vector3d t_rs = rs_camera_pose.C + d * rs_camera_pose.t;
    Vector3d eq = GetSkew(uh) * (K * (R_rs * world_points[i] + t_rs));
    err += eq.cwiseAbs().sum();
  }

  return err;
}

} // namespace theia
