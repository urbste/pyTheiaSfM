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

#include "mlrpnp.h"
#include "rolling_shutter_projection_utils.h"

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <iostream>

namespace theia {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void MLRPnPImpl(const std::vector<Eigen::Vector2d> &image_points,
                const std::vector<Eigen::Matrix2d> &reduced_covariance,
                const std::vector<Eigen::Vector3d> &world_points,
                const std::vector<Eigen::Matrix<double, 3, 2>> &nullspaces,
                const double start_row, const Eigen::Vector3d &init_v,
                const RSDirection rs_direction,
                RSLinearizedCameraPose *result) {

  const auto nr_pts = image_points.size();
  Eigen::MatrixXd A;
  A.setZero();
  A.resize(2 * nr_pts, 13);
  Eigen::MatrixXd Kll;
  Kll.setZero();
  Kll.resize(2*nr_pts, 2*nr_pts);

  for (auto i = 0; i < nr_pts; ++i) {
    const auto u = image_points[i][rs_direction] - start_row;
    const int i2 = 2 * (i + 1);
    const int a1 = i2 - 2;
    const int a2 = i2 - 1;

    Kll.block<2,2>(a1, a1) = reduced_covariance[i];

    const Vector3d r = nullspaces[i].col(0);
    const Vector3d s = nullspaces[i].col(1);
    const Vector3d X = world_points[i];

    const double r1X2 = r(1) * X(2);
    const double s1X2 = s(1) * X(2);
    const double r2X1 = r(2) * X(1);
    const double s2X1 = s(2) * X(1);
    const double r0X2 = r(0) * X(2);
    const double s0X2 = s(0) * X(2);
    const double r2X0 = r(2) * X(0);
    const double s2X0 = s(2) * X(0);
    const double r0X1 = r(0) * X(1);
    const double s0X1 = s(0) * X(1);
    const double r1X0 = r(1) * X(0);
    const double s1X0 = s(1) * X(0);

    const double r0X0 = r(0) * X(0);
    const double s0X0 = s(0) * X(0);
    const double r1X1 = r(1) * X(1);
    const double s1X1 = s(1) * X(1);
    const double r2X2 = r(2) * X(2);
    const double s2X2 = s(2) * X(2);

    // v1     -r[1,i]*X[i,2] + r[2,i]*X[i,1]
    A(a1, 0) = -r1X2 + r2X1;
    A(a2, 0) = -s1X2 + s2X1;
    // v2     r[0,i]*X[i,2] - r[2,i]*X[i,0]
    A(a1, 1) = r0X2 - r2X0;
    A(a2, 1) = s0X2 - s2X0;
    // v3     -r[0,i]*X[i,1] + r[1,i]*X[i,0]
    A(a1, 2) = -r0X1 + r1X0;
    A(a2, 2) = -s0X1 + s1X0;

    // w1     -r[1,i]*u*X[i,2] + r[2,i]*u*X[i,1] + r[1,i]*u*vk[1]*X[i,0] -
    // r[1,i]*u*vk[0]*X[i,1] + r[2,i]*u*vk[2]*X[i,0] - r[2,i]*u*vk[0]*X[i,2]
    A(a1, 3) = u * (-r1X2 + r2X1 + r1X0 * init_v(1) - r1X1 * init_v(0) +
                    r2X0 * init_v(2) - r2X2 * init_v(0));
    A(a2, 3) = u * (-s1X2 + s2X1 + s1X0 * init_v(1) - s1X1 * init_v(0) +
                    s2X0 * init_v(2) - s2X2 * init_v(0));
    // w2     r[0,i]*u*X[i,2] - r[2,i]*u*X[i,0] - r[0,i]*u*vk[1]*X[i,0] +
    // r[0,i]*u*vk[0]*X[i,1] + r[2,i]*u*vk[2]*X[i,1] - r[2,i]*u*vk[1]*X[i,2]
    A(a1, 4) = u * (r0X2 - r2X0 - r0X0 * init_v(1) + r0X1 * init_v(0) +
                    r2X1 * init_v(2) - r2X2 * init_v(1));
    A(a2, 4) = u * (s0X2 - s2X0 - s0X0 * init_v(1) + s0X1 * init_v(0) +
                    s2X1 * init_v(2) - s2X2 * init_v(1));
    // w3     -r[0,i]*u*X[i,1] + r[1,i]*u*X[i,0] - r[0,i]*u*vk[2]*X[i,0] +
    // r[0,i]*u*vk[0]*X[i,2] - r[1,i]*u*vk[2]*X[i,1] + r[1,i]*u*vk[1]*X[i,2]
    A(a1, 5) = u * (-r0X1 + r1X0 - r0X0 * init_v(2) + r0X2 * init_v(0) -
                    r1X1 * init_v(2) + r1X2 * init_v(1));
    A(a2, 5) = u * (-s0X1 + s1X0 - s0X0 * init_v(2) + s0X2 * init_v(0) -
                    s1X1 * init_v(2) + s1X2 * init_v(1));

    // C1
    A(a1, 6) = r(0);
    A(a2, 6) = s(0);
    // C2
    A(a1, 7) = r(1);
    A(a2, 7) = s(1);
    // C3
    A(a1, 8) = r(2);
    A(a2, 8) = s(2);

    // t1
    A(a1, 9) = r(0) * u;
    A(a2, 9) = s(0) * u;
    // t2
    A(a1, 10) = r(1) * u;
    A(a2, 10) = s(1) * u;
    // t3
    A(a1, 11) = r(2) * u;
    A(a2, 11) = s(2) * u;

    // const   r[0,i]*X[i,0] + r[1,i]*X[i,1] + r[2,i]*X[i,2]
    A(a1, 12) = r0X0 + r1X1 + r2X2;
    A(a2, 12) = s0X0 + s1X1 + s2X2;
  }

  Eigen::MatrixXd n = (Kll*A).fullPivLu().kernel();
  int end = n.cols() - 1;
  double s = n(12, end);
  (*result).v = n.col(end).head(3) / s;
  (*result).w = n.col(end).segment(3, 3) / s;
  (*result).C = n.col(end).segment(6, 3) / s;
  (*result).t = n.col(end).segment(9, 3) / s;
}

bool MLRPnP(const std::vector<Eigen::Vector2d> &image_points,
            const std::vector<Eigen::Matrix3d> &bearing_vectors_covariance,
            const std::vector<Eigen::Vector3d> &bearing_vectors,
            const std::vector<Eigen::Vector3d> &world_points,
            const int row_col_0, const RSDirection rs_direction,
            const int max_iter, RSLinearizedCameraPose &result) {

  std::vector<Vector3d> world_points_ = world_points;
  std::vector<Vector2d> image_points_ = image_points;

//  // flip x and y if column wise RS
//  if (rs_direction == RSDirection::ColWise) {
//    SwitchInputXY(world_points_, image_points_);
//  }

  result.v = Vector3d::Zero();
  result.C = Vector3d::Zero();
  result.w = Vector3d::Zero();
  result.t = Vector3d::Zero();
  result.f = 1.0;
  result.rd = 0.0;

  // calculate nullspaces
  std::vector<Eigen::Matrix<double, 3, 2>> nullspaces(bearing_vectors.size());
  std::vector<Eigen::Matrix2d> covs(bearing_vectors.size());
  for (auto i = 0; i < bearing_vectors.size(); ++i) {
    nullS_3x2_templated<double>(bearing_vectors[i], nullspaces[i]);
    Matrix3d J = (Matrix3d::Identity() - bearing_vectors[i]*bearing_vectors[i].transpose());
    Matrix3d Evv = J * bearing_vectors_covariance[i] * J.transpose();
    covs[i] = (nullspaces[i].transpose() * Evv * nullspaces[i]).inverse();
  }

  int k = 0;
  bool found = false;
  // iterate solver to find a solution
  Vector3d v_new = Vector3d::Zero();
  double err_prev = std::numeric_limits<double>::max();
  while (!found && k < max_iter) {
    MLRPnPImpl(image_points_, covs, world_points_,
               nullspaces, row_col_0, v_new, rs_direction, &result);
    double error = RSLinearizedProjectionNSError(
        nullspaces, image_points, world_points, result,
        RSProjectionType::DoubleLinearized, rs_direction, row_col_0);
    if (error < err_prev) {
      v_new = result.v;
      err_prev = error;
    }
    if (error < 1e-5) {
      return true;
    }
    ++k;
  }
  return false;
//  if (rs_direction == RSDirection::ColWise) {
//    result = SwitchOutputXY(result);
//  }
}
} // namespace theia
