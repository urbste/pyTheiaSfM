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

// Code taken from Cenek Albl: https://github.com/CenekAlbl/RnP

#include "six_point_rs_absolute_pose_iter.h"
#include "rolling_shutter_projection_utils.h"

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <iostream>

namespace theia {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

bool RSPoseFocalLengthRadialDistFromSevenPointsImpl(
    const std::vector<Eigen::Vector2d> &image_points,
    const std::vector<Eigen::Vector3d> &world_points, const double start_row,
    const Eigen::Vector3d &initial_rotational_velocity,
    std::vector<RSLinearizedCameraPose> *results) {

  Eigen::Matrix<double, 6, 3> X;
  Eigen::Matrix<double, 6, 2> u;
  for (int i = 0; i < 6; ++i) {
    X.row(i) = world_points[i];
    u.row(i) = image_points[i];
  }
    Eigen::Matrix<double,12,13> A = Eigen::Matrix<double,12,13>::Zero();
    A.col(0).head(6) << X.row(2).transpose().array() + X.row(1).transpose().array() * u.row(1).transpose().array();  
    A.col(0).tail(6) << -X.row(1).transpose().array() * u.row(0).transpose().array();
    A.col(1).head(6) << -X.row(0).transpose().array() * u.row(1).transpose().array();
    A.col(1).tail(6) << X.row(2).transpose().array() + X.row(0).transpose().array() * u.row(0).transpose().array();
    A.col(2).head(6) << - X.row(0).transpose().array();
    A.col(2).tail(6) << - X.row(1).transpose().array();
    A.col(3).head(6) << X.row(0).transpose().array() * vk(1) * (-u.row(0).transpose().array()) - X.row(2).transpose().array() * (- u.row(0).transpose().array()) - u.row(1).transpose().array() * (X.row(1).transpose().array() * ( -u.row(0).transpose().array()) + X.row(0).transpose().array() * vk(2) * (-u.row(0).transpose().array()) - X.row(2).transpose().array() * vk(0) * (-u.row(0).transpose().array())) - X.row(1).transpose().array() * vk(0) * (-u.row(0).transpose().array());
    A.col(3).tail(6) << u.row(0).transpose().array() * (X.row(1).transpose().array() * (-u.row(0).transpose().array()) + X.row(0).transpose().array() * vk(2) * (-u.row(0).transpose().array()) - X.row(2).transpose().array() * vk(0) * (-u.row(0).transpose().array()));
    A.col(4).head(6) << u.row(1).transpose().array() * (X.row(0).transpose().array() * (-u.row(0).transpose().array()) - X.row(1).transpose().array() * vk(2) * (-u.row(0).transpose().array()) + X.row(2).transpose().array() * vk(1) * (-u.row(0).transpose().array()));
    A.col(4).tail(6) << X.row(0).transpose().array() * vk(1) * (-u.row(0).transpose().array()) - X.row(2).transpose().array() * (-u.row(0).transpose().array()) - u.row(0).transpose().array() * (X.row(0).transpose().array() * (-u.row(0).transpose().array()) -X.row(1).transpose().array() * vk(2) * (-u.row(0).transpose().array()) + X.row(2).transpose().array() * vk(1) * (-u.row(0).transpose().array())) - X.row(1).transpose().array() * vk(0) * (-u.row(0).transpose().array());
    A.col(5).head(6) << X.row(0).transpose().array() * (-u.row(0).transpose().array()) - X.row(1).transpose().array() * vk(2) * (-u.row(0).transpose().array()) + X.row(2).transpose().array() * vk(1) * (-u.row(0).transpose().array());
    A.col(5).tail(6) << X.row(1).transpose().array() * (-u.row(0).transpose().array()) + X.row(0).transpose().array() * vk(2) * (-u.row(0).transpose().array()) - X.row(2).transpose().array() * vk(0) * (-u.row(0).transpose().array());
    A.col(6).tail(6) << Eigen::MatrixXd::Ones(6,1);
    A.col(7).head(6) << -Eigen::MatrixXd::Ones(6,1);
    A.col(8).head(6) << u.row(1).transpose().array();
    A.col(8).tail(6) << -u.row(0).transpose().array();
    A.col(9).tail(6) << u.row(0).transpose().array();
    A.col(10).head(6) << -u.row(0).transpose().array();
    A.col(11).head(6) << -u.row(1).transpose().array() * (-u.row(0).transpose().array());
    A.col(11).tail(6) << u.row(0).transpose().array() * (-u.row(0).transpose().array());
    A.col(12).head(6) << X.row(2).transpose().array() * u.row(1).transpose().array() - X.row(1).transpose().array();
    A.col(12).tail(6) << X.row(0).transpose().array() - X.row(2).transpose().array() * u.row(0).transpose().array();
    Eigen::MatrixXd n = A.fullPivLu().kernel();
    // std::cout << "A inverse done \n";
    int end = n.cols()-1;
    double s = n(12,end);
    // std::cout << "scalar done \n";
    Eigen::Vector3d v = n.col(end).head(3)/s; 
    // std::cout << "v: \n" << v << "\n";
    Eigen::Vector3d w = n.col(end).segment(3,3)/s;
    // std::cout << "w done \n" << w << "\n";
    Eigen::Vector3d C = n.col(end).segment(6,3)/s;
    // std::cout << "C done \n" << C << "\n";
    Eigen::Vector3d t = n.col(end).segment(9,3)/s;
    // std::cout << "t done \n" << t << "\n";

    results->push_back({v, C, w, t, 1, 0});
}

bool RSPoseFocalLengthRadialDistFromSevenPoints(
    const std::vector<Vector2d> &image_points,
    const std::vector<Vector3d> &world_points, const int row_col_0,
    const RSDirection rs_direction, const int max_iter,
    RSLinearizedCameraPose &result) {

  std::vector<Vector3d> world_points_ = world_points;
  std::vector<Vector2d> image_points_ = image_points;

  // flip x and y if column wise RS
  if (rs_direction == RSDirection::ColWise) {
    SwitchInputXY(world_points_, image_points_);
  }

  result.v = Vector3d::Zero();
  result.C = Vector3d::Zero();
  result.w = Vector3d::Zero();
  result.t = Vector3d::Zero();
  result.f = 1.0;
  result.rd = 0.0;

  int k = 0;
  bool found = false;
  // iterate solver to find a solution
  while (!found && k < max_iter) {
    std::vector<RSLinearizedCameraPose> results;
    double err_prev = std::numeric_limits<double>::max();
    RSPoseFocalLengthRadialDistFromSevenPointsImpl(
        image_points_, world_points_, row_col_0, result.v, &results);
    // if the inner solver returned no solution
    if (!results.size()) {
      std::cout << "RS7pfr solver returned no solution\n";
      return false;
    }

    for (auto const &res : results) {
      double error = RSLinearizedProjectionError(
          image_points_, world_points_, res, RSProjectionType::DoubleLinearized,
          rs_direction, row_col_0);
      if (error < err_prev) {
        result = res;
        err_prev = error;
      }
      if (error < 1e-10) {
        found = true;
        break;
      }
    }
    ++k;
  }

  if (rs_direction == RSDirection::ColWise) {
    result = SwitchOutputXY(result);
  }

  return true;
}
} // namespace theia
