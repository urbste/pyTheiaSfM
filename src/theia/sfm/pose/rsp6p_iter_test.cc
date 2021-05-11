// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/util.h"
#include "theia/sfm/pose/rolling_shutter_projection_utils.h"
#include "theia/sfm/pose/rsp6p_iter.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/types.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;

RandomNumberGenerator rng(55);

const int NR_PTS = 6;

bool EstimateRSP6PIterPose(const RSLinearizedCameraPose gt, RSDirection rs_dir,
                       const std::vector<Vector3d> &world_points,
                       const double focal_length,
                       const double noise_level) {

  std::vector<Vector2d> image_points(NR_PTS);
  Matrix3d K = Matrix3d::Zero();
  K.diagonal() << focal_length, focal_length, 1.0;
  // create exact 2D projections
  for (int i = 0; i < NR_PTS; ++i) {
    if (!RSLinearizedProjection(world_points[i], gt,
                                RSProjectionType::DoubleLinearized, rs_dir, 0,
                                image_points[i])) {
      std::cout << "R7Pfr test failed due to projection function\n";
      return false;
    }
    if (noise_level) {
      AddNoiseToProjection(noise_level, &rng, &image_points[i]);
    }
  }

  RSLinearizedCameraPose result;
  EXPECT_TRUE(RSP6PIter(
      image_points,  world_points, 0, rs_dir, 5, result));
  EXPECT_TRUE((gt.C - result.C).squaredNorm() < 1e-2);
  EXPECT_TRUE((gt.v - result.v).squaredNorm() < 1e-3);
  EXPECT_TRUE((gt.t - result.t).squaredNorm() < 1e-3);
  EXPECT_TRUE((gt.w - result.w).squaredNorm() < 1e-3);

  return true;
}



TEST(RSP6PIter, TestNoNoise) {
  std::vector<Vector3d> world_points(NR_PTS);

  const double noise = 0.0;
  // create random 3D points
  for (auto i = 0; i < NR_PTS; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.001 * Eigen::Vector3d(1, 0, 0);

  const Vector3d rot_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;
  const Vector3d trans_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;

  const RSDirection rs_dir = RSDirection::RowWise;

  //  test with random reasonable values

  RSLinearizedCameraPose gt;
  gt.v = v_gt;
  gt.C = t_gt;
  gt.w = rot_vel_gt;
  gt.t = trans_vel_gt;
  gt.rd = 0.0;
  gt.f = focal_length;
  EXPECT_TRUE(
      EstimateRSP6PIterPose(gt, rs_dir, world_points, focal_length, noise));
}

} // namespace theia
