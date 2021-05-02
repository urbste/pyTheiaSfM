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
#include "theia/sfm/pose/seven_point_rs_focal_length_radial_distortion.h"
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

RandomNumberGenerator rng(55);

bool EstimateRS7frPose(const RSLinearizedCameraPose gt, RSDirection rs_dir,
                       const std::vector<Vector3d> &world_points,
                       const double noise_level,
                       const double max_err_rd_f_percent) {

  std::vector<Vector2d> image_points(7);

  // create exact 2D projections
  for (int i = 0; i < 7; ++i) {
    if (!RSLinearizedProjection(world_points[i], gt,
                                RSProjectionType::DoubleLinearized, rs_dir, 0,
                                image_points[i])) {
      std::cout << "R7Pfr test failed due to projection function\n";
      return false;
    }
    if (noise_level != 0.0) {
      image_points[i] += rng.RandVector2d(-noise_level, noise_level);
    }
  }

  RSLinearizedCameraPose result;
  EXPECT_TRUE(RSPoseFocalLengthRadialDistFromSevenPoints(
      image_points, world_points, 0, rs_dir, 10, result));
  EXPECT_TRUE((gt.C - result.C).squaredNorm() < 1e-2);
  EXPECT_TRUE((gt.v - result.v).squaredNorm() < 1e-2);
  EXPECT_TRUE((gt.w - result.w).squaredNorm() < 1e-2);
  if (gt.rd != 0.0) {
    const double r_scaled = (result.f*result.f)*result.rd;
    const double r_gt_scaled = (gt.f*gt.f)*gt.rd;
    const double dist_rd = abs((r_gt_scaled - r_scaled) / r_gt_scaled) * 100;
    EXPECT_TRUE(dist_rd < max_err_rd_f_percent);
  }
  double dist_f = abs((gt.f - result.f) / gt.f) * 100;

  EXPECT_TRUE(dist_f < max_err_rd_f_percent);
  return true;
}

TEST(SevenPointRSfrTest, TestNoNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 0.0;
  const double max_err_percent_rd_f = 1.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.01 * Eigen::Vector3d(1, 0, 0);

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
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

TEST(SevenPointRSfrTest, TestNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 0.5;
  const double max_err_percent_rd_f = 10.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.01 * Eigen::Vector3d(1, 0, 0);

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
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

TEST(SevenPointRSfrTest, TestLargeNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 2.0;
  const double max_err_percent_rd_f = 20.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.1 * Eigen::Vector3d(1, 0, 0);

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
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

TEST(SevenPointRSfrTest, TestRadNoNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 0.0;
  const double max_err_percent_rd_f = 1.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.01 * Eigen::Vector3d(1, 0, 0);

  const Vector3d rot_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;
  const Vector3d trans_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;

  const RSDirection rs_dir = RSDirection::RowWise;

  //  test with random reasonable values

  RSLinearizedCameraPose gt;
  gt.v = v_gt;
  gt.C = t_gt;
  gt.w = rot_vel_gt;
  gt.t = trans_vel_gt;
  gt.rd = -0.25 / focal_length / focal_length;
  gt.f = focal_length;
  EXPECT_TRUE(
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

TEST(SevenPointRSfrTest, TestRadNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 0.5;
  const double max_err_percent_rd_f = 10.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 5);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.01 * Eigen::Vector3d(1, 0, 0);

  const Vector3d rot_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;
  const Vector3d trans_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;

  const RSDirection rs_dir = RSDirection::RowWise;

  //  test with random reasonable values

  RSLinearizedCameraPose gt;
  gt.v = v_gt;
  gt.C = t_gt;
  gt.w = rot_vel_gt;
  gt.t = trans_vel_gt;
  gt.rd = -0.25 / focal_length / focal_length;
  gt.f = focal_length;
  EXPECT_TRUE(
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

TEST(SevenPointRSfrTest, TestRadLargeNoise) {
  std::vector<Vector3d> world_points(7);

  const double noise = 2.0;
  const double max_err_percent_rd_f = 30.0;
  // create random 3D points
  for (auto i = 0; i < 7; ++i) {
    world_points[i] = rng.RandVector3d(-1, 1) + Eigen::Vector3d(0, 0, 10);
  }

  const double focal_length = 1000.0;
  const Vector3d t_gt = rng.RandVector3d(-1, 1);
  const Vector3d v_gt = 0.01 * Eigen::Vector3d(1, 0, 0);

  const Vector3d rot_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;
  const Vector3d trans_vel_gt = rng.RandVector3d(-1, 1) / focal_length / 10.0;

  const RSDirection rs_dir = RSDirection::RowWise;

  //  test with random reasonable values

  RSLinearizedCameraPose gt;
  gt.v = v_gt;
  gt.C = t_gt;
  gt.w = rot_vel_gt;
  gt.t = trans_vel_gt;
  gt.rd = -0.25 / focal_length / focal_length;
  gt.f = focal_length;
  EXPECT_TRUE(
      EstimateRS7frPose(gt, rs_dir, world_points, noise, max_err_percent_rd_f));
}

} // namespace theia
