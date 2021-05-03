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
// Author: Steffen Urban (urbste@googlemail.com)

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/util.h"
#include "theia/sfm/pose/rolling_shutter_projection_utils.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/sfm/types.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(55);

TEST(RSProjections, SingleLinProjection) {
  double row_col_0 = 0;

  RSLinearizedCameraPose pose;
  pose.v << 0, 0, 0;
  pose.w << 0, 0, 0;
  pose.C << 0, 0, 0;
  pose.t << 0, 0, 0;
  pose.f = 0.0;
  pose.rd = 0.0;

  Eigen::Vector3d world_point;
  world_point << 0, 0, 0;

  Eigen::Vector2d image_point;
  bool res = false;
  // test all zero input, should return NaN
//  res = RSLinearizedProjection(world_point, pose,
//                               RSProjectionType::SingleLinearized,
//                               RSDirection::RowWise, row_col_0, image_point);
//  EXPECT_TRUE(res);

//  // now test with focal length 1 and non-zero X, should return a number
//  pose.f = 1;
//  world_point << 1, 1, 1;
//  res = RSLinearizedProjection(world_point, pose,
//                               RSProjectionType::SingleLinearized,
//                               RSDirection::RowWise, row_col_0, image_point);
//  EXPECT_TRUE(res);

//  // now test with random X and C
//  for (int i = 0; i < 10; i++) {
//    world_point = rng.RandVector3d(-1.0, 1.0);
//    pose.C = rng.RandVector3d(-1.0, 1.0);
//    res = RSLinearizedProjection(world_point, pose,
//                                 RSProjectionType::SingleLinearized,
//                                 RSDirection::RowWise, row_col_0, image_point);
//    EXPECT_TRUE(res);
//  }

//  // now test with random  v
//  for (int i = 0; i < 10; i++) {
//    world_point << 0, 0, 2;
//    pose.C << 0, 0, 0;
//    pose.v = rng.RandVector3d(-1.0, 1.0);
//    res = RSLinearizedProjection(world_point, pose,
//                                 RSProjectionType::SingleLinearized,
//                                 RSDirection::RowWise, row_col_0, image_point);
//    EXPECT_TRUE(res);
//  }

//  // now test with random w
//  for (int i = 0; i < 10; i++) {
//    world_point << 0, 0, 2;
//    pose.C << 0, 0, 0;
//    pose.v << 0, 0, 0;
//    pose.w = rng.RandVector3d(-1.0, 1.0);
//    res = RSLinearizedProjection(world_point, pose,
//                                 RSProjectionType::SingleLinearized,
//                                 RSDirection::RowWise, row_col_0, image_point);
//    EXPECT_TRUE(res);
//  }

//  // now test with random rd

//  for (int i = 0; i < 10; i++) {
//    world_point << 0, 0, 2;
//    pose.C << 0, 0, 0;
//    pose.v << 0, 0, 0;
//    pose.w << 0, 0, 0;
//    pose.rd = rng.RandDouble(-0.5, 0.0);
//    res = RSLinearizedProjection(world_point, pose,
//                                 RSProjectionType::SingleLinearized,
//                                 RSDirection::RowWise, row_col_0, image_point);
//    EXPECT_TRUE(res);
//  }

//  // now test reasonable values of X,C,t,v,w,f,rd
//  for (int i = 0; i < 10; i++) {
//    pose.f = rng.RandDouble(1000.0, 1500.0);
//    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
//    pose.C << rng.RandVector3d(-1.0, 1.0);
//    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
//    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
//    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
//    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;
//    res = RSLinearizedProjection(world_point, pose,
//                                 RSProjectionType::SingleLinearized,
//                                 RSDirection::RowWise, row_col_0, image_point);
//    EXPECT_TRUE(res);
//  }

  // now verify that the solution is accurate
  for (int i = 0; i < 10; i++) {
    pose.f = rng.RandDouble(1000.0, 1500.0);
    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
    pose.C << rng.RandVector3d(-1.0, 1.0);
    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;

//    pose.f = 1000.0;//f_gen(random_engine);
//    world_point << Eigen::Vector3d(0.1,0.1,0.1) + Eigen::Vector3d(0,0,5);
//    pose.C  << Eigen::Vector3d(0.1,-0.1,-0.1);
//    pose.v << Eigen::Vector3d(0.1,0.0,0.0)/10;
//    pose.w << Eigen::Vector3d(0.1,0.1,0.1)/pose.f/10;
//    pose.t  << Eigen::Vector3d(0.1,0.1,0.1)/pose.f/10;
//    pose.rd = -0.25/pose.f/pose.f; //rd_gen(random_engine)/f/f;

    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::SingleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);

    std::vector<Vector3d> world_points = {world_point};
    std::vector<Vector2d> image_points = {image_point};
    double err = RSLinearizedProjectionError(image_points, world_points, pose,
                                             RSProjectionType::SingleLinearized,
                                             RSDirection::RowWise, row_col_0);
    EXPECT_TRUE(err < 1e-8);
  }
//  // Check other rolling shutter direction
  for (int i = 0; i < 10; i++) {
    pose.f = rng.RandDouble(1000.0, 1500.0);
    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
    pose.C << rng.RandVector3d(-1.0, 1.0);
    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::SingleLinearized,
                                 RSDirection::ColWise, row_col_0, image_point);
    std::vector<Vector3d> world_points = {world_point};
    std::vector<Vector2d> image_points = {image_point};
    double err = RSLinearizedProjectionError(image_points, world_points, pose,
                                             RSProjectionType::SingleLinearized,
                                             RSDirection::ColWise, row_col_0);
    EXPECT_TRUE(err < 1e-8);
  }
}

TEST(RollingShutterProjections, DoubleLinProjection) {
  double row_col_0 = 0;

  RSLinearizedCameraPose pose;
  pose.v << 0, 0, 0;
  pose.w << 0, 0, 0;
  pose.C << 0, 0, 0;
  pose.t << 0, 0, 0;
  pose.f = 0.0;
  pose.rd = 0.0;

  Eigen::Vector3d world_point;
  world_point << 0, 0, 0;

  Eigen::Vector2d image_point;
  bool res = false;
  // test all zero input, should return NaN
  res = RSLinearizedProjection(world_point, pose,
                               RSProjectionType::DoubleLinearized,
                               RSDirection::RowWise, row_col_0, image_point);
  EXPECT_TRUE(res);

  // now test with focal length 1 and non-zero X, should return a number
  pose.f = 1;
  world_point << 1, 1, 1;
  res = RSLinearizedProjection(world_point, pose,
                               RSProjectionType::DoubleLinearized,
                               RSDirection::RowWise, row_col_0, image_point);
  EXPECT_TRUE(res);

  // now test with random X and C
  for (int i = 0; i < 10; i++) {
    world_point = rng.RandVector3d(-1.0, 1.0);
    pose.C = rng.RandVector3d(-1.0, 1.0);
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);
    EXPECT_TRUE(res);
  }

  // now test with random  v
  for (int i = 0; i < 10; i++) {
    world_point << 0, 0, 2;
    pose.C << 0, 0, 0;
    pose.v = rng.RandVector3d(-1.0, 1.0);
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);
    EXPECT_TRUE(res);
  }

  // now test with random w
  for (int i = 0; i < 10; i++) {
    world_point << 0, 0, 2;
    pose.C << 0, 0, 0;
    pose.v << 0, 0, 0;
    pose.w = rng.RandVector3d(-1.0, 1.0);
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);
    EXPECT_TRUE(res);
  }

  // now test with random rd

  for (int i = 0; i < 10; i++) {
    world_point << 0, 0, 2;
    pose.C << 0, 0, 0;
    pose.v << 0, 0, 0;
    pose.w << 0, 0, 0;
    pose.rd = rng.RandDouble(-0.5, 0.0);
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);
    EXPECT_TRUE(res);
  }

  // now test reasonable values of X,C,t,v,w,f,rd
  for (int i = 0; i < 10; i++) {
    pose.f = rng.RandDouble(1000.0, 1500.0);
    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
    pose.C << rng.RandVector3d(-1.0, 1.0);
    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);
    EXPECT_TRUE(res);
  }

  // now verify that the solution is accurate
  for (int i = 0; i < 10; i++) {
    pose.f = rng.RandDouble(1000.0, 1500.0);
    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
    pose.C << rng.RandVector3d(-1.0, 1.0);
    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::RowWise, row_col_0, image_point);

    std::vector<Vector3d> world_points = {world_point};
    std::vector<Vector2d> image_points = {image_point};
    double err = RSLinearizedProjectionError(image_points, world_points, pose,
                                             RSProjectionType::DoubleLinearized,
                                             RSDirection::RowWise, row_col_0);
    EXPECT_TRUE(err < 1e-8);
  }
  // Check other rolling shutter direction
  for (int i = 0; i < 10; i++) {
    pose.f = rng.RandDouble(1000.0, 1500.0);
    world_point << rng.RandVector3d(-1., 1.) + Eigen::Vector3d(0, 0, 5);
    pose.C << rng.RandVector3d(-1.0, 1.0);
    pose.v << rng.RandVector3d(-1.0, 1.0) / 10;
    pose.w << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.t << rng.RandVector3d(-1.0, 1.0) / pose.f / 10;
    pose.rd = rng.RandDouble(-0.5, 0.) / pose.f / pose.f;
    res = RSLinearizedProjection(world_point, pose,
                                 RSProjectionType::DoubleLinearized,
                                 RSDirection::ColWise, row_col_0, image_point);
    std::vector<Vector3d> world_points = {world_point};
    std::vector<Vector2d> image_points = {image_point};
    double err = RSLinearizedProjectionError(image_points, world_points, pose,
                                             RSProjectionType::DoubleLinearized,
                                             RSDirection::ColWise, row_col_0);
    EXPECT_TRUE(err < 1e-8);
  }
}
} // namespace theia
