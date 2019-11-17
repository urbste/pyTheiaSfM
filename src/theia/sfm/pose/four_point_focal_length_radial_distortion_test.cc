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
// Please contact the author of this file if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// This file was created by Steffen Urban (urbste@googlemail.com) or
// company address (steffen.urban@zeiss.com)
// December 2018

#include <math.h>
#include <Eigen/Core>
#include <random>
#include "gtest/gtest.h"

#include "theia/sfm/pose/four_point_focal_length_radial_distortion.h"
#include "theia/sfm/pose/test_util.h"
#include "theia/test/test_utils.h"
#include "theia/util/random.h"

namespace theia {

namespace {

const int min_nr_points = 4;
const double image_width = 3000;

using Eigen::Array;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;

RandomNumberGenerator rng(64);

void DistortPoint(const Eigen::Vector2d& point2d, const double& distortion,
                  Eigen::Vector2d* distorted_point) {
  const double r_u_sq = point2d[0] * point2d[0] + point2d[1] * point2d[1];

  const double denom = 2.0 * distortion * r_u_sq;
  const double inner_sqrt = 1.0 - 4.0 * distortion * r_u_sq;

  // If the denominator is nearly zero then we can evaluate the distorted
  // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
  if (std::abs(denom) < 1e-15 || inner_sqrt < 0.0) {
    (*distorted_point)[0] = point2d[0];
    (*distorted_point)[1] = point2d[1];
  } else {
    const double scale = (1.0 - std::sqrt(inner_sqrt)) / denom;
    (*distorted_point)[0] = point2d[0] * scale;
    (*distorted_point)[1] = point2d[1] * scale;
  }
}

void UndistortPoint(const Eigen::Vector2d& distorted_point,
                    const double& radial_distortion,
                    Eigen::Vector2d* undistorted_point) {
  const double r_d_sq = distorted_point[0] * distorted_point[0] +
                        distorted_point[1] * distorted_point[1];

  const double undistortion = 1.0 / (1.0 + radial_distortion * r_d_sq);
  (*undistorted_point)[0] = distorted_point[0] * undistortion;
  (*undistorted_point)[1] = distorted_point[1] * undistortion;
}

void P4pfrTestWithNoise(const Matrix3d& gt_rotation,
                        const Vector3d& gt_translation,
                        const double focal_length,
                        const double radial_distortion,
                        const std::vector<Vector3d>& world_points_vector,
                        const double noise) {
  // Camera intrinsics matrix.
  const Matrix3d camera_matrix =
      Eigen::DiagonalMatrix<double, 3>(focal_length, focal_length, 1.0);
  Matrix<double, 3, 4> gt_transformation, gt_projection;
  gt_transformation << gt_rotation, gt_translation;
  gt_projection = camera_matrix * gt_transformation;

  // Reproject 3D points to get undistorted image points.
  std::vector<Eigen::Vector2d> distorted_image_points_vector(min_nr_points);
  for (int i = 0; i < min_nr_points; ++i) {
    Eigen::Vector3d point_in_cam =
        camera_matrix * (gt_rotation * world_points_vector[i] + gt_translation);
    Eigen::Vector2d undistorted_image_point = point_in_cam.hnormalized();
    DistortPoint(undistorted_image_point, radial_distortion,
                 &distorted_image_points_vector[i]);
    if (noise) {
      AddNoiseToProjection(noise, &rng, &distorted_image_points_vector[i]);
    }
  }

  // Run the P4Pfr algorithm.
  std::vector<Matrix3d> soln_rotations;
  std::vector<Vector3d> soln_translations;
  std::vector<double> soln_focal_lenghts;
  std::vector<double> soln_radial_distortions;

  CHECK(FourPointsPoseFocalLengthRadialDistortion(
      distorted_image_points_vector, world_points_vector, 2000.0, 0.0, -1e-5,
      -1e-10, &soln_rotations, &soln_translations, &soln_radial_distortions,
      &soln_focal_lenghts));

  double smallest_error = std::numeric_limits<double>::max();
  int solution_idx = -1;
  for (int i = 0; i < soln_radial_distortions.size(); ++i) {
    double error_sum = 0.0;
    // Check the reprojection error.
    for (int n = 0; n < min_nr_points; n++) {
      Matrix3d K = Vector3d(soln_focal_lenghts[i], soln_focal_lenghts[i], 1.0)
                       .asDiagonal();
      Eigen::Vector3d point_in_cam =
          K *
          (soln_rotations[i] * world_points_vector[n] + soln_translations[i]);
      Eigen::Vector2d undistorted_image_point = point_in_cam.hnormalized();
      Eigen::Vector2d undistorted_point;
      Eigen::Vector2d distorted_point;
      //UndistortPoint(distorted_image_points_vector[n],
      //               soln_radial_distortions[i],&undistorted_point);
      DistortPoint(undistorted_image_point, soln_radial_distortions[i],
                   &distorted_point);

      //double reproj_error =
      //    (undistorted_point - undistorted_image_point).squaredNorm();
      double reproj_error =
          (distorted_point - distorted_image_points_vector[n]).squaredNorm();
      error_sum += reproj_error;
    }
    if (error_sum < smallest_error) {
      smallest_error = error_sum;
      solution_idx = i;
    }
  }

  if (solution_idx >= 0) {
    // Expect poses are near.
    EXPECT_TRUE(test::ArraysEqualUpToScale(
        9, gt_rotation.data(), soln_rotations[solution_idx].data(), 1e-1));
    // The position is more noisy than the rotation usually.
    EXPECT_TRUE(test::ArraysEqualUpToScale(
        3, gt_translation.data(), soln_translations[solution_idx].data(),
        1e-1));

    // Expect focal length is near.
    static const double kFocalLengthTolerance = 0.3;
    EXPECT_NEAR(soln_focal_lenghts[solution_idx], focal_length,
                kFocalLengthTolerance * focal_length);
    // Expect radial distortion is near.
    EXPECT_NEAR(soln_radial_distortions[solution_idx], radial_distortion,
                std::abs(kFocalLengthTolerance * radial_distortion));
  } else {
    EXPECT_NEAR(1, 2, 0);
    VLOG(1) << "P4Pfr did not find a solution.";
  }
}

void BasicTest(const double noise) {
  // focal length
  const double focal_length = 1000;
  // radial distortion
  const double radial_distortion = -1e-7;

  const double x = -0.10;  // rotation of the view around x axis
  const double y = -0.20;  // rotation of the view around y axis
  const double z = 0.30;   // rotation of the view around z axis

  // Create a ground truth pose.
  Matrix3d Rz, Ry, Rx;
  Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
  Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
  Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
  const Matrix3d gt_rotation = Rz * Ry * Rx;
  const Vector3d gt_translation = Vector3d(-0.00950692, 0.0171496, 0.0508743);

  // Create 3D world points that are viable based on the camera intrinsics and
  // extrinsics.
  std::vector<Vector3d> world_points_vector(min_nr_points);
  Map<Matrix<double, 3, min_nr_points>> world_points(
      world_points_vector[0].data());
  world_points << -0.42941, 0.000621211, -0.350949, -1.45205, 0.415794,
      -0.556605, -1.92898, -1.89976, 1.4949, 0.838307, 1.41972, 1.25756;
  P4pfrTestWithNoise(gt_rotation, gt_translation, focal_length,
                     radial_distortion, world_points_vector, noise);
}

void PlanarTestWithNoise(const double noise) {
  // focal length
  const double focal_length = 1000;
  // radial distortion
  const double radial_distortion = -1e-7;

  const double size = 2;
  const double depth = 5;

  const double x = -0.10;  // rotation of the view around x axis
  const double y = -0.20;  // rotation of the view around y axis
  const double z = 0.10;   // rotation of the view around z axis

  // Create a ground truth pose.
  Matrix3d Rz, Ry, Rx;
  Rz << cos(z), sin(z), 0, -sin(z), cos(z), 0, 0, 0, 1;
  Ry << cos(y), 0, -sin(y), 0, 1, 0, sin(y), 0, cos(y);
  Rx << 1, 0, 0, 0, cos(x), sin(x), 0, -sin(x), cos(x);
  const Matrix3d gt_rotation = Rz * Ry * Rx;
  const Vector3d gt_translation = Vector3d(1.1, 0.2, 0.0);

  // Create 3D world points that are viable based on the camera intrinsics and
  // extrinsics.
  std::vector<Vector3d> world_points_vector(min_nr_points);
  world_points_vector[0] = Vector3d(-size / 2, -size / 2, depth + 1e-10);
  world_points_vector[1] = Vector3d(size / 2, -size / 2, depth - 1e-10);
  world_points_vector[2] = Vector3d(size / 2, size / 2, depth + 1e-8);
  world_points_vector[3] = Vector3d(-size / 2, size / 2, depth - 1e-7);

  P4pfrTestWithNoise(gt_rotation, gt_translation, focal_length,
                     radial_distortion, world_points_vector, noise);
}

TEST(P4Pfr, BasicTest) { BasicTest(0.0); }

TEST(P4Pfr, BasicNoiseTest) { BasicTest(0.5); }

TEST(P4Pfr, PlanarTestNoNoise) { PlanarTestWithNoise(0.0); }

TEST(P4Pfr, PlanarTestWithNoise) { PlanarTestWithNoise(0.5); }
}  // namespace
}  // namespace theia
