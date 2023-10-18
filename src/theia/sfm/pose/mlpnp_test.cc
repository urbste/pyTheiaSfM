// Copyright (C) 2023 Steffen Urban
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
// Author: Steffen Urban (urbste@gmail.com)

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/util.h"
#include "theia/sfm/pose/mlpnp.h"
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

void TestMLPnPWithNoise(const std::vector<Vector3d>& world_points,
                         const double projection_noise_std_dev,
                         const Matrix3d& expected_rotation,
                         const Vector3d& expected_translation,
                         const double max_reprojection_error,
                         const double max_rotation_difference,
                         const double max_translation_difference) {
  const int num_points = world_points.size();

  Matrix3x4d expected_transform;
  expected_transform << expected_rotation,
      expected_translation;

  std::vector<Vector2d> feature_points;
  feature_points.reserve(num_points);
  for (int i = 0; i < num_points; i++) {
    // Reproject 3D points into camera frame.
    feature_points.push_back(
        (expected_transform * world_points[i].homogeneous())
            .eval()
            .hnormalized());
  }

  if (projection_noise_std_dev) {
    // Adds noise to both of the rays.
    for (int i = 0; i < num_points; i++) {
      AddNoiseToProjection(projection_noise_std_dev, &rng, &feature_points[i]);
    }
  }

  // Run DLS PnP.
  Matrix3d soln_rotation;
  Vector3d soln_translation;
  MLPnP(feature_points, {}, world_points, &soln_rotation, &soln_translation);

  // Check solutions and verify at least one is close to the actual solution.
  bool matched_transform = false;

  Matrix3x4d soln_transform;
  soln_transform << soln_rotation, soln_translation;
  std::cout<<expected_transform<<std::endl;
  std::cout<<soln_transform<<std::endl;
  for (int j = 0; j < num_points; j++) {
    const Vector2d reprojected_point =
            (soln_transform * world_points[j].homogeneous()).eval().hnormalized();
    const double reprojection_error =
            (feature_points[j] - reprojected_point).squaredNorm();
    ASSERT_LE(reprojection_error, max_reprojection_error);
  }
}

TEST(MLPnP, NoNoiseTest) {
  const std::vector<Eigen::Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0),
                                           Vector3d(-1.0, -3.0, 2.0),
                                           Vector3d(1.0, -2.0, 1.0),
                                           Vector3d(-1.0, 4.0, 2.0),
                                           Vector3d(-2.0, 2.0, 3.0)};
  const Eigen::Matrix3d soln_rotation =
      Eigen::AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 0.0 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = DegToRad(0.25);
  const double kMaxAllowedTranslationDifference = 1e-2;

  TestMLPnPWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

// TEST(MLPnP, NoNoisePlanarTest) {
//   const std::vector<Eigen::Vector3d> points_3d = {Vector3d(-1.0, 3.0, 5.0),
//                                            Vector3d(1.0, -1.0, 5.0),
//                                            Vector3d(-1.0, 1.0, 5.0),
//                                            Vector3d(2.0, 1.0, 5.0),
//                                            Vector3d(-1.0, -3.0, 5.0),
//                                            Vector3d(1.0, -2.0, 5.0),
//                                            Vector3d(-1.0, 4.0, 5.0),
//                                            Vector3d(-2.0, 2.0, 5.0)};
//   const Eigen::Matrix3d soln_rotation =
//       Eigen::AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
//   const Vector3d soln_translation(1.0, 1.0, 1.0);
//   const double kNoise = 0.0 / 512.0;
//   const double kMaxReprojectionError = 5e-3;
//   const double kMaxAllowedRotationDifference = DegToRad(0.25);
//   const double kMaxAllowedTranslationDifference = 1e-2;

//   TestMLPnPWithNoise(points_3d,
//                       kNoise,
//                       soln_rotation,
//                       soln_translation,
//                       kMaxReprojectionError,
//                       kMaxAllowedRotationDifference,
//                       kMaxAllowedTranslationDifference);
// }

TEST(MLPnP, NoiseTest) {
  const std::vector<Eigen::Vector3d> points_3d = {Vector3d(-1.0, 3.0, 3.0),
                                           Vector3d(1.0, -1.0, 2.0),
                                           Vector3d(-1.0, 1.0, 2.0),
                                           Vector3d(2.0, 1.0, 3.0),
                                           Vector3d(-1.0, -3.0, 2.0),
                                           Vector3d(1.0, -2.0, 1.0),
                                           Vector3d(-1.0, 4.0, 2.0),
                                           Vector3d(-2.0, 2.0, 3.0)};
  const Eigen::Matrix3d soln_rotation =
      Eigen::AngleAxisd(DegToRad(13.0), Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 0.5 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = DegToRad(0.25);
  const double kMaxAllowedTranslationDifference = 1e-2;

  TestMLPnPWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

}  // namespace theia
