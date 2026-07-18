// Copyright (C) 2024 The Regents of the University of California (Regents).
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
// Equivalence test for the vectorized Sampson-distance helper added in Phase
// 1.2 of dev/TWO_VIEW_SPEEDUP_PLAN.md: SquaredSampsonDistances() must match
// SquaredSampsonDistance() called per-column, element-wise, to within 1e-12.

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "theia/sfm/pose/util.h"
#include "theia/util/random.h"

namespace theia {
namespace {

TEST(SquaredSampsonDistances, MatchesScalarPerColumn) {
  RandomNumberGenerator rng(123);

  const Eigen::Matrix3d F = (Eigen::Matrix3d() << 0.1, 0.4, -0.2, -0.3, 0.05,
                             0.7, 0.15, -0.6, 0.02)
                                .finished();

  constexpr int kNumPoints = 200;
  Eigen::Matrix3Xd x1(3, kNumPoints), x2(3, kNumPoints);
  for (int i = 0; i < kNumPoints; i++) {
    x1.col(i) = Eigen::Vector3d(
        rng.RandDouble(-1.0, 1.0), rng.RandDouble(-1.0, 1.0), 1.0);
    x2.col(i) = Eigen::Vector3d(
        rng.RandDouble(-1.0, 1.0), rng.RandDouble(-1.0, 1.0), 1.0);
  }

  const std::vector<double> vectorized = SquaredSampsonDistances(F, x1, x2);
  ASSERT_EQ(vectorized.size(), kNumPoints);

  for (int i = 0; i < kNumPoints; i++) {
    const double scalar = SquaredSampsonDistance(
        F, x1.col(i).hnormalized(), x2.col(i).hnormalized());
    EXPECT_NEAR(vectorized[i], scalar, 1e-12);
  }
}

}  // namespace
}  // namespace theia
