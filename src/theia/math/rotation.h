// Copyright (C) 2017 The Regents of the University of California (Regents).
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

// edited Steffen Urban (urbste@googlemail.com), 2021

#ifndef THEIA_MATH_ROTATION_H_
#define THEIA_MATH_ROTATION_H_

#include <Eigen/Core>

#include "theia/sfm/types.h"
#include "theia/util/random.h"
#include <unordered_map>
#include <vector>

namespace theia {

Eigen::Matrix3d GetSkew(const Eigen::Vector3d& f);

Eigen::MatrixXd ProjectToSOd(const Eigen::MatrixXd& M);

// Rotates the "rotation" set of orientations such that the orientations are
// most closely aligned in an L2 sense. That is, "rotation" is transformed such
// that R_rotation * R_gt_rotation^t is minimized.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation);

// Aligns rotations to the ground truth rotations via a similarity
// transformation.
void AlignOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations);

// Use Ceres to perform a stable composition of rotations. This is not as
// efficient as directly composing angle axis vectors (see the old
// implementation commented above) but is more stable.
Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rotation1,
                                  const Eigen::Vector3d& rotation2);

// Computes R_ij = R_j * R_i^t.
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2);

// Computes R_ij = R_j * R_i^t with noise
Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2,
    const double noise,
    theia::RandomNumberGenerator& rng);

// return R_j = R_ij * R_i.
Eigen::Vector3d ApplyRelativeRotation(const Eigen::Vector3d& rotation1,
                                      const Eigen::Vector3d& relative_rotation);

Eigen::Vector3d RelativeTranslationFromTwoPositions(
    const Eigen::Vector3d& position1,
    const Eigen::Vector3d& position2,
    const Eigen::Vector3d& rotation1);
}  // namespace theia

#endif  // THEIA_MATH_ROTATION_H_
