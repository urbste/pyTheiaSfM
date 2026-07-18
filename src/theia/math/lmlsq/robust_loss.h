// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// Adapted from PoseLib (https://github.com/PoseLib/PoseLib) robust/robust_loss.h,
// commit fa7280fee27f97aff31ae7f98bab7f583fac7d08 (BSD-3-Clause). See
// docs/licenses/POSELIB_LICENSE.txt for the full PoseLib license text.
//
// Lightweight robust losses for the dense LM local-optimization path used by
// RANSAC RefineModel (not Ceres).

#ifndef THEIA_MATH_LMLSQ_ROBUST_LOSS_H_
#define THEIA_MATH_LMLSQ_ROBUST_LOSS_H_

#include <algorithm>
#include <cmath>

namespace theia {

// ρ(r²) = r², weight = 1.
class TrivialLoss {
 public:
  double Loss(double r2) const { return r2; }
  double Weight(double r2) const {
    (void)r2;
    return 1.0;
  }
};

// Hard truncation on squared residual. `squared_threshold` is already in the
// same units as r² (for Sampson LO this equals RANSAC error_thresh).
class TruncatedLoss {
 public:
  explicit TruncatedLoss(double squared_threshold)
      : squared_threshold_(squared_threshold) {}

  double Loss(double r2) const { return std::min(r2, squared_threshold_); }
  double Weight(double r2) const {
    return (r2 < squared_threshold_) ? 1.0 : 0.0;
  }

 private:
  double squared_threshold_;
};

}  // namespace theia

#endif  // THEIA_MATH_LMLSQ_ROBUST_LOSS_H_
