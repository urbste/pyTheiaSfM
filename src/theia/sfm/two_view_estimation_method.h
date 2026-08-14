// Copyright (C) 2026, The pyTheiaSfM authors. All rights reserved.

#ifndef THEIA_SFM_TWO_VIEW_ESTIMATION_METHOD_H_
#define THEIA_SFM_TWO_VIEW_ESTIMATION_METHOD_H_

namespace theia {

// Method used to estimate two-view geometry / relative pose / essential matrix.
enum class TwoViewEstimationMethod {
  // Classic 5-point polynomial solver using 10x10 action-matrix eigendecomposition (Stewenius-style).
  FIVE_POINT_STEWENIUS = 0,

  // Fast minimal 5-point solver using polynomial elimination and Sturm sequences (Nistér/PoseLib).
  FIVE_POINT_STURM = 1,

  // Fast iterative 5-point Dogleg solver (Hedborg & Felsberg), constrained to forward-facing trajectories
  // with prior t ~ [0,0,1], R ~ I.
  FAST_ITERATIVE_FIVE_POINT = 2,

  // Monocular depth 3-point minimal solver (PoseLib RePoseD), requiring metric depth priors.
  MONODEPTH_THREE_POINT = 3,
};

}  // namespace theia

#endif  // THEIA_SFM_TWO_VIEW_ESTIMATION_METHOD_H_
