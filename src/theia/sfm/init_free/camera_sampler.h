// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_CAMERA_SAMPLER_H_
#define THEIA_SFM_INIT_FREE_CAMERA_SAMPLER_H_

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/ose_problem.h"

namespace theia {

// Samples `num_cameras` random projective cameras. The study in
// arXiv:2608.18028 shows that this distribution acts as a prior on the
// solution, so it is an explicit option rather than a hidden detail.
//
// UNIT_CIRCLE places camera i at angle 2*pi*i/n on the unit circle in the
// x-z plane, looking at the origin, with a small random perturbation. This is
// our reading of InitFreeBA's UNIT_CIRCLE option; check it against the
// reference implementation before comparing numbers.
OseCameraVector SampleInitialCameras(int num_cameras,
                                     InitFreeCameraInitType type,
                                     unsigned int seed);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_CAMERA_SAMPLER_H_
