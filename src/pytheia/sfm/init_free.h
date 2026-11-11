// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#pragma once

#include <pybind11/pybind11.h>

namespace pytheia {
namespace sfm {

// Bindings for the experimental initialization-free (OSE / VarPro) module.
// Must be called after ReconstructionEstimator is registered.
void pytheia_init_free(pybind11::module& m);

}  // namespace sfm
}  // namespace pytheia
