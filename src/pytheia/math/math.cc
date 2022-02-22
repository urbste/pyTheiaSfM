// Copyright (C) 2015 The Regents of the University of California (Regents).
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
// Author: Steffen Urban (urbse@googlemail.com), Shengyu Yin

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/polynomial.h"
#include "theia/math/rotation.h"

// for overloaded function in CameraInstrinsicsModel
template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

namespace py = pybind11;
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

namespace py = pybind11;

namespace pytheia {
namespace math {

void pytheia_math_classes(py::module& m) {
  m.def("FindQuadraticPolynomialRoots", &theia::FindQuadraticPolynomialRoots);
  
  // rotation.h
  m.def("AlignRotations", &theia::AlignRotations,
    "Solves a nonlinear least squares problem so that: rotations * R = gt_rotations.");
  m.def("AlignOrientations", &theia::AlignOrientations,
    "This functions takes as input a dictionary of view_ids to global orientations that should be aligned. Then it calls AlignRotations internally.");
  m.def("MultiplyRotations", &theia::MultiplyRotations, "return R = R1 * R2");
  m.def("RelativeRotationFromTwoRotations", 
    py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&>(
      &theia::RelativeRotationFromTwoRotations), "returns R12 = R2 * R1^T");
  m.def("ApplyRelativeRotation", &theia::ApplyRelativeRotation, "returns R2 = R12 * R1");
  m.def("RelativeTranslationFromTwoPositions", &theia::RelativeTranslationFromTwoPositions, "returns t12 = R1*(p2-p1)");
}

void pytheia_math(py::module& m) {
  py::module m_submodule = m.def_submodule("math");
  pytheia_math_classes(m_submodule);
}

}  // namespace math
}  // namespace pytheia