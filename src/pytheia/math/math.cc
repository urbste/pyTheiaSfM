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