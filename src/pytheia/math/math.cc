#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/polynomial.h"

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
  m.def("FindQuadraticPolynomialRoots", theia::FindQuadraticPolynomialRoots);
}

void pytheia_math(py::module& m) {
  py::module m_submodule = m.def_submodule("math");
  pytheia_math_classes(m_submodule);
}

}  // namespace math
}  // namespace pytheia