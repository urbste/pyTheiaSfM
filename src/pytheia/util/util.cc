#include "pytheia/solvers/solvers.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

#include "theia/util/map_util.h"

namespace py = pybind11;

namespace pytheia {
namespace util {

void pytheia_util_classes(py::module& m) {}

void pytheia_util(py::module& m) {
  py::module m_submodule = m.def_submodule("util");
  pytheia_util_classes(m_submodule);
}

}  // namespace util
}  // namespace pytheia