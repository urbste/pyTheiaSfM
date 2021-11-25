#pragma once

#include "../pytheia_pybind.h"

namespace py = pybind11;

namespace pytheia {
namespace solvers {

void pytheia_solvers(py::module& m);

}
}  // namespace pytheia
