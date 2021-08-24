#pragma once

#include "../pytheia_pybind.h"

namespace py = pybind11;

namespace pytheia {
namespace sfm {

void pytheia_sfm(py::module& m);

}
}  // namespace pytheia
