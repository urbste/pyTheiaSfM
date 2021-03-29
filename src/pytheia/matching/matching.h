#pragma once

#include "../pytheia_pybind.h"

namespace py = pybind11;

namespace pytheia {
namespace matching {

void pytheia_matching(py::module &m);

}
}