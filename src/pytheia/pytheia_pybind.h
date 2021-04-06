#pragma once

#include <pybind11/detail/internals.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>  // Include first to suppress compiler warnings
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace py::literals;

// PYBIND11_MAKE_OPAQUE(std::vector<int>);
// PYBIND11_MAKE_OPAQUE(std::vector<int64_t>);
// PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);
// PYBIND11_MAKE_OPAQUE(std::vector<float>);
// PYBIND11_MAKE_OPAQUE(std::vector<double>);
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3i>);
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector2d>);
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector2i>);
