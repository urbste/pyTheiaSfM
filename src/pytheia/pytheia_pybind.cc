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
// Author: Steffen Urban (urbse@googlemail.com)

#include <pybind11/pybind11.h>

#include "pytheia/io/io.h"
#include "pytheia/matching/matching.h"
#include "pytheia/math/math.h"
#include "pytheia/mvs/mvs.h"
#include "pytheia/sfm/sfm.h"
#include "pytheia/solvers/solvers.h"

#include <stdexcept>

namespace pytheia {

// Convert C++ exceptions to Python
static void TranslateExceptions(const std::exception& e) {
  // Handle standard exceptions
  if (dynamic_cast<const std::runtime_error*>(&e) != nullptr) {
    PyErr_SetString(PyExc_RuntimeError, e.what());
  }
  else if (dynamic_cast<const std::invalid_argument*>(&e) != nullptr) {
    PyErr_SetString(PyExc_ValueError, e.what());
  }
  else if (dynamic_cast<const std::out_of_range*>(&e) != nullptr) {
    PyErr_SetString(PyExc_IndexError, e.what());
  }
  else if (dynamic_cast<const std::domain_error*>(&e) != nullptr) {
    PyErr_SetString(PyExc_ValueError, e.what());
  }
  else {
    // Default to RuntimeError for unknown exceptions
    PyErr_SetString(PyExc_RuntimeError, e.what());
  }
}

PYBIND11_MODULE(pytheia, m) {
  // Register exception translator
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    }
    catch (const std::exception& e) {
      TranslateExceptions(e);
    }
  });

  m.doc() = R"pbdoc(
    Python bindings for TheiaSfM library
    -----------------------------------

    .. currentmodule:: pytheia

    This module provides Python bindings for TheiaSfM, a computer vision library for:
    - Structure from Motion (SfM) 
    - Multi-View Stereo (MVS)
    - Feature Detection and Matching
    - Geometric Vision and Camera Models
    - Robust Optimization and Estimation

    Submodules
    ----------
    io : Input/output operations for reconstructions and features
    matching : Feature detection and matching utilities
    math : Mathematical utilities and geometry operations
    mvs : Multi-view stereo reconstruction
    sfm : Structure from Motion pipeline and utilities  
    solvers : Geometric vision solvers and estimators
  )pbdoc";

  // Module version info
  m.attr("__version__") = "1.0.0";
  m.attr("__author__") = "Steffen Urban";

  // Register all submodules
  io::pytheia_io(m);
  matching::pytheia_matching(m);
  math::pytheia_math(m);
  mvs::pytheia_mvs(m);
  sfm::pytheia_sfm(m);
  solvers::pytheia_solvers(m);
}

}  // namespace pytheia