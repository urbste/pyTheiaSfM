
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

#include "pytheia/solvers/solvers.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

#include "theia/solvers/exhaustive_ransac.h"
#include "theia/solvers/exhaustive_sampler.h"
#include "theia/solvers/inlier_support.h"
#include "theia/solvers/lmed.h"
#include "theia/solvers/lmed_quality_measurement.h"
#include "theia/solvers/mle_quality_measurement.h"
#include "theia/solvers/prosac.h"
#include "theia/solvers/prosac_sampler.h"
#include "theia/solvers/quality_measurement.h"
#include "theia/solvers/random_sampler.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/solvers/sampler.h"
#include "theia/util/random.h"

namespace py = pybind11;

namespace pytheia {
namespace solvers {

void pytheia_solvers_classes(py::module& m) {
  py::class_<theia::RandomNumberGenerator>(m, "RandomNumberGenerator",
                                            "Reproducible random number generator for RANSAC/sampling.")
      .def(py::init<>(), "Default constructor (non-deterministic seed).")
      .def(py::init<int>(), py::arg("seed"), "Constructor with fixed seed.")
      .def("Seed", &theia::RandomNumberGenerator::Seed, py::arg("seed"),
           "Set the random seed.")
      .def("RandDouble", &theia::RandomNumberGenerator::RandDouble,
           "Return a random double in [0, 1).")
      .def("RandFloat", &theia::RandomNumberGenerator::RandFloat,
           "Return a random float in [0, 1).")
      .def("RandInt", &theia::RandomNumberGenerator::RandInt, py::arg("min"),
           py::arg("max"),
           "Return a random integer in [min, max] (inclusive).")
      .def("RandGaussian", &theia::RandomNumberGenerator::RandGaussian,
           py::arg("mean"), py::arg("std_dev"),
           "Return a sample from a Gaussian distribution.");

  py::class_<theia::RansacSummary>(m, "RansacSummary",
                                   "Summary of a RANSAC run: inliers, iterations, confidence.")
      .def_readwrite("inliers", &theia::RansacSummary::inliers)
      .def_readwrite("num_input_data_points",
                     &theia::RansacSummary::num_input_data_points)
      .def_readwrite("num_iterations", &theia::RansacSummary::num_iterations)
      .def_readwrite("confidence", &theia::RansacSummary::confidence)
      .def_readwrite("num_lo_iterations",
                     &theia::RansacSummary::num_lo_iterations);

  py::class_<theia::RansacParameters>(m, "RansacParameters",
                                      "Parameters for RANSAC: thresholds, iteration limits, MLE/LO options.")
      .def(py::init<>())
      .def_readwrite("error_thresh", &theia::RansacParameters::error_thresh)
      .def_readwrite("failure_probability",
                     &theia::RansacParameters::failure_probability)
      .def_readwrite("min_inlier_ratio",
                     &theia::RansacParameters::min_inlier_ratio)
      .def_readwrite("min_iterations", &theia::RansacParameters::min_iterations)
      .def_readwrite("max_iterations", &theia::RansacParameters::max_iterations)
      .def_readwrite("use_mle", &theia::RansacParameters::use_mle)
      .def_readwrite("use_lo", &theia::RansacParameters::use_lo)
      .def_readwrite("lo_start_iterations",
                     &theia::RansacParameters::lo_start_iterations)
      .def_readwrite("use_Tdd_test", &theia::RansacParameters::use_Tdd_test);
}

void pytheia_solvers(py::module& m) {
  py::module m_submodule = m.def_submodule(
      "solvers",
      "RANSAC and sample-consensus: parameters, summary, random number generator.");
  pytheia_solvers_classes(m_submodule);
}

}  // namespace solvers
}  // namespace pytheia