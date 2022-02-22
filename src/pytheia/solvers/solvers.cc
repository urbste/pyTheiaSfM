
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

#include "theia/solvers/evsac.h"
#include "theia/solvers/evsac_sampler.h"
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
  // RandomNumberGenerator
  py::class_<theia::RandomNumberGenerator>(m, "RandomNumberGenerator")
      .def(py::init<>())
      .def(py::init<int>())
      .def("Seed", &theia::RandomNumberGenerator::Seed)
      .def("RandDouble", &theia::RandomNumberGenerator::RandDouble)
      .def("RandFloat", &theia::RandomNumberGenerator::RandFloat)
      .def("RandInt", &theia::RandomNumberGenerator::RandInt)
      .def("RandGaussian", &theia::RandomNumberGenerator::RandGaussian)

      ;

  // RansacSummary
  py::class_<theia::RansacSummary>(m, "RansacSummary")
      .def_readwrite("inliers", &theia::RansacSummary::inliers)
      .def_readwrite("num_input_data_points",
                     &theia::RansacSummary::num_input_data_points)
      .def_readwrite("num_iterations", &theia::RansacSummary::num_iterations)
      .def_readwrite("confidence", &theia::RansacSummary::confidence)

      ;

  py::class_<theia::RansacParameters>(m, "RansacParameters")
      .def(py::init<>())
      .def_readwrite("error_thresh", &theia::RansacParameters::error_thresh)
      .def_readwrite("failure_probability",
                     &theia::RansacParameters::failure_probability)
      .def_readwrite("min_inlier_ratio",
                     &theia::RansacParameters::min_inlier_ratio)
      .def_readwrite("min_iterations", &theia::RansacParameters::min_iterations)
      .def_readwrite("max_iterations", &theia::RansacParameters::max_iterations)
      .def_readwrite("use_mle", &theia::RansacParameters::use_mle)
      .def_readwrite("use_Tdd_test", &theia::RansacParameters::use_Tdd_test);
  /*
  py::enum_<theia::FittingMethod>(m, "FittingMethod")
    .value("MLE", theia::FittingMethod::MLE)
    .value("QUANTILE_NLS", theia::FittingMethod::QUANTILE_NLS)
    .export_values()
  ;


  py::class_<theia::Sampler> sampler(m, "Sampler");


  py::class_<theia::ProsacSampler>(m, "ProsacSampler", sampler)
    .def(py::init<std::shared_ptr<theia::RandomNumberGenerator>, int>())
    .def("SetSampleNumber", &theia::ProsacSampler::SetSampleNumber)
    .def("Sample", &theia::ProsacSampler::Sample)
    .def("Initialize", &theia::ProsacSampler::Initialize)

  ;

  py::class_<theia::ExhaustiveSampler>(m, "ExhaustiveSampler", sampler)
    .def(py::init<std::shared_ptr<theia::RandomNumberGenerator>, int>())
    .def("Sample", &theia::ExhaustiveSampler::Sample)
    .def("Initialize", &theia::ExhaustiveSampler::Initialize)

  ;

  py::class_<theia::RandomSampler>(m, "RandomSampler", sampler)
    .def(py::init<std::shared_ptr<theia::RandomNumberGenerator>, int>())
    .def("Sample", &theia::RandomSampler::Sample)
    .def("Initialize", &theia::RandomSampler::Initialize)

  ;


  // templated subclass

  py::class_<theia::EvsacSampler<Eigen::Vector2d>>(m, "EvsacSampler", sampler)
    .def(py::init<int, Eigen::MatrixXd, double, theia::FittingMethod>())
    .def("Initialize", &theia::EvsacSampler<Eigen::Vector2d>::Initialize)
    .def("Sample", &theia::EvsacSampler<Eigen::Vector2d>::Sample)

  ;

  py::class_<theia::QualityMeasurement>(m, "QualityMeasurement")

    .def("Initialize", &theia::QualityMeasurement::Initialize)
  ;

  py::class_<theia::LmedQualityMeasurement, theia::QualityMeasurement>(m,
  "LmedQualityMeasurement") .def(py::init<int>()) .def("ComputeCost",
  &theia::LmedQualityMeasurement::ComputeCost)
  ;


  py::class_<theia::MLEQualityMeasurement, theia::QualityMeasurement>(m,
  "MLEQualityMeasurement") .def(py::init<double>()) .def("ComputeCost",
  &theia::MLEQualityMeasurement::ComputeCost)
  ;

  py::class_<theia::InlierSupport, theia::QualityMeasurement>(m,
  "InlierSupport") .def(py::init<double>()) .def("ComputeCost",
  &theia::InlierSupport::ComputeCost)
  ;
  */
  /*
  py::class_<theia::SampleConsensusEstimator>(m, "SampleConsensusEstimator")

    .def("Initialize", &theia::SampleConsensusEstimator::Initialize)
    .def("Estimate", &theia::SampleConsensusEstimator::Estimate)
  ;
  */
}

void pytheia_solvers(py::module& m) {
  py::module m_submodule = m.def_submodule("solvers");
  pytheia_solvers_classes(m_submodule);
}

}  // namespace solvers
}  // namespace pytheia