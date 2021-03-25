#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <pybind11/numpy.h>

#include "theia/solvers/sampler.h"
#include "theia/solvers/prosac.h"
#include "theia/solvers/prosac_sampler.h"
#include "theia/solvers/exhaustive_sampler.h"
#include "theia/util/random.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/solvers/quality_measurement.h"
#include "theia/solvers/random_sampler.h"
#include "theia/solvers/lmed.h"
#include "theia/solvers/lmed_quality_measurement.h"
#include "theia/solvers/mle_quality_measurement.h"
#include "theia/solvers/exhaustive_ransac.h"
#include "theia/solvers/inlier_support.h"
#include "theia/solvers/evsac.h"
#include "theia/solvers/evsac_sampler.h"


namespace py = pybind11;


PYBIND11_MODULE(pytheia_solvers, m) {

    //RandomNumberGenerator
    py::class_<theia::RandomNumberGenerator>(m, "RandomNumberGenerator")
      .def(py::init<>())
      .def(py::init<int>())
      .def("Seed", &theia::RandomNumberGenerator::Seed)
      .def("RandDouble", &theia::RandomNumberGenerator::RandDouble)
      .def("RandFloat", &theia::RandomNumberGenerator::RandFloat)
      .def("RandInt", &theia::RandomNumberGenerator::RandInt)
      .def("RandGaussian", &theia::RandomNumberGenerator::RandGaussian)

    ;

    //RansacSummary
    py::class_<theia::RansacSummary>(m, "RansacSummary")
      .def_readwrite("inliers", &theia::RansacSummary::inliers)
      .def_readwrite("num_input_data_points", &theia::RansacSummary::num_input_data_points)
      .def_readwrite("num_iterations", &theia::RansacSummary::num_iterations)
      .def_readwrite("confidence", &theia::RansacSummary::confidence)

    ;

    py::class_<theia::RansacParameters>(m, "RansacParameters")
      .def(py::init<>())
      .def_readwrite("error_thresh", &theia::RansacParameters::error_thresh)
      .def_readwrite("failure_probability", &theia::RansacParameters::failure_probability)
      .def_readwrite("min_inlier_ratio", &theia::RansacParameters::min_inlier_ratio)
      .def_readwrite("min_iterations", &theia::RansacParameters::min_iterations)
      .def_readwrite("max_iterations", &theia::RansacParameters::max_iterations)
      .def_readwrite("use_mle", &theia::RansacParameters::use_mle)
      .def_readwrite("use_Tdd_test", &theia::RansacParameters::use_Tdd_test)
    ;
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

    py::class_<theia::LmedQualityMeasurement, theia::QualityMeasurement>(m, "LmedQualityMeasurement")
      .def(py::init<int>())
      .def("ComputeCost", &theia::LmedQualityMeasurement::ComputeCost)
    ;


    py::class_<theia::MLEQualityMeasurement, theia::QualityMeasurement>(m, "MLEQualityMeasurement")
      .def(py::init<double>())
      .def("ComputeCost", &theia::MLEQualityMeasurement::ComputeCost)
    ;

    py::class_<theia::InlierSupport, theia::QualityMeasurement>(m, "InlierSupport")
      .def(py::init<double>())
      .def("ComputeCost", &theia::InlierSupport::ComputeCost)
    ;
    */
    /*
    py::class_<theia::SampleConsensusEstimator>(m, "SampleConsensusEstimator")

      .def("Initialize", &theia::SampleConsensusEstimator::Initialize)
      .def("Estimate", &theia::SampleConsensusEstimator::Estimate)
    ;
    */



}
