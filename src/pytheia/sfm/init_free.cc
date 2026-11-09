// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "pytheia/sfm/init_free.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <memory>
#include <string>

#include "theia/sfm/init_free/camera_sampler.h"
#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/init_free_reconstruction_estimator.h"
#include "theia/sfm/init_free/metric_upgrade.h"
#include "theia/sfm/init_free/ose_formulation.h"
#include "theia/sfm/init_free/ose_problem.h"
#include "theia/sfm/init_free/reduced_camera_solver.h"
#include "theia/sfm/init_free/varpro_solver.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view_graph/view_graph.h"

namespace py = pybind11;

namespace pytheia {
namespace sfm {

namespace {

std::vector<Eigen::Matrix<double, 3, 4>> ToList(
    const theia::OseCameraVector& cameras) {
  return {cameras.begin(), cameras.end()};
}

theia::OseCameraVector FromList(
    const std::vector<Eigen::Matrix<double, 3, 4>>& cameras) {
  return {cameras.begin(), cameras.end()};
}

}  // namespace

void pytheia_init_free(py::module& m) {
  // The enums are not exported to module scope: names like LINEAR, NONE and
  // NORMAL already exist there for other enums.
  py::enum_<theia::OseFormulationType>(m, "OseFormulationType")
      .value("POSE", theia::OseFormulationType::POSE)
      .value("ROSE", theia::OseFormulationType::ROSE)
      .value("RPOSE", theia::OseFormulationType::RPOSE)
      .value("EXPOSE", theia::OseFormulationType::EXPOSE)
      .value("POSE_ROT", theia::OseFormulationType::POSE_ROT);

  py::enum_<theia::OseObservationNormalization>(m,
                                                "OseObservationNormalization")
      .value("CALIBRATED_RAYS",
             theia::OseObservationNormalization::CALIBRATED_RAYS)
      .value("PIXEL_ISOTROPIC",
             theia::OseObservationNormalization::PIXEL_ISOTROPIC);

  py::enum_<theia::InitFreeCameraInitType>(m, "InitFreeCameraInitType")
      .value("NORMAL", theia::InitFreeCameraInitType::NORMAL)
      .value("UNIFORM", theia::InitFreeCameraInitType::UNIFORM)
      .value("UNIT_CIRCLE", theia::InitFreeCameraInitType::UNIT_CIRCLE);

  py::enum_<theia::ReducedCameraSolverType>(m, "ReducedCameraSolverType")
      .value("DENSE_CHOLESKY", theia::ReducedCameraSolverType::DENSE_CHOLESKY)
      .value("CUDA_DENSE_CHOLESKY",
             theia::ReducedCameraSolverType::CUDA_DENSE_CHOLESKY)
      .value("SPARSE_CHOLESKY", theia::ReducedCameraSolverType::SPARSE_CHOLESKY)
      .value("POWER_SERIES", theia::ReducedCameraSolverType::POWER_SERIES);

  py::enum_<theia::VarProRobustLossType>(m, "VarProRobustLossType")
      .value("NONE", theia::VarProRobustLossType::NONE)
      .value("CAUCHY", theia::VarProRobustLossType::CAUCHY);

  py::enum_<theia::MetricUpgradeType>(m, "MetricUpgradeType")
      .value("NONE", theia::MetricUpgradeType::NONE)
      .value("LINEAR", theia::MetricUpgradeType::LINEAR)
      .value("LINEAR_AND_NONLINEAR",
             theia::MetricUpgradeType::LINEAR_AND_NONLINEAR);

  using VO = theia::VarProOptions;
  py::class_<VO>(m, "VarProOptions")
      .def(py::init<>())
      .def_readwrite("max_num_iterations", &VO::max_num_iterations)
      .def_readwrite("function_tolerance", &VO::function_tolerance)
      .def_readwrite("parameter_tolerance", &VO::parameter_tolerance)
      .def_readwrite("initial_lambda", &VO::initial_lambda)
      .def_readwrite("min_lambda", &VO::min_lambda)
      .def_readwrite("max_lambda", &VO::max_lambda)
      .def_readwrite("lambda_decrease_factor", &VO::lambda_decrease_factor)
      .def_readwrite("lambda_increase_factor", &VO::lambda_increase_factor)
      .def_readwrite("linear_solver_type", &VO::linear_solver_type)
      .def_readwrite("power_series_max_iterations",
                     &VO::power_series_max_iterations)
      .def_readwrite("robust_loss_type", &VO::robust_loss_type)
      .def_readwrite("robust_loss_scale", &VO::robust_loss_scale)
      .def_readwrite("point_ridge", &VO::point_ridge)
      .def_readwrite("verbose", &VO::verbose);

  using IO = theia::InitFreeOptions;
  py::class_<IO>(m, "InitFreeOptions")
      .def(py::init<>())
      .def_readwrite("formulation", &IO::formulation)
      .def_readwrite("alpha", &IO::alpha)
      .def_readwrite("rotation_weight", &IO::rotation_weight)
      .def_readwrite("include_self_rotation_terms",
                     &IO::include_self_rotation_terms)
      .def_readwrite("observation_normalization",
                     &IO::observation_normalization)
      .def_readwrite("min_track_length", &IO::min_track_length)
      .def_readwrite("min_observations_per_view",
                     &IO::min_observations_per_view)
      .def_readwrite("camera_init", &IO::camera_init)
      .def_readwrite("num_random_restarts", &IO::num_random_restarts)
      .def_readwrite("seed", &IO::seed)
      .def_readwrite("varpro", &IO::varpro)
      .def_readwrite("projective_refinement_max_iterations",
                     &IO::projective_refinement_max_iterations)
      .def_readwrite("metric_upgrade", &IO::metric_upgrade)
      .def_readwrite("run_metric_bundle_adjustment",
                     &IO::run_metric_bundle_adjustment);

  py::class_<theia::OseRotationTerm>(m, "OseRotationTerm")
      .def(py::init<>())
      .def_readwrite("camera_a", &theia::OseRotationTerm::camera_a)
      .def_readwrite("camera_b", &theia::OseRotationTerm::camera_b)
      .def_readwrite("relative_rotation",
                     &theia::OseRotationTerm::relative_rotation)
      .def_readwrite("weight", &theia::OseRotationTerm::weight);

  using OP = theia::OseProblem;
  py::class_<OP>(m, "OseProblem")
      .def(py::init<>())
      .def("NumCameras", &OP::NumCameras)
      .def("NumPoints", &OP::NumPoints)
      .def("NumObservations", &OP::NumObservations)
      .def_readonly("view_ids", &OP::view_ids)
      .def_readonly("track_ids", &OP::track_ids)
      .def_readonly("observation_camera", &OP::observation_camera)
      .def_readonly("observation_point", &OP::observation_point)
      .def_readonly("observations", &OP::observations)
      .def_readonly("point_offsets", &OP::point_offsets)
      .def_readonly("normalizations", &OP::normalizations)
      .def_readonly("normalization", &OP::normalization)
      .def_readonly("rotation_terms", &OP::rotation_terms);

  m.def(
      "BuildOseProblem",
      [](const theia::Reconstruction& reconstruction,
         const theia::ViewGraph* view_graph,
         const theia::InitFreeOptions& options) {
        theia::OseProblem problem;
        std::string error;
        const bool ok = theia::BuildOseProblem(
            reconstruction, view_graph, options, &problem, &error);
        return std::make_tuple(ok, problem, error);
      },
      py::arg("reconstruction"),
      py::arg("view_graph"),
      py::arg("options"),
      "Builds an OseProblem. view_graph may be None (not for POSE_ROT). "
      "Returns (success, problem, error).");

  using OS = theia::OseSolution;
  py::class_<OS>(m, "OseSolution")
      .def(py::init<>())
      .def_property_readonly("cameras",
                             [](const OS& s) { return ToList(s.cameras); })
      .def_readonly("points", &OS::points)
      .def_readonly("initial_cost", &OS::initial_cost)
      .def_readonly("final_cost", &OS::final_cost)
      .def_readonly("cost_history", &OS::cost_history)
      .def_readonly("num_iterations", &OS::num_iterations)
      .def_readonly("converged", &OS::converged)
      .def_readonly("solve_time_seconds", &OS::solve_time_seconds)
      .def_readonly("message", &OS::message);

  m.def(
      "SampleInitialCameras",
      [](int num_cameras, theia::InitFreeCameraInitType type,
         unsigned int seed) {
        return ToList(theia::SampleInitialCameras(num_cameras, type, seed));
      },
      py::arg("num_cameras"),
      py::arg("type"),
      py::arg("seed"));

  m.def(
      "SolveOse",
      [](const theia::OseProblem& problem,
         const theia::InitFreeOptions& options,
         const std::vector<Eigen::Matrix<double, 3, 4>>& initial_cameras) {
        std::string error;
        auto formulation = theia::CreateOseFormulation(options, &error);
        theia::OseSolution solution;
        if (formulation == nullptr) {
          solution.message = error;
          return std::make_tuple(false, solution);
        }
        py::gil_scoped_release release;
        const bool ok = theia::SolveOseVarPro(problem,
                                              *formulation,
                                              options.varpro,
                                              FromList(initial_cameras),
                                              &solution);
        return std::make_tuple(ok, solution);
      },
      py::arg("problem"),
      py::arg("options"),
      py::arg("initial_cameras"),
      "Runs VarPro from the given 3x4 cameras. Returns (success, solution).");

  m.def(
      "EvaluateOseCost",
      [](const theia::OseProblem& problem,
         const theia::InitFreeOptions& options,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras) {
        std::string error;
        auto formulation = theia::CreateOseFormulation(options, &error);
        if (formulation == nullptr) {
          throw std::invalid_argument(error);
        }
        Eigen::Matrix3Xd points;
        const double cost = theia::EvaluateOseCost(problem,
                                                   *formulation,
                                                   FromList(cameras),
                                                   options.varpro.point_ridge,
                                                   &points);
        return std::make_tuple(cost, points);
      },
      py::arg("problem"),
      py::arg("options"),
      py::arg("cameras"),
      "Solves the points for fixed cameras. Returns (cost, points).");

  using RD = theia::InitFreeRestartDiagnostics;
  py::class_<RD>(m, "InitFreeRestartDiagnostics")
      .def(py::init<>())
      .def_readonly("seed", &RD::seed)
      .def_readonly("initial_cost", &RD::initial_cost)
      .def_readonly("final_cost", &RD::final_cost)
      .def_readonly("num_iterations", &RD::num_iterations)
      .def_readonly("converged", &RD::converged)
      .def_readonly("solve_time_seconds", &RD::solve_time_seconds)
      .def_readonly("message", &RD::message);

  using ID = theia::InitFreeDiagnostics;
  py::class_<ID>(m, "InitFreeDiagnostics")
      .def(py::init<>())
      .def_readonly("num_cameras", &ID::num_cameras)
      .def_readonly("num_points", &ID::num_points)
      .def_readonly("num_observations", &ID::num_observations)
      .def_readonly("num_rotation_terms", &ID::num_rotation_terms)
      .def_readonly("restarts", &ID::restarts)
      .def_readonly("best_restart", &ID::best_restart)
      .def_readonly("metric_upgrade_success", &ID::metric_upgrade_success)
      .def_readonly("fraction_in_front", &ID::fraction_in_front)
      .def_readonly("mean_orthogonality_error", &ID::mean_orthogonality_error)
      .def_readonly("problem_build_time", &ID::problem_build_time)
      .def_readonly("ose_time", &ID::ose_time)
      .def_readonly("metric_upgrade_time", &ID::metric_upgrade_time)
      .def_readonly("bundle_adjustment_time", &ID::bundle_adjustment_time);

  m.def(
      "RunOseRestarts",
      [](const theia::OseProblem& problem,
         const theia::InitFreeOptions& options) {
        theia::OseSolution best;
        theia::InitFreeDiagnostics diagnostics;
        std::string error;
        bool ok;
        {
          py::gil_scoped_release release;
          ok = theia::RunOseRestarts(
              problem, options, &best, &diagnostics, &error);
        }
        return std::make_tuple(ok, best, diagnostics, error);
      },
      py::arg("problem"),
      py::arg("options"),
      "Runs options.num_random_restarts VarPro solves from sampled cameras. "
      "Returns (success, best_solution, diagnostics, error).");

  using MU = theia::MetricUpgradeResult;
  py::class_<MU>(m, "MetricUpgradeResult")
      .def(py::init<>())
      .def_readonly("H", &MU::H)
      .def_readonly("rotations", &MU::rotations)
      .def_readonly("translations", &MU::translations)
      .def_readonly("points", &MU::points)
      .def_property_readonly(
          "point_valid",
          [](const MU& r) {
            return std::vector<bool>(r.point_valid.begin(),
                                     r.point_valid.end());
          })
      .def_readonly("fraction_in_front", &MU::fraction_in_front)
      .def_readonly("mean_orthogonality_error",
                    &MU::mean_orthogonality_error);

  m.def(
      "UpgradeToMetricCalibrated",
      [](const theia::OseProblem& problem,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras,
         const Eigen::Matrix3Xd& points) {
        theia::MetricUpgradeResult result;
        std::string error;
        const bool ok = theia::UpgradeToMetricCalibrated(
            problem, FromList(cameras), points, &result, &error);
        return std::make_tuple(ok, result, error);
      },
      py::arg("problem"),
      py::arg("cameras"),
      py::arg("points"),
      "Linear metric upgrade for normalized (calibrated) observations. "
      "Returns (success, result, error).");

  m.def("CudaReducedCameraSolverAvailable",
        &theia::CudaReducedCameraSolverAvailable,
        "True if pytheia was built with -DTHEIA_WITH_CUDA_SOLVERS=ON and a "
        "CUDA device is visible.");

  py::class_<theia::InitFreeReconstructionEstimator,
             theia::ReconstructionEstimator,
             std::unique_ptr<theia::InitFreeReconstructionEstimator>>(
      m, "InitFreeReconstructionEstimator")
      .def(py::init<theia::ReconstructionEstimatorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::InitFreeReconstructionEstimator::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"))
      .def("Diagnostics",
           &theia::InitFreeReconstructionEstimator::Diagnostics);
}

}  // namespace sfm
}  // namespace pytheia
