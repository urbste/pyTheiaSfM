// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "pytheia/sfm/init_free.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Geometry>

#include <memory>
#include <stdexcept>
#include <string>

#include "theia/sfm/init_free/camera_sampler.h"
#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/init_free/init_free_reconstruction_estimator.h"
#include "theia/sfm/init_free/metric_upgrade.h"
#include "theia/sfm/init_free/ose_formulation.h"
#include "theia/sfm/init_free/ose_problem.h"
#include "theia/sfm/init_free/projective_refinement.h"
#include "theia/sfm/init_free/reduced_camera_solver.h"
#include "theia/sfm/init_free/rotation_weighting.h"
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

// Accepts 3 x N Cartesian or 4 x N homogeneous points.
Eigen::Matrix4Xd ToHomogeneous(const Eigen::MatrixXd& points) {
  if (points.rows() == 4) {
    return points;
  }
  if (points.rows() == 3) {
    return points.colwise().homogeneous();
  }
  throw std::invalid_argument("points must be 3 x N or 4 x N.");
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
      .value("POWER_SERIES", theia::ReducedCameraSolverType::POWER_SERIES)
      .value("CUDA_POWER_SERIES",
             theia::ReducedCameraSolverType::CUDA_POWER_SERIES);

  py::enum_<theia::RotationWeightingType>(m, "RotationWeightingType")
      .value("SCALAR", theia::RotationWeightingType::SCALAR)
      .value("TWO_VIEW_INFORMATION",
             theia::RotationWeightingType::TWO_VIEW_INFORMATION);

  py::enum_<theia::InitFreeRestartSelection>(m, "InitFreeRestartSelection")
      .value("OSE_COST", theia::InitFreeRestartSelection::OSE_COST)
      .value("REPROJECTION_ERROR",
             theia::InitFreeRestartSelection::REPROJECTION_ERROR);

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
      .def_readwrite("power_series_tolerance", &VO::power_series_tolerance)
      .def_readwrite("robust_loss_type", &VO::robust_loss_type)
      .def_readwrite("robust_loss_scale", &VO::robust_loss_scale)
      .def_readwrite("num_threads", &VO::num_threads)
      .def_readwrite("point_ridge", &VO::point_ridge)
      .def_readwrite("verbose", &VO::verbose);

  using PRO = theia::ProjectiveRefinementOptions;
  py::class_<PRO>(m, "ProjectiveRefinementOptions")
      .def(py::init<>())
      .def_readwrite("max_num_iterations", &PRO::max_num_iterations)
      .def_readwrite("function_tolerance", &PRO::function_tolerance)
      .def_readwrite("robust_loss_width", &PRO::robust_loss_width)
      .def_readwrite("verbose", &PRO::verbose);

  using IO = theia::InitFreeOptions;
  py::class_<IO>(m, "InitFreeOptions")
      .def(py::init<>())
      .def_readwrite("formulation", &IO::formulation)
      .def_readwrite("alpha", &IO::alpha)
      .def_readwrite("rotation_weight", &IO::rotation_weight)
      .def_readwrite("rotation_weighting", &IO::rotation_weighting)
      .def_readwrite("rotation_normal_weight", &IO::rotation_normal_weight)
      .def_readwrite("min_rotation_information_points",
                     &IO::min_rotation_information_points)
      .def_readwrite("expose_warmup_iterations",
                     &IO::expose_warmup_iterations)
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
      .def_readwrite("projective_refinement", &IO::projective_refinement)
      .def_readwrite("restart_selection", &IO::restart_selection)
      .def_readwrite("min_fraction_in_front", &IO::min_fraction_in_front)
      .def_readwrite("max_mean_orthogonality_error",
                     &IO::max_mean_orthogonality_error)
      .def_readwrite("num_threads", &IO::num_threads)
      .def_readwrite("metric_upgrade", &IO::metric_upgrade)
      .def_readwrite("shared_focal_length", &IO::shared_focal_length)
      .def_readwrite("run_metric_bundle_adjustment",
                     &IO::run_metric_bundle_adjustment);

  py::class_<theia::OseRotationTerm>(m, "OseRotationTerm")
      .def(py::init<>())
      .def_readwrite("camera_a", &theia::OseRotationTerm::camera_a)
      .def_readwrite("camera_b", &theia::OseRotationTerm::camera_b)
      .def_readwrite("relative_rotation",
                     &theia::OseRotationTerm::relative_rotation)
      .def_readwrite("weight", &theia::OseRotationTerm::weight)
      .def_readwrite("sqrt_weight_matrix",
                     &theia::OseRotationTerm::sqrt_weight_matrix);

  using TVW = theia::TwoViewRotationWeight;
  py::class_<TVW>(m, "TwoViewRotationWeight")
      .def(py::init<>())
      .def_readonly("sqrt_weight", &TVW::sqrt_weight)
      .def_readonly("rotation_information", &TVW::rotation_information)
      .def_readonly("translation", &TVW::translation)
      .def_readonly("num_points", &TVW::num_points);

  m.def(
      "ComputeTwoViewRotationWeight",
      [](const Eigen::Matrix2Xd& m_a,
         const Eigen::Matrix2Xd& m_b,
         const Eigen::Matrix3d& R_ab,
         const Eigen::Vector3d& t_ab,
         double normal_weight,
         int min_points) {
        if (m_a.cols() != m_b.cols()) {
          throw std::invalid_argument("m_a and m_b need the same size.");
        }
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
            a, b;
        for (int k = 0; k < m_a.cols(); ++k) {
          a.push_back(m_a.col(k));
          b.push_back(m_b.col(k));
        }
        TVW weight;
        const bool ok = theia::ComputeTwoViewRotationWeight(
            a, b, R_ab, t_ab, normal_weight, min_points, &weight);
        return std::make_tuple(ok, weight);
      },
      py::arg("m_a"),
      py::arg("m_b"),
      py::arg("R_ab"),
      py::arg("t_ab"),
      py::arg("normal_weight") = 1.0,
      py::arg("min_points") = 10,
      "9x9 rotation weight of Olsson & Nilsson for one view pair with "
      "x_a = R_ab x_b + t_ab (2 x N normalized observations; t_ab = 0 "
      "estimates the translation direction). Returns (success, weight).");

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
                                              theia::ResolvedVarProOptions(options),
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
      .def_readonly("message", &RD::message)
      .def_readonly("ose_rms_error", &RD::ose_rms_error)
      .def_readonly("refined_rms_error", &RD::refined_rms_error)
      .def_readonly("refinement_iterations", &RD::refinement_iterations)
      .def_readonly("refinement_time_seconds",
                    &RD::refinement_time_seconds)
      .def_readonly("upgrade_attempted", &RD::upgrade_attempted)
      .def_readonly("upgrade_success", &RD::upgrade_success)
      .def_readonly("upgrade_fraction_in_front",
                    &RD::upgrade_fraction_in_front)
      .def_readonly("upgrade_mean_orthogonality_error",
                    &RD::upgrade_mean_orthogonality_error)
      .def_readonly("upgrade_message", &RD::upgrade_message);

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
      .def_readonly("linear_mean_orthogonality_error",
                    &ID::linear_mean_orthogonality_error)
      .def_readonly("focal_lengths", &ID::focal_lengths)
      .def_readonly("problem_build_time", &ID::problem_build_time)
      .def_readonly("ose_time", &ID::ose_time)
      .def_readonly("metric_upgrade_time", &ID::metric_upgrade_time)
      .def_readonly("bundle_adjustment_time", &ID::bundle_adjustment_time);

  py::class_<theia::ProjectiveReconstruction>(m, "ProjectiveReconstruction")
      .def(py::init<>())
      .def_property_readonly("cameras",
                             [](const theia::ProjectiveReconstruction& r) {
                               return ToList(r.cameras);
                             })
      .def_readonly("points", &theia::ProjectiveReconstruction::points);

  m.def(
      "RunProjectiveStages",
      [](const theia::OseProblem& problem,
         const theia::InitFreeOptions& options) {
        theia::ProjectiveReconstruction best;
        theia::InitFreeDiagnostics diagnostics;
        std::string error;
        bool ok;
        {
          py::gil_scoped_release release;
          ok = theia::RunProjectiveStages(
              problem, options, &best, &diagnostics, &error);
        }
        return std::make_tuple(ok, best, diagnostics, error);
      },
      py::arg("problem"),
      py::arg("options"),
      "Runs options.num_random_restarts random starts of the OSE stage and "
      "(if enabled) projective refinement. Returns (success, best, "
      "diagnostics, error); best.points are homogeneous (4 x N).");

  using PRR = theia::ProjectiveRefinementResult;
  py::class_<PRR>(m, "ProjectiveRefinementResult")
      .def(py::init<>())
      .def_property_readonly("cameras",
                             [](const PRR& r) { return ToList(r.cameras); })
      .def_readonly("points", &PRR::points)
      .def_readonly("initial_rms_error", &PRR::initial_rms_error)
      .def_readonly("final_rms_error", &PRR::final_rms_error)
      .def_readonly("num_iterations", &PRR::num_iterations)
      .def_readonly("converged", &PRR::converged)
      .def_readonly("solve_time_seconds", &PRR::solve_time_seconds)
      .def_readonly("message", &PRR::message);

  m.def(
      "RefineProjectiveReconstruction",
      [](const theia::OseProblem& problem,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras,
         const Eigen::MatrixXd& points,
         const theia::ProjectiveRefinementOptions& options,
         int num_threads) {
        const Eigen::Matrix4Xd homogeneous = ToHomogeneous(points);
        theia::ProjectiveRefinementResult result;
        bool ok;
        {
          py::gil_scoped_release release;
          ok = theia::RefineProjectiveReconstruction(problem,
                                                     FromList(cameras),
                                                     homogeneous,
                                                     options,
                                                     num_threads,
                                                     &result);
        }
        return std::make_tuple(ok, result);
      },
      py::arg("problem"),
      py::arg("cameras"),
      py::arg("points"),
      py::arg("options"),
      py::arg("num_threads") = 1,
      "Projective bundle adjustment. points: 3 x N (Cartesian) or 4 x N "
      "(homogeneous). Returns (success, result).");

  m.def(
      "ProjectiveRmsError",
      [](const theia::OseProblem& problem,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras,
         const Eigen::MatrixXd& points) {
        return theia::ProjectiveRmsError(
            problem, FromList(cameras), ToHomogeneous(points));
      },
      py::arg("problem"),
      py::arg("cameras"),
      py::arg("points"));

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
                    &MU::mean_orthogonality_error)
      .def_readonly("linear_mean_orthogonality_error",
                    &MU::linear_mean_orthogonality_error)
      .def_readonly("focal_lengths", &MU::focal_lengths)
      .def_readonly("nonlinear_refinement_run",
                    &MU::nonlinear_refinement_run);

  using MUO = theia::MetricUpgradeOptions;
  py::class_<MUO>(m, "MetricUpgradeOptions")
      .def(py::init<>())
      .def_readwrite("nonlinear_refinement", &MUO::nonlinear_refinement)
      .def_readwrite("shared_focal_length", &MUO::shared_focal_length)
      .def_readwrite("radial_cameras", &MUO::radial_cameras)
      .def_readwrite("max_num_iterations", &MUO::max_num_iterations);

  m.def(
      "UpgradeToMetric",
      [](const theia::OseProblem& problem,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras,
         const Eigen::MatrixXd& points,
         const theia::MetricUpgradeOptions& options) {
        theia::MetricUpgradeResult result;
        std::string error;
        const bool ok = theia::UpgradeToMetric(problem,
                                               FromList(cameras),
                                               ToHomogeneous(points),
                                               options,
                                               &result,
                                               &error);
        return std::make_tuple(ok, result, error);
      },
      py::arg("problem"),
      py::arg("cameras"),
      py::arg("points"),
      py::arg("options") = theia::MetricUpgradeOptions(),
      "Metric upgrade by self-calibration: K = I for CALIBRATED_RAYS, one "
      "focal length per camera (in observation units) for PIXEL_ISOTROPIC. "
      "points: 3 x N or 4 x N. Returns (success, result, error).");

  m.def(
      "UpgradeToMetricCalibrated",
      [](const theia::OseProblem& problem,
         const std::vector<Eigen::Matrix<double, 3, 4>>& cameras,
         const Eigen::MatrixXd& points) {
        theia::MetricUpgradeResult result;
        std::string error;
        const bool ok = theia::UpgradeToMetricCalibrated(
            problem, FromList(cameras), ToHomogeneous(points), &result, &error);
        return std::make_tuple(ok, result, error);
      },
      py::arg("problem"),
      py::arg("cameras"),
      py::arg("points"),
      "Linear metric upgrade for normalized (calibrated) observations. "
      "points: 3 x N (Cartesian) or 4 x N (homogeneous). "
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
