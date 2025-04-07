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

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/closed_form_polynomial_solver.h"
#include "theia/math/find_polynomial_roots_companion_matrix.h"
#include "theia/math/find_polynomial_roots_jenkins_traub.h"
#include "theia/math/l1_solver.h"
#include "theia/math/polynomial.h"
#include "theia/math/math_wrapper.h"
#include "theia/math/qp_solver.h"
#include "theia/math/rotation.h"

namespace py = pybind11;
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

namespace pytheia {
namespace math {

void pytheia_math_classes(py::module& m) {
  // Existing bindings
  m.def("FindQuadraticPolynomialRoots", &theia::FindQuadraticPolynomialRoots);

  // rotation.h
  m.def("AlignRotations",
        &theia::AlignRotations,
        "Solves a nonlinear least squares problem so that: rotations * R = "
        "gt_rotations.");
  m.def("AlignOrientations",
        &theia::AlignOrientationsWrapper,
        "This functions takes as input a dictionary of view_ids to global "
        "orientations that should be aligned. Then it calls AlignRotations "
        "internally.");
  m.def("MultiplyRotations", &theia::MultiplyRotations, "return R = R1 * R2");
  m.def("RelativeRotationFromTwoRotations",
        py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&>(
            &theia::RelativeRotationFromTwoRotations),
        "returns R12 = R2 * R1^T");
  m.def("ApplyRelativeRotation",
        &theia::ApplyRelativeRotation,
        "returns R2 = R12 * R1");
  m.def("RelativeTranslationFromTwoPositions",
        &theia::RelativeTranslationFromTwoPositions,
        "returns t12 = R1*(p2-p1)");

  // ---- New bindings ----
  
  // Closed form polynomial solver bindings - using wrapper functions
  m.def("SolveQuadraticReals", 
        &theia::SolveQuadraticRealsWrapper,
        "Solves quadratic equations with real roots: ax^2 + bx + c = 0");
  m.def("SolveQuadratic", 
        &theia::SolveQuadraticWrapper,
        "Solves quadratic equations with possibly complex roots: ax^2 + bx + c = 0");
  m.def("SolveCubicReals", 
        &theia::SolveCubicRealsWrapper,
        "Solves cubic equations with real roots: ax^3 + bx^2 + cx + d = 0");
  m.def("SolveQuarticReals", 
        &theia::SolveQuarticRealsWrapper,
        "Solves quartic equations with real roots: ax^4 + bx^3 + cx^2 + dx + e = 0");
  
  // Polynomial functions
  m.def("FindRootIterativeLaguerre", 
        &theia::FindRootIterativeLaguerre, 
        "Finds a root of a polynomial using the Laguerre method");
  m.def("FindRootIterativeNewton", 
        &theia::FindRootIterativeNewton, 
        "Finds a root of a polynomial using Newton's method");
  m.def("DifferentiatePolynomial", 
        &theia::DifferentiatePolynomial, 
        "Computes the derivative of a polynomial");
  m.def("MultiplyPolynomials", 
        &theia::MultiplyPolynomials, 
        "Multiplies two polynomials");
  // Use wrapper for DividePolynomial to handle Eigen vectors properly
  m.def("DividePolynomial", 
        &theia::DividePolynomialWrapper, 
        "Divides one polynomial by another, returning quotient and remainder");
  // Use wrapper for MinimizePolynomial to handle Eigen vectors properly
  m.def("MinimizePolynomial", 
        &theia::MinimizePolynomialWrapper, 
        "Minimizes a polynomial within the given bounds");
  
  // Polynomial root-finding algorithms
  m.def("FindPolynomialRootsCompanionMatrix", 
        &theia::FindPolynomialRootsCompanionMatrix, 
        "Finds all roots of a polynomial using the companion matrix method");
  m.def("FindPolynomialRootsJenkinsTraub", 
        &theia::FindPolynomialRootsJenkinsTraub, 
        "Finds all roots of a polynomial using the Jenkins-Traub method");
  
  // L1 Solver
  // Define the L1 Solver Options struct
  py::class_<theia::L1Solver<Eigen::MatrixXd>::Options>(m, "L1SolverOptions")
      .def(py::init<>())
      .def_readwrite("max_num_iterations", &theia::L1Solver<Eigen::MatrixXd>::Options::max_num_iterations)
      .def_readwrite("rho", &theia::L1Solver<Eigen::MatrixXd>::Options::rho)
      .def_readwrite("alpha", &theia::L1Solver<Eigen::MatrixXd>::Options::alpha)
      .def_readwrite("absolute_tolerance", &theia::L1Solver<Eigen::MatrixXd>::Options::absolute_tolerance)
      .def_readwrite("relative_tolerance", &theia::L1Solver<Eigen::MatrixXd>::Options::relative_tolerance);
      
  // Define the L1 Solver class
  py::class_<theia::L1Solver<Eigen::MatrixXd>>(m, "L1Solver")
      .def(py::init<const theia::L1Solver<Eigen::MatrixXd>::Options&, const Eigen::MatrixXd&>())
      .def("SetMaxIterations", &theia::L1Solver<Eigen::MatrixXd>::SetMaxIterations)
      .def("Solve", &theia::L1Solver<Eigen::MatrixXd>::Solve);
  
  // QP Solver
  // Define the QP Solver Options struct
  py::class_<theia::QPSolver::Options>(m, "QPSolverOptions")
      .def(py::init<>())
      .def_readwrite("max_num_iterations", &theia::QPSolver::Options::max_num_iterations)
      .def_readwrite("rho", &theia::QPSolver::Options::rho)
      .def_readwrite("alpha", &theia::QPSolver::Options::alpha)
      .def_readwrite("absolute_tolerance", &theia::QPSolver::Options::absolute_tolerance)
      .def_readwrite("relative_tolerance", &theia::QPSolver::Options::relative_tolerance);
      
  // Define the QP Solver class
  py::class_<theia::QPSolver>(m, "QPSolver")
      .def(py::init<const theia::QPSolver::Options&, 
                    const Eigen::SparseMatrix<double>&, 
                    const Eigen::VectorXd&,
                    const double>())
      .def("SetMaxIterations", &theia::QPSolver::SetMaxIterations)
      .def("SetLowerBound", &theia::QPSolver::SetLowerBound)
      .def("SetUpperBound", &theia::QPSolver::SetUpperBound)
      .def("Solve", &theia::QPSolver::Solve);
}

void pytheia_math(py::module& m) {
  py::module m_submodule = m.def_submodule("math");
  pytheia_math_classes(m_submodule);
}

}  // namespace math
}  // namespace pytheia