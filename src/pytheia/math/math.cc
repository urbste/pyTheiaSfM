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

#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>

#include "theia/math/math_wrapper.h"
#include "theia/math/polynomial.h"
#include "theia/math/rotation.h"

// Sophus includes
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "sophus/so3.hpp"
#include "sophus/rxso3.hpp"
#include "sophus/types.hpp"

namespace py = pybind11;
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>
#include <sstream>
#include <stdexcept>

namespace pytheia {
namespace math {

void pytheia_math_classes(py::module& m) {
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

  // Sophus SE3d bindings
  py::class_<Sophus::SE3d>(m, "SE3d")
      .def(py::init<>(), "Default constructor")
      .def(py::init<const Sophus::SO3d&, const Eigen::Vector3d&>(),
           py::arg("so3"), py::arg("translation"),
           "Constructor from SO3 and translation")
      .def(py::init<const Eigen::Quaterniond&, const Eigen::Vector3d&>(),
           py::arg("quaternion"), py::arg("translation"),
           "Constructor from quaternion and translation")
      .def(py::init<const Eigen::Matrix3d&, const Eigen::Vector3d&>(),
           py::arg("rotation_matrix"), py::arg("translation"),
           "Constructor from rotation matrix and translation")
      .def(py::init([](const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation) {
          return Sophus::SE3d(rotation_matrix, translation);
      }), py::arg("rotation_matrix"), py::arg("translation"),
           "Constructor from rotation matrix and translation")
      .def(py::init([](const Eigen::Vector4d& quaternion, const Eigen::Vector3d& translation) {
          Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
          return Sophus::SE3d(q, translation);
      }), py::arg("quaternion"), py::arg("translation"),
           "Constructor from quaternion [w, x, y, z] and translation")
      .def("matrix", &Sophus::SE3d::matrix, "Get 4x4 transformation matrix")
      .def("matrix3x4", &Sophus::SE3d::matrix3x4, "Get 3x4 matrix")
      .def("rotation_matrix", &Sophus::SE3d::rotationMatrix, "Get 3x3 rotation matrix")
      .def("translation", [](const Sophus::SE3d& se3) { return se3.translation(); }, "Get translation vector")
      .def("inverse", &Sophus::SE3d::inverse, "Get inverse transformation")
      .def("log", &Sophus::SE3d::log, "Get Lie algebra (tangent vector)")
      .def("adjoint", &Sophus::SE3d::Adj, "Get adjoint matrix")
      .def("params", &Sophus::SE3d::params, "Get internal parameters")
      .def("set_quaternion", &Sophus::SE3d::setQuaternion, py::arg("quat"),
           "Set quaternion")
      .def("set_rotation_matrix", &Sophus::SE3d::setRotationMatrix, py::arg("R"),
           "Set rotation matrix")
      .def("__mul__", [](const Sophus::SE3d& a, const Sophus::SE3d& b) { return a * b; },
           py::is_operator())
      .def("__mul__", [](const Sophus::SE3d& se3, const Eigen::Vector3d& point) { return se3 * point; },
           py::is_operator())
      .def("__mul__", [](const Sophus::SE3d& se3, const Eigen::Vector4d& hpoint) { return se3 * hpoint; },
           py::is_operator())
      .def("__repr__", [](const Sophus::SE3d& se3) {
          std::ostringstream oss;
          oss << "SE3d(rotation=" << se3.rotationMatrix() << ", translation=" << se3.translation() << ")";
          return oss.str();
      })
      .def_static("exp", [](const Sophus::Vector6d& tangent) { 
          return Sophus::SE3d::exp(tangent); 
      }, py::arg("tangent"), "Exponential map")
      .def_static("rot_x", &Sophus::SE3d::rotX, py::arg("x"), "Rotation around X axis")
      .def_static("rot_y", &Sophus::SE3d::rotY, py::arg("y"), "Rotation around Y axis")
      .def_static("rot_z", &Sophus::SE3d::rotZ, py::arg("z"), "Rotation around Z axis")
      .def_static("trans", [](const Eigen::Vector3d& xyz) { return Sophus::SE3d::trans(xyz); },
                  py::arg("xyz"), "Translation")
      .def_static("trans", [](double x, double y, double z) { return Sophus::SE3d::trans(x, y, z); },
                  py::arg("x"), py::arg("y"), py::arg("z"), "Translation")
      .def_static("trans_x", &Sophus::SE3d::transX, py::arg("x"), "Translation along X axis")
      .def_static("trans_y", &Sophus::SE3d::transY, py::arg("y"), "Translation along Y axis")
      .def_static("trans_z", &Sophus::SE3d::transZ, py::arg("z"), "Translation along Z axis");

  // Sophus SIM3d bindings
  py::class_<Sophus::Sim3d>(m, "Sim3d")
      .def(py::init<>(), "Default constructor")
      .def(py::init<const Eigen::Quaterniond&, const Eigen::Vector3d&>(),
           py::arg("quaternion"), py::arg("translation"),
           "Constructor from quaternion and translation")
      .def(py::init([](const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation, double scale) {
          Sophus::RxSO3d rxso3(rotation_matrix * scale);
          return Sophus::Sim3d(rxso3, translation);
      }), py::arg("rotation_matrix"), py::arg("translation"), py::arg("scale"),
           "Constructor from rotation matrix, translation, and scale")
      .def(py::init([](const Eigen::Vector4d& quaternion, const Eigen::Vector3d& translation, double scale) {
          Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
          Sophus::RxSO3d rxso3(q.toRotationMatrix() * scale);
          return Sophus::Sim3d(rxso3, translation);
      }), py::arg("quaternion"), py::arg("translation"), py::arg("scale"),
           "Constructor from quaternion [w, x, y, z], translation, and scale")
      .def("matrix", &Sophus::Sim3d::matrix, "Get 4x4 transformation matrix")
      .def("matrix3x4", &Sophus::Sim3d::matrix3x4, "Get 3x4 matrix")
      .def("quaternion", &Sophus::Sim3d::quaternion, "Get quaternion")
      .def("translation", [](const Sophus::Sim3d& sim3) { return sim3.translation(); }, "Get translation vector")
      .def("scale", &Sophus::Sim3d::scale, "Get scale factor")
      .def("inverse", &Sophus::Sim3d::inverse, "Get inverse transformation")
      .def("log", &Sophus::Sim3d::log, "Get Lie algebra (tangent vector)")
      .def("adjoint", &Sophus::Sim3d::Adj, "Get adjoint matrix")
      .def("params", &Sophus::Sim3d::params, "Get internal parameters")
      .def("set_quaternion", &Sophus::Sim3d::setQuaternion, py::arg("quat"),
           "Set quaternion")
      .def("set_rotation_matrix", &Sophus::Sim3d::setRotationMatrix, py::arg("R"),
           "Set rotation matrix")
      .def("set_scaled_rotation_matrix", &Sophus::Sim3d::setScaledRotationMatrix, py::arg("sR"),
           "Set scaled rotation matrix")
      .def("set_scale", &Sophus::Sim3d::setScale, py::arg("scale"),
           "Set scale factor")
      .def("__mul__", [](const Sophus::Sim3d& a, const Sophus::Sim3d& b) { return a * b; },
           py::is_operator())
      .def("__mul__", [](const Sophus::Sim3d& sim3, const Eigen::Vector3d& point) { return sim3 * point; },
           py::is_operator())
      .def("__mul__", [](const Sophus::Sim3d& sim3, const Eigen::Vector4d& hpoint) { return sim3 * hpoint; },
           py::is_operator())
      .def("__repr__", [](const Sophus::Sim3d& sim3) {
          std::ostringstream oss;
          oss << "Sim3d(rotation=" << sim3.rotationMatrix() << ", translation=" << sim3.translation() 
              << ", scale=" << sim3.scale() << ")";
          return oss.str();
      })
      .def_static("exp", [](const Sophus::Vector7d& tangent) { 
          return Sophus::Sim3d::exp(tangent); 
      }, py::arg("tangent"), "Exponential map");

  // Create a Sophus submodule for better organization
  py::module sophus_module = m.def_submodule("Sophus", "Sophus Lie group bindings");
  
  // Add the classes to the Sophus submodule as well
  sophus_module.attr("SE3d") = m.attr("SE3d");
  sophus_module.attr("Sim3d") = m.attr("Sim3d");

  // Helper functions for creating transformations
  m.def("Sim3FromRotationTranslationScale", 
        [](const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation, double scale) {
            Sophus::RxSO3d rxso3(rotation * scale);
            return Sophus::Sim3d(rxso3, translation);
        },
        py::arg("rotation"), py::arg("translation"), py::arg("scale"),
        "Create Sim3 from rotation matrix, translation vector, and scale factor");

  m.def("SE3FromRotationTranslation", 
        [](const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
            return Sophus::SE3d(rotation, translation);
        },
        py::arg("rotation"), py::arg("translation"),
        "Create SE3 from rotation matrix and translation vector");
}

void pytheia_math(py::module& m) {
  py::module m_submodule = m.def_submodule("math");
  pytheia_math_classes(m_submodule);
}

}  // namespace math
}  // namespace pytheia