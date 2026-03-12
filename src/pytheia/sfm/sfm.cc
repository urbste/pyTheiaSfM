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

#include "pytheia/sfm/sfm.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>
#include <ceres/types.h>

#include "theia/math/polynomial.h"
#include "theia/sfm/pose/util.h"

#include "theia/sfm/feature.h"

#include "theia/sfm/pose/pose_wrapper.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/triangulation/triangulation_wrapper.h"

#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/transformation/align_reconstructions.h"
#include "theia/sfm/transformation/gdls_similarity_transform.h"
#include "theia/sfm/transformation/transformation_wrapper.h"

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/camera_wrapper.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/orthographic_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"

#include "theia/sfm/bundle_adjustment/bundle_adjust_two_views.h"
#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment_wrapper.h"
#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"

#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"
#include "theia/sfm/global_pose_estimation/global_pose_estimation_wrapper.h"
#include "theia/sfm/global_pose_estimation/hybrid_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/lagrange_dual_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/global_pose_estimation/robust_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/rotation_estimator.h"

// reconstruction view track
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/remove_disconnected_view_pairs.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/sfm/visibility_pyramid.h"

#include "theia/sfm/global_reconstruction_estimator.h"
#include "theia/sfm/hybrid_reconstruction_estimator.h"
#include "theia/sfm/incremental_reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/track_builder.h"

#include "theia/sfm/rigid_transformation.h"
#include "theia/sfm/similarity_transformation.h"

// estimator
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_dominant_plane_from_points.h"
#include "theia/sfm/estimators/estimate_radial_distortion_homography.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_relative_pose.h"
#include "theia/sfm/estimators/estimators_wrapper.h"
#include "theia/solvers/sample_consensus_estimator.h"

#include "theia/sfm/colorize_reconstruction.h"
#include "theia/sfm/estimate_track.h"
#include "theia/sfm/estimate_twoview_info.h"
#include "theia/sfm/extract_maximally_parallel_rigid_subgraph.h"
#include "theia/sfm/feature_extractor.h"
#include "theia/sfm/feature_extractor_and_matcher.h"
#include "theia/sfm/filter_view_graph_cycles_by_rotation.h"
#include "theia/sfm/filter_view_pairs_from_orientation.h"
#include "theia/sfm/filter_view_pairs_from_relative_translation.h"
#include "theia/sfm/find_common_tracks_in_views.h"
#include "theia/sfm/find_common_views_by_name.h"
#include "theia/sfm/gps_converter.h"
#include "theia/sfm/localize_view_to_reconstruction.h"
#include "theia/sfm/pose/upnp.h"
#include "theia/sfm/select_good_tracks_for_bundle_adjustment.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/sfm_wrapper.h"

// for overloaded function in CameraInstrinsicsModel
template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

namespace py = pybind11;
#include <iostream>
#include <pybind11/numpy.h>
#include <vector>

// Initialize gtest
// google::InitGoogleLogging(argv[0]);

template <int N>
void AddIntrinsicsPriorType(py::module& m, const std::string& name) {
  py::class_<theia::Prior<N>>(m, ("Prior" + name).c_str(),
                              "Optional prior for intrinsics (is_set, value).")
      .def(py::init())
      .def_readwrite("is_set", &theia::Prior<N>::is_set)
      .def_property("value",
                    &theia::Prior<N>::GetParametersValues,
                    &theia::Prior<N>::SetParametersValues);
}

namespace pytheia {
namespace sfm {

void pytheia_sfm_classes(py::module& m) {
  m.attr("kInvalidTrackId") = theia::kInvalidTrackId;
  m.attr("kInvalidViewId") = theia::kInvalidViewId;
  // camera
  AddIntrinsicsPriorType<1>(m, "Scalar");
  AddIntrinsicsPriorType<2>(m, "Vector2d");
  AddIntrinsicsPriorType<3>(m, "Vector3d");
  AddIntrinsicsPriorType<4>(m, "Vector4d");
  /*
  AddIntrinsicsPriorType<1>(m, "Focallength");
  AddIntrinsicsPriorType<2>(m, "Principalpoint");
  AddIntrinsicsPriorType<1>(m, "Aspectratio");
  AddIntrinsicsPriorType<1>(m, "Skew");
  AddIntrinsicsPriorType<4>(m, "Radialdistortion");
  AddIntrinsicsPriorType<2>(m, "Tangentialdistortion");
  AddIntrinsicsPriorType<3>(m, "Position");
  AddIntrinsicsPriorType<3>(m, "Orientation");
  AddIntrinsicsPriorType<1>(m, "Latitude");
  AddIntrinsicsPriorType<1>(m, "Longitude");
  AddIntrinsicsPriorType<1>(m, "Altitude");
  */

  // abstract superclass and 5 subclasses (camera models)
  py::class_<theia::CameraIntrinsicsModel,
             std::shared_ptr<theia::CameraIntrinsicsModel>>
      camera_intrinsics_model(m, "CameraIntrinsicsModel",
                              "Base class for camera intrinsic models (focal length, principal point, distortion).");
  camera_intrinsics_model
      .def("SetFocalLength", &theia::CameraIntrinsicsModel::SetFocalLength)
      .def("FocalLength", &theia::CameraIntrinsicsModel::FocalLength)
      .def("PrincipalPointX", &theia::CameraIntrinsicsModel::PrincipalPointX)
      .def("PrincipalPointY", &theia::CameraIntrinsicsModel::PrincipalPointY)
      .def("SetPrincipalPoint",
           &theia::CameraIntrinsicsModel::SetPrincipalPoint)
      .def("CameraToImageCoordinates",
           &theia::CameraIntrinsicsModel::CameraToImageCoordinates)
      .def("ImageToCameraCoordinates",
           &theia::CameraIntrinsicsModel::ImageToCameraCoordinates)
      .def("GetParameter", &theia::CameraIntrinsicsModel::GetParameter)
      .def("SetParameter", &theia::CameraIntrinsicsModel::SetParameter)
      // .def("DistortPoint", py::overload_cast<const
      // Eigen::Vector2d>(&theia::CameraIntrinsicsModel::DistortPoint,
      // py::const_)) .def("DistortPoint",   py::overload_cast<const
      // Eigen::Vector2d&>(&theia::CameraIntrinsicsModel::DistortPoint,
      // py::const_)); .def("DistortPoint", static_cast<Eigen::Vector2d
      // (theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d
      // &)>(&theia::CameraIntrinsicsModel::DistortPoint), "Set the pet's name")
      // .def("DistortPoint", overload_cast_<const
      // Eigen::Vector2d>()(&theia::CameraIntrinsicsModel::DistortPoint), "Set
      // the pet's age") .def("DistortPoint",
      //      (Eigen::Vector2d
      //       (theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d& ))
      //       &theia::CameraIntrinsicsModel::DistortPoint,
      //      py::return_value_policy::reference_internal)

      //.def("UndistortPoint",(Eigen::Vector2d
      //(theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d& )) &
      //     theia::CameraIntrinsicsModel::UndistortPoint,
      //     py::arg("UndistortPoint"))
      ;

  // FisheyeCameraModel
  py::class_<theia::FisheyeCameraModel,
             std::shared_ptr<theia::FisheyeCameraModel>>(
      m, "FisheyeCameraModel", camera_intrinsics_model,
      "Fisheye camera model with radial distortion (e.g. Kannala-Brandt).")
      .def(py::init<>())
      .def("Type", &theia::FisheyeCameraModel::Type)
      .def("NumParameters", &theia::FisheyeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::FisheyeCameraModel::SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::FisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::FisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix",
           &theia::FisheyeCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics", &theia::FisheyeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::FisheyeCameraModel::NumParameters)
      .def("AspectRatio", &theia::FisheyeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::FisheyeCameraModel::SetAspectRatio)
      .def("Skew", &theia::FisheyeCameraModel::Skew)
      .def("SetSkew", &theia::FisheyeCameraModel::SetSkew)
      .def("RadialDistortion1", &theia::FisheyeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::FisheyeCameraModel::RadialDistortion2)
      .def("RadialDistortion3", &theia::FisheyeCameraModel::RadialDistortion3)
      .def("RadialDistortion4", &theia::FisheyeCameraModel::RadialDistortion4)
      .def("SetRadialDistortion",
           &theia::FisheyeCameraModel::SetRadialDistortion);

  // PinholeRadialTangentialCameraModel
  py::class_<theia::PinholeRadialTangentialCameraModel,
             std::shared_ptr<theia::PinholeRadialTangentialCameraModel>>(
      m, "PinholeRadialTangentialCameraModel", camera_intrinsics_model,
      "Pinhole model with radial and tangential distortion.")
      .def(py::init<>())
      .def("Type", &theia::PinholeRadialTangentialCameraModel::Type)
      .def("NumParameters",
           &theia::PinholeRadialTangentialCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::PinholeRadialTangentialCameraModel::
               SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::PinholeRadialTangentialCameraModel::
               CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::PinholeRadialTangentialCameraModel::
               GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix",
           &theia::PinholeRadialTangentialCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics",
           &theia::PinholeRadialTangentialCameraModel::PrintIntrinsics)
      .def_property_readonly(
          "kIntrinsicsSize",
          &theia::PinholeRadialTangentialCameraModel::NumParameters)
      .def("AspectRatio",
           &theia::PinholeRadialTangentialCameraModel::AspectRatio)
      .def("SetAspectRatio",
           &theia::PinholeRadialTangentialCameraModel::SetAspectRatio)
      .def("Skew", &theia::PinholeRadialTangentialCameraModel::Skew)
      .def("SetSkew", &theia::PinholeRadialTangentialCameraModel::SetSkew)
      .def("RadialDistortion1",
           &theia::PinholeRadialTangentialCameraModel::RadialDistortion1)
      .def("RadialDistortion2",
           &theia::PinholeRadialTangentialCameraModel::RadialDistortion2)
      .def("RadialDistortion3",
           &theia::PinholeRadialTangentialCameraModel::RadialDistortion3)
      .def("TangentialDistortion1",
           &theia::PinholeRadialTangentialCameraModel::TangentialDistortion1)
      .def("TangentialDistortion2",
           &theia::PinholeRadialTangentialCameraModel::TangentialDistortion2)
      .def("SetRadialDistortion",
           &theia::PinholeRadialTangentialCameraModel::SetRadialDistortion)
      .def("SetTangentialDistortion",
           &theia::PinholeRadialTangentialCameraModel::SetTangentialDistortion);

  // DivisionUndistortionCameraModel
  py::class_<theia::DivisionUndistortionCameraModel,
             std::shared_ptr<theia::DivisionUndistortionCameraModel>>(
      m, "DivisionUndistortionCameraModel", camera_intrinsics_model,
      "Division model for wide-angle distortion.")
      .def(py::init<>())
      .def("Type", &theia::DivisionUndistortionCameraModel::Type)
      .def("NumParameters",
           &theia::DivisionUndistortionCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::DivisionUndistortionCameraModel::
               SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::DivisionUndistortionCameraModel::
               CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::DivisionUndistortionCameraModel::
               GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix",
           &theia::DivisionUndistortionCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics",
           &theia::DivisionUndistortionCameraModel::PrintIntrinsics)
      .def_property_readonly(
          "kIntrinsicsSize",
          &theia::DivisionUndistortionCameraModel::NumParameters)
      .def("AspectRatio", &theia::DivisionUndistortionCameraModel::AspectRatio)
      .def("SetAspectRatio",
           &theia::DivisionUndistortionCameraModel::SetAspectRatio)
      .def("RadialDistortion1",
           &theia::DivisionUndistortionCameraModel::RadialDistortion1)
      .def("SetRadialDistortion",
           &theia::DivisionUndistortionCameraModel::SetRadialDistortion);

  // PinholeCameraModel
  py::class_<theia::PinholeCameraModel,
             std::shared_ptr<theia::PinholeCameraModel>>(
      m, "PinholeCameraModel", camera_intrinsics_model,
      "Simple pinhole model with optional radial distortion (k1, k2).")
      .def(py::init<>())
      .def("Type", &theia::PinholeCameraModel::Type)
      .def("NumParameters", &theia::PinholeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::PinholeCameraModel::SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::PinholeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::PinholeCameraModel::GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix",
           &theia::PinholeCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics", &theia::PinholeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::PinholeCameraModel::NumParameters)
      .def("AspectRatio", &theia::PinholeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::PinholeCameraModel::SetAspectRatio)
      .def("Skew", &theia::PinholeCameraModel::Skew)
      .def("SetSkew", &theia::PinholeCameraModel::SetSkew)
      .def("RadialDistortion1", &theia::PinholeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::PinholeCameraModel::RadialDistortion2)
      .def("SetRadialDistortion",
           &theia::PinholeCameraModel::SetRadialDistortion);

  // OrthographicCameraModel
  py::class_<theia::OrthographicCameraModel,
             std::shared_ptr<theia::OrthographicCameraModel>>(
      m, "OrthographicCameraModel", camera_intrinsics_model,
      "Orthographic (affine) camera model.")
      .def(py::init<>())
      .def("Type", &theia::OrthographicCameraModel::Type)
      .def("NumParameters", &theia::OrthographicCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::OrthographicCameraModel::SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::OrthographicCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::OrthographicCameraModel::GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix",
           &theia::OrthographicCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics", &theia::OrthographicCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::OrthographicCameraModel::NumParameters)
      .def("AspectRatio", &theia::OrthographicCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::OrthographicCameraModel::SetAspectRatio)
      .def("Skew", &theia::OrthographicCameraModel::Skew)
      .def("SetSkew", &theia::OrthographicCameraModel::SetSkew)
      .def("RadialDistortion1",
           &theia::OrthographicCameraModel::RadialDistortion1)
      .def("RadialDistortion2",
           &theia::OrthographicCameraModel::RadialDistortion2)
      .def("SetRadialDistortion",
           &theia::OrthographicCameraModel::SetRadialDistortion);

  // FOVCameraModel
  py::class_<theia::FOVCameraModel, std::shared_ptr<theia::FOVCameraModel>>(
      m, "FOVCameraModel", camera_intrinsics_model,
      "Field-of-view (FOV) camera model.")
      .def(py::init<>())
      .def("Type", &theia::FOVCameraModel::Type)
      .def("NumParameters", &theia::FOVCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::FOVCameraModel::SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::FOVCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::FOVCameraModel::GetSubsetFromOptimizeIntrinsicsType)
      .def("GetCalibrationMatrix", &theia::FOVCameraModel::GetCalibrationMatrix)
      .def("PrintIntrinsics", &theia::FOVCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::FOVCameraModel::NumParameters)
      .def("AspectRatio", &theia::FOVCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::FOVCameraModel::SetAspectRatio)
      .def("RadialDistortion1", &theia::FOVCameraModel::RadialDistortion1)
      .def("SetRadialDistortion", &theia::FOVCameraModel::SetRadialDistortion);

  m.def("ComposeProjectionMatrix", theia::ComposeProjectionMatrixWrapper,
        "Compose 3x4 projection matrix from rotation, position, and calibration matrix.");
  m.def("DecomposeProjectionMatrix", theia::DecomposeProjectionMatrixWrapper,
        "Decompose 3x4 projection matrix into rotation, position, and calibration matrix.");
  m.def("CalibrationMatrixToIntrinsics",
        theia::CalibrationMatrixToIntrinsicsWrapper,
        "Extract focal length and principal point from 3x3 calibration matrix.");
  m.def("IntrinsicsToCalibrationMatrix",
        theia::IntrinsicsToCalibrationMatrixWrapper,
        "Build 3x3 calibration matrix from focal length and principal point.");

  py::class_<theia::Camera, std::shared_ptr<theia::Camera>>(m, "Camera",
                                                            "Camera with intrinsics and extrinsics (pose).")
      .def(py::init())
      .def(py::init<theia::Camera>())
      .def(py::init<theia::CameraIntrinsicsModelType>())
      .def("DeepCopy", &theia::Camera::DeepCopy)
      .def("CameraIntrinsics",
           &theia::Camera::CameraIntrinsics,
           py::return_value_policy::reference)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::Camera::SetFromCameraIntrinsicsPriors)
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::Camera::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetCameraIntrinsicsModelType",
           &theia::Camera::GetCameraIntrinsicsModelType)
      .def("SetCameraIntrinsicsModelType",
           &theia::Camera::SetCameraIntrinsicsModelType)
      .def("InitializeFromProjectionMatrix",
           &theia::Camera::InitializeFromProjectionMatrix)
      .def("GetCalibrationMatrix", &theia::Camera::GetCalibrationMatrixWrapper)
      .def("GetProjectionMatrix", &theia::Camera::GetProjectionMatrixWrapper)
      .def("FocalLength", &theia::Camera::FocalLength)
      .def("SetFocalLength", &theia::Camera::SetFocalLength)
      .def("ImageHeight", &theia::Camera::ImageHeight)
      .def("ImageWidth", &theia::Camera::ImageWidth)
      .def("SetImageSize", &theia::Camera::SetImageSize)
      .def("PrincipalPointX", &theia::Camera::PrincipalPointX)
      .def("PrincipalPointY", &theia::Camera::PrincipalPointY)
      .def("SetPrincipalPoint", &theia::Camera::SetPrincipalPoint)
      .def("GetOrientationAsAngleAxis",
           &theia::Camera::GetOrientationAsAngleAxis)
      .def("GetOrientationAsRotationMatrix",
           &theia::Camera::GetOrientationAsRotationMatrix)
      .def("SetOrientationFromAngleAxis",
           &theia::Camera::SetOrientationFromAngleAxis)
      .def("SetOrientationFromRotationMatrix",
           &theia::Camera::SetOrientationFromRotationMatrix)
      .def("GetPosition", &theia::Camera::GetPosition)
      .def("SetPosition", &theia::Camera::SetPosition)
      .def("PrintCameraIntrinsics", &theia::Camera::PrintCameraIntrinsics)
      .def("PixelToNormalizedCoordinates",
           &theia::Camera::PixelToNormalizedCoordinates)
      .def("PixelToUnitDepthRay", &theia::Camera::PixelToUnitDepthRay)
      .def("ProjectPoint", (std::tuple<double, Eigen::Vector2d> (theia::Camera::*)(const Eigen::Vector4d&)) &theia::Camera::ProjectPointWrapper)
      .def("ProjectPoint", (std::tuple<double, Eigen::Vector2d> (theia::Camera::*)(const Eigen::Vector3d&)) &theia::Camera::ProjectPointWrapper)
      //.def_readonly_static("kExtrinsicsSize", &theia::Camera::kExtrinsicsSize)
      ;

  py::enum_<theia::CameraIntrinsicsModelType>(m, "CameraIntrinsicsModelType",
                                               "Type of camera intrinsic model (pinhole, fisheye, FOV, etc.).")
      .value("INVALID", theia::CameraIntrinsicsModelType::INVALID)
      .value("PINHOLE", theia::CameraIntrinsicsModelType::PINHOLE)
      .value("PINHOLE_RADIAL_TANGENTIAL",
             theia::CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL)
      .value("FISHEYE", theia::CameraIntrinsicsModelType::FISHEYE)
      .value("FOV", theia::CameraIntrinsicsModelType::FOV)
      .value("DIVISION_UNDISTORTION",
             theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION)
      .value("DOUBLE_SPHERE", theia::CameraIntrinsicsModelType::DOUBLE_SPHERE)
      .value("EXTENDED_UNIFIED",
             theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED)
      .value("ORTHOGRAPHIC", theia::CameraIntrinsicsModelType::ORTHOGRAPHIC)
      .export_values();

  py::class_<theia::CameraIntrinsicsPrior>(m, "CameraIntrinsicsPrior",
                                           "Priors for camera intrinsics (focal length, principal point, distortion, etc.).")
      .def(py::init())
      .def_readwrite("image_width", &theia::CameraIntrinsicsPrior::image_width)
      .def_readwrite("image_height",
                     &theia::CameraIntrinsicsPrior::image_height)
      .def_readwrite(
          "camera_intrinsics_model_type",
          &theia::CameraIntrinsicsPrior::camera_intrinsics_model_type)
      .def_readwrite("focal_length",
                     &theia::CameraIntrinsicsPrior::focal_length)
      .def_readwrite("principal_point",
                     &theia::CameraIntrinsicsPrior::principal_point)
      .def_readwrite("aspect_ratio",
                     &theia::CameraIntrinsicsPrior::aspect_ratio)
      .def_readwrite("skew", &theia::CameraIntrinsicsPrior::skew)
      .def_readwrite("radial_distortion",
                     &theia::CameraIntrinsicsPrior::radial_distortion)
      .def_readwrite("tangential_distortion",
                     &theia::CameraIntrinsicsPrior::tangential_distortion)
      .def_readwrite("position", &theia::CameraIntrinsicsPrior::position)
      .def_readwrite("orientation", &theia::CameraIntrinsicsPrior::orientation)
      .def_readwrite("latitude", &theia::CameraIntrinsicsPrior::latitude)
      .def_readwrite("longitude", &theia::CameraIntrinsicsPrior::longitude)
      .def_readwrite("altitude", &theia::CameraIntrinsicsPrior::altitude)

      ;

  // pose
  m.def("PoseFromThreePoints", theia::PoseFromThreePointsWrapper,
        "Compute camera pose from 3D-2D point correspondences (P3P).");
  m.def("NormalizedEightPointFundamentalMatrix",
        theia::NormalizedEightPointFundamentalMatrixWrapper,
        "Estimate fundamental matrix from point correspondences using normalized 8-point algorithm.");
  m.def("FivePointRelativePose", theia::FivePointRelativePoseWrapper,
        "Estimate relative pose from five point correspondences (calibrated).");
  m.def("FourPointPoseAndFocalLength",
        theia::FourPointPoseAndFocalLengthWrapper,
        "Estimate pose and focal length from four 2D-3D correspondences.");
  m.def("FourPointHomography", theia::FourPointHomographyWrapper,
        "Estimate homography from four point correspondences.");
  m.def("FourPointsPoseFocalLengthRadialDistortion",
        theia::FourPointsPoseFocalLengthRadialDistortionWrapper,
        "Estimate pose, focal length, and radial distortion from four correspondences.");
  m.def("FourPointRelativePosePartialRotation",
        theia::FourPointRelativePosePartialRotationWrapper,
        "Estimate relative pose with known rotation from four points.");
  m.def("FivePointFocalLengthRadialDistortion",
        theia::FivePointFocalLengthRadialDistortionWrapper,
        "Estimate relative pose and radial distortion from five points.");
  m.def("ThreePointRelativePosePartialRotation",
        theia::ThreePointRelativePosePartialRotationWrapper,
        "Estimate relative translation with known rotation from three points.");
  m.def("TwoPointPosePartialRotation",
        theia::TwoPointPosePartialRotationWrapper,
        "Estimate pose with known rotation from two points.");
  m.def("DlsPnp", theia::DlsPnpWrapper,
        "Direct least-squares PnP (Kneip et al.).");
  m.def("SQPnP", theia::SQPnPWrapper,
        "SQPnP solver for perspective-n-point.");
  m.def("PositionFromTwoRays", theia::PositionFromTwoRaysWrapper,
        "Triangulate 3D position from two camera rays.");
  m.def("RelativePoseFromTwoPointsWithKnownRotation",
        theia::RelativePoseFromTwoPointsWithKnownRotationWrapper,
        "Compute relative translation from two point correspondences with known rotation.");
  m.def("SevenPointFundamentalMatrix",
        theia::SevenPointFundamentalMatrixWrapper,
        "Estimate fundamental matrix from seven point correspondences.");
  m.def("SimTransformPartialRotation",
        theia::SimTransformPartialRotationWrapper,
        "Estimate similarity transform with known rotation.");
  m.def("DecomposeEssentialMatrix", theia::DecomposeEssentialMatrixWrapper,
        "Extract rotation and translation from essential matrix.");
  m.def("EssentialMatrixFromTwoProjectionMatrices",
        theia::EssentialMatrixFromTwoProjectionMatricesWrapper,
        "Compute essential matrix from two projection matrices.");
  m.def("GetBestPoseFromEssentialMatrix",
        theia::GetBestPoseFromEssentialMatrixWrapper,
        "Select the correct pose (R, t) from four candidates using cheirality.");
  m.def("FocalLengthsFromFundamentalMatrix",
        theia::FocalLengthsFromFundamentalMatrixWrapper,
        "Extract focal lengths of two cameras from fundamental matrix.");
  m.def("SharedFocalLengthsFromFundamentalMatrix",
        theia::SharedFocalLengthsFromFundamentalMatrixWrapper,
        "Extract common focal length from fundamental matrix (same focal length assumed).");
  m.def("ProjectionMatricesFromFundamentalMatrix",
        theia::ProjectionMatricesFromFundamentalMatrixWrapper,
        "Get canonical projection matrices from fundamental matrix and focal lengths.");
  m.def("FundamentalMatrixFromProjectionMatrices",
        theia::FundamentalMatrixFromProjectionMatricesWrapper,
        "Compute fundamental matrix from two projection matrices.");
  m.def("EssentialMatrixFromFundamentalMatrix",
        theia::EssentialMatrixFromFundamentalMatrixWrapper,
        "Compute essential matrix from fundamental matrix and calibration matrices.");
  m.def("ComposeFundamentalMatrix", theia::ComposeFundamentalMatrixWrapper,
        "Compose fundamental matrix from two projection matrices.");
  m.def("PlanarUncalibratedOrthographicPose",
        theia::PlanarUncalibratedOrthographicPoseWrapper,
        "Estimate orthographic pose for planar scene (uncalibrated).");

  // transformation
  m.def("AlignPointCloudsUmeyama", theia::AlignPointCloudsUmeyamaWrapper,
        "Compute similarity transform (rotation, translation, scale) aligning two point clouds (Umeyama).");
  m.def("AlignPointCloudsUmeyamaWithWeights",
        theia::AlignPointCloudsUmeyamaWithWeightsWrapper,
        "Weighted Umeyama alignment of two point clouds.");
  m.def("GdlsSimilarityTransform", theia::GdlsSimilarityTransformWrapper,
        "Globally optimal similarity transform via GDLS.");
  m.def("AlignRotations", theia::AlignRotationsWrapper,
        "Align two reconstructions by rotation (e.g. to ground truth).");
  m.def("AlignReconstructions", theia::AlignReconstructionsWrapper,
        "Align two reconstructions with similarity transform.");
  m.def("AlignReconstructionsRobust", theia::AlignReconstructionsRobustWrapper,
        "Robust alignment of two reconstructions (outlier-resistant).");
  m.def("TransformReconstruction", theia::TransformReconstructionWrapper,
        "Apply similarity transform to a reconstruction.");
  m.def("TransformReconstruction4", theia::TransformReconstructionWrapper4,
        "Apply 4x4 homogeneous transform to a reconstruction.");

  // SIM3 Point Cloud Alignment
  py::enum_<theia::Sim3AlignmentType>(m, "Sim3AlignmentType",
                                      "Type of Sim3 point cloud alignment (point-to-point, robust, point-to-plane).")
      .value("POINT_TO_POINT", theia::Sim3AlignmentType::POINT_TO_POINT)
      .value("ROBUST_POINT_TO_POINT", theia::Sim3AlignmentType::ROBUST_POINT_TO_POINT)
      .value("POINT_TO_PLANE", theia::Sim3AlignmentType::POINT_TO_PLANE)
      .export_values();

  py::class_<theia::Sim3AlignmentOptions>(m, "Sim3AlignmentOptions",
                                          "Options for Sim3 alignment (type, weights, thresholds, iterations).")
      .def(py::init<>())
      .def_readwrite("alignment_type", &theia::Sim3AlignmentOptions::alignment_type)
      .def_readwrite("point_weight", &theia::Sim3AlignmentOptions::point_weight)
      .def_readwrite("huber_threshold", &theia::Sim3AlignmentOptions::huber_threshold)
      .def_readwrite("outlier_threshold", &theia::Sim3AlignmentOptions::outlier_threshold)
      .def_readwrite("max_iterations", &theia::Sim3AlignmentOptions::max_iterations)
      .def_readwrite("perform_optimization", &theia::Sim3AlignmentOptions::perform_optimization)
      .def_readwrite("verbose", &theia::Sim3AlignmentOptions::verbose)
      .def("set_initial_sim3_params", &theia::Sim3AlignmentOptions::SetInitialSim3Params,
           py::arg("params"), "Set initial SIM3 parameters")
      .def("set_target_normals", &theia::Sim3AlignmentOptions::SetTargetNormals,
           py::arg("normals"), "Set target normals for point-to-plane alignment")
      .def("set_point_weights", &theia::Sim3AlignmentOptions::SetPointWeights,
           py::arg("weights"), "Set point weights")
      .def("clear_initial_sim3_params", &theia::Sim3AlignmentOptions::ClearInitialSim3Params,
           "Clear initial SIM3 parameters")
      .def("clear_target_normals", &theia::Sim3AlignmentOptions::ClearTargetNormals,
           "Clear target normals")
      .def("clear_point_weights", &theia::Sim3AlignmentOptions::ClearPointWeights,
           "Clear point weights");

  py::class_<theia::Sim3AlignmentSummary>(m, "Sim3AlignmentSummary",
                                          "Summary of Sim3 alignment (success, params, error, iterations).")
      .def(py::init<>())
      .def_readwrite("success", &theia::Sim3AlignmentSummary::success)
      .def_readwrite("sim3_params", &theia::Sim3AlignmentSummary::sim3_params)
      .def_readwrite("alignment_error", &theia::Sim3AlignmentSummary::alignment_error)
      .def_readwrite("num_iterations", &theia::Sim3AlignmentSummary::num_iterations)
      .def_readwrite("final_cost", &theia::Sim3AlignmentSummary::final_cost);

  m.def("OptimizeAlignmentSim3", theia::OptimizeAlignmentSim3Wrapper,
        "Optimize Sim3 alignment between two point clouds (optionally with normals/weights).");
  m.def("Sim3FromRotationTranslationScale", theia::Sim3FromRotationTranslationScaleWrapper,
        "Build Sim3 from rotation matrix, translation vector, and scale.");
  m.def("Sim3ToRotationTranslationScale", theia::Sim3ToRotationTranslationScaleWrapper,
        "Decompose Sim3 into rotation, translation, and scale.");
  m.def("Sim3ToHomogeneousMatrix", theia::Sim3ToHomogeneousMatrixWrapper,
        "Convert Sim3 to 4x4 homogeneous matrix.");

  py::class_<theia::SimilarityTransformation>(m, "SimilarityTransformation",
                                              "Similarity transform: rotation, translation, scale.")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::SimilarityTransformation::rotation)
      .def_readwrite("translation",
                     &theia::SimilarityTransformation::translation)
      .def_readwrite("scale", &theia::SimilarityTransformation::scale);

  py::class_<theia::RigidTransformation>(m, "RigidTransformation",
                                        "Rigid transform: rotation and translation (no scale).")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::RigidTransformation::rotation)
      .def_readwrite("translation", &theia::RigidTransformation::translation);

  py::class_<theia::Feature>(m, "Feature",
                            "2D feature (point, optional covariance, depth prior).")
      .def(py::init<>())
      .def(py::init<double, double>())
      .def(py::init<double, double, double>())
      .def(py::init<Eigen::Vector2d>())
      .def(py::init<Eigen::Vector2d, double>())
      .def(py::init<Eigen::Vector2d, Eigen::Matrix2d>())
      .def(py::init<Eigen::Vector2d, Eigen::Matrix2d, double, double>())
      .def_readwrite("point", &theia::Feature::point_)
      .def_readwrite("covariance", &theia::Feature::covariance_)
      .def_readwrite("depth_prior", &theia::Feature::depth_prior_)
      .def_readwrite("depth_prior_variance",
                     &theia::Feature::depth_prior_variance_)
      .def("x", &theia::Feature::x)
      .def("y", &theia::Feature::y)
      .def("get_depth_prior", &theia::Feature::depth_prior)
      .def("get_depth_prior_variance", &theia::Feature::depth_prior_variance);

  py::class_<theia::CameraAndFeatureCorrespondence2D3D>(
      m, "CameraAndFeatureCorrespondence2D3D",
      "Camera plus 2D-3D correspondence for absolute pose.")
      .def(py::init<>())
      .def_readwrite("camera",
                     &theia::CameraAndFeatureCorrespondence2D3D::camera)
      .def_readwrite("observation",
                     &theia::CameraAndFeatureCorrespondence2D3D::observation)
      .def_readwrite("point3d",
                     &theia::CameraAndFeatureCorrespondence2D3D::point3d);

  py::class_<theia::FeatureCorrespondence2D3D>(m, "FeatureCorrespondence2D3D",
                                              "2D-3D correspondence (feature position and 3D point).")
      .def(py::init<>())
      .def(py::init<Eigen::Vector2d, Eigen::Vector3d>())
      .def_readwrite("feature", &theia::FeatureCorrespondence2D3D::feature)
      .def_readwrite("world_point",
                     &theia::FeatureCorrespondence2D3D::world_point);

  py::class_<theia::CalibratedAbsolutePose>(m, "CalibratedAbsolutePose",
                                            "Calibrated absolute pose: rotation and position.")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::CalibratedAbsolutePose::rotation)
      .def_readwrite("position", &theia::CalibratedAbsolutePose::position);

  py::class_<theia::UncalibratedAbsolutePose>(m, "UncalibratedAbsolutePose",
                                             "Uncalibrated absolute pose: rotation, position, focal length.")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::UncalibratedAbsolutePose::rotation)
      .def_readwrite("position", &theia::UncalibratedAbsolutePose::position)
      .def_readwrite("focal_length",
                     &theia::UncalibratedAbsolutePose::focal_length);

  py::class_<theia::UncalibratedRelativePose>(m, "UncalibratedRelativePose",
                                             "Uncalibrated relative pose: fundamental matrix, focal lengths, R, t.")
      .def(py::init<>())
      .def_readwrite("fundamental_matrix",
                     &theia::UncalibratedRelativePose::fundamental_matrix)
      .def_readwrite("focal_length1",
                     &theia::UncalibratedRelativePose::focal_length1)
      .def_readwrite("focal_length2",
                     &theia::UncalibratedRelativePose::focal_length2)
      .def_readwrite("rotation", &theia::UncalibratedRelativePose::rotation)
      .def_readwrite("position", &theia::UncalibratedRelativePose::position);

  py::class_<theia::RelativePose>(m, "RelativePose",
                                 "Calibrated relative pose: essential matrix, rotation, position.")
      .def(py::init<>())
      .def_readwrite("essential_matrix", &theia::RelativePose::essential_matrix)
      .def_readwrite("rotation", &theia::RelativePose::rotation)
      .def_readwrite("position", &theia::RelativePose::position);

  py::class_<theia::Plane>(m, "Plane",
                          "Plane defined by point and unit normal.")
      .def(py::init<>())
      .def_readwrite("point", &theia::Plane::point)
      .def_readwrite("unit_normal", &theia::Plane::unit_normal);

  py::class_<theia::RadialDistortionFeatureCorrespondence>(
      m, "RadialDistortionFeatureCorrespondence",
      "Feature correspondence with radial distortion and focal estimates.")
      .def(py::init<>())
      .def_readwrite(
          "feature_left",
          &theia::RadialDistortionFeatureCorrespondence::feature_left)
      .def_readwrite(
          "feature_right",
          &theia::RadialDistortionFeatureCorrespondence::feature_right)
      .def_readwrite("normalized_feature_left",
                     &theia::RadialDistortionFeatureCorrespondence::
                         normalized_feature_left)
      .def_readwrite("normalized_feature_right",
                     &theia::RadialDistortionFeatureCorrespondence::
                         normalized_feature_right)
      .def_readwrite("focal_length_estimate_left",
                     &theia::RadialDistortionFeatureCorrespondence::
                         focal_length_estimate_left)
      .def_readwrite("focal_length_estimate_right",
                     &theia::RadialDistortionFeatureCorrespondence::
                         focal_length_estimate_right)
      .def_readwrite(
          "min_radial_distortion",
          &theia::RadialDistortionFeatureCorrespondence::min_radial_distortion)
      .def_readwrite(
          "max_radial_distortion",
          &theia::RadialDistortionFeatureCorrespondence::max_radial_distortion);

  py::class_<theia::RadialDistUncalibratedAbsolutePoseMetaData>(
      m, "RadialDistUncalibratedAbsolutePoseMetaData",
      "Metadata for radial distortion absolute pose (focal and distortion ranges).")
      .def(py::init<>())
      .def_readwrite(
          "min_focal_length",
          &theia::RadialDistUncalibratedAbsolutePoseMetaData::min_focal_length)
      .def_readwrite(
          "max_focal_length",
          &theia::RadialDistUncalibratedAbsolutePoseMetaData::max_focal_length)
      .def_readwrite("min_radial_distortion",
                     &theia::RadialDistUncalibratedAbsolutePoseMetaData::
                         min_radial_distortion)
      .def_readwrite("max_radial_distortion",
                     &theia::RadialDistUncalibratedAbsolutePoseMetaData::
                         max_radial_distortion);

  // estimator ransac
  py::enum_<theia::RansacType>(m, "RansacType",
                               "RANSAC variant: RANSAC, PROSAC, LMED, EXHAUSTIVE.")
      .value("RANSAC", theia::RansacType::RANSAC)
      .value("PROSAC", theia::RansacType::PROSAC)
      .value("LMED", theia::RansacType::LMED)
      .value("EXHAUSTIVE", theia::RansacType::EXHAUSTIVE)
      .export_values();

  py::enum_<theia::PnPType>(m, "PnPType",
                           "PnP solver type: KNEIP, DLS, SQPnP.")
      .value("KNEIP", theia::PnPType::KNEIP)
      .value("DLS", theia::PnPType::DLS)
      .value("SQPnP", theia::PnPType::SQPnP)
      .export_values();

  m.def("EstimateAbsolutePoseWithKnownOrientation",
        theia::EstimateAbsolutePoseWithKnownOrientationWrapper,
        "RANSAC: absolute pose from 2D-3D correspondences with known orientation.");
  m.def("EstimateCalibratedAbsolutePose",
        theia::EstimateCalibratedAbsolutePoseWrapper,
        "RANSAC: calibrated absolute pose (PnP) from 2D-3D correspondences.");
  m.def("EstimateDominantPlaneFromPoints",
        theia::EstimateDominantPlaneFromPointsWrapper,
        "RANSAC: fit dominant plane to 3D points.");

  m.def("EstimateEssentialMatrix", theia::EstimateEssentialMatrixWrapper,
        "RANSAC: essential matrix from calibrated point correspondences.");
  m.def("EstimateFundamentalMatrix", theia::EstimateFundamentalMatrixWrapper,
        "RANSAC: fundamental matrix from point correspondences.");
  m.def("EstimateHomography", theia::EstimateHomographyWrapper,
        "RANSAC: homography from point correspondences.");
  m.def("EstimateRadialHomographyMatrix",
        theia::EstimateRadialHomographyMatrixWrapper,
        "RANSAC: radial distortion homography.");
  m.def("EstimateRelativePose", theia::EstimateRelativePoseWrapper,
        "RANSAC: relative pose from calibrated correspondences.");
  m.def("EstimateRelativePoseWithKnownOrientation",
        theia::EstimateRelativePoseWithKnownOrientationWrapper,
        "RANSAC: relative translation with known rotation.");
  m.def("EstimateRigidTransformation2D3D",
        theia::EstimateRigidTransformation2D3DWrapper,
        "RANSAC: rigid 2D-3D transformation.");
  m.def("EstimateRigidTransformation2D3DNormalized",
        theia::EstimateRigidTransformation2D3DNormalizedWrapper,
        "RANSAC: rigid 2D-3D transformation (normalized coordinates).");
  m.def("EstimateTriangulation", theia::EstimateTriangulationWrapper,
        "RANSAC: triangulate point from multi-view correspondences.");
  m.def("EstimateUncalibratedAbsolutePose",
        theia::EstimateUncalibratedAbsolutePoseWrapper,
        "RANSAC: absolute pose and focal length from 2D-3D correspondences.");
  m.def("EstimateUncalibratedRelativePose",
        theia::EstimateUncalibratedRelativePoseWrapper,
        "RANSAC: fundamental matrix and relative pose (uncalibrated).");

  // triangulation
  m.def("Triangulate", theia::TriangulateWrapper,
        "Triangulate 3D point from two or more rays/correspondences.");
  m.def("TriangulateMidpoint", theia::TriangulateMidpointWrapper,
        "Triangulate using midpoint method.");
  m.def("TriangulateDLT", theia::TriangulateDLTWrapper,
        "Triangulate using DLT.");
  m.def("TriangulateNViewSVD", theia::TriangulateNViewSVDWrapper,
        "Triangulate from N views using SVD.");
  m.def("TriangulateNView", theia::TriangulateNViewWrapper,
        "Triangulate 3D point from N views.");
  m.def("IsTriangulatedPointInFrontOfCameras",
        theia::IsTriangulatedPointInFrontOfCameras,
        "Check if triangulated point is in front of all cameras.");
  m.def("SufficientTriangulationAngle", theia::SufficientTriangulationAngle,
        "Check if ray angle is above threshold for stable triangulation.");

  // function in the sfm folder

  py::class_<theia::EstimateTwoViewInfoOptions>(m, "EstimateTwoViewInfoOptions",
                                                "Options for two-view geometry estimation (RANSAC, focal range).")
      .def(py::init<>())
      .def_readwrite("ransac_type",
                     &theia::EstimateTwoViewInfoOptions::ransac_type)
      .def_readwrite(
          "max_sampson_error_pixels",
          &theia::EstimateTwoViewInfoOptions::max_sampson_error_pixels)
      .def_readwrite(
          "expected_ransac_confidence",
          &theia::EstimateTwoViewInfoOptions::expected_ransac_confidence)
      .def_readwrite("min_ransac_iterations",
                     &theia::EstimateTwoViewInfoOptions::min_ransac_iterations)
      .def_readwrite("max_ransac_iterations",
                     &theia::EstimateTwoViewInfoOptions::max_ransac_iterations)
      .def_readwrite("use_mle", &theia::EstimateTwoViewInfoOptions::use_mle)
      .def_readwrite("use_lo", &theia::EstimateTwoViewInfoOptions::use_lo)
      .def_readwrite("lo_start_iterations",
                     &theia::EstimateTwoViewInfoOptions::lo_start_iterations)
      .def_readwrite("min_focal_length",
                     &theia::EstimateTwoViewInfoOptions::min_focal_length)
      .def_readwrite("max_focal_length",
                     &theia::EstimateTwoViewInfoOptions::max_focal_length);

  py::class_<theia::FilterViewPairsFromRelativeTranslationOptions>(
      m, "FilterViewPairsFromRelativeTranslationOptions",
      "Options for filtering view pairs by relative translation consistency.")
      .def(py::init<>())
      .def_readwrite(
          "num_threads",
          &theia::FilterViewPairsFromRelativeTranslationOptions::num_threads)
      .def_readwrite(
          "num_iterations",
          &theia::FilterViewPairsFromRelativeTranslationOptions::num_iterations)
      .def_readwrite("translation_projection_tolerance",
                     &theia::FilterViewPairsFromRelativeTranslationOptions::
                         translation_projection_tolerance);

  py::class_<theia::LocalizeViewToReconstructionOptions>(
      m, "LocalizeViewToReconstructionOptions",
      "Options for adding a new view to a reconstruction (reprojection threshold, RANSAC, PnP type).")
      .def(py::init<>())
      .def_readwrite("reprojection_error_threshold_pixels",
                     &theia::LocalizeViewToReconstructionOptions::
                         reprojection_error_threshold_pixels)
      .def_readwrite(
          "assume_known_orientation",
          &theia::LocalizeViewToReconstructionOptions::assume_known_orientation)
      .def_readwrite("ransac_params",
                     &theia::LocalizeViewToReconstructionOptions::ransac_params)
      .def_readwrite(
          "bundle_adjust_view",
          &theia::LocalizeViewToReconstructionOptions::bundle_adjust_view)
      .def_readwrite("ba_options",
                     &theia::LocalizeViewToReconstructionOptions::ba_options)
      .def_readwrite(
          "min_num_inliers",
          &theia::LocalizeViewToReconstructionOptions::min_num_inliers)
      .def_readwrite("pnp_type",
                     &theia::LocalizeViewToReconstructionOptions::pnp_type);

  m.def("EstimateTwoViewInfo", theia::EstimateTwoViewInfoWrapper,
        "Estimate two-view geometry (relative pose, focal lengths) from correspondences with RANSAC.");
  m.def("ColorizeReconstruction", theia::ColorizeReconstruction,
        "Assign colors to 3D points from image observations.");
  m.def("ExtractMaximallyParallelRigidSubgraph",
        theia::ExtractMaximallyParallelRigidSubgraph,
        "Extract maximal rigid subgraph from view graph.");
  m.def("FilterViewGraphCyclesByRotation",
        theia::FilterViewGraphCyclesByRotation,
        "Remove inconsistent edges using rotation cycle constraints.");
  m.def("FilterViewPairsFromOrientation",
        theia::FilterViewPairsFromOrientation,
        "Filter view pairs inconsistent with global orientations.");
  m.def("FilterViewPairsFromRelativeTranslation",
        theia::FilterViewPairsFromRelativeTranslation,
        "Filter view pairs by relative translation consistency.");
  m.def("LocalizeViewToReconstruction", theia::LocalizeViewToReconstruction,
        "Register a new view/camera into an existing reconstruction (PnP + optional BA).");
  m.def("SelectGoodTracksForBundleAdjustment",
        theia::SelectGoodTracksForBundleAdjustmentWrapper,
        "Select tracks suitable for bundle adjustment.");
  m.def("SetOutlierTracksToUnestimated",
        theia::SetOutlierTracksToUnestimatedWrapper,
        "Mark outlier tracks as unestimated.");
  m.def("SetCameraIntrinsicsFromPriors", theia::SetCameraIntrinsicsFromPriors,
        "Initialize camera intrinsics from prior per view.");
  m.def("FindCommonViewsByName", theia::FindCommonViewsByName,
        "Find view IDs common to two reconstructions by name.");
  m.def("FindCommonTracksInViews", theia::FindCommonTracksInViews,
        "Find tracks visible in a set of views.");
  m.def("RemoveDisconnectedViewPairs", theia::RemoveDisconnectedViewPairs,
        "Remove view pairs that disconnect the view graph.");
  m.def("AddFeatureCorrespondencesToTrackBuilder", theia::AddFeatureCorrespondencesToTrackBuilderWrapper,
        "Add feature correspondences to track builder for track creation.");
  m.def("AddFullFeatureCorrespondencesToTrackBuilder", theia::AddFullFeatureCorrespondencesToTrackBuilderWrapper,
        "Add full feature correspondences to track builder.");
  m.def("UpdateFeaturesInView", theia::UpdateFeaturesInViewWrapper,
        "Update features stored in a view.");
  m.def("FindCommonTracksByFeatureInReconstructions",
        theia::FindCommonTracksByFeatureInReconstructionsWrapper,
        "Find common tracks between two reconstructions by feature overlap.");
  m.def("AddObservations", theia::AddObservationsWrapper,
        "Add observations to reconstruction.");
  m.def("AddTracks", theia::AddTracksWrapper,
        "Add tracks to reconstruction.");

  // View class
  py::class_<theia::View>(m, "View",
                          "A single image/view: name, camera, features, intrinsics priors.")
      .def(py::init<>())
      .def(py::init<std::string>())
      .def("Name", &theia::View::Name)
      .def("IsEstimated", &theia::View::IsEstimated)
      .def("SetIsEstimated", &theia::View::SetEstimated)
      .def("NumFeatures", &theia::View::NumFeatures)
      .def("AddFeature", &theia::View::AddFeature)
      .def("UpdateFeature", &theia::View::UpdateFeature)
      .def("RemoveFeature", &theia::View::RemoveFeature)
      .def("TrackIds", &theia::View::TrackIds)
      .def("GetFeature",
           &theia::View::GetFeature,
           py::return_value_policy::reference)
      .def("GetTrack", &theia::View::GetTrack)
      .def("GetTimestamp", &theia::View::GetTimestamp)
      .def("Camera", &theia::View::Camera, "Camera class object")
      .def("CameraIntrinsicsPrior", &theia::View::CameraIntrinsicsPrior)
      .def("SetCameraIntrinsicsPrior", &theia::View::SetCameraIntrinsicsPrior)
      .def("MutableCameraIntrinsicsPrior",
           &theia::View::MutableCameraIntrinsicsPrior,
           py::return_value_policy::reference)
      .def("MutableCamera",
           &theia::View::MutableCamera,
           py::return_value_policy::reference_internal)
      .def("GetPositionPrior", &theia::View::GetPositionPrior)
      .def("GetPositionPriorSqrtInformation",
           &theia::View::GetPositionPriorSqrtInformation)
      .def("SetPositionPrior", &theia::View::SetPositionPrior)
      .def("HasPositionPrior", &theia::View::HasPositionPrior)
      .def("GetGravityPrior", &theia::View::GetGravityPrior)
      .def("GetGravityPriorSqrtInformation",
           &theia::View::GetGravityPriorSqrtInformation)
      .def("SetGravityPrior", &theia::View::SetGravityPrior)
      .def("HasGravityPrior", &theia::View::HasGravityPrior)
      .def("GetOrientationPrior", &theia::View::GetOrientationPrior)
      .def("GetOrientationPriorSqrtInformation",
           &theia::View::GetOrientationPriorSqrtInformation)
      .def("SetOrientationPrior", &theia::View::SetOrientationPrior)
      .def("HasOrientationPrior", &theia::View::HasOrientationPrior);

  // Visibility pyramid
  py::class_<theia::VisibilityPyramid>(m, "VisibilityPyramid",
                                       "Pyramid for visibility scoring (view overlap).")
      .def(py::init<int, int, int>())
      .def("AddPoint", &theia::VisibilityPyramid::AddPoint)
      .def("ComputeScore", &theia::VisibilityPyramid::ComputeScore);

  // Track class
  py::class_<theia::Track>(m, "Track",
                           "3D point track: view IDs, 3D point, color, estimated flag.")
      .def(py::init<>())
      .def("SetIsEstimated", &theia::Track::SetEstimated)
      .def("IsEstimated", &theia::Track::IsEstimated)
      .def("NumViews", &theia::Track::NumViews)
      .def("AddView", &theia::Track::AddView)
      .def("RemoveView", &theia::Track::RemoveView)
      .def("ViewIds", &theia::Track::ViewIds)
      .def("Point", &theia::Track::Point)
      .def("SetPoint", &theia::Track::SetPoint)
      .def("Color", &theia::Track::Color)
      .def("SetColor", &theia::Track::SetColor)
      .def("ReferenceViewId", &theia::Track::ReferenceViewId)
      .def("InverseDepth", &theia::Track::InverseDepth)
      .def("SetInverseDepth", &theia::Track::SetInverseDepth)
      .def("SetReferenceBearingVector",
           &theia::Track::SetReferenceBearingVector)
      .def("ReferenceBearingVector", &theia::Track::ReferenceBearingVector)
      .def("SetReferenceDescriptor", &theia::Track::SetReferenceDescriptor)
      .def("ReferenceDescriptor", &theia::Track::ReferenceDescriptor);

  // Track builder class
  py::class_<theia::TrackBuilder>(m, "TrackBuilder",
                                  "Build tracks from feature correspondences across views.")
      .def(py::init<int, int>())
      .def("AddFeatureCorrespondence",
           &theia::TrackBuilder::AddFeatureCorrespondence)
      .def("BuildTracks", &theia::TrackBuilder::BuildTracks)
      .def("BuildTracksIncremental",
           &theia::TrackBuilder::BuildTracksIncremental);

  py::class_<theia::BundleAdjustmentOptions>(m, "BundleAdjustmentOptions",
                                            "Bundle adjustment options (loss, solver, priors, tolerances).")
      .def(py::init<>())
      .def_readwrite("loss_function_type",
                     &theia::BundleAdjustmentOptions::loss_function_type)
      .def_readwrite("robust_loss_width",
                     &theia::BundleAdjustmentOptions::robust_loss_width)
      .def_readwrite("linear_solver_type",
                     &theia::BundleAdjustmentOptions::linear_solver_type)
      .def_readwrite("preconditioner_type",
                     &theia::BundleAdjustmentOptions::preconditioner_type)
      .def_readwrite("visibility_clustering_type",
                     &theia::BundleAdjustmentOptions::visibility_clustering_type)
      .def_readwrite("verbose", &theia::BundleAdjustmentOptions::verbose)
      .def_readwrite(
          "constant_camera_orientation",
          &theia::BundleAdjustmentOptions::constant_camera_orientation)
      .def_readwrite("constant_camera_position",
                     &theia::BundleAdjustmentOptions::constant_camera_position)
      .def_readwrite("intrinsics_to_optimize",
                     &theia::BundleAdjustmentOptions::intrinsics_to_optimize)
      .def_readwrite("num_threads",
                     &theia::BundleAdjustmentOptions::num_threads)
      .def_readwrite("max_num_iterations",
                     &theia::BundleAdjustmentOptions::max_num_iterations)
      .def_readwrite(
          "max_solver_time_in_seconds",
          &theia::BundleAdjustmentOptions::max_solver_time_in_seconds)
      .def_readwrite("use_inner_iterations",
                     &theia::BundleAdjustmentOptions::use_inner_iterations)
      .def_readwrite("function_tolerance",
                     &theia::BundleAdjustmentOptions::function_tolerance)
      .def_readwrite("gradient_tolerance",
                     &theia::BundleAdjustmentOptions::gradient_tolerance)
      .def_readwrite("parameter_tolerance",
                     &theia::BundleAdjustmentOptions::parameter_tolerance)
      .def_readwrite("max_trust_region_radius",
                     &theia::BundleAdjustmentOptions::max_trust_region_radius)
      .def_readwrite("use_position_priors",
                     &theia::BundleAdjustmentOptions::use_position_priors)
      .def_readwrite("use_orientation_priors",
                     &theia::BundleAdjustmentOptions::use_orientation_priors)
      .def_readwrite("use_gravity_priors",
                     &theia::BundleAdjustmentOptions::use_gravity_priors)
      .def_readwrite("use_depth_priors",
                     &theia::BundleAdjustmentOptions::use_depth_priors)
      .def_readwrite("orthographic_camera",
                     &theia::BundleAdjustmentOptions::orthographic_camera)
      .def_readwrite("use_homogeneous_point_parametrization",
                     &theia::BundleAdjustmentOptions::
                         use_homogeneous_point_parametrization)
      .def_readwrite("robust_loss_width_depth_prior",
                     &theia::BundleAdjustmentOptions::robust_loss_width_depth_prior)
      .def_readwrite(
          "use_inverse_depth_parametrization",
          &theia::BundleAdjustmentOptions::use_inverse_depth_parametrization)
      .def_readwrite("use_mixed_precision_solves",
                     &theia::BundleAdjustmentOptions::use_mixed_precision_solves)
      .def_readwrite("max_num_refinement_iterations",
                     &theia::BundleAdjustmentOptions::max_num_refinement_iterations)
      .def_readwrite("dense_linear_algebra_library_type",
                     &theia::BundleAdjustmentOptions::dense_linear_algebra_library_type)
      .def_readwrite("sparse_linear_algebra_library_type",
                     &theia::BundleAdjustmentOptions::sparse_linear_algebra_library_type)
      .def_readwrite("optimize_for_forward_facing_trajectory",
                     &theia::BundleAdjustmentOptions::optimize_for_forward_facing_trajectory);

  // Reconstruction Options
  py::enum_<theia::TriangulationMethodType>(m, "TriangulationMethodType",
                                            "Triangulation method: MIDPOINT, SVD, L2_MINIMIZATION.")
      .value("MIDPOINT", theia::TriangulationMethodType::MIDPOINT)
      .value("SVD", theia::TriangulationMethodType::SVD)
      .value("L2_MINIMIZATION", theia::TriangulationMethodType::L2_MINIMIZATION)
      .export_values();

  // Track Estimator Options
  py::class_<theia::TrackEstimator::Options>(m, "TrackEstimatorOptions",
                                             "Options for track estimation (reprojection error, triangulation method).")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::TrackEstimator::Options::num_threads)
      .def_readwrite("max_acceptable_reprojection_error_pixels",
                     &theia::TrackEstimator::Options::
                         max_acceptable_reprojection_error_pixels)
      .def_readwrite(
          "min_triangulation_angle_degrees",
          &theia::TrackEstimator::Options::min_triangulation_angle_degrees)
      .def_readwrite("bundle_adjustment",
                     &theia::TrackEstimator::Options::bundle_adjustment)
      .def_readwrite("multithreaded_step_size",
                     &theia::TrackEstimator::Options::multithreaded_step_size)
      .def_readwrite("triangulation_method",
                     &theia::TrackEstimator::Options::triangulation_method);

  // Track Estimator Summary
  py::class_<theia::TrackEstimator::Summary>(m, "TrackEstimatorSummary",
                                             "Summary of track estimation (attempts, estimated count).")
      .def_readwrite(
          "input_num_estimated_tracks",
          &theia::TrackEstimator::Summary::input_num_estimated_tracks)
      .def_readwrite(
          "num_triangulation_attempts",
          &theia::TrackEstimator::Summary::num_triangulation_attempts)
      .def_readwrite("estimated_tracks",
                     &theia::TrackEstimator::Summary::estimated_tracks);

  // Track Estimator class
  py::class_<theia::TrackEstimator>(m, "TrackEstimator",
                                    "Estimate 3D tracks from views and correspondences.")
      .def(py::init<const theia::TrackEstimator::Options&,
                    theia::Reconstruction*>())
      .def("EstimateAllTracks", &theia::TrackEstimator::EstimateAllTracks)
      .def("EstimateTracks", &theia::TrackEstimator::EstimateTracks);

  // ReconstructionEstimatorSummary
  py::class_<theia::ReconstructionEstimatorSummary>(
      m, "ReconstructionEstimatorSummary",
      "Summary of reconstruction estimation (success, timings, message).")
      .def_readwrite("success", &theia::ReconstructionEstimatorSummary::success)
      .def_readwrite("estimated_tracks",
                     &theia::ReconstructionEstimatorSummary::estimated_tracks)
      .def_readwrite("estimated_views",
                     &theia::ReconstructionEstimatorSummary::estimated_views)
      .def_readwrite("camera_intrinsics_calibration_time",
                     &theia::ReconstructionEstimatorSummary::
                         camera_intrinsics_calibration_time)
      .def_readwrite(
          "pose_estimation_time",
          &theia::ReconstructionEstimatorSummary::pose_estimation_time)
      .def_readwrite("triangulation_time",
                     &theia::ReconstructionEstimatorSummary::triangulation_time)
      .def_readwrite(
          "bundle_adjustment_time",
          &theia::ReconstructionEstimatorSummary::bundle_adjustment_time)
      .def_readwrite("total_time",
                     &theia::ReconstructionEstimatorSummary::total_time)
      .def_readwrite("message",
                     &theia::ReconstructionEstimatorSummary::message);

  // Reconstruction Estimator class
  py::class_<theia::ReconstructionEstimator>(m, "ReconstructionEstimator",
                                             "Base class for SfM reconstruction estimators.")
      .def_static("Create",
                  &theia::ReconstructionEstimator::Create,
                  py::return_value_policy::reference);

  py::class_<theia::GlobalReconstructionEstimator,
             theia::ReconstructionEstimator>(m, "GlobalReconstructionEstimator",
                                             "Global SfM: estimate all poses then triangulate.")
      .def(py::init<theia::ReconstructionEstimatorOptions>())
      .def("Estimate", &theia::GlobalReconstructionEstimator::Estimate);

  py::class_<theia::IncrementalReconstructionEstimator,
             theia::ReconstructionEstimator>(
      m, "IncrementalReconstructionEstimator",
      "Incremental SfM: add views and tracks step by step.")
      .def(py::init<theia::ReconstructionEstimatorOptions>())
      .def("Estimate", &theia::IncrementalReconstructionEstimator::Estimate);

  py::class_<theia::HybridReconstructionEstimator,
             theia::ReconstructionEstimator>(m, "HybridReconstructionEstimator",
                                            "Hybrid SfM: combine global and incremental strategies.")
      .def(py::init<theia::ReconstructionEstimatorOptions>())
      .def("Estimate", &theia::HybridReconstructionEstimator::Estimate);

  py::enum_<theia::ReconstructionEstimatorType>(m,
                                                "ReconstructionEstimatorType",
                                                "SfM strategy: GLOBAL, INCREMENTAL, HYBRID.")
      .value("GLOBAL", theia::ReconstructionEstimatorType::GLOBAL)
      .value("INCREMENTAL", theia::ReconstructionEstimatorType::INCREMENTAL)
      .value("HYBRID", theia::ReconstructionEstimatorType::HYBRID)
      .export_values();

  py::enum_<theia::GlobalPositionEstimatorType>(m,
                                                "GlobalPositionEstimatorType",
                                                "Global position estimator: NONLINEAR, LINEAR_TRIPLET, LEAST_UNSQUARED_DEVIATION, LIGT.")
      .value("NONLINEAR", theia::GlobalPositionEstimatorType::NONLINEAR)
      .value("LINEAR_TRIPLET",
             theia::GlobalPositionEstimatorType::LINEAR_TRIPLET)
      .value("LEAST_UNSQUARED_DEVIATION",
             theia::GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION)
      .value("LIGT", theia::GlobalPositionEstimatorType::LIGT)
      .export_values();

  py::enum_<theia::GlobalRotationEstimatorType>(m,
                                                "GlobalRotationEstimatorType",
                                                "Global rotation estimator: ROBUST_L1L2, NONLINEAR, LINEAR, LAGRANGE_DUAL, HYBRID.")
      .value("ROBUST_L1L2", theia::GlobalRotationEstimatorType::ROBUST_L1L2)
      .value("NONLINEAR", theia::GlobalRotationEstimatorType::NONLINEAR)
      .value("LINEAR", theia::GlobalRotationEstimatorType::LINEAR)
      .value("LAGRANGE_DUAL", theia::GlobalRotationEstimatorType::LAGRANGE_DUAL)
      .value("HYBRID", theia::GlobalRotationEstimatorType::HYBRID)
      .export_values();

  py::class_<theia::ReconstructionEstimatorOptions>(
      m, "ReconstructionEstimatorOptions",
      "Options for reconstruction estimation (strategy, RANSAC, BA, triangulation).")
      .def(py::init<>())

      .def_readwrite(
          "reconstruction_estimator_type",
          &theia::ReconstructionEstimatorOptions::reconstruction_estimator_type)
      .def_readwrite("global_position_estimator_type",
                     &theia::ReconstructionEstimatorOptions::
                         global_position_estimator_type)
      .def_readwrite("global_rotation_estimator_type",
                     &theia::ReconstructionEstimatorOptions::
                         global_rotation_estimator_type)
      .def_readwrite("num_threads",
                     &theia::ReconstructionEstimatorOptions::num_threads)
      .def_readwrite("max_reprojection_error_in_pixels",
                     &theia::ReconstructionEstimatorOptions::
                         max_reprojection_error_in_pixels)
      .def_readwrite(
          "min_num_two_view_inliers",
          &theia::ReconstructionEstimatorOptions::min_num_two_view_inliers)
      .def_readwrite("ransac_confidence",
                     &theia::ReconstructionEstimatorOptions::ransac_confidence)
      .def_readwrite(
          "ransac_min_iterations",
          &theia::ReconstructionEstimatorOptions::ransac_min_iterations)
      .def_readwrite(
          "ransac_max_iterations",
          &theia::ReconstructionEstimatorOptions::ransac_max_iterations)
      .def_readwrite("ransac_use_mle",
                     &theia::ReconstructionEstimatorOptions::ransac_use_mle)
      .def_readwrite("ransac_use_lo",
                     &theia::ReconstructionEstimatorOptions::ransac_use_lo)
      .def_readwrite("ransac_lo_start_iterations",
                        &theia::ReconstructionEstimatorOptions::ransac_lo_start_iterations)
      .def_readwrite("rotation_filtering_max_difference_degrees",
                     &theia::ReconstructionEstimatorOptions::
                         rotation_filtering_max_difference_degrees)
      .def_readwrite("refine_relative_translations_after_rotation_estimation",
                     &theia::ReconstructionEstimatorOptions::
                         refine_relative_translations_after_rotation_estimation)
      .def_readwrite("extract_maximal_rigid_subgraph",
                     &theia::ReconstructionEstimatorOptions::
                         extract_maximal_rigid_subgraph)
      .def_readwrite("filter_relative_translations_with_1dsfm",
                     &theia::ReconstructionEstimatorOptions::
                         filter_relative_translations_with_1dsfm)
      .def_readwrite("translation_filtering_num_iterations",
                     &theia::ReconstructionEstimatorOptions::
                         translation_filtering_num_iterations)
      .def_readwrite("translation_filtering_projection_tolerance",
                     &theia::ReconstructionEstimatorOptions::
                         translation_filtering_projection_tolerance)
      .def_readwrite("rotation_estimation_robust_loss_scale",
                     &theia::ReconstructionEstimatorOptions::
                         rotation_estimation_robust_loss_scale)
      .def_readwrite("nonlinear_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         nonlinear_position_estimator_options)
      .def_readwrite("linear_triplet_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         linear_triplet_position_estimator_options)
      .def_readwrite("least_unsquared_deviation_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         least_unsquared_deviation_position_estimator_options)
      .def_readwrite(
          "refine_camera_positions_and_points_after_position_estimation",
          &theia::ReconstructionEstimatorOptions::
              refine_camera_positions_and_points_after_position_estimation)
      .def_readwrite("multiple_view_localization_ratio",
                     &theia::ReconstructionEstimatorOptions::
                         multiple_view_localization_ratio)
      .def_readwrite("absolute_pose_reprojection_error_threshold",
                     &theia::ReconstructionEstimatorOptions::
                         absolute_pose_reprojection_error_threshold)
      .def_readwrite(
          "min_num_absolute_pose_inliers",
          &theia::ReconstructionEstimatorOptions::min_num_absolute_pose_inliers)
      .def_readwrite("full_bundle_adjustment_growth_percent",
                     &theia::ReconstructionEstimatorOptions::
                         full_bundle_adjustment_growth_percent)
      .def_readwrite("partial_bundle_adjustment_num_views",
                     &theia::ReconstructionEstimatorOptions::
                         partial_bundle_adjustment_num_views)
      .def_readwrite("relative_position_estimation_max_sampson_error_pixels",
                     &theia::ReconstructionEstimatorOptions::
                         relative_position_estimation_max_sampson_error_pixels)
      .def_readwrite("min_triangulation_angle_degrees",
                     &theia::ReconstructionEstimatorOptions::
                         min_triangulation_angle_degrees)
      .def_readwrite(
          "bundle_adjust_tracks",
          &theia::ReconstructionEstimatorOptions::bundle_adjust_tracks)
      .def_readwrite(
          "triangulation_method",
          &theia::ReconstructionEstimatorOptions::triangulation_method)
      .def_readwrite("num_retriangulation_iterations",
                     &theia::ReconstructionEstimatorOptions::
                         num_retriangulation_iterations)
      .def_readwrite("bundle_adjustment_loss_function_type",
                     &theia::ReconstructionEstimatorOptions::
                         bundle_adjustment_loss_function_type)
      .def_readwrite("bundle_adjustment_robust_loss_width",
                     &theia::ReconstructionEstimatorOptions::
                         bundle_adjustment_robust_loss_width)
      .def_readwrite("min_cameras_for_iterative_solver",
                     &theia::ReconstructionEstimatorOptions::
                         min_cameras_for_iterative_solver)
      .def_readwrite("intrinsics_to_optimize",
          &theia::ReconstructionEstimatorOptions::intrinsics_to_optimize)
      .def_readwrite("subsample_tracks_for_bundle_adjustment",
                     &theia::ReconstructionEstimatorOptions::
                         subsample_tracks_for_bundle_adjustment)
      .def_readwrite("track_subset_selection_long_track_length_threshold",
                     &theia::ReconstructionEstimatorOptions::
                         track_subset_selection_long_track_length_threshold)
      .def_readwrite("track_selection_image_grid_cell_size_pixels",
                     &theia::ReconstructionEstimatorOptions::
                         track_selection_image_grid_cell_size_pixels)
      .def_readwrite("min_num_optimized_tracks_per_view",
                     &theia::ReconstructionEstimatorOptions::
                         min_num_optimized_tracks_per_view)
      .def_readwrite("track_parametrization_type",
          &theia::ReconstructionEstimatorOptions::track_parametrization_type)
      .def_readwrite("localization_pnp_type",
                     &theia::ReconstructionEstimatorOptions::localization_pnp_type)
      .def_readwrite("dense_linear_algebra_library_type",
                     &theia::ReconstructionEstimatorOptions::dense_linear_algebra_library_type)
      .def_readwrite("sparse_linear_algebra_library_type",
                     &theia::ReconstructionEstimatorOptions::sparse_linear_algebra_library_type)
      .def_readwrite("linear_solver_type",
                     &theia::ReconstructionEstimatorOptions::linear_solver_type)
      .def_readwrite("preconditioner_type",
                     &theia::ReconstructionEstimatorOptions::preconditioner_type)
      .def_readwrite("visibility_clustering_type",
                     &theia::ReconstructionEstimatorOptions::visibility_clustering_type)
      .def_readwrite("use_inner_iterations",
                     &theia::ReconstructionEstimatorOptions::use_inner_iterations)
      .def_readwrite("max_num_iterations",
                     &theia::ReconstructionEstimatorOptions::max_num_iterations)
      .def_readwrite("optimize_for_forward_facing_trajectory",
                     &theia::ReconstructionEstimatorOptions::optimize_for_forward_facing_trajectory);
  // Reconstruction class
  py::class_<theia::Reconstruction>(m, "Reconstruction",
                                    "SfM reconstruction: views, tracks, cameras.")
      .def(py::init<>())
      .def("NumViews", &theia::Reconstruction::NumViews)
      .def("ViewIdFromName", &theia::Reconstruction::ViewIdFromName)
      .def("AddView",
           (theia::ViewId(theia::Reconstruction::*)(const std::string&,
                                                    const double)) &
               theia::Reconstruction::AddView,
           py::return_value_policy::reference_internal)
      .def("AddView",
           (theia::ViewId(theia::Reconstruction::*)(
               const std::string&,
               const theia::CameraIntrinsicsGroupId,
               const double)) &
               theia::Reconstruction::AddView,
           py::return_value_policy::reference_internal)
      .def("RemoveView", &theia::Reconstruction::RemoveView)
      .def("ViewIds", &theia::Reconstruction::ViewIds)
      .def("NumTracks", &theia::Reconstruction::NumTracks)
      //   .def("AddTrack",
      //        (theia::TrackId(theia::Reconstruction::*)()) &
      //            theia::Reconstruction::AddTrack,
      //        py::return_value_policy::reference_internal)
      //   .def("AddTrack",
      //        ((theia::Reconstruction::*)(const theia::TrackId&)) &
      //            theia::Reconstruction::AddTrack,
      //        py::return_value_policy::reference_internal)
      //   .def("AddTrack",
      //        (theia::TrackId(theia::Reconstruction::*)()) &
      //            theia::Reconstruction::AddTrack,
      //        py::return_value_policy::reference_internal)
      //   .def("AddTrack",
      //        ((theia::Reconstruction::*)(const theia::TrackId&)) &
      //            theia::Reconstruction::AddTrack,
      //        py::return_value_policy::reference_internal)
      .def("AddTrack",
           static_cast<theia::TrackId (theia::Reconstruction::*)()>(
               &theia::Reconstruction::AddTrack))
      .def("AddTrack",
           static_cast<void (theia::Reconstruction::*)(const theia::TrackId&)>(
               &theia::Reconstruction::AddTrack))
      .def("AddTrack",
           static_cast<theia::TrackId (theia::Reconstruction::*)(
               const std::vector<std::pair<theia::ViewId, theia::Feature>>&)>(
               &theia::Reconstruction::AddTrack))
      .def("RemoveTrack", &theia::Reconstruction::RemoveTrack)
      .def("TrackIds", &theia::Reconstruction::TrackIds)
      .def("AddObservation", &theia::Reconstruction::AddObservation)
      .def("NumCameraIntrinsicGroups",
           &theia::Reconstruction::NumCameraIntrinsicGroups)
      .def("Normalize", &theia::Reconstruction::Normalize)
      .def("CameraIntrinsicsGroupIdFromViewId",
           &theia::Reconstruction::CameraIntrinsicsGroupIdFromViewId)
      .def("CameraIntrinsicsGroupIds",
           &theia::Reconstruction::CameraIntrinsicsGroupIds)
      .def("View",
           &theia::Reconstruction::View,
           py::return_value_policy::reference_internal)
      .def("MutableView",
           &theia::Reconstruction::MutableView,
           py::return_value_policy::reference_internal)
      .def("Track",
           &theia::Reconstruction::Track,
           py::return_value_policy::reference_internal)
      .def("MutableTrack",
           &theia::Reconstruction::MutableTrack,
           py::return_value_policy::reference_internal)
      .def("GetViewsInCameraIntrinsicGroup",
           &theia::Reconstruction::GetViewsInCameraIntrinsicGroup)
      .def("InitializeInverseDepth",
           &theia::Reconstruction::InitializeInverseDepth)
      //.def("GetSubReconstruction",
      //&theia::Reconstruction::GetSubReconstructionWrapper)
      ;

  // Reconstruction Estimator Helpers
  m.def("SetUnderconstrainedTracksToUnestimated",
        &theia::SetUnderconstrainedTracksToUnestimated,
        "Mark underconstrained tracks as unestimated.");
  m.def("SetUnderconstrainedViewsToUnestimated",
        &theia::SetUnderconstrainedViewsToUnestimated,
        "Mark underconstrained views as unestimated.");
  m.def("NumEstimatedViews", &theia::NumEstimatedViews,
        "Return number of estimated views in reconstruction.");
  m.def("NumEstimatedTracks", &theia::NumEstimatedTracks,
        "Return number of estimated tracks in reconstruction.");
  m.def("SetReconstructionFromEstimatedPoses",
        &theia::SetReconstructionFromEstimatedPoses,
        "Set camera poses in reconstruction from estimated poses.");
  m.def("CreateEstimatedSubreconstruction",
        &theia::CreateEstimatedSubreconstruction,
        "Build subreconstruction containing only estimated views and tracks.");
  m.def("RelativeRotationsFromViewGraph",
        &theia::RelativeRotationsFromViewGraph,
        "Extract relative rotations from view graph.");
  m.def("GetEstimatedViewsFromReconstruction",
        &theia::GetEstimatedViewsFromReconstructionWrapper,
        "Return list of estimated view IDs.");
  m.def("GetEstimatedTracksFromReconstruction",
        &theia::GetEstimatedTracksFromReconstructionWrapper,
        "Return list of estimated track IDs.");
        
  py::class_<theia::TwoViewInfo>(m, "TwoViewInfo",
                                 "Two-view geometry: focal lengths, relative rotation and position, match count.")
      .def(py::init<>())
      .def_readwrite("focal_length_1", &theia::TwoViewInfo::focal_length_1)
      .def_readwrite("focal_length_2", &theia::TwoViewInfo::focal_length_2)
      .def_readwrite("position_2", &theia::TwoViewInfo::position_2)
      .def_readwrite("rotation_2", &theia::TwoViewInfo::rotation_2)
      .def_readwrite("num_verified_matches",
                     &theia::TwoViewInfo::num_verified_matches)
      .def_readwrite("num_homography_inliers",
                     &theia::TwoViewInfo::num_homography_inliers)
      .def_readwrite("visibility_score", &theia::TwoViewInfo::visibility_score)
      .def_readwrite("scale_estimate", &theia::TwoViewInfo::scale_estimate);

  m.def("SwapCameras", &theia::SwapCameras,
        "Swap camera indices in two-view info.");

  py::class_<theia::ViewGraph>(m, "ViewGraph",
                               "Graph of view pairs with two-view geometry (edges = TwoViewInfo).")
      .def(py::init<>())
      //.def_property_readonly("Name", &theia::View::Name)
      .def("ReadFromDisk", &theia::ViewGraph::ReadFromDisk)
      .def("WriteToDisk", &theia::ViewGraph::WriteToDisk)
      .def("HasView", &theia::ViewGraph::HasView)
      .def("RemoveView", &theia::ViewGraph::RemoveView)
      .def("HasEdge", &theia::ViewGraph::HasEdge)
      .def("AddEdge", &theia::ViewGraph::AddEdge)
      .def("RemoveEdge", &theia::ViewGraph::RemoveEdge)
      .def("NumViews", &theia::ViewGraph::NumViews)
      .def("NumEdges", &theia::ViewGraph::NumEdges)
      .def("GetNeighborIdsForView",
           &theia::ViewGraph::GetNeighborIdsForView,
           py::return_value_policy::reference)
      .def("GetEdge",
           &theia::ViewGraph::GetEdge,
           py::return_value_policy::reference)
      .def("GetAllEdges", &theia::ViewGraph::GetAllEdges)

      // not sure pointer as input
      //.def("ExtractSubgraph", &theia::ViewGraph::ExtractSubgraph)
      //.def("GetLargestConnectedComponentIds",
      //&theia::ViewGraph::GetLargestConnectedComponentIds)

      ;

  py::class_<theia::GPSConverter>(m, "GPSConverter",
                                 "Convert between ECEF and LLA (lat/lon/alt) coordinates.")
      .def(py::init<>())
      .def_static("ECEFToLLA", theia::GPSConverter::ECEFToLLA)
      .def_static("LLAToECEF", theia::GPSConverter::LLAToECEF);

  // Bundle Adjustment
  // Expose selected Ceres enums to configure solver from Python.
  py::enum_<ceres::LinearSolverType>(m, "LinearSolverType")
      .value("DENSE_QR", ceres::LinearSolverType::DENSE_QR)
      .value("DENSE_NORMAL_CHOLESKY", ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY)
      .value("DENSE_SCHUR", ceres::LinearSolverType::DENSE_SCHUR)
      .value("SPARSE_NORMAL_CHOLESKY", ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY)
      .value("SPARSE_SCHUR", ceres::LinearSolverType::SPARSE_SCHUR)
      .value("ITERATIVE_SCHUR", ceres::LinearSolverType::ITERATIVE_SCHUR)
      .value("CGNR", ceres::LinearSolverType::CGNR)
      .export_values();

  py::enum_<ceres::PreconditionerType>(m, "PreconditionerType")
      .value("IDENTITY", ceres::PreconditionerType::IDENTITY)
      .value("JACOBI", ceres::PreconditionerType::JACOBI)
      .value("SCHUR_JACOBI", ceres::PreconditionerType::SCHUR_JACOBI)
      .value("CLUSTER_JACOBI", ceres::PreconditionerType::CLUSTER_JACOBI)
      .value("CLUSTER_TRIDIAGONAL", ceres::PreconditionerType::CLUSTER_TRIDIAGONAL)
      .export_values();

  py::enum_<ceres::VisibilityClusteringType>(m, "VisibilityClusteringType")
      .value("SINGLE_LINKAGE", ceres::VisibilityClusteringType::SINGLE_LINKAGE)
      .value("CANONICAL_VIEWS", ceres::VisibilityClusteringType::CANONICAL_VIEWS)
      .export_values();

  py::enum_<ceres::DenseLinearAlgebraLibraryType>(m, "DenseLinearAlgebraLibraryType")
      .value("EIGEN", ceres::DenseLinearAlgebraLibraryType::EIGEN)
      .value("LAPACK", ceres::DenseLinearAlgebraLibraryType::LAPACK)
#if THEIA_CERES_HAS_CUDA_DENSE
      .value("CUDA", ceres::DenseLinearAlgebraLibraryType::CUDA)
#endif
      .export_values();

  py::enum_<ceres::SparseLinearAlgebraLibraryType>(m, "SparseLinearAlgebraLibraryType")
      .value("SUITE_SPARSE", ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE)
      .value("EIGEN_SPARSE", ceres::SparseLinearAlgebraLibraryType::EIGEN_SPARSE)
      .value("ACCELERATE_SPARSE", ceres::SparseLinearAlgebraLibraryType::ACCELERATE_SPARSE)
#if THEIA_CERES_HAS_CUDA_SPARSE
      .value("CUDA_SPARSE", ceres::SparseLinearAlgebraLibraryType::CUDA_SPARSE)
#endif
      .export_values();

  py::enum_<theia::OptimizeIntrinsicsType>(m, "OptimizeIntrinsicsType")
      .value("NONE", theia::OptimizeIntrinsicsType::NONE)
      .value("FOCAL_LENGTH", theia::OptimizeIntrinsicsType::FOCAL_LENGTH)
      .value("ASPECT_RATIO", theia::OptimizeIntrinsicsType::ASPECT_RATIO)
      .value("SKEW", theia::OptimizeIntrinsicsType::SKEW)
      .value("PRINCIPAL_POINTS",
             theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS)
      .value("RADIAL_DISTORTION",
             theia::OptimizeIntrinsicsType::RADIAL_DISTORTION)
      .value("TANGENTIAL_DISTORTION",
             theia::OptimizeIntrinsicsType::TANGENTIAL_DISTORTION)
      .value("DISTORTION", theia::OptimizeIntrinsicsType::DISTORTION)
      .value("FOCAL_LENGTH_DISTORTION",
             theia::OptimizeIntrinsicsType::FOCAL_LENGTH_DISTORTION)
      .value("FOCAL_LENGTH_RADIAL_DISTORTION",
             theia::OptimizeIntrinsicsType::FOCAL_LENGTH_RADIAL_DISTORTION)
      .value("ALL", theia::OptimizeIntrinsicsType::ALL)
      .export_values();

  py::enum_<theia::LossFunctionType>(m, "LossFunctionType")
      .value("TRIVIAL", theia::LossFunctionType::TRIVIAL)
      .value("HUBER", theia::LossFunctionType::HUBER)
      .value("SOFTLONE", theia::LossFunctionType::SOFTLONE)
      .value("CAUCHY", theia::LossFunctionType::CAUCHY)
      .value("ARCTAN", theia::LossFunctionType::ARCTAN)
      .value("TUKEY", theia::LossFunctionType::TUKEY)
      .export_values();

  py::enum_<theia::TrackParametrizationType>(m, "TrackParametrizationType")
      .value("XYZW", theia::TrackParametrizationType::XYZW)
      .value("XYZW_MANIFOLD", theia::TrackParametrizationType::XYZW_MANIFOLD)
      .value("INVERSE_DEPTH", theia::TrackParametrizationType::INVERSE_DEPTH)
      .export_values();

  py::class_<theia::TwoViewBundleAdjustmentOptions>(
      m, "TwoViewBundleAdjustmentOptions",
      "Options for two-view bundle adjustment.")
      .def(py::init<>())
      .def_readwrite("ba_options",
                     &theia::TwoViewBundleAdjustmentOptions::ba_options)
      .def_readwrite(
          "constant_camera1_intrinsics",
          &theia::TwoViewBundleAdjustmentOptions::constant_camera1_intrinsics)
      .def_readwrite(
          "constant_camera2_intrinsics",
          &theia::TwoViewBundleAdjustmentOptions::constant_camera2_intrinsics);

  py::class_<theia::BundleAdjustmentSummary>(m, "BundleAdjustmentSummary",
                                             "Summary of bundle adjustment (success, initial/final cost, timings).")
      .def(py::init<>())
      .def_readwrite("success", &theia::BundleAdjustmentSummary::success)
      .def_readwrite("initial_cost",
                     &theia::BundleAdjustmentSummary::initial_cost)
      .def_readwrite("final_cost", &theia::BundleAdjustmentSummary::final_cost)
      .def_readwrite("setup_time_in_seconds",
                     &theia::BundleAdjustmentSummary::setup_time_in_seconds)
      .def_readwrite("solve_time_in_seconds",
                     &theia::BundleAdjustmentSummary::solve_time_in_seconds);

  m.def("BundleAdjustPartialReconstruction",
        theia::BundleAdjustPartialReconstructionWrapper,
        "Bundle adjustment over a subset of views and tracks.");
  m.def("BundleAdjustPartialViewsConstant",
        theia::BundleAdjustPartialViewsConstantWrapper,
        "Bundle adjustment with some views held constant.");
  m.def("BundleAdjustReconstruction", theia::BundleAdjustReconstructionWrapper,
        "Full bundle adjustment of reconstruction.");
  m.def("BundleAdjustView", theia::BundleAdjustViewWrapper,
        "Bundle adjustment for a single view.");
  m.def("BundleAdjustViews", theia::BundleAdjustViewsWrapper,
        "Bundle adjustment for a set of views.");
  m.def("BundleAdjustViewWithCov", theia::BundleAdjustViewWithCovWrapper,
        "Bundle adjustment for one view with covariance output.");
  m.def("BundleAdjustViewsWithCov", theia::BundleAdjustViewsWithCovWrapper,
        "Bundle adjustment for views with covariance output.");

  m.def("BundleAdjustTrack", theia::BundleAdjustTrackWrapper,
        "Bundle adjustment for a single track.");
  m.def("BundleAdjustTracks", theia::BundleAdjustTracksWrapper,
        "Bundle adjustment for a set of tracks.");
  m.def("BundleAdjustTrackWithCov", theia::BundleAdjustTrackWithCovWrapper,
        "Bundle adjustment for one track with covariance output.");
  m.def("BundleAdjustTracksWithCov", theia::BundleAdjustTracksWithCovWrapper,
        "Bundle adjustment for tracks with covariance output.");
  m.def("BundleAdjustTwoViewsAngular",
        theia::BundleAdjustTwoViewsAngularWrapper,
        "Two-view bundle adjustment using angular reprojection error.");
  m.def("OptimizeRelativePositionWithKnownRotation",
        theia::OptimizeRelativePositionWithKnownRotationWrapper,
        "Optimize relative translation with rotation fixed.");

  m.def("OptimizeAbsolutePoseOnNormFeatures",
        theia::OptimizeAbsolutePoseOnNormFeatures,
        "Refine absolute pose on normalized bearing vectors.");
  py::class_<theia::BundleAdjuster>(m, "BundleAdjuster",
                                   "Incremental bundle adjuster (add views/tracks, optimize).")
      // constructor uses pointer of an object as input
      //.def(py::init<theia::BundleAdjustmentOptions, theia::Reconstruction>())
      .def("AddView", &theia::BundleAdjuster::AddView)
      .def("AddTrack", &theia::BundleAdjuster::AddTrack)
      .def("Optimize", &theia::BundleAdjuster::Optimize)
      //.def("SetCameraExtrinsicsParameterization",
      //&theia::BundleAdjuster::SetCameraExtrinsicsParameterization)
      //.def("SetCameraIntrinsicsParameterization",
      //&theia::BundleAdjuster::SetCameraIntrinsicsParameterization)
      //.def("SetCameraExtrinsicsConstant",
      //&theia::BundleAdjuster::SetCameraExtrinsicsConstant)
      //.def("SetCameraPositionConstant",
      //&theia::BundleAdjuster::SetCameraPositionConstant)
      //.def("SetCameraOrientationConstant",
      //&theia::BundleAdjuster::SetCameraOrientationConstant)
      //.def("SetTrackConstant", &theia::BundleAdjuster::SetTrackConstant)
      //.def("SetTrackVariable", &theia::BundleAdjuster::SetTrackVariable)
      //.def("SetCameraSchurGroups",
      //&theia::BundleAdjuster::SetCameraSchurGroups) .def("SetTrackSchurGroup",
      //&theia::BundleAdjuster::SetTrackSchurGroup)
      //.def("AddReprojectionErrorResidual",
      //&theia::BundleAdjuster::AddReprojectionErrorResidual)
      ;

  // Global SfM
  m.def("ComputeTripletBaselineRatios",
        theia::ComputeTripletBaselineRatiosWrapper,
        "Compute baseline ratios for view triplets (global SfM).");

  py::class_<theia::PositionEstimator>(m, "PositionEstimator",
                                       "Base class for global camera position estimators.");

  py::class_<theia::LinearPositionEstimator::Options>(
      m, "LinearPositionEstimatorOptions",
      "Options for linear position estimator (threads, eigensolver).")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::LinearPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_power_iterations",
          &theia::LinearPositionEstimator::Options::max_power_iterations)
      .def_readwrite(
          "eigensolver_threshold",
          &theia::LinearPositionEstimator::Options::eigensolver_threshold);

  py::class_<theia::LinearPositionEstimator, theia::PositionEstimator>(
      m, "LinearPositionEstimator",
      "Linear global position estimator (power iteration).")
      .def(py::init<const theia::LinearPositionEstimator::Options&,
                    const theia::Reconstruction&>())
      .def("EstimatePositions",
           &theia::LinearPositionEstimator::EstimatePositionsWrapper);

  py::class_<theia::NonlinearPositionEstimator::Options>(
      m, "NonlinearPositionEstimatorOptions",
      "Options for nonlinear position estimator.")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::NonlinearPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_power_iterations",
          &theia::NonlinearPositionEstimator::Options::max_num_iterations)
      .def_readwrite(
          "eigensolver_threshold",
          &theia::NonlinearPositionEstimator::Options::robust_loss_width)
      .def_readwrite("min_num_points_per_view",
                     &theia::NonlinearPositionEstimator::Options::num_threads)
      .def_readwrite(
          "point_to_camera_weight",
          &theia::NonlinearPositionEstimator::Options::max_num_iterations);

  py::class_<theia::NonlinearPositionEstimator, theia::PositionEstimator>(
      m, "NonlinearPositionEstimator",
      "Nonlinear global position estimator.")
      .def(py::init<const theia::NonlinearPositionEstimator::Options&,
                    const theia::Reconstruction&>())
      .def("EstimatePositions",
           &theia::NonlinearPositionEstimator::EstimatePositionsWrapper)
      .def("EstimateRemainingPositionsInRecon",
           &theia::NonlinearPositionEstimator::
               EstimateRemainingPositionsInReconWrapper);

  py::class_<theia::LeastUnsquaredDeviationPositionEstimator::Options>(
      m, "LeastUnsquaredDeviationPositionEstimatorOptions",
      "Options for LUD position estimator.")
      .def(py::init<>())
      .def_readwrite("max_num_iterations",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         max_num_iterations)
      .def_readwrite("max_num_reweighted_iterations",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         max_num_reweighted_iterations)
      .def_readwrite("convergence_criterion",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         convergence_criterion);

  py::class_<theia::LeastUnsquaredDeviationPositionEstimator,
             theia::PositionEstimator>(
      m, "LeastUnsquaredDeviationPositionEstimator",
      "Least unsquared deviation global position estimator.")
      .def(py::init<theia::LeastUnsquaredDeviationPositionEstimator::Options>())
      .def("EstimatePositions",
           &theia::LeastUnsquaredDeviationPositionEstimator::
               EstimatePositionsWrapper);

  py::class_<theia::LiGTPositionEstimator::Options>(
      m, "LiGTPositionEstimatorOptions",
      "Options for LiGT position estimator.")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::LiGTPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_power_iterations",
          &theia::LiGTPositionEstimator::Options::max_power_iterations)
      .def_readwrite(
          "eigensolver_threshold",
          &theia::LiGTPositionEstimator::Options::eigensolver_threshold)
      .def_readwrite("max_num_views_svd",
                     &theia::LiGTPositionEstimator::Options::max_num_views_svd);

  py::class_<theia::LiGTPositionEstimator, theia::PositionEstimator>(
      m, "LiGTPositionEstimator",
      "LiGT global position estimator.")
      .def(py::init<const theia::LiGTPositionEstimator::Options&,
                    const theia::Reconstruction&>())
      .def("EstimatePositions",
           &theia::LiGTPositionEstimator::EstimatePositionsWrapper);

  py::class_<theia::RotationEstimator>(m, "RotationEstimator",
                                       "Base class for global rotation estimators.");

  py::class_<theia::RobustRotationEstimator::Options>(
      m, "RobustRotationEstimatorOptions",
      "Options for robust L1/L2 rotation estimator.")
      .def(py::init<>())
      .def_readwrite(
          "max_num_l1_iterations",
          &theia::RobustRotationEstimator::Options::max_num_l1_iterations)
      .def_readwrite("l1_step_convergence_threshold",
                     &theia::RobustRotationEstimator::Options::
                         l1_step_convergence_threshold)
      .def_readwrite(
          "max_num_irls_iterations",
          &theia::RobustRotationEstimator::Options::max_num_irls_iterations)
      .def_readwrite("irls_step_convergence_threshold",
                     &theia::RobustRotationEstimator::Options::
                         irls_step_convergence_threshold)
      .def_readwrite(
          "irls_loss_parameter_sigma",
          &theia::RobustRotationEstimator::Options::irls_loss_parameter_sigma);

  py::class_<theia::RobustRotationEstimator, theia::RotationEstimator>(
      m, "RobustRotationEstimator",
      "Robust global rotation estimator (L1/L2).")
      .def(py::init<const theia::RobustRotationEstimator::Options&>())
      .def("EstimateRotations",
           &theia::RobustRotationEstimator::EstimateRotationsWrapper)
      .def("AddRelativeRotationConstraint",
           &theia::RobustRotationEstimator::AddRelativeRotationConstraint)
      .def("SetFixedGlobalRotations",
           &theia::RobustRotationEstimator::SetFixedGlobalRotations);

  py::class_<theia::NonlinearRotationEstimator, theia::RotationEstimator>(
      m, "NonlinearRotationEstimator",
      "Nonlinear global rotation estimator.")
      .def(py::init<>())
      .def(py::init<double>())
      .def("EstimateRotations",
           &theia::NonlinearRotationEstimator::EstimateRotationsWrapper);

  py::class_<theia::LinearRotationEstimator, theia::RotationEstimator>(
      m, "LinearRotationEstimator",
      "Linear global rotation estimator.")
      .def(py::init<>())
      .def("AddRelativeRotationConstraint",
           &theia::LinearRotationEstimator::AddRelativeRotationConstraint)
      .def("EstimateRotations",
           &theia::LinearRotationEstimator::EstimateRotationsWrapper);

  py::class_<theia::LagrangeDualRotationEstimator, theia::RotationEstimator>(
      m, "LagrangeDualRotationEstimator",
      "Lagrange dual global rotation estimator.")
      .def(py::init<>())
      .def("EstimateRotations",
           &theia::LagrangeDualRotationEstimator::EstimateRotationsWrapper);

  py::class_<theia::HybridRotationEstimator, theia::RotationEstimator>(
      m, "HybridRotationEstimator",
      "Hybrid global rotation estimator.")
      .def(py::init<>())
      .def("EstimateRotations",
           &theia::HybridRotationEstimator::EstimateRotationsWrapper);
}

void pytheia_sfm(py::module& m) {
  py::module m_submodule = m.def_submodule(
      "sfm",
      "Structure-from-motion: cameras, reconstruction, views, tracks, view graph, "
      "pose estimation, triangulation, bundle adjustment, estimators.");
  pytheia_sfm_classes(m_submodule);
}

}  // namespace sfm
}  // namespace pytheia
