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
#include "pytheia/sfm/init_free.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <optional>
#include <unordered_set>

#include <Eigen/Dense>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>
#include <ceres/types.h>
#include <Sophus/sophus/sim3.hpp>

#include "theia/math/polynomial.h"
#include "theia/sfm/pose/util.h"

#include "theia/sfm/feature.h"

#include "theia/sfm/pose/pose_wrapper.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/triangulation/triangulation_wrapper.h"

#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/transformation/align_reconstructions.h"
#include "theia/sfm/transformation/gdls_similarity_transform.h"
#include "theia/sfm/transformation/cross_reconstruction_pose_graph_types.h"
#include "theia/sfm/transformation/cross_reconstruction_sim3_pose_graph_optimizer.h"
#include "theia/sfm/transformation/transformation_wrapper.h"

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/camera_wrapper.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/full_opencv_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/orthographic_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/sfm/camera/thin_prism_fisheye_camera_model.h"

#include "theia/sfm/bundle_adjustment/bundle_adjust_two_views.h"
#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment_wrapper.h"
#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"

#include "theia/sfm/global_pose_estimation/art_position_estimator.h"
#include "theia/sfm/global_pose_estimation/rotation_only_refiner.h"
#include "theia/sfm/global_pose_estimation/LiGT_position_estimator.h"
#include "theia/sfm/global_pose_estimation/glomap_position_estimator.h"
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
#include "theia/sfm/incremental_rig_reconstructor.h"
#include "theia/sfm/global_rig_reconstructor.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/rig/rig_edge_scales.h"
#include "theia/sfm/rig/rig_utils.h"
#include "theia/sfm/rig/capture_view_graph.h"
#include "theia/sfm/track_builder.h"

#include "theia/sfm/rigid_transformation.h"
#include "theia/sfm/similarity_transformation.h"

// estimator
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_dominant_plane_from_points.h"
#include "theia/sfm/estimators/estimate_radial_distortion_homography.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_relative_rig_info.h"
#include "theia/sfm/pose/generalized_ray_correspondence.h"
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
#include "theia/sfm/filter_view_graph_by_relative_inliers.h"
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
  py::class_<theia::Prior<N>>(m, ("Prior" + name).c_str())
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
  AddIntrinsicsPriorType<6>(m, "Vector6d");
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
      camera_intrinsics_model(m, "CameraIntrinsicsModel");
  camera_intrinsics_model
      .def("SetFocalLength", &theia::CameraIntrinsicsModel::SetFocalLength,
           py::arg("focal_length"))
      .def("FocalLength", &theia::CameraIntrinsicsModel::FocalLength)
      .def("PrincipalPointX", &theia::CameraIntrinsicsModel::PrincipalPointX)
      .def("PrincipalPointY", &theia::CameraIntrinsicsModel::PrincipalPointY)
      .def("SetPrincipalPoint",
           &theia::CameraIntrinsicsModel::SetPrincipalPoint,
           py::arg("principal_point_x"), py::arg("principal_point_y"))
      .def("CameraToImageCoordinates",
           &theia::CameraIntrinsicsModel::CameraToImageCoordinates,
           py::arg("point"))
      .def("ImageToCameraCoordinates",
           &theia::CameraIntrinsicsModel::ImageToCameraCoordinates,
           py::arg("pixel"))
      .def("GetParameter", &theia::CameraIntrinsicsModel::GetParameter,
           py::arg("parameter_index"))
      .def("SetParameter", &theia::CameraIntrinsicsModel::SetParameter,
           py::arg("parameter_index"), py::arg("parameter_value"))
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
      m, "FisheyeCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::FisheyeCameraModel::Type)
      .def("NumParameters", &theia::FisheyeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::FisheyeCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::FisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::FisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::FisheyeCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::FisheyeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::FisheyeCameraModel::NumParameters)
      .def("AspectRatio", &theia::FisheyeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::FisheyeCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::FisheyeCameraModel::Skew)
      .def("SetSkew", &theia::FisheyeCameraModel::SetSkew, py::arg("skew"))
      .def("RadialDistortion1", &theia::FisheyeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::FisheyeCameraModel::RadialDistortion2)
      .def("RadialDistortion3", &theia::FisheyeCameraModel::RadialDistortion3)
      .def("RadialDistortion4", &theia::FisheyeCameraModel::RadialDistortion4)
      .def("SetRadialDistortion",
           &theia::FisheyeCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"),
           py::arg("radial_distortion_3"), py::arg("radial_distortion_4"));

  // PinholeRadialTangentialCameraModel
  py::class_<theia::PinholeRadialTangentialCameraModel,
             std::shared_ptr<theia::PinholeRadialTangentialCameraModel>>(
      m, "PinholeRadialTangentialCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::PinholeRadialTangentialCameraModel::Type)
      .def("NumParameters",
           &theia::PinholeRadialTangentialCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::PinholeRadialTangentialCameraModel::
               SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::PinholeRadialTangentialCameraModel::
               CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::PinholeRadialTangentialCameraModel::
               GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::PinholeRadialTangentialCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics",
           &theia::PinholeRadialTangentialCameraModel::PrintIntrinsics)
      .def_property_readonly(
          "kIntrinsicsSize",
          &theia::PinholeRadialTangentialCameraModel::NumParameters)
      .def("AspectRatio",
           &theia::PinholeRadialTangentialCameraModel::AspectRatio)
      .def("SetAspectRatio",
           &theia::PinholeRadialTangentialCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::PinholeRadialTangentialCameraModel::Skew)
      .def("SetSkew", &theia::PinholeRadialTangentialCameraModel::SetSkew,
           py::arg("skew"))
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
           &theia::PinholeRadialTangentialCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"),
           py::arg("radial_distortion_3"))
      .def("SetTangentialDistortion",
           &theia::PinholeRadialTangentialCameraModel::SetTangentialDistortion,
           py::arg("tangential_distortion_1"),
           py::arg("tangential_distortion_2"));

  // DivisionUndistortionCameraModel
  py::class_<theia::DivisionUndistortionCameraModel,
             std::shared_ptr<theia::DivisionUndistortionCameraModel>>(
      m, "DivisionUndistortionCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::DivisionUndistortionCameraModel::Type)
      .def("NumParameters",
           &theia::DivisionUndistortionCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::DivisionUndistortionCameraModel::
               SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::DivisionUndistortionCameraModel::
               CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::DivisionUndistortionCameraModel::
               GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::DivisionUndistortionCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics",
           &theia::DivisionUndistortionCameraModel::PrintIntrinsics)
      .def_property_readonly(
          "kIntrinsicsSize",
          &theia::DivisionUndistortionCameraModel::NumParameters)
      .def("AspectRatio", &theia::DivisionUndistortionCameraModel::AspectRatio)
      .def("SetAspectRatio",
           &theia::DivisionUndistortionCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1",
           &theia::DivisionUndistortionCameraModel::RadialDistortion1)
      .def("SetRadialDistortion",
           &theia::DivisionUndistortionCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"));

  // DoubleSphereCameraModel
  py::class_<theia::DoubleSphereCameraModel,
             std::shared_ptr<theia::DoubleSphereCameraModel>>(
      m, "DoubleSphereCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::DoubleSphereCameraModel::Type)
      .def("NumParameters", &theia::DoubleSphereCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::DoubleSphereCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::DoubleSphereCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::DoubleSphereCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::DoubleSphereCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::DoubleSphereCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                               &theia::DoubleSphereCameraModel::NumParameters)
      .def("AspectRatio", &theia::DoubleSphereCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::DoubleSphereCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::DoubleSphereCameraModel::Skew)
      .def("SetSkew", &theia::DoubleSphereCameraModel::SetSkew,
           py::arg("skew"))
      .def("Alpha", &theia::DoubleSphereCameraModel::Alpha)
      .def("Xi", &theia::DoubleSphereCameraModel::Xi)
      .def("SetAlphaXiDistortion",
           &theia::DoubleSphereCameraModel::SetAlphaXiDistortion,
           py::arg("alpha"), py::arg("xi"));

  // ExtendedUnifiedCameraModel
  py::class_<theia::ExtendedUnifiedCameraModel,
             std::shared_ptr<theia::ExtendedUnifiedCameraModel>>(
      m, "ExtendedUnifiedCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::ExtendedUnifiedCameraModel::Type)
      .def("NumParameters", &theia::ExtendedUnifiedCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::ExtendedUnifiedCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::ExtendedUnifiedCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::ExtendedUnifiedCameraModel::
               GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::ExtendedUnifiedCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics",
           &theia::ExtendedUnifiedCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::ExtendedUnifiedCameraModel::NumParameters)
      .def("AspectRatio", &theia::ExtendedUnifiedCameraModel::AspectRatio)
      .def("SetAspectRatio",
           &theia::ExtendedUnifiedCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::ExtendedUnifiedCameraModel::Skew)
      .def("SetSkew", &theia::ExtendedUnifiedCameraModel::SetSkew,
           py::arg("skew"))
      .def("Alpha", &theia::ExtendedUnifiedCameraModel::Alpha)
      .def("Beta", &theia::ExtendedUnifiedCameraModel::Beta)
      .def("SetAlphaBetaDistortion",
           &theia::ExtendedUnifiedCameraModel::SetAlphaBetaDistortion,
           py::arg("alpha"), py::arg("beta"));

  // PinholeCameraModel
  py::class_<theia::PinholeCameraModel,
             std::shared_ptr<theia::PinholeCameraModel>>(
      m, "PinholeCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::PinholeCameraModel::Type)
      .def("NumParameters", &theia::PinholeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::PinholeCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::PinholeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::PinholeCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::PinholeCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::PinholeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::PinholeCameraModel::NumParameters)
      .def("AspectRatio", &theia::PinholeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::PinholeCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::PinholeCameraModel::Skew)
      .def("SetSkew", &theia::PinholeCameraModel::SetSkew, py::arg("skew"))
      .def("RadialDistortion1", &theia::PinholeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::PinholeCameraModel::RadialDistortion2)
      .def("SetRadialDistortion",
           &theia::PinholeCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"));

  // OrthographicCameraModel
  py::class_<theia::OrthographicCameraModel,
             std::shared_ptr<theia::OrthographicCameraModel>>(
      m, "OrthographicCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::OrthographicCameraModel::Type)
      .def("NumParameters", &theia::OrthographicCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::OrthographicCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::OrthographicCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::OrthographicCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::OrthographicCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::OrthographicCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::OrthographicCameraModel::NumParameters)
      .def("AspectRatio", &theia::OrthographicCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::OrthographicCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("Skew", &theia::OrthographicCameraModel::Skew)
      .def("SetSkew", &theia::OrthographicCameraModel::SetSkew,
           py::arg("skew"))
      .def("RadialDistortion1",
           &theia::OrthographicCameraModel::RadialDistortion1)
      .def("RadialDistortion2",
           &theia::OrthographicCameraModel::RadialDistortion2)
      .def("SetRadialDistortion",
           &theia::OrthographicCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"));

  // FOVCameraModel
  py::class_<theia::FOVCameraModel, std::shared_ptr<theia::FOVCameraModel>>(
      m, "FOVCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::FOVCameraModel::Type)
      .def("NumParameters", &theia::FOVCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::FOVCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::FOVCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      // OptimizeIntrinsicsType not defined
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::FOVCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix", &theia::FOVCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::FOVCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::FOVCameraModel::NumParameters)
      .def("AspectRatio", &theia::FOVCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::FOVCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1", &theia::FOVCameraModel::RadialDistortion1)
      .def("SetRadialDistortion", &theia::FOVCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"));

  // FullOpenCVCameraModel
  py::class_<theia::FullOpenCVCameraModel, std::shared_ptr<theia::FullOpenCVCameraModel>>(
      m, "FullOpenCVCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::FullOpenCVCameraModel::Type)
      .def("NumParameters", &theia::FullOpenCVCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::FullOpenCVCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::FullOpenCVCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::FullOpenCVCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::FullOpenCVCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::FullOpenCVCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::FullOpenCVCameraModel::NumParameters)
      .def("AspectRatio", &theia::FullOpenCVCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::FullOpenCVCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1", &theia::FullOpenCVCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::FullOpenCVCameraModel::RadialDistortion2)
      .def("RadialDistortion3", &theia::FullOpenCVCameraModel::RadialDistortion3)
      .def("RadialDistortion4", &theia::FullOpenCVCameraModel::RadialDistortion4)
      .def("RadialDistortion5", &theia::FullOpenCVCameraModel::RadialDistortion5)
      .def("RadialDistortion6", &theia::FullOpenCVCameraModel::RadialDistortion6)
      .def("TangentialDistortion1", &theia::FullOpenCVCameraModel::TangentialDistortion1)
      .def("TangentialDistortion2", &theia::FullOpenCVCameraModel::TangentialDistortion2)
      .def("SetRadialDistortion", &theia::FullOpenCVCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"), py::arg("radial_distortion_3"), py::arg("radial_distortion_4"), py::arg("radial_distortion_5"), py::arg("radial_distortion_6"))
      .def("SetTangentialDistortion", &theia::FullOpenCVCameraModel::SetTangentialDistortion,
           py::arg("tangential_distortion_1"), py::arg("tangential_distortion_2"));
  // RadialFisheyeCameraModel
  py::class_<theia::RadialFisheyeCameraModel, std::shared_ptr<theia::RadialFisheyeCameraModel>>(
      m, "RadialFisheyeCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::RadialFisheyeCameraModel::Type)
      .def("NumParameters", &theia::RadialFisheyeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::RadialFisheyeCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::RadialFisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::RadialFisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::RadialFisheyeCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::RadialFisheyeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::RadialFisheyeCameraModel::NumParameters)
      .def("AspectRatio", &theia::RadialFisheyeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::RadialFisheyeCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1", &theia::RadialFisheyeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::RadialFisheyeCameraModel::RadialDistortion2)
      .def("SetRadialDistortion", &theia::RadialFisheyeCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"));
  // ThinPrismFisheyeCameraModel
  py::class_<theia::ThinPrismFisheyeCameraModel, std::shared_ptr<theia::ThinPrismFisheyeCameraModel>>(
      m, "ThinPrismFisheyeCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::ThinPrismFisheyeCameraModel::Type)
      .def("NumParameters", &theia::ThinPrismFisheyeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::ThinPrismFisheyeCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::ThinPrismFisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::ThinPrismFisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::ThinPrismFisheyeCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::ThinPrismFisheyeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::ThinPrismFisheyeCameraModel::NumParameters)
      .def("AspectRatio", &theia::ThinPrismFisheyeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::ThinPrismFisheyeCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1", &theia::ThinPrismFisheyeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::ThinPrismFisheyeCameraModel::RadialDistortion2)
      .def("RadialDistortion3", &theia::ThinPrismFisheyeCameraModel::RadialDistortion3)
      .def("RadialDistortion4", &theia::ThinPrismFisheyeCameraModel::RadialDistortion4)
      .def("SetRadialDistortion", &theia::ThinPrismFisheyeCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"), py::arg("radial_distortion_3"), py::arg("radial_distortion_4"))
      .def("TangentialDistortion1", &theia::ThinPrismFisheyeCameraModel::TangentialDistortion1)
      .def("TangentialDistortion2", &theia::ThinPrismFisheyeCameraModel::TangentialDistortion2)
      .def("SetTangentialDistortion", &theia::ThinPrismFisheyeCameraModel::SetTangentialDistortion,
           py::arg("tangential_distortion_1"), py::arg("tangential_distortion_2"))
      .def("ThinPrismDistortion1", &theia::ThinPrismFisheyeCameraModel::ThinPrismDistortion1)
      .def("ThinPrismDistortion2", &theia::ThinPrismFisheyeCameraModel::ThinPrismDistortion2)
      .def("SetThinPrismDistortion", &theia::ThinPrismFisheyeCameraModel::SetThinPrismDistortion,
           py::arg("thin_prism_distortion_1"), py::arg("thin_prism_distortion_2"));
  // RadTanThinPrismFisheyeCameraModel
  py::class_<theia::RadTanThinPrismFisheyeCameraModel, std::shared_ptr<theia::RadTanThinPrismFisheyeCameraModel>>(
      m, "RadTanThinPrismFisheyeCameraModel", camera_intrinsics_model)
      .def(py::init<>())
      .def("Type", &theia::RadTanThinPrismFisheyeCameraModel::Type)
      .def("NumParameters", &theia::RadTanThinPrismFisheyeCameraModel::NumParameters)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::RadTanThinPrismFisheyeCameraModel::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::RadTanThinPrismFisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetSubsetFromOptimizeIntrinsicsType",
           &theia::RadTanThinPrismFisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType,
           py::arg("intrinsics_to_optimize"))
      .def("GetCalibrationMatrix",
           &theia::RadTanThinPrismFisheyeCameraModel::GetCalibrationMatrix,
           py::arg("kmatrix"))
      .def("PrintIntrinsics", &theia::RadTanThinPrismFisheyeCameraModel::PrintIntrinsics)
      .def_property_readonly("kIntrinsicsSize",
                             &theia::RadTanThinPrismFisheyeCameraModel::NumParameters)
      .def("AspectRatio", &theia::RadTanThinPrismFisheyeCameraModel::AspectRatio)
      .def("SetAspectRatio", &theia::RadTanThinPrismFisheyeCameraModel::SetAspectRatio,
           py::arg("aspect_ratio"))
      .def("RadialDistortion1", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion1)
      .def("RadialDistortion2", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion2)
      .def("RadialDistortion3", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion3)
      .def("RadialDistortion4", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion4)
      .def("RadialDistortion5", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion5)
      .def("RadialDistortion6", &theia::RadTanThinPrismFisheyeCameraModel::RadialDistortion6)
      .def("SetRadialDistortion", &theia::RadTanThinPrismFisheyeCameraModel::SetRadialDistortion,
           py::arg("radial_distortion_1"), py::arg("radial_distortion_2"), py::arg("radial_distortion_3"), py::arg("radial_distortion_4"), py::arg("radial_distortion_5"), py::arg("radial_distortion_6"))
      .def("TangentialDistortion1", &theia::RadTanThinPrismFisheyeCameraModel::TangentialDistortion1)
      .def("TangentialDistortion2", &theia::RadTanThinPrismFisheyeCameraModel::TangentialDistortion2)
      .def("SetTangentialDistortion", &theia::RadTanThinPrismFisheyeCameraModel::SetTangentialDistortion,
           py::arg("tangential_distortion_1"), py::arg("tangential_distortion_2"))
      .def("ThinPrismDistortion1", &theia::RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion1)
      .def("ThinPrismDistortion2", &theia::RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion2)
      .def("ThinPrismDistortion3", &theia::RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion3)
      .def("ThinPrismDistortion4", &theia::RadTanThinPrismFisheyeCameraModel::ThinPrismDistortion4)
      .def("SetThinPrismDistortion", &theia::RadTanThinPrismFisheyeCameraModel::SetThinPrismDistortion,
           py::arg("thin_prism_distortion_1"), py::arg("thin_prism_distortion_2"), py::arg("thin_prism_distortion_3"), py::arg("thin_prism_distortion_4"));

  m.def("ComposeProjectionMatrix", theia::ComposeProjectionMatrixWrapper,
        py::arg("calibration_matrix"), py::arg("rotation"),
        py::arg("position"));
  m.def("DecomposeProjectionMatrix", theia::DecomposeProjectionMatrixWrapper,
        py::arg("pmatrix"));
  m.def("CalibrationMatrixToIntrinsics",
        theia::CalibrationMatrixToIntrinsicsWrapper,
        py::arg("calibration_matrix"));
  m.def("IntrinsicsToCalibrationMatrix",
        theia::IntrinsicsToCalibrationMatrixWrapper,
        py::arg("focal_length"), py::arg("skew"), py::arg("aspect_ratio"),
        py::arg("principal_point_x"), py::arg("principal_point_y"));

  py::class_<theia::Camera, std::shared_ptr<theia::Camera>>(m, "Camera")
      .def(py::init())
      .def(py::init<theia::Camera>(), py::arg("camera"))
      .def(py::init<theia::CameraIntrinsicsModelType>(),
           py::arg("camera_type"))
      .def("DeepCopy", &theia::Camera::DeepCopy, py::arg("camera"))
      .def("CameraIntrinsics",
           &theia::Camera::CameraIntrinsics,
           py::return_value_policy::reference_internal)
      .def("SetFromCameraIntrinsicsPriors",
           &theia::Camera::SetFromCameraIntrinsicsPriors,
           py::arg("prior"))
      .def("CameraIntrinsicsPriorFromIntrinsics",
           &theia::Camera::CameraIntrinsicsPriorFromIntrinsics)
      .def("GetCameraIntrinsicsModelType",
           &theia::Camera::GetCameraIntrinsicsModelType)
      .def("SetCameraIntrinsicsModelType",
           &theia::Camera::SetCameraIntrinsicsModelType,
           py::arg("camera_model_type"))
      .def("InitializeFromProjectionMatrix",
           &theia::Camera::InitializeFromProjectionMatrix,
           py::arg("image_width"), py::arg("image_height"),
           py::arg("projection_matrix"))
      .def("GetCalibrationMatrix", &theia::Camera::GetCalibrationMatrixWrapper)
      .def("GetProjectionMatrix", &theia::Camera::GetProjectionMatrixWrapper)
      .def("FocalLength", &theia::Camera::FocalLength)
      .def("SetFocalLength", &theia::Camera::SetFocalLength,
           py::arg("focal_length"))
      .def("ImageHeight", &theia::Camera::ImageHeight)
      .def("ImageWidth", &theia::Camera::ImageWidth)
      .def("SetImageSize", &theia::Camera::SetImageSize,
           py::arg("image_width"), py::arg("image_height"))
      .def("PrincipalPointX", &theia::Camera::PrincipalPointX)
      .def("PrincipalPointY", &theia::Camera::PrincipalPointY)
      .def("SetPrincipalPoint", &theia::Camera::SetPrincipalPoint,
           py::arg("principal_point_x"), py::arg("principal_point_y"))
      .def("GetOrientationAsAngleAxis",
           &theia::Camera::GetOrientationAsAngleAxis)
      .def("GetOrientationAsRotationMatrix",
           &theia::Camera::GetOrientationAsRotationMatrix)
      .def("SetOrientationFromAngleAxis",
           &theia::Camera::SetOrientationFromAngleAxis,
           py::arg("angle_axis"))
      .def("SetOrientationFromRotationMatrix",
           &theia::Camera::SetOrientationFromRotationMatrix,
           py::arg("rotation"))
      .def("GetPosition", &theia::Camera::GetPosition)
      .def("SetPosition", &theia::Camera::SetPosition, py::arg("position"))
      .def("PrintCameraIntrinsics", &theia::Camera::PrintCameraIntrinsics)
      .def("PixelToNormalizedCoordinates",
           &theia::Camera::PixelToNormalizedCoordinates, py::arg("pixel"))
      .def("PixelToUnitDepthRay", &theia::Camera::PixelToUnitDepthRay,
           py::arg("pixel"))
      .def("ProjectPoint", (std::tuple<double, Eigen::Vector2d> (theia::Camera::*)(const Eigen::Vector4d&)) &theia::Camera::ProjectPointWrapper,
           py::arg("point"))
      .def("ProjectPoint", (std::tuple<double, Eigen::Vector2d> (theia::Camera::*)(const Eigen::Vector3d&)) &theia::Camera::ProjectPointWrapper,
           py::arg("point"))
      //.def_readonly_static("kExtrinsicsSize", &theia::Camera::kExtrinsicsSize)
      ;

  // tested
  py::enum_<theia::CameraIntrinsicsModelType>(m, "CameraIntrinsicsModelType")
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
      .value("FULL_OPENCV", theia::CameraIntrinsicsModelType::FULL_OPENCV)
      .value("RADIAL_FISHEYE", theia::CameraIntrinsicsModelType::RADIAL_FISHEYE)
      .value("THIN_PRISM_FISHEYE",
             theia::CameraIntrinsicsModelType::THIN_PRISM_FISHEYE)
      .value("RAD_TAN_THIN_PRISM_FISHEYE",
             theia::CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE)
      .export_values();

  py::class_<theia::CameraIntrinsicsPrior>(m, "CameraIntrinsicsPrior")
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
      .def_readwrite("radial_distortion_6",
                     &theia::CameraIntrinsicsPrior::radial_distortion_6)
      .def_readwrite("thin_prism_distortion",
                     &theia::CameraIntrinsicsPrior::thin_prism_distortion)
      .def_readwrite("position", &theia::CameraIntrinsicsPrior::position)
      .def_readwrite("orientation", &theia::CameraIntrinsicsPrior::orientation)
      .def_readwrite("latitude", &theia::CameraIntrinsicsPrior::latitude)
      .def_readwrite("longitude", &theia::CameraIntrinsicsPrior::longitude)
      .def_readwrite("altitude", &theia::CameraIntrinsicsPrior::altitude)

      ;

  // pose
  m.def("PoseFromThreePoints", theia::PoseFromThreePointsWrapper,
        py::arg("feature_points_in"), py::arg("points_3d_in"));
  m.def("MLPnP", theia::MLPnPWrapper,
        "Maximum-likelihood PnP. Requires at least six normalized 2D-3D "
        "correspondences (see kMLPnPMinimumPoints).",
        py::arg("norm_feature_points"), py::arg("feature_covariances"),
        py::arg("world_points"));
  m.def("UPMLPnP", theia::UPMLPnPWrapper,
        "Upright maximum-likelihood PnP with gravity and per-feature "
        "uncertainty. Requires at least three normalized 2D-3D correspondences "
        "(see kUPMLPnPMinimumPoints).",
        py::arg("norm_feature_points"), py::arg("feature_covariances"),
        py::arg("world_points"), py::arg("gravity_camera"),
        py::arg("gravity_world"), py::arg("gravity_covariance"),
        py::arg("run_refinement"));
  m.def("UPMLPnPWithCovariance", theia::UPMLPnPWithCovarianceWrapper,
        "UPMLPnP with a 6x6 posterior pose covariance (Laplace approximation).",
        py::arg("norm_feature_points"), py::arg("feature_covariances"),
        py::arg("world_points"), py::arg("gravity_camera"),
        py::arg("gravity_world"), py::arg("gravity_covariance"),
        py::arg("run_refinement"));
  m.def("NormalizedEightPointFundamentalMatrix",
        theia::NormalizedEightPointFundamentalMatrixWrapper,
        py::arg("image_1_points"), py::arg("image_2_points"));
  m.def("FivePointRelativePose", theia::FivePointRelativePoseWrapper,
        py::arg("image1_points"), py::arg("image2_points"));
  m.def("FivePointOnePointGeneralizedRelativePose",
        theia::FivePointOnePointGeneralizedRelativePoseWrapper,
        py::arg("origins1"), py::arg("directions1"), py::arg("origins2"),
        py::arg("directions2"));
  m.def("FourPointUprightGeneralizedRelativePose",
        theia::FourPointUprightGeneralizedRelativePoseWrapper,
        py::arg("gravity_axis"), py::arg("origins1"), py::arg("directions1"),
        py::arg("origins2"), py::arg("directions2"));
  m.def("FourPointPoseAndFocalLength",
        theia::FourPointPoseAndFocalLengthWrapper,
        py::arg("feature_vectors"), py::arg("world_points"));
  m.def("FourPointHomography", theia::FourPointHomographyWrapper,
        py::arg("image_1_points"), py::arg("image_2_points"));
  m.def("FourPointsPoseFocalLengthRadialDistortion",
        theia::FourPointsPoseFocalLengthRadialDistortionWrapper,
        py::arg("feature_vectors"), py::arg("world_points"),
        py::arg("settings"));
  m.def("FourPointRelativePosePartialRotation",
        theia::FourPointRelativePosePartialRotationWrapper,
        py::arg("rotation_axis"), py::arg("image_one_ray_directions_in"),
        py::arg("image_one_ray_origins_in"),
        py::arg("image_two_ray_directions_in"),
        py::arg("image_two_ray_origins_in"));
  m.def("FivePointFocalLengthRadialDistortion",
        theia::FivePointFocalLengthRadialDistortionWrapper,
        py::arg("feature_positions"), py::arg("world_points"),
        py::arg("num_radial_distortion_params"));
  m.def("ThreePointRelativePosePartialRotation",
        theia::ThreePointRelativePosePartialRotationWrapper,
        py::arg("rotation_axis"), py::arg("image_1_rays_in"),
        py::arg("image_2_rays_in"));
  m.def("TwoPointPosePartialRotation",
        theia::TwoPointPosePartialRotationWrapper,
        py::arg("axis"), py::arg("model_point_1"), py::arg("model_point_2"),
        py::arg("image_ray_1"), py::arg("image_ray_2"));
  m.def("DlsPnp", theia::DlsPnpWrapper, py::arg("feature_positions"),
        py::arg("world_point"));
  m.def("SQPnP", theia::SQPnPWrapper, py::arg("feature_positions"),
        py::arg("world_point"));
  m.def("PositionFromTwoRays", theia::PositionFromTwoRaysWrapper,
        py::arg("rotated_feature1"), py::arg("point1"),
        py::arg("rotated_feature2"), py::arg("point2"));
  m.def("RelativePoseFromTwoPointsWithKnownRotation",
        theia::RelativePoseFromTwoPointsWithKnownRotationWrapper,
        py::arg("rotated_features1_in"), py::arg("rotated_features2_in"));
  m.def("SevenPointFundamentalMatrix",
        theia::SevenPointFundamentalMatrixWrapper,
        py::arg("image1_points"), py::arg("image2_points"));
  m.def("SimTransformPartialRotation",
        theia::SimTransformPartialRotationWrapper,
        py::arg("rotation_axis"), py::arg("image_one_ray_directions_in"),
        py::arg("image_one_ray_origins_in"),
        py::arg("image_two_ray_directions_in"),
        py::arg("image_two_ray_origins_in"));
  m.def("DecomposeEssentialMatrix", theia::DecomposeEssentialMatrixWrapper,
        py::arg("essential_matrix"));
  m.def("EssentialMatrixFromTwoProjectionMatrices",
        theia::EssentialMatrixFromTwoProjectionMatricesWrapper,
        py::arg("pose1"), py::arg("pose2"));
  m.def("GetBestPoseFromEssentialMatrix",
        theia::GetBestPoseFromEssentialMatrixWrapper,
        py::arg("essential_matrix"), py::arg("normalized_correspondences"));
  m.def("FocalLengthsFromFundamentalMatrix",
        theia::FocalLengthsFromFundamentalMatrixWrapper, py::arg("fmatrix"));
  m.def("SharedFocalLengthsFromFundamentalMatrix",
        theia::SharedFocalLengthsFromFundamentalMatrixWrapper,
        py::arg("fmatrix"));
  m.def("ProjectionMatricesFromFundamentalMatrix",
        theia::ProjectionMatricesFromFundamentalMatrixWrapper,
        py::arg("fmatrix"));
  m.def("FundamentalMatrixFromProjectionMatrices",
        theia::FundamentalMatrixFromProjectionMatricesWrapper,
        py::arg("pmatrix1"), py::arg("pmatrix2"));
  m.def("EssentialMatrixFromFundamentalMatrix",
        theia::EssentialMatrixFromFundamentalMatrixWrapper,
        py::arg("fmatrix"), py::arg("focal_length1"), py::arg("focal_length2"));
  m.def("ComposeFundamentalMatrix", theia::ComposeFundamentalMatrixWrapper,
        py::arg("focal_length1"), py::arg("focal_length2"),
        py::arg("rotation"), py::arg("translation"));
  m.def("PlanarUncalibratedOrthographicPose",
        theia::PlanarUncalibratedOrthographicPoseWrapper,
        py::arg("correspondences"), py::arg("principal_point"));

  // transformation
  m.def("AlignPointCloudsUmeyama", theia::AlignPointCloudsUmeyamaWrapper,
        py::arg("left"), py::arg("right"));
  m.def("AlignPointCloudsUmeyamaWithWeights",
        theia::AlignPointCloudsUmeyamaWithWeightsWrapper,
        py::arg("left"), py::arg("right"), py::arg("weights"));
  m.def("GdlsSimilarityTransform", theia::GdlsSimilarityTransformWrapper,
        py::arg("ray_origin"), py::arg("ray_direction"),
        py::arg("world_point"));
  m.def("AlignRotations", theia::AlignRotationsWrapper,
        py::arg("gt_rotation"));
  m.def("AlignReconstructions", theia::AlignReconstructionsWrapper,
        py::arg("fixed_reconstruction"), py::arg("variable_reconstruction"));
  m.def("AlignReconstructionsRobust", theia::AlignReconstructionsRobustWrapper,
        py::arg("robust_error_threshold"), py::arg("fixed_reconstruction"),
        py::arg("variable_reconstruction"));
  m.def("TransformReconstruction", theia::TransformReconstructionWrapper,
        py::arg("reconstruction"), py::arg("rotation"), py::arg("translation"),
        py::arg("scale"));
  m.def("TransformReconstruction4", theia::TransformReconstructionWrapper4,
        py::arg("reconstruction"), py::arg("transformation"));

  // SIM3 Point Cloud Alignment
  py::enum_<theia::Sim3AlignmentType>(m, "Sim3AlignmentType")
      .value("POINT_TO_POINT", theia::Sim3AlignmentType::POINT_TO_POINT)
      .value("ROBUST_POINT_TO_POINT", theia::Sim3AlignmentType::ROBUST_POINT_TO_POINT)
      .value("POINT_TO_PLANE", theia::Sim3AlignmentType::POINT_TO_PLANE)
      .export_values();

  py::class_<theia::Sim3AlignmentOptions>(m, "Sim3AlignmentOptions")
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

  py::class_<theia::Sim3AlignmentSummary>(m, "Sim3AlignmentSummary")
      .def(py::init<>())
      .def_readwrite("success", &theia::Sim3AlignmentSummary::success)
      .def_readwrite("sim3_params", &theia::Sim3AlignmentSummary::sim3_params)
      .def_readwrite("alignment_error", &theia::Sim3AlignmentSummary::alignment_error)
      .def_readwrite("num_iterations", &theia::Sim3AlignmentSummary::num_iterations)
      .def_readwrite("final_cost", &theia::Sim3AlignmentSummary::final_cost);

  m.def("OptimizeAlignmentSim3", theia::OptimizeAlignmentSim3Wrapper,
        py::arg("source_points"), py::arg("target_points"), py::arg("options"));
  m.def("Sim3FromRotationTranslationScale", theia::Sim3FromRotationTranslationScaleWrapper,
        py::arg("rotation"), py::arg("translation"), py::arg("scale"));
  m.def("Sim3ToRotationTranslationScale", theia::Sim3ToRotationTranslationScaleWrapper,
        py::arg("sim3_params"));
  m.def("Sim3ToHomogeneousMatrix", theia::Sim3ToHomogeneousMatrixWrapper,
        py::arg("sim3_params"));

  // Cross-reconstruction Sim(3) pose graph alignment
  py::class_<theia::CrossReconstructionPoseGraphOptions>(
      m, "CrossReconstructionPoseGraphOptions")
      .def(py::init<>())
      .def_readwrite("sequential_weight",
                     &theia::CrossReconstructionPoseGraphOptions::
                         sequential_weight)
      .def_readwrite("anchor_weight",
                     &theia::CrossReconstructionPoseGraphOptions::anchor_weight)
      .def_readwrite("sequential_translation_magnitude_weight",
                     &theia::CrossReconstructionPoseGraphOptions::
                         sequential_translation_magnitude_weight)
      .def_readwrite("scale_smooth_weight",
                     &theia::CrossReconstructionPoseGraphOptions::
                         scale_smooth_weight)
      .def_readwrite("huber_delta_anchor",
                     &theia::CrossReconstructionPoseGraphOptions::
                         huber_delta_anchor)
      .def_readwrite("auto_scale_smoothness",
                     &theia::CrossReconstructionPoseGraphOptions::
                         auto_scale_smoothness)
      .def_readwrite("max_num_iterations",
                     &theia::CrossReconstructionPoseGraphOptions::
                         max_num_iterations)
      .def_readwrite("verbose",
                     &theia::CrossReconstructionPoseGraphOptions::verbose)
      .def_readwrite("debug_cost_breakdown",
                     &theia::CrossReconstructionPoseGraphOptions::
                         debug_cost_breakdown);

  py::class_<theia::CrossReconstructionPoseGraphSummary>(
      m, "CrossReconstructionPoseGraphSummary")
      .def(py::init<>())
      .def_readwrite("success",
                     &theia::CrossReconstructionPoseGraphSummary::success)
      .def_readwrite("initial_cost",
                     &theia::CrossReconstructionPoseGraphSummary::initial_cost)
      .def_readwrite("final_cost",
                     &theia::CrossReconstructionPoseGraphSummary::final_cost)
      .def_readwrite("num_iterations",
                     &theia::CrossReconstructionPoseGraphSummary::
                         num_iterations)
      .def_readwrite("poses_finite_before",
                     &theia::CrossReconstructionPoseGraphSummary::
                         poses_finite_before)
      .def_readwrite("poses_finite_after",
                     &theia::CrossReconstructionPoseGraphSummary::
                         poses_finite_after)
      .def_readwrite("sequential_residual_cost",
                     &theia::CrossReconstructionPoseGraphSummary::
                         sequential_residual_cost)
      .def_readwrite("anchor_residual_cost",
                     &theia::CrossReconstructionPoseGraphSummary::
                         anchor_residual_cost)
      .def_readwrite("scale_smooth_residual_cost",
                     &theia::CrossReconstructionPoseGraphSummary::
                         scale_smooth_residual_cost);

  py::class_<theia::SequentialSim3Edge>(m, "SequentialSim3Edge")
      .def(py::init<>())
      .def_readwrite("view_id_i", &theia::SequentialSim3Edge::view_id_i)
      .def_readwrite("view_id_j", &theia::SequentialSim3Edge::view_id_j)
      .def_property(
          "measured_S_ji_log",
          [](const theia::SequentialSim3Edge& e) {
            return e.measured_S_ji.log();
          },
          [](theia::SequentialSim3Edge& e, const Eigen::Matrix<double, 7, 1>& v) {
            e.measured_S_ji = Sophus::Sim3d::exp(v);
          },
          "Relative Sim3 S_ji as 7-vector lie algebra log")
      .def_readwrite("sqrt_information",
                     &theia::SequentialSim3Edge::sqrt_information,
                     "7x7 sqrt-information for the relative Sim3 residual; "
                     "tangent order [translation(3), rotation(3), scale(1)]");

  py::class_<theia::CrossViewAnchorEdge>(m, "CrossViewAnchorEdge")
      .def(py::init<>())
      .def_readwrite("variable_view_id",
                     &theia::CrossViewAnchorEdge::variable_view_id)
      .def_property(
          "measured_S_run_in_seg_log",
          [](const theia::CrossViewAnchorEdge& e) {
            return e.measured_S_run_in_seg.log();
          },
          [](theia::CrossViewAnchorEdge& e, const Eigen::Matrix<double, 7, 1>& v) {
            e.measured_S_run_in_seg = Sophus::Sim3d::exp(v);
          },
          "PnP Sim3 pose of run camera in segment world (7-vector lie log)")
      .def_readwrite("weight", &theia::CrossViewAnchorEdge::weight);

  py::class_<theia::RelativePoseConstraint>(m, "RelativePoseConstraint")
      .def(py::init<>())
      .def_readwrite("view_id_i", &theia::RelativePoseConstraint::view_id_i)
      .def_readwrite("view_id_j", &theia::RelativePoseConstraint::view_id_j)
      .def_readwrite("translation_sqrt_weight",
                     &theia::RelativePoseConstraint::translation_sqrt_weight)
      .def_readwrite("rotation_sqrt_weight",
                     &theia::RelativePoseConstraint::rotation_sqrt_weight)
      .def_readwrite("scale_invariant_translation",
                     &theia::RelativePoseConstraint::scale_invariant_translation)
      .def_readwrite("translation_direction_sqrt_weight",
                     &theia::RelativePoseConstraint::
                         translation_direction_sqrt_weight)
      .def_readwrite("translation_magnitude_sqrt_weight",
                     &theia::RelativePoseConstraint::
                         translation_magnitude_sqrt_weight);

  py::class_<theia::CrossReconstructionConstraints>(
      m, "CrossReconstructionConstraints")
      .def(py::init<>())
      .def_readwrite("variable_keyframe_view_ids",
                     &theia::CrossReconstructionConstraints::
                         variable_keyframe_view_ids)
      .def_readwrite("fixed_anchor_view_ids",
                     &theia::CrossReconstructionConstraints::
                         fixed_anchor_view_ids)
      .def_readwrite("sequential_edges",
                     &theia::CrossReconstructionConstraints::sequential_edges)
      .def_readwrite("cross_view_edges",
                     &theia::CrossReconstructionConstraints::cross_view_edges);

  py::class_<theia::CrossReconstructionSim3PoseGraphOptimizer>(
      m, "CrossReconstructionSim3PoseGraphOptimizer")
      .def(py::init<const theia::CrossReconstructionPoseGraphOptions&>(),
           py::arg("options") = theia::CrossReconstructionPoseGraphOptions())
      .def("set_fixed_reconstruction",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::
               SetFixedReconstruction,
           py::arg("fixed_reconstruction"), py::arg("anchor_view_ids"))
      .def("set_variable_reconstruction",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::
               SetVariableReconstruction,
           py::arg("variable_reconstruction"), py::arg("keyframe_view_ids"))
      .def(
          "set_initial_variable_poses",
          [](theia::CrossReconstructionSim3PoseGraphOptimizer& optimizer,
             py::dict poses_dict) {
            theia::Sim3LieMap poses;
            for (auto item : poses_dict) {
              const theia::ViewId view_id =
                  py::cast<theia::ViewId>(item.first);
              poses[view_id] =
                  py::cast<Eigen::Matrix<double, 7, 1>>(item.second);
            }
            optimizer.SetInitialVariablePoses(poses);
          },
          py::arg("initial_poses"),
          "Override initial optimizer poses (view_id -> 7-vector lie) without "
          "mutating the reconstruction.")
      .def("add_sequential_edge",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::AddSequentialEdge,
           py::arg("edge"))
      .def("add_cross_view_edge",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::AddCrossViewEdge,
           py::arg("edge"))
      .def("add_scale_smoothness_edge",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::AddScaleSmoothnessEdge,
           py::arg("view_id_i"), py::arg("view_id_j"), py::arg("weight"))
      .def("set_constraints",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::SetConstraints,
           py::arg("constraints"))
      .def("optimize",
           &theia::CrossReconstructionSim3PoseGraphOptimizerOptimizeWrapper,
           "Run Ceres pose-graph optimization; returns (ok, summary).")
      .def("apply_to_variable_reconstruction",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::
               ApplyToVariableReconstruction,
           py::arg("variable_reconstruction"),
           py::arg("transform_tracks") = true)
      .def("variable_poses",
           &theia::CrossReconstructionSim3PoseGraphOptimizer::variable_poses,
           py::return_value_policy::reference_internal);

  m.def("AlignReconstructionsWithPoseGraph",
        &theia::AlignReconstructionsWithPoseGraphWrapper,
        py::arg("fixed_reconstruction"),
        py::arg("variable_reconstruction"),
        py::arg("constraints"),
        py::arg("options") = theia::CrossReconstructionPoseGraphOptions(),
        py::arg("apply_to_variable_reconstruction") = true,
        "Align variable_reconstruction to fixed via Sim(3) pose graph; "
        "returns (ok, CrossReconstructionPoseGraphSummary).");

  m.def("GetSim3LieFromView", &theia::GetSim3LieFromViewWrapper, py::arg("view"),
        "Sim(3) camera pose as 7-vector lie algebra log (Sophus convention).");
  m.def("RelativeSim3BetweenViews", &theia::RelativeSim3BetweenViewsWrapper,
        py::arg("view_i"), py::arg("view_j"),
        "Relative Sim(3) S_i^{-1} S_j as 7-vector lie log.");

  py::class_<theia::SimilarityTransformation>(m, "SimilarityTransformation")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::SimilarityTransformation::rotation)
      .def_readwrite("translation",
                     &theia::SimilarityTransformation::translation)
      .def_readwrite("scale", &theia::SimilarityTransformation::scale);

  py::class_<theia::RigidTransformation>(m, "RigidTransformation")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::RigidTransformation::rotation)
      .def_readwrite("translation", &theia::RigidTransformation::translation);

  py::class_<theia::Feature>(m, "Feature")
      .def(py::init<>())
      .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
           py::arg("depth_prior"))
      .def(py::init<Eigen::Vector2d>(), py::arg("point"))
      .def(py::init<Eigen::Vector2d, double>(), py::arg("point"),
           py::arg("depth_prior"))
      .def(py::init<Eigen::Vector2d, Eigen::Matrix2d>(), py::arg("point"),
           py::arg("covariance"))
      .def(py::init<Eigen::Vector2d, Eigen::Matrix2d, double, double>(),
           py::arg("point"), py::arg("covariance"), py::arg("depth_prior"),
           py::arg("depth_prior_variance"))
      .def_readwrite("point", &theia::Feature::point_)
      .def_readwrite("covariance", &theia::Feature::covariance_)
      .def_readwrite("depth_prior", &theia::Feature::depth_prior_)
      .def_readwrite("depth_prior_variance",
                     &theia::Feature::depth_prior_variance_)
      .def("set_isotropic_sigma", &theia::Feature::SetIsotropicSigma,
           py::arg("sigma"),
           "Sets covariance = sigma^2 * I (sigma in pixels). Used to weight "
           "the feature in bundle adjustment.")
      .def("sqrt_information", &theia::Feature::SqrtInformation,
           "W with W^T W = covariance^-1.")
      .def("x", &theia::Feature::x)
      .def("y", &theia::Feature::y)
      .def("get_depth_prior", &theia::Feature::depth_prior)
      .def("get_depth_prior_variance", &theia::Feature::depth_prior_variance);

  py::class_<theia::CameraAndFeatureCorrespondence2D3D>(
      m, "CameraAndFeatureCorrespondence2D3D")
      .def(py::init<>())
      .def_readwrite("camera",
                     &theia::CameraAndFeatureCorrespondence2D3D::camera)
      .def_readwrite("observation",
                     &theia::CameraAndFeatureCorrespondence2D3D::observation)
      .def_readwrite("point3d",
                     &theia::CameraAndFeatureCorrespondence2D3D::point3d);

  py::class_<theia::FeatureCorrespondence2D3D>(m, "FeatureCorrespondence2D3D")
      .def(py::init<>())
      .def(py::init<Eigen::Vector2d, Eigen::Vector3d>(), py::arg("feature"),
           py::arg("world_point"))
      .def_readwrite("feature", &theia::FeatureCorrespondence2D3D::feature)
      .def_readwrite("world_point",
                     &theia::FeatureCorrespondence2D3D::world_point);

  py::class_<theia::CalibratedAbsolutePose>(m, "CalibratedAbsolutePose")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::CalibratedAbsolutePose::rotation)
      .def_readwrite("position", &theia::CalibratedAbsolutePose::position);

  py::class_<theia::UncalibratedAbsolutePose>(m, "UncalibratedAbsolutePose")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::UncalibratedAbsolutePose::rotation)
      .def_readwrite("position", &theia::UncalibratedAbsolutePose::position)
      .def_readwrite("focal_length",
                     &theia::UncalibratedAbsolutePose::focal_length);

  py::class_<theia::RadialDistUncalibratedAbsolutePose>(
      m, "RadialDistUncalibratedAbsolutePose")
      .def(py::init<>())
      .def_readwrite("rotation",
                     &theia::RadialDistUncalibratedAbsolutePose::rotation)
      .def_readwrite("translation",
                     &theia::RadialDistUncalibratedAbsolutePose::translation)
      .def_readwrite(
          "focal_length",
          &theia::RadialDistUncalibratedAbsolutePose::focal_length)
      .def_readwrite("radial_distortion",
                     &theia::RadialDistUncalibratedAbsolutePose::
                         radial_distortion);

  py::class_<theia::UncalibratedRelativePose>(m, "UncalibratedRelativePose")
      .def(py::init<>())
      .def_readwrite("fundamental_matrix",
                     &theia::UncalibratedRelativePose::fundamental_matrix)
      .def_readwrite("focal_length1",
                     &theia::UncalibratedRelativePose::focal_length1)
      .def_readwrite("focal_length2",
                     &theia::UncalibratedRelativePose::focal_length2)
      .def_readwrite("rotation", &theia::UncalibratedRelativePose::rotation)
      .def_readwrite("position", &theia::UncalibratedRelativePose::position);

  py::class_<theia::RelativePose>(m, "RelativePose")
      .def(py::init<>())
      .def_readwrite("essential_matrix", &theia::RelativePose::essential_matrix)
      .def_readwrite("rotation", &theia::RelativePose::rotation)
      .def_readwrite("position", &theia::RelativePose::position);

  py::class_<theia::GeneralizedRayCorrespondence>(
      m, "GeneralizedRayCorrespondence")
      .def(py::init<>())
      .def_readwrite("origin1", &theia::GeneralizedRayCorrespondence::origin1)
      .def_readwrite("direction1",
                     &theia::GeneralizedRayCorrespondence::direction1)
      .def_readwrite("origin2", &theia::GeneralizedRayCorrespondence::origin2)
      .def_readwrite("direction2",
                     &theia::GeneralizedRayCorrespondence::direction2);

  py::class_<theia::RelativeRigInfo>(m, "RelativeRigInfo")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::RelativeRigInfo::rotation)
      .def_readwrite("translation", &theia::RelativeRigInfo::translation)
      .def_readwrite("position", &theia::RelativeRigInfo::position)
      .def("ToTwoViewInfo",
           [](const theia::RelativeRigInfo& self) {
             theia::TwoViewInfo info;
             self.ToTwoViewInfo(&info);
             return info;
           });

  py::class_<theia::MonoDepthRelativePoseResult>(
      m, "MonoDepthRelativePoseResult")
      .def(py::init<>())
      .def_readwrite("rotation", &theia::MonoDepthRelativePoseResult::rotation)
      .def_readwrite("position", &theia::MonoDepthRelativePoseResult::position)
      .def_readwrite("scale", &theia::MonoDepthRelativePoseResult::scale)
      .def_readwrite("shift1", &theia::MonoDepthRelativePoseResult::shift1)
      .def_readwrite("shift2", &theia::MonoDepthRelativePoseResult::shift2)
      .def_readwrite("focal_length1",
                     &theia::MonoDepthRelativePoseResult::focal_length1)
      .def_readwrite("focal_length2",
                     &theia::MonoDepthRelativePoseResult::focal_length2);

  py::class_<theia::Plane>(m, "Plane")
      .def(py::init<>())
      .def_readwrite("point", &theia::Plane::point)
      .def_readwrite("unit_normal", &theia::Plane::unit_normal);

  py::class_<theia::RadialDistortionFeatureCorrespondence>(
      m, "RadialDistortionFeatureCorrespondence")
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
      m, "RadialDistUncalibratedAbsolutePoseMetaData")
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
  py::enum_<theia::RansacType>(m, "RansacType")
      .value("RANSAC", theia::RansacType::RANSAC)
      .value("PROSAC", theia::RansacType::PROSAC)
      .value("LMED", theia::RansacType::LMED)
      .value("EXHAUSTIVE", theia::RansacType::EXHAUSTIVE)
      .export_values();

  py::enum_<theia::PnPType>(m, "PnPType")
      .value("KNEIP", theia::PnPType::KNEIP)
      .value("DLS", theia::PnPType::DLS)
      .value("SQPnP", theia::PnPType::SQPnP)
      .value("MLPnP", theia::PnPType::MLPnP);
  // Do not export MLPnP at module scope: it would shadow the MLPnP() solver.
  m.attr("KNEIP") = theia::PnPType::KNEIP;
  m.attr("DLS") = theia::PnPType::DLS;
  // Do not export SQPnP at module scope either: it would shadow the SQPnP()
  // solver. Use PnPType.SQPnP.

  m.def("EstimateAbsolutePoseWithKnownOrientation",
        theia::EstimateAbsolutePoseWithKnownOrientationWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("camera_orientation"), py::arg("normalized_correspondences"));
  m.def("EstimateCalibratedAbsolutePose",
        theia::EstimateCalibratedAbsolutePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"), py::arg("pnp_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateDominantPlaneFromPoints",
        theia::EstimateDominantPlaneFromPointsWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"), py::arg("points"));

  m.def("EstimateEssentialMatrix", theia::EstimateEssentialMatrixWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateFundamentalMatrix", theia::EstimateFundamentalMatrixWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("correspondences"));
  m.def("EstimateHomography", theia::EstimateHomographyWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("correspondences"));
  m.def("EstimateRadialHomographyMatrix",
        theia::EstimateRadialHomographyMatrixWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateRelativePose", theia::EstimateRelativePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateRelativeRigInfo", theia::EstimateRelativeRigInfoWrapper,
        py::arg("ransac_params"), py::arg("central_matches"),
        py::arg("generalized_matches"));
  m.def("EstimateRelativeRigInfoUpright",
        theia::EstimateRelativeRigInfoUprightWrapper,
        py::arg("ransac_params"), py::arg("gravity_axis"), py::arg("matches"));
  m.def("EstimateMonoDepthRelativePose",
        theia::EstimateMonoDepthRelativePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"),
        py::arg("depth_reprojection_weight") = 0.0);
  m.def("EstimateMonoDepthRelativePoseSharedFocal",
        theia::EstimateMonoDepthRelativePoseSharedFocalWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("centered_correspondences"));
  m.def("EstimateMonoDepthRelativePoseVaryingFocal",
        theia::EstimateMonoDepthRelativePoseVaryingFocalWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("centered_correspondences"));
  m.def("EstimateRelativePoseWithKnownOrientation",
        theia::EstimateRelativePoseWithKnownOrientationWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("rotated_correspondences"));
  m.def("EstimateRigidTransformation2D3D",
        theia::EstimateRigidTransformation2D3DWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("correspondences"));
  m.def("EstimateRigidTransformation2D3DNormalized",
        theia::EstimateRigidTransformation2D3DNormalizedWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateSimilarityTransformation2D3D",
        theia::EstimateSimilarityTransformation2D3DWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("correspondences"));
  m.def("EstimateTriangulation", theia::EstimateTriangulationWrapper,
        py::arg("ransac_params"), py::arg("cameras"), py::arg("features"));
  m.def("EstimateUncalibratedAbsolutePose",
        theia::EstimateUncalibratedAbsolutePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"));
  m.def("EstimateRadialDistUncalibratedAbsolutePose",
        theia::EstimateRadialDistUncalibratedAbsolutePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("normalized_correspondences"), py::arg("meta_data"));
  m.def("EstimateUncalibratedRelativePose",
        theia::EstimateUncalibratedRelativePoseWrapper,
        py::arg("ransac_params"), py::arg("ransac_type"),
        py::arg("centered_correspondences"), py::arg("min_max_focal_length"));

  // triangulation
  m.def("Triangulate", theia::TriangulateWrapper,
        py::arg("pose1"), py::arg("pose2"), py::arg("point1"),
        py::arg("point2"));
  m.def("TriangulateMidpoint", theia::TriangulateMidpointWrapper,
        py::arg("origins"), py::arg("ray_directions"));
  m.def("TriangulateDLT", theia::TriangulateDLTWrapper,
        py::arg("pose1"), py::arg("pose2"), py::arg("point1"),
        py::arg("point2"));
  m.def("TriangulateNViewSVD", theia::TriangulateNViewSVDWrapper,
        py::arg("poses"), py::arg("points"));
  m.def("TriangulateNView", theia::TriangulateNViewWrapper,
        py::arg("poses"), py::arg("points"));
  m.def("IsTriangulatedPointInFrontOfCameras",
        theia::IsTriangulatedPointInFrontOfCameras,
        py::arg("correspondence"), py::arg("rotation"), py::arg("position"));
  m.def("SufficientTriangulationAngle", theia::SufficientTriangulationAngle,
        py::arg("ray_directions"), py::arg("min_triangulation_angle_degrees"));

  // function in the sfm folder

  py::class_<theia::EstimateTwoViewInfoOptions>(m, "EstimateTwoViewInfoOptions")
      .def(py::init<>())
      .def_readwrite("ransac_type",
                     &theia::EstimateTwoViewInfoOptions::ransac_type)
      .def_readwrite(
          "max_sampson_error_pixels",
          &theia::EstimateTwoViewInfoOptions::max_sampson_error_pixels)
      .def_readwrite(
          "expected_ransac_confidence",
          &theia::EstimateTwoViewInfoOptions::expected_ransac_confidence)
      .def_readwrite("scale_threshold_with_resolution",
                     &theia::EstimateTwoViewInfoOptions::
                         scale_threshold_with_resolution)
      .def_readwrite("min_inlier_ratio",
                     &theia::EstimateTwoViewInfoOptions::min_inlier_ratio)
      .def_readwrite("min_ransac_iterations",
                     &theia::EstimateTwoViewInfoOptions::min_ransac_iterations)
      .def_readwrite("max_ransac_iterations",
                     &theia::EstimateTwoViewInfoOptions::max_ransac_iterations)
      .def_readwrite("use_mle", &theia::EstimateTwoViewInfoOptions::use_mle)
      .def_readwrite("use_lo", &theia::EstimateTwoViewInfoOptions::use_lo)
      .def_readwrite("lo_start_iterations",
                     &theia::EstimateTwoViewInfoOptions::lo_start_iterations)
      .def_readwrite("use_sturm_5pt",
                     &theia::EstimateTwoViewInfoOptions::use_sturm_5pt)
      .def_readwrite("use_monodepth",
                     &theia::EstimateTwoViewInfoOptions::use_monodepth)
      .def_readwrite(
          "monodepth_shared_focal",
          &theia::EstimateTwoViewInfoOptions::monodepth_shared_focal)
      .def_readwrite(
          "monodepth_reprojection_weight",
          &theia::EstimateTwoViewInfoOptions::monodepth_reprojection_weight)
      .def_readwrite("min_focal_length",
                     &theia::EstimateTwoViewInfoOptions::min_focal_length)
      .def_readwrite("max_focal_length",
                     &theia::EstimateTwoViewInfoOptions::max_focal_length);

  py::class_<theia::FilterViewPairsFromRelativeTranslationOptions>(
      m, "FilterViewPairsFromRelativeTranslationOptions")
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
      m, "LocalizeViewToReconstructionOptions")
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
        py::arg("options"), py::arg("intrinsics1"), py::arg("intrinsics2"),
        py::arg("correspondences"));
  m.def(
      "BulkEstimateTwoViewInfo",
      [](py::array_t<uint64_t, py::array::c_style | py::array::forcecast>
             pair_offsets,
         py::array_t<float, py::array::c_style | py::array::forcecast>
             points_i,
         py::array_t<float, py::array::c_style | py::array::forcecast>
             points_j,
         const theia::CameraIntrinsicsPrior& camera_prior_i,
         const theia::CameraIntrinsicsPrior& camera_prior_j,
         const theia::EstimateTwoViewInfoOptions& options,
         int num_threads,
         py::object depth_i,
         py::object depth_j) {
        const auto offsets_buf = pair_offsets.unchecked<1>();
        const auto pi = points_i.unchecked<2>();
        const auto pj = points_j.unchecked<2>();
        if (pi.shape(1) != 2 || pj.shape(1) != 2) {
          throw py::value_error("points_i/points_j must have shape [N, 2]");
        }
        if (pi.shape(0) != pj.shape(0)) {
          throw py::value_error("points_i and points_j must have equal length");
        }
        if (offsets_buf.shape(0) < 1) {
          throw py::value_error("pair_offsets must not be empty");
        }
        const size_t num_corrs = static_cast<size_t>(pi.shape(0));
        if (static_cast<size_t>(offsets_buf(offsets_buf.shape(0) - 1)) !=
            num_corrs) {
          throw py::value_error(
              "pair_offsets final entry must equal correspondence count");
        }

        const bool has_depth_i = !depth_i.is_none();
        const bool has_depth_j = !depth_j.is_none();
        if (has_depth_i != has_depth_j) {
          throw py::value_error(
              "depth_i and depth_j must both be provided or both None");
        }

        py::array_t<float, py::array::c_style | py::array::forcecast>
            depth_i_arr;
        py::array_t<float, py::array::c_style | py::array::forcecast>
            depth_j_arr;
        if (has_depth_i) {
          depth_i_arr =
              depth_i
                  .cast<py::array_t<float,
                                    py::array::c_style | py::array::forcecast>>();
          depth_j_arr =
              depth_j
                  .cast<py::array_t<float,
                                    py::array::c_style | py::array::forcecast>>();
          if (depth_i_arr.ndim() != 1 || depth_j_arr.ndim() != 1) {
            throw py::value_error("depth_i/depth_j must be 1-D arrays");
          }
          if (static_cast<size_t>(depth_i_arr.shape(0)) != num_corrs ||
              static_cast<size_t>(depth_j_arr.shape(0)) != num_corrs) {
            throw py::value_error(
                "depth_i/depth_j length must equal correspondence count");
          }
        }

        std::vector<uint64_t> offsets(offsets_buf.shape(0));
        for (py::ssize_t i = 0; i < offsets_buf.shape(0); ++i) {
          offsets[i] = offsets_buf(i);
        }
        std::vector<theia::FeatureCorrespondence> correspondences;
        correspondences.reserve(num_corrs);
        if (has_depth_i) {
          const auto di = depth_i_arr.unchecked<1>();
          const auto dj = depth_j_arr.unchecked<1>();
          for (size_t i = 0; i < num_corrs; ++i) {
            const double depth1 = static_cast<double>(di(i));
            const double depth2 = static_cast<double>(dj(i));
            correspondences.emplace_back(
                theia::Feature(static_cast<double>(pi(i, 0)),
                               static_cast<double>(pi(i, 1)),
                               depth1 > 0.0 ? depth1 : 0.0),
                theia::Feature(static_cast<double>(pj(i, 0)),
                               static_cast<double>(pj(i, 1)),
                               depth2 > 0.0 ? depth2 : 0.0));
          }
        } else {
          for (size_t i = 0; i < num_corrs; ++i) {
            correspondences.emplace_back(
                theia::Feature(static_cast<double>(pi(i, 0)),
                               static_cast<double>(pi(i, 1))),
                theia::Feature(static_cast<double>(pj(i, 0)),
                               static_cast<double>(pj(i, 1))));
          }
        }

        std::vector<uint8_t> success;
        std::vector<Eigen::Vector3d> rotations;
        std::vector<Eigen::Vector3d> positions;
        std::vector<uint64_t> inlier_offsets;
        std::vector<int> inlier_indices;
        std::vector<double> scales;
        {
          py::gil_scoped_release release;
          std::tie(success, rotations, positions, inlier_offsets,
                   inlier_indices, scales) =
              theia::BulkEstimateTwoViewInfoWrapper(options,
                                                    camera_prior_i,
                                                    camera_prior_j,
                                                    offsets,
                                                    correspondences,
                                                    num_threads);
        }

        const py::ssize_t num_pairs = static_cast<py::ssize_t>(success.size());
        py::array_t<bool> success_arr(num_pairs);
        py::array_t<double> orientations_arr({num_pairs, py::ssize_t(3)});
        py::array_t<double> positions_arr({num_pairs, py::ssize_t(3)});
        py::array_t<double> scales_arr(num_pairs);
        auto success_mut = success_arr.mutable_unchecked<1>();
        auto orient_mut = orientations_arr.mutable_unchecked<2>();
        auto pos_mut = positions_arr.mutable_unchecked<2>();
        auto scales_mut = scales_arr.mutable_unchecked<1>();
        for (py::ssize_t p = 0; p < num_pairs; ++p) {
          success_mut(p) = success[p] != 0;
          scales_mut(p) = scales[p];
          for (py::ssize_t k = 0; k < 3; ++k) {
            orient_mut(p, k) = rotations[p][k];
            pos_mut(p, k) = positions[p][k];
          }
        }
        py::array_t<uint64_t> inlier_offsets_arr(
            static_cast<py::ssize_t>(inlier_offsets.size()));
        std::copy(inlier_offsets.begin(),
                  inlier_offsets.end(),
                  inlier_offsets_arr.mutable_data());
        py::array_t<int64_t> inlier_indices_arr(
            static_cast<py::ssize_t>(inlier_indices.size()));
        std::copy(inlier_indices.begin(),
                  inlier_indices.end(),
                  inlier_indices_arr.mutable_data());

        py::dict out;
        out["success"] = std::move(success_arr);
        out["orientations"] = std::move(orientations_arr);
        out["positions"] = std::move(positions_arr);
        out["inlier_offsets"] = std::move(inlier_offsets_arr);
        out["inlier_indices"] = std::move(inlier_indices_arr);
        out["scales"] = std::move(scales_arr);
        return out;
      },
      py::arg("pair_offsets"),
      py::arg("points_i"),
      py::arg("points_j"),
      py::arg("camera_prior_i"),
      py::arg("camera_prior_j"),
      py::arg("options"),
      py::arg("num_threads") = 0,
      py::arg("depth_i") = py::none(),
      py::arg("depth_j") = py::none());
  // The C++ implementation was a stub (image loading was removed from Theia and
  // the body never assigned colors), so it silently left all tracks black.
  // Fail loudly instead; set colors with Track.SetColor from your own image
  // loader (see pyexamples/eth3d for a pixel sampling example).
  m.def("ColorizeReconstruction",
        [](const std::string&, const int, theia::Reconstruction*) {
          throw py::value_error(
              "ColorizeReconstruction is not implemented (Theia has no image "
              "I/O). Sample pixel colors in Python and call Track.SetColor.");
        },
        py::arg("image_directory"),
        py::arg("num_threads"),
        py::arg("reconstruction"));
  m.def("ExtractMaximallyParallelRigidSubgraph",
        theia::ExtractMaximallyParallelRigidSubgraph,
        py::arg("orientations"),
        py::arg("view_graph"));
  m.def("FilterViewGraphCyclesByRotation",
        theia::FilterViewGraphCyclesByRotation,
        py::arg("max_loop_error_degrees"),
        py::arg("view_pairs"));
  m.def("FilterViewGraphByRelativeInliers",
        theia::FilterViewGraphByRelativeInliers,
        py::arg("min_relative_inliers"),
        py::arg("view_graph"),
        "Remove edges with fewer verified matches than min_relative_inliers "
        "times the strongest edge of either endpoint view. Returns the number "
        "of removed edges. 0 disables.");
  m.def("PruneTracksToViewGraph",
        theia::PruneTracksToViewGraph,
        py::arg("view_graph"),
        py::arg("reconstruction"),
        "Remove track observations whose view has no remaining view graph "
        "edge to another view of the track; removes tracks with < 2 views. "
        "Returns the number of removed observations.");
  m.def("FilterViewPairsFromOrientation",
        theia::FilterViewPairsFromOrientation,
        py::arg("orientations"),
        py::arg("max_relative_rotation_difference_degrees"),
        py::arg("view_pairs"));
  m.def("FilterViewPairsFromRelativeTranslation",
        theia::FilterViewPairsFromRelativeTranslation,
        py::arg("options"),
        py::arg("orientations"),
        py::arg("view_graph"));
  m.def("LocalizeViewToReconstruction",
        theia::LocalizeViewToReconstruction,
        py::arg("view_to_localize"),
        py::arg("options"),
        py::arg("reconstruction"),
        py::arg("summary"));
  m.def("SelectGoodTracksForBundleAdjustment",
        theia::SelectGoodTracksForBundleAdjustmentWrapper,
        py::arg("reconstruction"),
        py::arg("view_ids"),
        py::arg("long_track_length_threshold"),
        py::arg("image_grid_cell_size_pixels"),
        py::arg("min_num_optimized_tracks_per_view"));
  m.def("SetOutlierTracksToUnestimated",
        theia::SetOutlierTracksToUnestimatedWrapper,
        py::arg("tracks"),
        py::arg("max_inlier_reprojection_error"),
        py::arg("min_triangulation_angle_degrees"),
        py::arg("reconstruction"));
  m.def("ComputeMeanReprojectionError",
        theia::ComputeMeanReprojectionErrorWrapper,
        py::arg("reconstruction"),
        "Mean L2 pixel reprojection error over estimated views and tracks.");
  m.def("SetCameraIntrinsicsFromPriors",
        theia::SetCameraIntrinsicsFromPriors,
        py::arg("reconstruction"));
  m.def("FindCommonViewsByName",
        theia::FindCommonViewsByName,
        py::arg("reconstruction1"),
        py::arg("reconstruction2"));
  m.def("FindCommonTracksInViews",
        theia::FindCommonTracksInViews,
        py::arg("reconstruction"),
        py::arg("views"));
  m.def("RemoveDisconnectedViewPairs",
        theia::RemoveDisconnectedViewPairs,
        py::arg("view_graph"));
  m.def("AddFeatureCorrespondencesToTrackBuilder",
        theia::AddFeatureCorrespondencesToTrackBuilderWrapper,
        py::arg("view_id1"),
        py::arg("features1"),
        py::arg("view_id2"),
        py::arg("features2"),
        py::arg("track_builder"));
  m.def("AddFullFeatureCorrespondencesToTrackBuilder",
        theia::AddFullFeatureCorrespondencesToTrackBuilderWrapper,
        py::arg("view_id1"),
        py::arg("features1"),
        py::arg("view_id2"),
        py::arg("features2"),
        py::arg("track_builder"));
  m.def("BulkAddViewGraphEdgesAndCorrespondences",
        theia::BulkAddViewGraphEdgesAndCorrespondencesWrapper,
        py::arg("view_graph"),
        py::arg("track_builder"),
        py::arg("view_ids1"),
        py::arg("view_ids2"),
        py::arg("twoview_infos"),
        py::arg("features1"),
        py::arg("features2"));
  m.def("UpdateFeaturesInView",
        theia::UpdateFeaturesInViewWrapper,
        py::arg("view_id"),
        py::arg("track_ids"),
        py::arg("new_features"),
        py::arg("covariances"),
        py::arg("reconstruction"));
  m.def("FindCommonTracksByFeatureInReconstructions", 
        theia::FindCommonTracksByFeatureInReconstructionsWrapper,
        py::arg("reconstruction_ref"),
        py::arg("reconstruction_qry"),
        py::arg("view_graph_matches_ref_qry"));
  m.def("AddObservations",
        theia::AddObservationsWrapper,
        py::arg("view_id"),
        py::arg("track_ids"),
        py::arg("features"),
        py::arg("reconstruction"));
  m.def("AddTracks",
        theia::AddTracksWrapper,
        py::arg("tracks"),
        py::arg("colors"),
        py::arg("reconstruction"));

  // View class
  py::class_<theia::View>(m, "View")
      .def(py::init<>())
      .def(py::init<std::string>(), py::arg("name"))
      .def("Name", &theia::View::Name)
      .def("IsEstimated", &theia::View::IsEstimated)
      .def("SetIsEstimated", &theia::View::SetEstimated, py::arg("is_estimated"))
      .def("NumFeatures", &theia::View::NumFeatures)
      .def("AddFeature",
           &theia::View::AddFeature,
           py::arg("track_id"),
           py::arg("feature"))
      .def("UpdateFeature",
           &theia::View::UpdateFeature,
           py::arg("track_id"),
           py::arg("feature"))
      .def("RemoveFeature", &theia::View::RemoveFeature, py::arg("track_id"))
      .def("TrackIds", &theia::View::TrackIds)
      .def("GetFeature",
           &theia::View::GetFeature,
           py::arg("track_id"),
           py::return_value_policy::reference_internal)
      .def("GetTrack", &theia::View::GetTrack, py::arg("feature"))
      .def("GetTimestamp", &theia::View::GetTimestamp)
      .def("Camera", &theia::View::Camera, "Camera class object")
      .def("CameraIntrinsicsPrior", &theia::View::CameraIntrinsicsPrior)
      .def("SetCameraIntrinsicsPrior",
           &theia::View::SetCameraIntrinsicsPrior,
           py::arg("prior"))
      .def("MutableCameraIntrinsicsPrior",
           &theia::View::MutableCameraIntrinsicsPrior,
           py::return_value_policy::reference_internal)
      .def("MutableCamera",
           &theia::View::MutableCamera,
           py::return_value_policy::reference_internal)
      .def("GetPositionPrior", &theia::View::GetPositionPrior)
      .def("GetPositionPriorSqrtInformation",
           &theia::View::GetPositionPriorSqrtInformation)
      .def("SetPositionPrior",
           &theia::View::SetPositionPrior,
           py::arg("position_prior"),
           py::arg("position_prior_information"),
           "Set the position prior. NOTE: despite its name, the matrix argument is "
           "the SQUARE ROOT of the information matrix (e.g. np.eye(3) / sigma), "
           "which bundle adjustment uses directly as the residual weight.")
      .def("HasPositionPrior", &theia::View::HasPositionPrior)
      .def("GetGravityPrior", &theia::View::GetGravityPrior)
      .def("GetGravityPriorSqrtInformation",
           &theia::View::GetGravityPriorSqrtInformation)
      .def("SetGravityPrior",
           &theia::View::SetGravityPrior,
           py::arg("gravity_prior"),
           py::arg("gravity_prior_information"),
           "Set the gravity prior. NOTE: despite its name, the matrix argument is "
           "the SQUARE ROOT of the information matrix (e.g. np.eye(3) / sigma), "
           "which bundle adjustment uses directly as the residual weight.")
      .def("HasGravityPrior", &theia::View::HasGravityPrior)
      .def("GetOrientationPrior", &theia::View::GetOrientationPrior)
      .def("GetOrientationPriorSqrtInformation",
           &theia::View::GetOrientationPriorSqrtInformation)
      .def("SetOrientationPrior",
           &theia::View::SetOrientationPrior,
           py::arg("orientation_prior"),
           py::arg("orientation_prior_information"),
           "Set the orientation prior. NOTE: despite its name, the matrix argument is "
           "the SQUARE ROOT of the information matrix (e.g. np.eye(3) / sigma), "
           "which bundle adjustment uses directly as the residual weight.")
      .def("HasOrientationPrior", &theia::View::HasOrientationPrior);

  // Visibility pyramid
  py::class_<theia::VisibilityPyramid>(m, "VisibilityPyramid")
      .def(py::init<int, int, int>(),
           py::arg("width"),
           py::arg("height"),
           py::arg("num_pyramid_levels"))
      .def("AddPoint", &theia::VisibilityPyramid::AddPoint, py::arg("point"))
      .def("ComputeScore", &theia::VisibilityPyramid::ComputeScore);

  // Track class
  py::class_<theia::Track>(m, "Track")
      .def(py::init<>())
      .def("SetIsEstimated", &theia::Track::SetEstimated, py::arg("is_estimated"))
      .def("IsEstimated", &theia::Track::IsEstimated)
      .def("NumViews", &theia::Track::NumViews)
      .def("AddView", &theia::Track::AddView, py::arg("view_id"))
      .def("RemoveView", &theia::Track::RemoveView, py::arg("view_id"))
      .def("ViewIds", &theia::Track::ViewIds)
      .def("Point", &theia::Track::Point)
      .def("SetPoint", &theia::Track::SetPoint, py::arg("point"))
      .def("Color", &theia::Track::Color)
      .def("SetColor", &theia::Track::SetColor, py::arg("color"))
      .def("ReferenceViewId", &theia::Track::ReferenceViewId)
      .def("SetReferenceViewId",
           &theia::Track::SetReferenceViewId,
           py::arg("view_id"))
      .def("InverseDepth", &theia::Track::InverseDepth)
      .def("SetInverseDepth", &theia::Track::SetInverseDepth, py::arg("inverse_depth"))
      .def("SetReferenceBearingVector",
           &theia::Track::SetReferenceBearingVector,
           py::arg("ref_bearing"))
      .def("ReferenceBearingVector", &theia::Track::ReferenceBearingVector)
      .def("SetReferenceDescriptor", &theia::Track::SetReferenceDescriptor, py::arg("descriptor"))
      .def("ReferenceDescriptor", &theia::Track::ReferenceDescriptor)
      .def("SetPositionPrior",
           &theia::Track::SetPositionPrior,
           py::arg("position"),
           py::arg("sqrt_information"))
      .def("SetPositionPriorCovariance",
           &theia::Track::SetPositionPriorCovariance,
           py::arg("position"),
           py::arg("covariance"))
      .def("HasPositionPrior", &theia::Track::HasPositionPrior)
      .def("PositionPrior", &theia::Track::PositionPrior)
      .def("PositionPriorSqrtInformation",
           &theia::Track::PositionPriorSqrtInformation)
      .def("RemovePositionPrior", &theia::Track::RemovePositionPrior);

  // Track builder class
  py::class_<theia::TrackBuilder>(m, "TrackBuilder")
      .def(py::init<int, int>(),
           py::arg("min_track_length"),
           py::arg("max_track_length"))
      .def("AddFeatureCorrespondence",
           &theia::TrackBuilder::AddFeatureCorrespondence,
           py::arg("view_id1"),
           py::arg("feature1"),
           py::arg("view_id2"),
           py::arg("feature2"))
      .def("BuildTracks",
           &theia::TrackBuilder::BuildTracks,
           py::arg("reconstruction"))
      .def("BuildTracksIncremental",
           &theia::TrackBuilder::BuildTracksIncremental,
           py::arg("reconstruction"));

  py::class_<theia::BundleAdjustmentOptions>(m, "BundleAdjustmentOptions")
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
      .def_readwrite("gravity_prior_error_type",
                     &theia::BundleAdjustmentOptions::gravity_prior_error_type)
      .def_readwrite("gravity_world_direction",
                     &theia::BundleAdjustmentOptions::gravity_world_direction)
      .def_readwrite("use_depth_priors",
                     &theia::BundleAdjustmentOptions::use_depth_priors)
      .def_readwrite("orthographic_camera",
                     &theia::BundleAdjustmentOptions::orthographic_camera)
      .def_readwrite("analytic_jacobians",
                     &theia::BundleAdjustmentOptions::analytic_jacobians)
      .def_readwrite("use_homogeneous_point_parametrization",
                     &theia::BundleAdjustmentOptions::
                         use_homogeneous_point_parametrization)
      .def_readwrite("robust_loss_width_depth_prior",
                     &theia::BundleAdjustmentOptions::robust_loss_width_depth_prior)
      .def_readwrite("use_point_priors",
                     &theia::BundleAdjustmentOptions::use_point_priors)
      .def_readwrite("robust_loss_width_point_prior",
                     &theia::BundleAdjustmentOptions::
                         robust_loss_width_point_prior)
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
  py::enum_<theia::TriangulationMethodType>(m, "TriangulationMethodType")
      .value("MIDPOINT", theia::TriangulationMethodType::MIDPOINT)
      .value("SVD", theia::TriangulationMethodType::SVD)
      .value("L2_MINIMIZATION", theia::TriangulationMethodType::L2_MINIMIZATION)
      .export_values();

  // Track Estimator Options
  py::class_<theia::TrackEstimator::Options>(m, "TrackEstimatorOptions")
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
  py::class_<theia::TrackEstimator::Summary>(m, "TrackEstimatorSummary")
      .def_readwrite(
          "input_num_estimated_tracks",
          &theia::TrackEstimator::Summary::input_num_estimated_tracks)
      .def_readwrite(
          "num_triangulation_attempts",
          &theia::TrackEstimator::Summary::num_triangulation_attempts)
      .def_readwrite("estimated_tracks",
                     &theia::TrackEstimator::Summary::estimated_tracks);

  // Track Estimator class
  py::class_<theia::TrackEstimator>(m, "TrackEstimator")
      .def(py::init<const theia::TrackEstimator::Options&,
                    theia::Reconstruction*>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimateAllTracks", &theia::TrackEstimator::EstimateAllTracks)
      .def("EstimateTracks",
           &theia::TrackEstimator::EstimateTracks,
           py::arg("track_ids"));

  // ReconstructionEstimatorSummary
  py::class_<theia::ReconstructionEstimatorSummary>(
      m, "ReconstructionEstimatorSummary")
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

  // Reconstruction Estimator class (unique_ptr holder: Create transfers ownership)
  py::class_<theia::ReconstructionEstimator,
             std::unique_ptr<theia::ReconstructionEstimator>>(
      m, "ReconstructionEstimator")
      .def_static("Create",
                  &theia::ReconstructionEstimator::Create,
                  py::arg("options"));

  py::class_<theia::GlobalReconstructionEstimator,
             theia::ReconstructionEstimator,
             std::unique_ptr<theia::GlobalReconstructionEstimator>>(
      m, "GlobalReconstructionEstimator")
      .def(py::init<theia::ReconstructionEstimatorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::GlobalReconstructionEstimator::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"));

  py::class_<theia::IncrementalReconstructionEstimator,
             theia::ReconstructionEstimator,
             std::unique_ptr<theia::IncrementalReconstructionEstimator>>(
      m, "IncrementalReconstructionEstimator")
      .def(py::init<theia::ReconstructionEstimatorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::IncrementalReconstructionEstimator::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"));

  py::class_<theia::IncrementalRigReconstructorOptions>(
      m, "IncrementalRigReconstructorOptions")
      .def(py::init<>())
      .def_readwrite("min_num_abs_pose_features",
                     &theia::IncrementalRigReconstructorOptions::
                         min_num_abs_pose_features)
      .def_readwrite("min_num_inliers_for_localization",
                     &theia::IncrementalRigReconstructorOptions::
                         min_num_inliers_for_localization)
      .def_readwrite("max_reprojection_error_in_pixels",
                     &theia::IncrementalRigReconstructorOptions::
                         max_reprojection_error_in_pixels)
      .def_readwrite("min_triangulation_angle_degrees",
                     &theia::IncrementalRigReconstructorOptions::
                         min_triangulation_angle_degrees)
      .def_readwrite("bundle_adjust_after_localize",
                     &theia::IncrementalRigReconstructorOptions::
                         bundle_adjust_after_localize)
      .def_readwrite("bundle_adjust_every_n_captures",
                     &theia::IncrementalRigReconstructorOptions::
                         bundle_adjust_every_n_captures)
      .def_readwrite("num_seed_candidates",
                     &theia::IncrementalRigReconstructorOptions::
                         num_seed_candidates)
      .def_readwrite("use_generalized_localization",
                     &theia::IncrementalRigReconstructorOptions::
                         use_generalized_localization)
      .def_readwrite("use_rig_constrained_ba",
                     &theia::IncrementalRigReconstructorOptions::
                         use_rig_constrained_ba)
      .def_readwrite(
          "ba_options",
          &theia::IncrementalRigReconstructorOptions::ba_options);

  py::class_<theia::IncrementalRigReconstructor>(
      m, "IncrementalRigReconstructor")
      .def(py::init<theia::IncrementalRigReconstructorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::IncrementalRigReconstructor::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"));

  py::class_<theia::BuildCaptureViewGraphOptions>(
      m, "BuildCaptureViewGraphOptions")
      .def(py::init<>())
      .def_readwrite(
          "use_metric_relative_rig_pose",
          &theia::BuildCaptureViewGraphOptions::use_metric_relative_rig_pose)
      .def_readwrite(
          "fallback_to_twoview_strip",
          &theia::BuildCaptureViewGraphOptions::fallback_to_twoview_strip)
      .def_readwrite(
          "relative_rig_ransac",
          &theia::BuildCaptureViewGraphOptions::relative_rig_ransac);

  py::class_<theia::GlobalRigReconstructorOptions>(
      m, "GlobalRigReconstructorOptions")
      .def(py::init<>())
      .def_readwrite("sfm_options",
                     &theia::GlobalRigReconstructorOptions::sfm_options)
      .def_readwrite(
          "capture_graph_options",
          &theia::GlobalRigReconstructorOptions::capture_graph_options)
      .def_readwrite(
          "rescale_positions_to_metric_edges",
          &theia::GlobalRigReconstructorOptions::
              rescale_positions_to_metric_edges)
      .def_readwrite(
          "use_rig_constraints_in_position_estimation",
          &theia::GlobalRigReconstructorOptions::
              use_rig_constraints_in_position_estimation)
      .def_readwrite("use_rig_constrained_ba",
                     &theia::GlobalRigReconstructorOptions::
                         use_rig_constrained_ba)
      .def_readwrite("use_rig_edge_scales",
                     &theia::GlobalRigReconstructorOptions::
                         use_rig_edge_scales);

  py::class_<theia::GlobalRigReconstructor>(m, "GlobalRigReconstructor")
      .def(py::init<theia::GlobalRigReconstructorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::GlobalRigReconstructor::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"));

  m.def("BuildCaptureViewGraph",
        [](const theia::Reconstruction& reconstruction,
           const theia::ViewGraph& view_graph,
           theia::ViewGraph& capture_view_graph) {
          return theia::BuildCaptureViewGraph(
              reconstruction, view_graph, &capture_view_graph);
        },
        py::arg("reconstruction"),
        py::arg("view_graph"),
        py::arg("capture_view_graph"));
  m.def("BuildCaptureViewGraph",
        [](const theia::Reconstruction& reconstruction,
           const theia::ViewGraph& view_graph,
           theia::ViewGraph& capture_view_graph,
           const theia::BuildCaptureViewGraphOptions& options) {
          return theia::BuildCaptureViewGraph(
              reconstruction, view_graph, &capture_view_graph, options);
        },
        py::arg("reconstruction"),
        py::arg("view_graph"),
        py::arg("capture_view_graph"),
        py::arg("options"));

  py::class_<theia::HybridReconstructionEstimator,
             theia::ReconstructionEstimator,
             std::unique_ptr<theia::HybridReconstructionEstimator>>(
      m, "HybridReconstructionEstimator")
      .def(py::init<theia::ReconstructionEstimatorOptions>(),
           py::arg("options"))
      .def("Estimate",
           &theia::HybridReconstructionEstimator::Estimate,
           py::arg("view_graph"),
           py::arg("reconstruction"));

  // Reconstruction Options
  py::enum_<theia::ReconstructionEstimatorType>(m,
                                                "ReconstructionEstimatorType")
      .value("GLOBAL", theia::ReconstructionEstimatorType::GLOBAL)
      .value("INCREMENTAL", theia::ReconstructionEstimatorType::INCREMENTAL)
      .value("HYBRID", theia::ReconstructionEstimatorType::HYBRID)
      .value("INIT_FREE", theia::ReconstructionEstimatorType::INIT_FREE)
      .export_values();

  py::enum_<theia::GlobalPositionEstimatorType>(m,
                                                "GlobalPositionEstimatorType")
      .value("NONLINEAR", theia::GlobalPositionEstimatorType::NONLINEAR)
      .value("LINEAR_TRIPLET",
             theia::GlobalPositionEstimatorType::LINEAR_TRIPLET)
      .value("LEAST_UNSQUARED_DEVIATION",
             theia::GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION)
      .value("LIGT", theia::GlobalPositionEstimatorType::LIGT)
      .value("GLOMAP", theia::GlobalPositionEstimatorType::GLOMAP)
      .value("ART", theia::GlobalPositionEstimatorType::ART)
      .export_values();

  py::enum_<theia::GlobalRotationEstimatorType>(m,
                                                "GlobalRotationEstimatorType")
      .value("ROBUST_L1L2", theia::GlobalRotationEstimatorType::ROBUST_L1L2)
      .value("NONLINEAR", theia::GlobalRotationEstimatorType::NONLINEAR)
      .value("LINEAR", theia::GlobalRotationEstimatorType::LINEAR)
      .value("LAGRANGE_DUAL", theia::GlobalRotationEstimatorType::LAGRANGE_DUAL)
      .value("HYBRID", theia::GlobalRotationEstimatorType::HYBRID)
      .export_values();

  // ReconstructionEstimatorOptions
  py::class_<theia::ReconstructionEstimatorOptions>(
      m, "ReconstructionEstimatorOptions")
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
      .def_readwrite("view_graph_min_relative_inliers",
                     &theia::ReconstructionEstimatorOptions::
                         view_graph_min_relative_inliers)
      .def_readwrite("view_graph_max_cycle_rotation_error_degrees",
                     &theia::ReconstructionEstimatorOptions::
                         view_graph_max_cycle_rotation_error_degrees)
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
      .def_readwrite("ligt_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         ligt_position_estimator_options)
      .def_readwrite("art_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         art_position_estimator_options)
      .def_readwrite("refine_rotations_with_rotation_only_error",
                     &theia::ReconstructionEstimatorOptions::
                         refine_rotations_with_rotation_only_error)
      .def_readwrite("rotation_only_refiner_options",
                     &theia::ReconstructionEstimatorOptions::
                         rotation_only_refiner_options)
      .def_readwrite("glomap_position_estimator_options",
                     &theia::ReconstructionEstimatorOptions::
                         glomap_position_estimator_options)
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
          "triangulation_max_reprojection_error_in_pixels",
          &theia::ReconstructionEstimatorOptions::
              triangulation_max_reprojection_error_in_pixels)
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
                     &theia::ReconstructionEstimatorOptions::optimize_for_forward_facing_trajectory)
      .def_readwrite("init_free_options",
                     &theia::ReconstructionEstimatorOptions::init_free_options);
  // Reconstruction class
  py::class_<theia::Reconstruction>(m, "Reconstruction")
      .def(py::init<>())
      .def("NumViews", &theia::Reconstruction::NumViews)
      .def("ViewIdFromName",
           &theia::Reconstruction::ViewIdFromName,
           py::arg("view_name"))
      .def("AddView",
           (theia::ViewId(theia::Reconstruction::*)(const std::string&,
                                                    const double)) &
               theia::Reconstruction::AddView,
           py::arg("view_name"),
           py::arg("timestamp"),
           py::return_value_policy::reference_internal)
      .def("AddView",
           (theia::ViewId(theia::Reconstruction::*)(
               const std::string&,
               const theia::CameraIntrinsicsGroupId,
               const double)) &
               theia::Reconstruction::AddView,
           py::arg("view_name"),
           py::arg("group_id"),
           py::arg("timestamp"),
           py::return_value_policy::reference_internal)
      .def("RemoveView", &theia::Reconstruction::RemoveView, py::arg("view_id"))
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
               &theia::Reconstruction::AddTrack),
           py::arg("track_id"))
      .def("AddTrack",
           static_cast<theia::TrackId (theia::Reconstruction::*)(
               const std::vector<std::pair<theia::ViewId, theia::Feature>>&)>(
               &theia::Reconstruction::AddTrack),
           py::arg("track"))
      .def("RemoveTrack",
           &theia::Reconstruction::RemoveTrack,
           py::arg("track_id"))
      .def("TrackIds", &theia::Reconstruction::TrackIds)
      .def("AddObservation",
           &theia::Reconstruction::AddObservation,
           py::arg("view_id"),
           py::arg("track_id"),
           py::arg("feature"))
      .def("NumCameraIntrinsicGroups",
           &theia::Reconstruction::NumCameraIntrinsicGroups)
      .def("Normalize", &theia::Reconstruction::Normalize)
      .def("CameraIntrinsicsGroupIdFromViewId",
           &theia::Reconstruction::CameraIntrinsicsGroupIdFromViewId,
           py::arg("view_id"))
      .def("CameraIntrinsicsGroupIds",
           &theia::Reconstruction::CameraIntrinsicsGroupIds)
      .def("View",
           &theia::Reconstruction::View,
           py::arg("view_id"),
           py::return_value_policy::reference_internal)
      .def("MutableView",
           &theia::Reconstruction::MutableView,
           py::arg("view_id"),
           py::return_value_policy::reference_internal)
      .def("Track",
           &theia::Reconstruction::Track,
           py::arg("track_id"),
           py::return_value_policy::reference_internal)
      .def("MutableTrack",
           &theia::Reconstruction::MutableTrack,
           py::arg("track_id"),
           py::return_value_policy::reference_internal)
      .def("GetViewsInCameraIntrinsicGroup",
           &theia::Reconstruction::GetViewsInCameraIntrinsicGroup,
           py::arg("group_id"))
      .def("InitializeInverseDepth",
           &theia::Reconstruction::InitializeInverseDepth)
      .def("InitializeTrackInverseDepth",
           &theia::Reconstruction::InitializeTrackInverseDepth,
           py::arg("track_id"))
      .def("AddCameraRig",
           &theia::Reconstruction::AddCameraRig,
           py::arg("camera_rig"))
      .def("GetCameraRig",
           &theia::Reconstruction::GetCameraRig,
           py::arg("rig_id"),
           py::return_value_policy::reference_internal)
      .def("MutableCameraRig",
           &theia::Reconstruction::MutableCameraRig,
           py::arg("rig_id"),
           py::return_value_policy::reference_internal)
      .def("RigIds", &theia::Reconstruction::RigIds)
      .def("NumCameraRigs", &theia::Reconstruction::NumCameraRigs)
      .def("AddCapture",
           &theia::Reconstruction::AddCapture,
           py::arg("rig_id"),
           py::arg("timestamp"))
      .def("AddRigCapture",
           &theia::Reconstruction::AddRigCapture,
           py::arg("rig_id"),
           py::arg("timestamp"),
           py::arg("rig_camera_id_to_view_name"))
      .def("SetViewRigMembership",
           &theia::Reconstruction::SetViewRigMembership,
           py::arg("view_id"),
           py::arg("rig_id"),
           py::arg("rig_camera_id"),
           py::arg("capture_id"))
      .def("ViewHasRigMembership",
           &theia::Reconstruction::ViewHasRigMembership,
           py::arg("view_id"))
      .def("GetViewRigMembership",
           &theia::Reconstruction::GetViewRigMembership,
           py::arg("view_id"),
           py::return_value_policy::reference_internal)
      .def("GetRigCapture",
           &theia::Reconstruction::GetRigCapture,
           py::arg("capture_id"),
           py::return_value_policy::reference_internal)
      .def("MutableRigCapture",
           &theia::Reconstruction::MutableRigCapture,
           py::arg("capture_id"),
           py::return_value_policy::reference_internal)
      .def("CaptureIds", &theia::Reconstruction::CaptureIds)
      .def("CaptureIdsForRig",
           &theia::Reconstruction::CaptureIdsForRig,
           py::arg("rig_id"))
      .def("CaptureIdFromRigAndTimestamp",
           &theia::Reconstruction::CaptureIdFromRigAndTimestamp,
           py::arg("rig_id"),
           py::arg("timestamp"))
      .def("NumCaptures", &theia::Reconstruction::NumCaptures)
      //.def("GetSubReconstruction",
      //&theia::Reconstruction::GetSubReconstructionWrapper)
      ;

  py::class_<theia::RigSensor>(m, "RigSensor")
      .def(py::init<>())
      .def_readwrite("name", &theia::RigSensor::name)
      .def_readwrite("position", &theia::RigSensor::position)
      .def_readwrite("orientation", &theia::RigSensor::orientation)
      .def_readwrite("optimize_extrinsics",
                     &theia::RigSensor::optimize_extrinsics)
      .def_readwrite("intrinsics_group_id",
                     &theia::RigSensor::intrinsics_group_id)
      .def("GetOrientationAsRotationMatrix",
           &theia::RigSensor::GetOrientationAsRotationMatrix)
      .def("SetOrientationFromRotationMatrix",
           &theia::RigSensor::SetOrientationFromRotationMatrix,
           py::arg("rotation"))
      .def("SetOrientationFromAngleAxis",
           &theia::RigSensor::SetOrientationFromAngleAxis,
           py::arg("angle_axis"))
      .def("SetPosition",
           &theia::RigSensor::SetPosition,
           py::arg("position_in_rig"));

  py::class_<theia::CameraRig>(m, "CameraRig")
      .def(py::init<>())
      .def(py::init<const std::string&>(), py::arg("name"))
      .def("Name", &theia::CameraRig::Name)
      .def("SetName", &theia::CameraRig::SetName, py::arg("name"))
      .def("AddSensor",
           (theia::RigCameraId(theia::CameraRig::*)(
               const std::string&,
               const Eigen::Vector3d&,
               const Eigen::Vector3d&)) &
               theia::CameraRig::AddSensor,
           py::arg("sensor_name"),
           py::arg("position_in_rig"),
           py::arg("orientation_rig_to_cam"))
      .def("AddSensor",
           (theia::RigCameraId(theia::CameraRig::*)(const theia::RigSensor&)) &
               theia::CameraRig::AddSensor,
           py::arg("sensor"))
      .def("HasSensor",
           &theia::CameraRig::HasSensor,
           py::arg("rig_camera_id"))
      .def("GetSensor",
           &theia::CameraRig::GetSensor,
           py::arg("rig_camera_id"),
           py::return_value_policy::reference_internal)
      .def("MutableSensor",
           &theia::CameraRig::MutableSensor,
           py::arg("rig_camera_id"),
           py::return_value_policy::reference_internal)
      .def("SensorIds", &theia::CameraRig::SensorIds)
      .def("NumSensors", &theia::CameraRig::NumSensors);

  py::class_<theia::ViewRigMembership>(m, "ViewRigMembership")
      .def(py::init<>())
      .def_readwrite("rig_id", &theia::ViewRigMembership::rig_id)
      .def_readwrite("rig_camera_id", &theia::ViewRigMembership::rig_camera_id)
      .def_readwrite("capture_id", &theia::ViewRigMembership::capture_id);

  py::class_<theia::RigCapture>(m, "RigCapture")
      .def(py::init<>())
      .def(py::init<theia::RigId, double>(),
           py::arg("rig_id"),
           py::arg("timestamp"))
      .def("GetRigId", &theia::RigCapture::GetRigId)
      .def("GetTimestamp", &theia::RigCapture::GetTimestamp)
      .def("SetTimestamp",
           &theia::RigCapture::SetTimestamp,
           py::arg("timestamp"))
      .def("SetEstimated",
           &theia::RigCapture::SetEstimated,
           py::arg("is_estimated"))
      .def("IsEstimated", &theia::RigCapture::IsEstimated)
      .def("SetPosition",
           &theia::RigCapture::SetPosition,
           py::arg("position"))
      .def("GetPosition", &theia::RigCapture::GetPosition)
      .def("SetOrientationFromRotationMatrix",
           &theia::RigCapture::SetOrientationFromRotationMatrix,
           py::arg("rotation"))
      .def("SetOrientationFromAngleAxis",
           &theia::RigCapture::SetOrientationFromAngleAxis,
           py::arg("angle_axis"))
      .def("GetOrientationAsRotationMatrix",
           &theia::RigCapture::GetOrientationAsRotationMatrix)
      .def("GetOrientationAsAngleAxis",
           &theia::RigCapture::GetOrientationAsAngleAxis)
      .def("ViewIdForCamera",
           &theia::RigCapture::ViewIdForCamera,
           py::arg("rig_camera_id"))
      .def("GetViewIds", &theia::RigCapture::GetViewIds)
      .def("NumViews", &theia::RigCapture::NumViews);

  m.def("PropagateCameraPosesForCapture",
        &theia::PropagateCameraPosesForCapture,
        py::arg("capture_id"),
        py::arg("reconstruction"),
        py::arg("update_view_estimated") = true);
  m.def("EstimateMetricEdgeScales",
        &theia::EstimateMetricEdgeScales,
        py::arg("reconstruction"),
        py::arg("view_graph"),
        py::arg("min_points") = 10,
        "Sets TwoViewInfo.scale_estimate on same-sensor capture edges from "
        "rig geometry. Returns the number of edges that received a scale.");
  m.def("PropagateAllEstimatedCapturePoses",
        &theia::PropagateAllEstimatedCapturePoses,
        py::arg("reconstruction"));

  // Reconstruction Estimator Helpers
  m.def("SetUnderconstrainedTracksToUnestimated",
        &theia::SetUnderconstrainedTracksToUnestimated,
        py::arg("reconstruction"));
  m.def("SetUnderconstrainedViewsToUnestimated",
        &theia::SetUnderconstrainedViewsToUnestimated,
        py::arg("reconstruction"));
  m.def("NumEstimatedViews",
        &theia::NumEstimatedViews,
        py::arg("reconstruction"));
  m.def("NumEstimatedTracks",
        &theia::NumEstimatedTracks,
        py::arg("reconstruction"));
  m.def("SetReconstructionFromEstimatedPoses",
        &theia::SetReconstructionFromEstimatedPoses,
        py::arg("orientations"),
        py::arg("positions"),
        py::arg("reconstruction"));
  m.def("CreateEstimatedSubreconstruction",
        &theia::CreateEstimatedSubreconstruction,
        py::arg("input_reconstruction"),
        py::arg("estimated_reconstruction"));
  m.def("RelativeRotationsFromViewGraph",
        &theia::RelativeRotationsFromViewGraph,
        py::arg("view_graph"));
  m.def("GetEstimatedViewsFromReconstruction",
        &theia::GetEstimatedViewsFromReconstructionWrapper,
        py::arg("reconstruction"));
  m.def("GetEstimatedTracksFromReconstruction",
        &theia::GetEstimatedTracksFromReconstructionWrapper,
        py::arg("reconstruction"));
        
  // Reconstruction Estimator
  // TwoViewInfo
  py::class_<theia::TwoViewInfo>(m, "TwoViewInfo")
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

  m.def("SwapCameras", &theia::SwapCameras, py::arg("twoview_info"));

  // ViewGraph
  py::class_<theia::ViewGraph>(m, "ViewGraph")
      .def(py::init<>())
      //.def_property_readonly("Name", &theia::View::Name)
      .def("ReadFromDisk",
           &theia::ViewGraph::ReadFromDisk,
           py::arg("input_filepath"))
      .def("WriteToDisk",
           &theia::ViewGraph::WriteToDisk,
           py::arg("output_filepath"))
      .def("HasView", &theia::ViewGraph::HasView, py::arg("view_id"))
      .def("RemoveView", &theia::ViewGraph::RemoveView, py::arg("view_id"))
      .def("HasEdge",
           &theia::ViewGraph::HasEdge,
           py::arg("view_id_1"),
           py::arg("view_id_2"))
      .def("AddEdge",
           &theia::ViewGraph::AddEdge,
           py::arg("view_id_1"),
           py::arg("view_id_2"),
           py::arg("two_view_info"))
      .def("RemoveEdge",
           &theia::ViewGraph::RemoveEdge,
           py::arg("view_id_1"),
           py::arg("view_id_2"))
      .def("NumViews", &theia::ViewGraph::NumViews)
      .def("NumEdges", &theia::ViewGraph::NumEdges)
      .def("GetNeighborIdsForView",
           &theia::ViewGraph::GetNeighborIdsForView,
           py::arg("view_id"),
           py::return_value_policy::reference_internal)
      .def("GetEdge",
           &theia::ViewGraph::GetEdge,
           py::arg("view_id_1"),
           py::arg("view_id_2"),
           py::return_value_policy::reference_internal)
      .def("GetAllEdges", &theia::ViewGraph::GetAllEdges)

      // not sure pointer as input
      //.def("ExtractSubgraph", &theia::ViewGraph::ExtractSubgraph)
      //.def("GetLargestConnectedComponentIds",
      //&theia::ViewGraph::GetLargestConnectedComponentIds)

      ;

  // GPS converter
  py::class_<theia::GPSConverter>(m, "GPSConverter")
      .def(py::init<>())
      .def_static("ECEFToLLA",
                  theia::GPSConverter::ECEFToLLA,
                  py::arg("ecef"))
      .def_static("LLAToECEF",
                  theia::GPSConverter::LLAToECEF,
                  py::arg("lla"));

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

  py::enum_<theia::GravityPriorErrorType>(m, "GravityPriorErrorType")
      .value("VECTOR_DIFF", theia::GravityPriorErrorType::VECTOR_DIFF)
      .value("DIRECTION_CROSS", theia::GravityPriorErrorType::DIRECTION_CROSS)
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

  // TwoViewBundleAdjustmentOptions
  py::class_<theia::TwoViewBundleAdjustmentOptions>(
      m, "TwoViewBundleAdjustmentOptions")
      .def(py::init<>())
      .def_readwrite("ba_options",
                     &theia::TwoViewBundleAdjustmentOptions::ba_options)
      .def_readwrite(
          "constant_camera1_intrinsics",
          &theia::TwoViewBundleAdjustmentOptions::constant_camera1_intrinsics)
      .def_readwrite(
          "constant_camera2_intrinsics",
          &theia::TwoViewBundleAdjustmentOptions::constant_camera2_intrinsics);

  py::class_<theia::BundleAdjustmentSummary>(m, "BundleAdjustmentSummary")
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
        py::arg("options"),
        py::arg("views_to_optimize"),
        py::arg("tracks_to_optimize"),
        py::arg("reconstruction"));
  m.def("BundleAdjustPartialViewsConstant",
        theia::BundleAdjustPartialViewsConstantWrapper,
        py::arg("options"),
        py::arg("var_view_ids"),
        py::arg("const_view_ids"),
        py::arg("reconstruction"));
  m.def("BundleAdjustReconstructionWithConstantTracks",
        theia::BundleAdjustReconstructionWithConstantTracksWrapper,
        py::arg("options"),
        py::arg("constant_track_ids"),
        py::arg("reconstruction"));
  m.def("BundleAdjustReconstructionWithRelativePoseEdges",
        theia::BundleAdjustReconstructionWithRelativePoseEdgesWrapper,
        py::arg("options"),
        py::arg("relative_pose_constraints"),
        py::arg("reconstruction"));
  m.def("BundleAdjustPartialViewsConstantWithRelativePoseEdges",
        theia::BundleAdjustPartialViewsConstantWithRelativePoseEdgesWrapper,
        py::arg("options"),
        py::arg("var_view_ids"),
        py::arg("const_view_ids"),
        py::arg("relative_pose_constraints"),
        py::arg("reconstruction"));
  m.def("BundleAdjustReconstruction",
        theia::BundleAdjustReconstructionWrapper,
        py::arg("options"),
        py::arg("reconstruction"));
  m.def("BundleAdjustRigReconstruction",
        theia::BundleAdjustRigReconstructionWrapper,
        py::arg("options"),
        py::arg("reconstruction"));
  m.def("BundleAdjustView",
        theia::BundleAdjustViewWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("view_id"));
  m.def("BundleAdjustViews",
        theia::BundleAdjustViewsWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("view_ids"));
  m.def("BundleAdjustViewWithCov",
        theia::BundleAdjustViewWithCovWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("view_id"));
  m.def("BundleAdjustViewsWithCov",
        theia::BundleAdjustViewsWithCovWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("view_ids"));

  m.def("BundleAdjustTrack",
        theia::BundleAdjustTrackWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("track_id"));
  m.def("BundleAdjustTracks",
        theia::BundleAdjustTracksWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("track_ids"));
  m.def("BundleAdjustTrackWithCov",
        theia::BundleAdjustTrackWithCovWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("track_id"));
  m.def("BundleAdjustTracksWithCov",
        theia::BundleAdjustTracksWithCovWrapper,
        py::arg("reconstruction"),
        py::arg("options"),
        py::arg("track_ids"));
  // m.def("BundleAdjustTwoViews", theia::BundleAdjustTwoViewsWrapper);
  m.def("BundleAdjustTwoViewsAngular",
        theia::BundleAdjustTwoViewsAngularWrapper,
        py::arg("options"),
        py::arg("correspondences"),
        py::arg("two_view_info_prior"));
  m.def("OptimizeRelativePositionWithKnownRotation",
        theia::OptimizeRelativePositionWithKnownRotationWrapper,
        py::arg("correspondences"),
        py::arg("rotation1"),
        py::arg("rotation2"));

  m.def("OptimizeAbsolutePoseOnNormFeatures",
        theia::OptimizeAbsolutePoseOnNormFeatures,
        py::arg("correspondences_2d_3d"),
        py::arg("rotation_init"),
        py::arg("position_init"),
        py::arg("ba_options"));
  // Bundle Adjuster
  py::class_<theia::BundleAdjuster>(m, "BundleAdjuster")
      // constructor uses pointer of an object as input
      //.def(py::init<theia::BundleAdjustmentOptions, theia::Reconstruction>())
      .def("AddView", &theia::BundleAdjuster::AddView, py::arg("view_id"))
      .def("AddTrack", &theia::BundleAdjuster::AddTrack, py::arg("track_id"))
      .def(
          "AddInvTrack",
          [](theia::BundleAdjuster& self,
             const theia::TrackId track_id,
             const bool views_constant,
             const std::optional<std::unordered_set<theia::ViewId>>&
                 variable_views) {
            return self.AddInvTrack(
                track_id,
                views_constant,
                variable_views ? &*variable_views : nullptr);
          },
          py::arg("track_id"),
          py::arg("views_constant") = false,
          py::arg("variable_views") = py::none())
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
        py::arg("triplet"),
        py::arg("feature1"),
        py::arg("feature2"),
        py::arg("feature3"));

  // Global position estimator options
  py::class_<theia::PositionEstimator>(m, "PositionEstimator");

  py::class_<theia::LinearPositionEstimator::Options>(
      m, "LinearPositionEstimatorOptions")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::LinearPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_power_iterations",
          &theia::LinearPositionEstimator::Options::max_power_iterations)
      .def_readwrite(
          "eigensolver_threshold",
          &theia::LinearPositionEstimator::Options::eigensolver_threshold);

  // Global Position Estimators
  py::class_<theia::LinearPositionEstimator, theia::PositionEstimator>(
      m, "LinearPositionEstimator")
      .def(py::init<const theia::LinearPositionEstimator::Options&,
                    const theia::Reconstruction&>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimatePositions",
           &theia::LinearPositionEstimator::EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientation"));

  py::class_<theia::NonlinearPositionEstimator::Options>(
      m, "NonlinearPositionEstimatorOptions")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::NonlinearPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_num_iterations",
          &theia::NonlinearPositionEstimator::Options::max_num_iterations)
      .def_readwrite(
          "robust_loss_width",
          &theia::NonlinearPositionEstimator::Options::robust_loss_width)
      .def_readwrite(
          "min_num_points_per_view",
          &theia::NonlinearPositionEstimator::Options::min_num_points_per_view)
      .def_readwrite(
          "point_to_camera_weight",
          &theia::NonlinearPositionEstimator::Options::point_to_camera_weight)
      // Deprecated aliases kept for backward compatibility.
      .def_readwrite(
          "max_power_iterations",
          &theia::NonlinearPositionEstimator::Options::max_num_iterations)
      .def_readwrite(
          "eigensolver_threshold",
          &theia::NonlinearPositionEstimator::Options::robust_loss_width);

  py::class_<theia::NonlinearPositionEstimator, theia::PositionEstimator>(
      m, "NonlinearPositionEstimator")
      .def(py::init<const theia::NonlinearPositionEstimator::Options&,
                    const theia::Reconstruction&>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimatePositions",
           &theia::NonlinearPositionEstimator::EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientation"))
      .def("EstimateRemainingPositionsInRecon",
           &theia::NonlinearPositionEstimator::
               EstimateRemainingPositionsInReconWrapper,
           py::arg("fixed_views"),
           py::arg("views_in_subrecon"),
           py::arg("view_pairs_sub_recon"));

  py::class_<theia::LeastUnsquaredDeviationPositionEstimator::Options>(
      m, "LeastUnsquaredDeviationPositionEstimatorOptions")
      .def(py::init<>())
      .def_readwrite("max_num_iterations",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         max_num_iterations)
      .def_readwrite("max_num_reweighted_iterations",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         max_num_reweighted_iterations)
      .def_readwrite("convergence_criterion",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         convergence_criterion)
      .def_readwrite("use_scale_estimates",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         use_scale_estimates)
      .def_readwrite("min_valid_scale_estimate",
                     &theia::LeastUnsquaredDeviationPositionEstimator::Options::
                         min_valid_scale_estimate);

  py::class_<theia::LeastUnsquaredDeviationPositionEstimator,
             theia::PositionEstimator>(
      m, "LeastUnsquaredDeviationPositionEstimator")
      .def(py::init<theia::LeastUnsquaredDeviationPositionEstimator::Options>(),
           py::arg("options"))
      .def("EstimatePositions",
           &theia::LeastUnsquaredDeviationPositionEstimator::
               EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientation"));

  py::class_<theia::LiGTPositionEstimator::Options>(
      m, "LiGTPositionEstimatorOptions")
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
                     &theia::LiGTPositionEstimator::Options::max_num_views_svd)
      .def_readwrite(
          "normalize_constraints",
          &theia::LiGTPositionEstimator::Options::normalize_constraints)
      .def_readwrite(
          "num_irls_iterations",
          &theia::LiGTPositionEstimator::Options::num_irls_iterations)
      .def_readwrite(
          "irls_scale_multiplier",
          &theia::LiGTPositionEstimator::Options::irls_scale_multiplier)
      .def_readwrite(
          "irls_convergence_threshold",
          &theia::LiGTPositionEstimator::Options::irls_convergence_threshold)
      .def_readwrite(
          "use_scale_constraint",
          &theia::LiGTPositionEstimator::Options::use_scale_constraint)
      .def_readwrite(
          "min_tracks_per_view",
          &theia::LiGTPositionEstimator::Options::min_tracks_per_view);

  py::class_<theia::LiGTPositionEstimator, theia::PositionEstimator>(
      m, "LiGTPositionEstimator")
      .def(py::init<const theia::LiGTPositionEstimator::Options&,
                    const theia::Reconstruction&>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimatePositions",
           &theia::LiGTPositionEstimator::EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientation"));

  py::class_<theia::ARTPositionEstimator::Options>(
      m, "ARTPositionEstimatorOptions")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::ARTPositionEstimator::Options::num_threads)
      .def_readwrite(
          "min_tracks_per_triplet",
          &theia::ARTPositionEstimator::Options::min_tracks_per_triplet)
      .def_readwrite(
          "min_tracks_per_triplet_fallback",
          &theia::ARTPositionEstimator::Options::min_tracks_per_triplet_fallback)
      .def_readwrite(
          "num_coverage_rounds",
          &theia::ARTPositionEstimator::Options::num_coverage_rounds)
      .def_readwrite(
          "num_robust_samples",
          &theia::ARTPositionEstimator::Options::num_robust_samples)
      .def_readwrite(
          "min_bridge_tracks",
          &theia::ARTPositionEstimator::Options::min_bridge_tracks)
      .def_readwrite(
          "use_track_covisibility_triplets",
          &theia::ARTPositionEstimator::Options::use_track_covisibility_triplets)
      .def_readwrite(
          "max_tracks_per_triplet",
          &theia::ARTPositionEstimator::Options::max_tracks_per_triplet)
      .def_readwrite(
          "max_triplets_per_view",
          &theia::ARTPositionEstimator::Options::max_triplets_per_view)
      .def_readwrite("max_eigen_ratio",
                     &theia::ARTPositionEstimator::Options::max_eigen_ratio)
      .def_readwrite(
          "min_c_conditioning",
          &theia::ARTPositionEstimator::Options::min_c_conditioning)
      .def_readwrite(
          "num_obs_filter_iterations",
          &theia::ARTPositionEstimator::Options::num_obs_filter_iterations)
      .def_readwrite("obs_mad_factor",
                     &theia::ARTPositionEstimator::Options::obs_mad_factor)
      .def_readwrite("triplet_conf_factor",
                     &theia::ARTPositionEstimator::Options::triplet_conf_factor)
      .def_readwrite("num_irls_iterations",
                     &theia::ARTPositionEstimator::Options::num_irls_iterations)
      .def_readwrite(
          "irls_scale_multiplier",
          &theia::ARTPositionEstimator::Options::irls_scale_multiplier)
      .def_readwrite("irls_scale",
                     &theia::ARTPositionEstimator::Options::irls_scale)
      .def_readwrite(
          "irls_convergence_threshold",
          &theia::ARTPositionEstimator::Options::irls_convergence_threshold);

  py::class_<theia::RotationOnlyRefinerOptions>(m, "RotationOnlyRefinerOptions")
      .def(py::init<>())
      .def_readwrite("num_outer_iterations",
                     &theia::RotationOnlyRefinerOptions::num_outer_iterations)
      .def_readwrite("max_num_iterations",
                     &theia::RotationOnlyRefinerOptions::max_num_iterations)
      .def_readwrite("loss_scale_px",
                     &theia::RotationOnlyRefinerOptions::loss_scale_px)
      .def_readwrite("max_observations_per_view",
                     &theia::RotationOnlyRefinerOptions::
                         max_observations_per_view)
      .def_readwrite("max_tracks_per_triplet",
                     &theia::RotationOnlyRefinerOptions::max_tracks_per_triplet)
      .def_readwrite("use_two_view_tuples",
                     &theia::RotationOnlyRefinerOptions::use_two_view_tuples)
      .def_readwrite("require_view_graph_edges",
                     &theia::RotationOnlyRefinerOptions::
                         require_view_graph_edges)
      .def_readwrite("max_reference_candidates",
                     &theia::RotationOnlyRefinerOptions::
                         max_reference_candidates)
      .def_readwrite("tuple_gate_px",
                     &theia::RotationOnlyRefinerOptions::tuple_gate_px)
      .def_readwrite(
          "verify_observations_px",
          &theia::RotationOnlyRefinerOptions::verify_observations_px)
      .def_readwrite("num_threads",
                     &theia::RotationOnlyRefinerOptions::num_threads);

  // Returns (success, refined orientations). On failure the orientations are
  // returned unchanged.
  m.def(
      "RefineRotationsWithRotationOnlyError",
      [](const theia::RotationOnlyRefinerOptions& options,
         const theia::Reconstruction& reconstruction,
         const std::unordered_map<theia::ViewIdPair, theia::TwoViewInfo>&
             view_pairs,
         const std::unordered_map<theia::ViewId, Eigen::Vector3d>&
             orientations) {
        std::unordered_map<theia::ViewId, Eigen::Vector3d> refined = orientations;
        const bool success = theia::RefineRotationsWithRotationOnlyError(
            options, reconstruction, view_pairs, &refined);
        return std::make_tuple(success, success ? refined : orientations);
      },
      py::arg("options"), py::arg("reconstruction"), py::arg("view_pairs"),
      py::arg("orientations"));

  py::class_<theia::ARTPositionEstimator, theia::PositionEstimator>(
      m, "ARTPositionEstimator")
      .def(py::init<const theia::ARTPositionEstimator::Options&,
                    const theia::Reconstruction&>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimatePositions",
           &theia::ARTPositionEstimator::EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientation"));

  py::class_<theia::GlomapPositionEstimator::Options>(
      m, "GlomapPositionEstimatorOptions")
      .def(py::init<>())
      .def_readwrite("num_threads",
                     &theia::GlomapPositionEstimator::Options::num_threads)
      .def_readwrite(
          "max_num_iterations",
          &theia::GlomapPositionEstimator::Options::max_num_iterations)
      .def_readwrite("robust_loss_width",
                     &theia::GlomapPositionEstimator::Options::robust_loss_width)
      .def_readwrite("min_track_length",
                     &theia::GlomapPositionEstimator::Options::min_track_length)
      .def_readwrite("max_num_tracks",
                     &theia::GlomapPositionEstimator::Options::max_num_tracks)
      .def_readwrite("write_points_to_reconstruction",
                     &theia::GlomapPositionEstimator::Options::
                         write_points_to_reconstruction)
      .def_readwrite("use_pairwise_scale_priors",
                     &theia::GlomapPositionEstimator::Options::
                         use_pairwise_scale_priors)
      .def_readwrite("pairwise_scale_prior_weight",
                     &theia::GlomapPositionEstimator::Options::
                         pairwise_scale_prior_weight);

  py::class_<theia::GlomapPositionEstimator, theia::PositionEstimator>(
      m, "GlomapPositionEstimator")
      .def(py::init<const theia::GlomapPositionEstimator::Options&,
                    theia::Reconstruction*>(),
           py::arg("options"),
           py::arg("reconstruction"))
      .def("EstimatePositions",
           &theia::GlomapPositionEstimator::EstimatePositionsWrapper,
           py::arg("view_pairs"),
           py::arg("orientations"));

  // base RotationEstimator class
  py::class_<theia::RotationEstimator>(m, "RotationEstimator");

  py::class_<theia::RobustRotationEstimator::Options>(
      m, "RobustRotationEstimatorOptions")
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

  // Global Rotation Estimators
  py::class_<theia::RobustRotationEstimator, theia::RotationEstimator>(
      m, "RobustRotationEstimator")
      .def(py::init<const theia::RobustRotationEstimator::Options&>(),
           py::arg("options"))
      .def("EstimateRotations",
           &theia::RobustRotationEstimator::EstimateRotationsWrapper,
           py::arg("view_pairs"),
           py::arg("global_orientations"))
      .def("AddRelativeRotationConstraint",
           &theia::RobustRotationEstimator::AddRelativeRotationConstraint,
           py::arg("view_id_pair"),
           py::arg("relative_rotation"))
      .def("SetFixedGlobalRotations",
           &theia::RobustRotationEstimator::SetFixedGlobalRotations,
           py::arg("fixed_views"));

  py::class_<theia::NonlinearRotationEstimator, theia::RotationEstimator>(
      m, "NonlinearRotationEstimator")
      .def(py::init<>())
      .def(py::init<double>(), py::arg("robust_loss_width"))
      .def("EstimateRotations",
           &theia::NonlinearRotationEstimator::EstimateRotationsWrapper,
           py::arg("view_pairs"),
           py::arg("global_orientations"));

  py::class_<theia::LinearRotationEstimator, theia::RotationEstimator>(
      m, "LinearRotationEstimator")
      .def(py::init<>())
      .def("AddRelativeRotationConstraint",
           &theia::LinearRotationEstimator::AddRelativeRotationConstraint,
           py::arg("view_id_pair"),
           py::arg("relative_rotation"))
      .def("EstimateRotations",
           &theia::LinearRotationEstimator::EstimateRotationsWrapper,
           py::arg("view_pairs"),
           py::arg("global_orientations"));

  py::class_<theia::LagrangeDualRotationEstimator, theia::RotationEstimator>(
      m, "LagrangeDualRotationEstimator")
      .def(py::init<>())
      .def("EstimateRotations",
           &theia::LagrangeDualRotationEstimator::EstimateRotationsWrapper,
           py::arg("view_pairs"),
           py::arg("global_orientations"));

  py::class_<theia::HybridRotationEstimator, theia::RotationEstimator>(
      m, "HybridRotationEstimator")
      .def(py::init<>())
      .def("EstimateRotations",
           &theia::HybridRotationEstimator::EstimateRotationsWrapper,
           py::arg("view_pairs"),
           py::arg("global_orientations"));
}

void pytheia_sfm(py::module& m) {
  py::module m_submodule = m.def_submodule("sfm");
  pytheia_sfm_classes(m_submodule);
  pytheia_init_free(m_submodule);
}

}  // namespace sfm
}  // namespace pytheia
