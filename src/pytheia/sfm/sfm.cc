
#include "pytheia/sfm/sfm.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>


#include <glog/logging.h>
#include <math.h>
#include <Eigen/Dense>
#include <complex>
#include <algorithm>

#include "theia/math/polynomial.h"
#include "theia/sfm/pose/util.h"

#include "theia/sfm/feature.h"
#include "theia/matching/features_and_matches_database.h"

#include "theia/sfm/pose/pose_wrapper.h"
#include "theia/sfm/triangulation/triangulation.h"
#include "theia/sfm/triangulation/triangulation_wrapper.h"

#include "theia/sfm/transformation/transformation_wrapper.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/transformation/gdls_similarity_transform.h"
#include "theia/sfm/transformation/align_reconstructions.h"

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_wrapper.h"
#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"


#include "theia/sfm/bundle_adjustment/bundle_adjustment.h"
#include "theia/sfm/bundle_adjustment/create_loss_function.h"
#include "theia/sfm/bundle_adjustment/bundle_adjustment_wrapper.h"
#include "theia/sfm/bundle_adjustment/bundle_adjuster.h"
#include "theia/sfm/bundle_adjustment/bundle_adjust_two_views.h"
#include "theia/sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.h"


#include "theia/sfm/global_pose_estimation/position_estimator.h"
#include "theia/sfm/global_pose_estimation/rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/robust_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/linear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_position_estimator.h"
#include "theia/sfm/global_pose_estimation/nonlinear_rotation_estimator.h"
#include "theia/sfm/global_pose_estimation/global_pose_estimation_wrapper.h"

// reconstruction view track
#include "theia/sfm/view.h"
#include "theia/sfm/track.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/sfm/visibility_pyramid.h"
#include "theia/sfm/two_view_match_geometric_verification.h"
#include "theia/matching/keypoints_and_descriptors.h"

#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/reconstruction_builder.h"
#include "theia/sfm/global_reconstruction_estimator.h"
#include "theia/sfm/incremental_reconstruction_estimator.h"
#include "theia/sfm/hybrid_reconstruction_estimator.h"
#include "theia/sfm/track_builder.h"
#include "theia/sfm/reconstruction_builder.h"

#include "theia/sfm/similarity_transformation.h"
#include "theia/sfm/rigid_transformation.h"

// estimator
#include "theia/sfm/estimators/estimators_wrapper.h"
#include "theia/sfm/estimators/estimate_calibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_dominant_plane_from_points.h"
#include "theia/sfm/estimators/estimate_radial_distortion_homography.h"
#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h"
#include "theia/sfm/estimators/estimate_uncalibrated_relative_pose.h"
#include "theia/sfm/create_and_initialize_ransac_variant.h"
#include "theia/solvers/sample_consensus_estimator.h"


#include "theia/sfm/sfm_wrapper.h"
#include "theia/sfm/estimate_twoview_info.h"
#include "theia/sfm/estimate_track.h"
#include "theia/sfm/colorize_reconstruction.h"
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
#include "theia/sfm/select_good_tracks_for_bundle_adjustment.h"
#include "theia/sfm/set_camera_intrinsics_from_priors.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/undistort_image.h"
#include "theia/sfm/pose/upnp.h"



// for overloaded function in CameraInstrinsicsModel
template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

namespace py = pybind11;
#include <vector>
#include <iostream>
#include <pybind11/numpy.h>

// Initialize gtest
//google::InitGoogleLogging(argv[0]);

template <int N>
void AddIntrinsicsPriorType(py::module& m, const std::string& name) {
  py::class_<theia::Prior<N>>(m, ("Prior" + name).c_str())
      .def(py::init())
      .def_readwrite("is_set", &theia::Prior<N>::is_set)
      .def_property("value", &theia::Prior<N>::GetParametersValues, &theia::Prior<N>::SetParametersValues)
    ;
  }

namespace py = pybind11;

namespace pytheia {
namespace sfm {

void pytheia_sfm_classes(py::module &m) {

  //camera
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
  py::class_<theia::CameraIntrinsicsModel, std::shared_ptr<theia::CameraIntrinsicsModel>> camera_intrinsics_model(m, "CameraIntrinsicsModel");
  camera_intrinsics_model.def_property("FocalLength", &theia::CameraIntrinsicsModel::FocalLength, &theia::CameraIntrinsicsModel::SetFocalLength)
    .def_property_readonly("PrincipalPointX", &theia::CameraIntrinsicsModel::PrincipalPointX)
    .def_property_readonly("PrincipalPointY", &theia::CameraIntrinsicsModel::PrincipalPointY)
    .def("SetPrincipalPoint", &theia::CameraIntrinsicsModel::SetPrincipalPoint)
    .def("CameraToImageCoordinates", &theia::CameraIntrinsicsModel::CameraToImageCoordinates)
    .def("ImageToCameraCoordinates", &theia::CameraIntrinsicsModel::ImageToCameraCoordinates)
    .def("GetParameter", &theia::CameraIntrinsicsModel::GetParameter)
    .def("SetParameter", &theia::CameraIntrinsicsModel::SetParameter)

    // .def("DistortPoint", py::overload_cast<const Eigen::Vector2d>(&theia::CameraIntrinsicsModel::DistortPoint, py::const_))
    // .def("DistortPoint",   py::overload_cast<const Eigen::Vector2d&>(&theia::CameraIntrinsicsModel::DistortPoint, py::const_));
    // .def("DistortPoint", static_cast<Eigen::Vector2d (theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d &)>(&theia::CameraIntrinsicsModel::DistortPoint), "Set the pet's name")
    // .def("DistortPoint", overload_cast_<const Eigen::Vector2d>()(&theia::CameraIntrinsicsModel::DistortPoint), "Set the pet's age")


    // .def("DistortPoint",
    //      (Eigen::Vector2d
    //       (theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d& )) &theia::CameraIntrinsicsModel::DistortPoint,
    //      py::return_value_policy::reference_internal)

    //.def("UndistortPoint",(Eigen::Vector2d (theia::CameraIntrinsicsModel::*)(const Eigen::Vector2d& )) &
    //     theia::CameraIntrinsicsModel::UndistortPoint, py::arg("UndistortPoint"))





  ;
  // FisheyeCameraModel
  py::class_<theia::FisheyeCameraModel, std::shared_ptr<theia::FisheyeCameraModel>>(m, "FisheyeCameraModel", camera_intrinsics_model)
    .def(py::init<>())
    .def("Type", &theia::FisheyeCameraModel::Type)
    .def("NumParameters", &theia::FisheyeCameraModel::NumParameters)
    .def("SetFromCameraIntrinsicsPriors", &theia::FisheyeCameraModel::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::FisheyeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
    // OptimizeIntrinsicsType not defined
    .def("GetSubsetFromOptimizeIntrinsicsType", &theia::FisheyeCameraModel::GetSubsetFromOptimizeIntrinsicsType)
    .def("GetCalibrationMatrix", &theia::FisheyeCameraModel::GetCalibrationMatrix)
    .def("PrintIntrinsics", &theia::FisheyeCameraModel::PrintIntrinsics)
    .def_property_readonly("kIntrinsicsSize", &theia::FisheyeCameraModel::NumParameters)
    .def_property("AspectRatio", &theia::FisheyeCameraModel::AspectRatio, &theia::FisheyeCameraModel::SetAspectRatio)
    .def_property("Skew", &theia::FisheyeCameraModel::Skew, &theia::FisheyeCameraModel::SetSkew)
    .def_property_readonly("RadialDistortion1", &theia::FisheyeCameraModel::RadialDistortion1)
    .def_property_readonly("RadialDistortion2", &theia::FisheyeCameraModel::RadialDistortion2)
    .def_property_readonly("RadialDistortion3", &theia::FisheyeCameraModel::RadialDistortion3)
    .def_property_readonly("RadialDistortion4", &theia::FisheyeCameraModel::RadialDistortion4)
    .def("SetRadialDistortion", &theia::FisheyeCameraModel::SetRadialDistortion)

  ;
  // PinholeRadialTangentialCameraModel
  py::class_<theia::PinholeRadialTangentialCameraModel, std::shared_ptr<theia::PinholeRadialTangentialCameraModel>>(m, "PinholeRadialTangentialCameraModel", camera_intrinsics_model)
    .def(py::init<>())
    .def("Type", &theia::PinholeRadialTangentialCameraModel::Type)
    .def("NumParameters", &theia::PinholeRadialTangentialCameraModel::NumParameters)
    .def("SetFromCameraIntrinsicsPriors", &theia::PinholeRadialTangentialCameraModel::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::PinholeRadialTangentialCameraModel::CameraIntrinsicsPriorFromIntrinsics)
    // OptimizeIntrinsicsType not defined
    .def("GetSubsetFromOptimizeIntrinsicsType", &theia::PinholeRadialTangentialCameraModel::GetSubsetFromOptimizeIntrinsicsType)
    .def("GetCalibrationMatrix", &theia::PinholeRadialTangentialCameraModel::GetCalibrationMatrix)
    .def("PrintIntrinsics", &theia::PinholeRadialTangentialCameraModel::PrintIntrinsics)
    .def_property_readonly("kIntrinsicsSize", &theia::PinholeRadialTangentialCameraModel::NumParameters)
    .def_property("AspectRatio", &theia::PinholeRadialTangentialCameraModel::AspectRatio, &theia::PinholeRadialTangentialCameraModel::SetAspectRatio)
    .def_property("Skew", &theia::PinholeRadialTangentialCameraModel::Skew, &theia::PinholeRadialTangentialCameraModel::SetSkew)
    .def_property_readonly("RadialDistortion1", &theia::PinholeRadialTangentialCameraModel::RadialDistortion1)
    .def_property_readonly("RadialDistortion2", &theia::PinholeRadialTangentialCameraModel::RadialDistortion2)
    .def_property_readonly("RadialDistortion3", &theia::PinholeRadialTangentialCameraModel::RadialDistortion3)
    .def_property_readonly("TangentialDistortion1", &theia::PinholeRadialTangentialCameraModel::TangentialDistortion1)
    .def_property_readonly("TangentialDistortion2", &theia::PinholeRadialTangentialCameraModel::TangentialDistortion2)
    .def("SetRadialDistortion", &theia::PinholeRadialTangentialCameraModel::SetRadialDistortion)
    .def("SetTangentialDistortion", &theia::PinholeRadialTangentialCameraModel::SetTangentialDistortion)

  ;
  // DivisionUndistortionCameraModel
  py::class_<theia::DivisionUndistortionCameraModel, std::shared_ptr<theia::DivisionUndistortionCameraModel>>(m, "DivisionUndistortionCameraModel", camera_intrinsics_model)
    .def(py::init<>())
    .def("Type", &theia::DivisionUndistortionCameraModel::Type)
    .def("NumParameters", &theia::DivisionUndistortionCameraModel::NumParameters)
    .def("SetFromCameraIntrinsicsPriors", &theia::DivisionUndistortionCameraModel::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::DivisionUndistortionCameraModel::CameraIntrinsicsPriorFromIntrinsics)
    // OptimizeIntrinsicsType not defined
    .def("GetSubsetFromOptimizeIntrinsicsType", &theia::DivisionUndistortionCameraModel::GetSubsetFromOptimizeIntrinsicsType)
    .def("GetCalibrationMatrix", &theia::DivisionUndistortionCameraModel::GetCalibrationMatrix)
    .def("PrintIntrinsics", &theia::DivisionUndistortionCameraModel::PrintIntrinsics)
    .def_property_readonly("kIntrinsicsSize", &theia::DivisionUndistortionCameraModel::NumParameters)
    .def_property("AspectRatio", &theia::DivisionUndistortionCameraModel::AspectRatio, &theia::DivisionUndistortionCameraModel::SetAspectRatio)
    .def_property_readonly("RadialDistortion1", &theia::DivisionUndistortionCameraModel::RadialDistortion1)
    .def("SetRadialDistortion", &theia::DivisionUndistortionCameraModel::SetRadialDistortion)

  ;

  // PinholeCameraModel
  py::class_<theia::PinholeCameraModel, std::shared_ptr<theia::PinholeCameraModel>>(m, "PinholeCameraModel", camera_intrinsics_model)
    .def(py::init<>())
    .def("Type", &theia::PinholeCameraModel::Type)
    .def("NumParameters", &theia::PinholeCameraModel::NumParameters)
    .def("SetFromCameraIntrinsicsPriors", &theia::PinholeCameraModel::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::PinholeCameraModel::CameraIntrinsicsPriorFromIntrinsics)
    // OptimizeIntrinsicsType not defined
    .def("GetSubsetFromOptimizeIntrinsicsType", &theia::PinholeCameraModel::GetSubsetFromOptimizeIntrinsicsType)
    .def("GetCalibrationMatrix", &theia::PinholeCameraModel::GetCalibrationMatrix)
    .def("PrintIntrinsics", &theia::PinholeCameraModel::PrintIntrinsics)
    .def_property_readonly("kIntrinsicsSize", &theia::PinholeCameraModel::NumParameters)
    .def_property("AspectRatio", &theia::PinholeCameraModel::AspectRatio, &theia::PinholeCameraModel::SetAspectRatio)
    .def_property("Skew", &theia::PinholeCameraModel::Skew, &theia::PinholeCameraModel::SetSkew)
    .def_property_readonly("RadialDistortion1", &theia::PinholeCameraModel::RadialDistortion1)
    .def_property_readonly("RadialDistortion2", &theia::PinholeCameraModel::RadialDistortion2)
    .def("SetRadialDistortion", &theia::PinholeCameraModel::SetRadialDistortion)

  ;

  // FOVCameraModel
  py::class_<theia::FOVCameraModel, std::shared_ptr<theia::FOVCameraModel>>(m, "FOVCameraModel", camera_intrinsics_model)
    .def(py::init<>())
    .def("Type", &theia::FOVCameraModel::Type)
    .def("NumParameters", &theia::FOVCameraModel::NumParameters)
    .def("SetFromCameraIntrinsicsPriors", &theia::FOVCameraModel::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::FOVCameraModel::CameraIntrinsicsPriorFromIntrinsics)
    // OptimizeIntrinsicsType not defined
    .def("GetSubsetFromOptimizeIntrinsicsType", &theia::FOVCameraModel::GetSubsetFromOptimizeIntrinsicsType)
    .def("GetCalibrationMatrix", &theia::FOVCameraModel::GetCalibrationMatrix)
    .def("PrintIntrinsics", &theia::FOVCameraModel::PrintIntrinsics)
    .def_property_readonly("kIntrinsicsSize", &theia::FOVCameraModel::NumParameters)
    .def_property("AspectRatio", &theia::FOVCameraModel::AspectRatio, &theia::FOVCameraModel::SetAspectRatio)
    .def_property_readonly("RadialDistortion1", &theia::FOVCameraModel::RadialDistortion1)
    .def("SetRadialDistortion", &theia::FOVCameraModel::SetRadialDistortion)

  ;

  m.def("ComposeProjectionMatrix", theia::ComposeProjectionMatrixWrapper);
  m.def("DecomposeProjectionMatrix", theia::DecomposeProjectionMatrixWrapper);
  m.def("CalibrationMatrixToIntrinsics", theia::CalibrationMatrixToIntrinsicsWrapper);
  m.def("IntrinsicsToCalibrationMatrix", theia::IntrinsicsToCalibrationMatrixWrapper);



  py::class_<theia::Camera, std::shared_ptr<theia::Camera>>(m, "Camera")
    .def(py::init())
    .def(py::init<theia::Camera>())
    .def(py::init<theia::CameraIntrinsicsModelType>())
    .def("DeepCopy", &theia::Camera::DeepCopy)
    .def("CameraIntrinsics", &theia::Camera::CameraIntrinsics, py::return_value_policy::reference)
    .def("SetFromCameraIntrinsicsPriors", &theia::Camera::SetFromCameraIntrinsicsPriors)
    .def("CameraIntrinsicsPriorFromIntrinsics", &theia::Camera::CameraIntrinsicsPriorFromIntrinsics)
    .def("GetCameraIntrinsicsModelType", &theia::Camera::GetCameraIntrinsicsModelType)
    .def("SetCameraIntrinsicsModelType", &theia::Camera::SetCameraIntrinsicsModelType)
    .def("InitializeFromProjectionMatrix", &theia::Camera::InitializeFromProjectionMatrix)
    .def("GetCalibrationMatrix", &theia::Camera::GetCalibrationMatrixWrapper)
    .def("GetProjectionMatrix", &theia::Camera::GetProjectionMatrixWrapper)
    .def_property("FocalLength", &theia::Camera::FocalLength, &theia::Camera::SetFocalLength)
    .def_property_readonly("ImageHeight", &theia::Camera::ImageHeight)
    .def_property_readonly("ImageWidth", &theia::Camera::ImageWidth)
    .def("SetImageSize", &theia::Camera::SetImageSize)
    .def_property_readonly("PrincipalPointX", &theia::Camera::PrincipalPointX)
    .def_property_readonly("PrincipalPointY", &theia::Camera::PrincipalPointY)
    .def("SetPrincipalPoint", &theia::Camera::SetPrincipalPoint)
    .def("GetOrientationAsAngleAxis", &theia::Camera::GetOrientationAsAngleAxis)
    .def("GetOrientationAsRotationMatrix", &theia::Camera::GetOrientationAsRotationMatrix)
    .def("SetOrientationFromAngleAxis", &theia::Camera::SetOrientationFromAngleAxis)
    .def("SetOrientationFromRotationMatrix", &theia::Camera::SetOrientationFromRotationMatrix)
    .def_property("Position", &theia::Camera::GetPosition, &theia::Camera::SetPosition)
    .def("PrintCameraIntrinsics", &theia::Camera::PrintCameraIntrinsics)
    .def("PixelToNormalizedCoordinates", &theia::Camera::PixelToNormalizedCoordinates)
    .def("PixelToUnitDepthRay", &theia::Camera::PixelToUnitDepthRay)
    .def("ProjectPoint", &theia::Camera::ProjectPointWrapper)
    //.def_readonly_static("kExtrinsicsSize", &theia::Camera::kExtrinsicsSize)
  ;


  // tested
  py::enum_<theia::CameraIntrinsicsModelType>(m, "CameraIntrinsicsModelType")
    .value("INVALID", theia::CameraIntrinsicsModelType::INVALID)
    .value("PINHOLE", theia::CameraIntrinsicsModelType::PINHOLE)
    .value("PINHOLE_RADIAL_TANGENTIAL", theia::CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL)
    .value("FISHEYE", theia::CameraIntrinsicsModelType::FISHEYE)
    .value("FOV", theia::CameraIntrinsicsModelType::FOV)
    .value("DIVISION_UNDISTORTION", theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION)
    .value("DOUBLE_SPHERE", theia::CameraIntrinsicsModelType::DOUBLE_SPHERE)
    .value("EXTENDED_UNIFIED", theia::CameraIntrinsicsModelType::EXTENDED_UNIFIED)
    .export_values()
  ;

  py::class_<theia::CameraIntrinsicsPrior>(m, "CameraIntrinsicsPrior")
    .def(py::init())

    .def_readwrite("image_width", &theia::CameraIntrinsicsPrior::image_width)
    .def_readwrite("image_height", &theia::CameraIntrinsicsPrior::image_height)
    .def_readwrite("camera_intrinsics_model_type", &theia::CameraIntrinsicsPrior::camera_intrinsics_model_type)
    .def_readwrite("focal_length", &theia::CameraIntrinsicsPrior::focal_length)
    .def_readwrite("principal_point", &theia::CameraIntrinsicsPrior::principal_point)
    .def_readwrite("aspect_ratio", &theia::CameraIntrinsicsPrior::aspect_ratio)
    .def_readwrite("skew", &theia::CameraIntrinsicsPrior::skew)
    .def_readwrite("radial_distortion", &theia::CameraIntrinsicsPrior::radial_distortion)
    .def_readwrite("tangential_distortion", &theia::CameraIntrinsicsPrior::tangential_distortion)
    .def_readwrite("position", &theia::CameraIntrinsicsPrior::position)
    .def_readwrite("orientation", &theia::CameraIntrinsicsPrior::orientation)
    .def_readwrite("latitude", &theia::CameraIntrinsicsPrior::latitude)
    .def_readwrite("longitude", &theia::CameraIntrinsicsPrior::longitude)
    .def_readwrite("altitude", &theia::CameraIntrinsicsPrior::altitude)

  ;


  //pose
  m.def("PoseFromThreePoints", theia::PoseFromThreePointsWrapper);
  m.def("NormalizedEightPointFundamentalMatrix", theia::NormalizedEightPointFundamentalMatrixWrapper);
  m.def("FivePointRelativePose", theia::FivePointRelativePoseWrapper);
  m.def("FourPointPoseAndFocalLength", theia::FourPointPoseAndFocalLengthWrapper);
  m.def("FourPointHomography", theia::FourPointHomographyWrapper);
  m.def("FourPointsPoseFocalLengthRadialDistortion", theia::FourPointsPoseFocalLengthRadialDistortionWrapper);
  m.def("FourPointRelativePosePartialRotation", theia::FourPointRelativePosePartialRotationWrapper);
  m.def("FivePointFocalLengthRadialDistortion", theia::FivePointFocalLengthRadialDistortionWrapper);
  m.def("ThreePointRelativePosePartialRotation", theia::ThreePointRelativePosePartialRotationWrapper);
  m.def("TwoPointPosePartialRotation", theia::TwoPointPosePartialRotationWrapper);
  m.def("DlsPnp", theia::DlsPnpWrapper);
  m.def("PositionFromTwoRays", theia::PositionFromTwoRaysWrapper);
  m.def("RelativePoseFromTwoPointsWithKnownRotation", theia::RelativePoseFromTwoPointsWithKnownRotationWrapper);
  m.def("SevenPointFundamentalMatrix", theia::SevenPointFundamentalMatrixWrapper);
  m.def("SimTransformPartialRotation", theia::SimTransformPartialRotationWrapper);
  m.def("DecomposeEssentialMatrix", theia::DecomposeEssentialMatrixWrapper);
  m.def("EssentialMatrixFromTwoProjectionMatrices", theia::EssentialMatrixFromTwoProjectionMatricesWrapper);
  m.def("GetBestPoseFromEssentialMatrix", theia::GetBestPoseFromEssentialMatrixWrapper);
  m.def("FocalLengthsFromFundamentalMatrix", theia::FocalLengthsFromFundamentalMatrixWrapper);
  m.def("SharedFocalLengthsFromFundamentalMatrix", theia::SharedFocalLengthsFromFundamentalMatrixWrapper);
  m.def("ProjectionMatricesFromFundamentalMatrix", theia::ProjectionMatricesFromFundamentalMatrixWrapper);
  m.def("FundamentalMatrixFromProjectionMatrices", theia::FundamentalMatrixFromProjectionMatricesWrapper);
  m.def("EssentialMatrixFromFundamentalMatrix", theia::EssentialMatrixFromFundamentalMatrixWrapper);
  m.def("ComposeFundamentalMatrix", theia::ComposeFundamentalMatrixWrapper);

  //transformation
  m.def("AlignPointCloudsUmeyama", theia::AlignPointCloudsUmeyamaWrapper);
  m.def("AlignPointCloudsUmeyamaWithWeights", theia::AlignPointCloudsUmeyamaWithWeightsWrapper);
  m.def("GdlsSimilarityTransform" ,theia::GdlsSimilarityTransformWrapper);
  m.def("AlignRotations", theia::AlignRotationsWrapper);
  m.def("AlignReconstructions", theia::AlignReconstructionsWrapper);
  m.def("AlignReconstructionsRobust", theia::AlignReconstructionsRobustWrapper);
  m.def("TransformReconstruction", theia::TransformReconstructionWrapper);

  py::class_<theia::SimilarityTransformation>(m, "SimilarityTransformation")
    .def(py::init<>())
    .def_readwrite("rotation", &theia::SimilarityTransformation::rotation)
    .def_readwrite("translation", &theia::SimilarityTransformation::translation)
    .def_readwrite("scale", &theia::SimilarityTransformation::scale)
  ;

  py::class_<theia::RigidTransformation>(m, "RigidTransformation")
    .def(py::init<>())
    .def_readwrite("rotation", &theia::RigidTransformation::rotation)
    .def_readwrite("translation", &theia::RigidTransformation::translation)
  ;

  // estimator

  py::class_<theia::Feature>(m, "Feature")
    .def(py::init<>())
    .def(py::init<double, double>())
    .def(py::init<Eigen::Vector2d>())
    .def(py::init<Eigen::Vector2d,Eigen::Matrix2d>())
    .def_readwrite("point", &theia::Feature::point_)
    .def_readwrite("covariance", &theia::Feature::covariance_)
    .def("x", &theia::Feature::x)
    .def("y", &theia::Feature::y)
  ;

  py::class_<theia::CameraAndFeatureCorrespondence2D3D>(m, "CameraAndFeatureCorrespondence2D3D")
    .def(py::init<>())
    .def_readwrite("camera", &theia::CameraAndFeatureCorrespondence2D3D::camera)
    .def_readwrite("observation", &theia::CameraAndFeatureCorrespondence2D3D::observation)
    .def_readwrite("point3d", &theia::CameraAndFeatureCorrespondence2D3D::point3d)
  ;

  py::class_<theia::FeatureCorrespondence2D3D>(m, "FeatureCorrespondence2D3D")
    .def(py::init<>())
    .def_readwrite("feature", &theia::FeatureCorrespondence2D3D::feature)
    .def_readwrite("world_point", &theia::FeatureCorrespondence2D3D::world_point)
  ;

  py::class_<theia::CalibratedAbsolutePose>(m, "CalibratedAbsolutePose")
    .def(py::init<>())
    .def_readwrite("rotation", &theia::CalibratedAbsolutePose::rotation)
    .def_readwrite("position", &theia::CalibratedAbsolutePose::position)
  ;

  py::class_<theia::UncalibratedAbsolutePose>(m, "UncalibratedAbsolutePose")
    .def(py::init<>())
    .def_readwrite("rotation", &theia::UncalibratedAbsolutePose::rotation)
    .def_readwrite("position", &theia::UncalibratedAbsolutePose::position)
    .def_readwrite("position", &theia::UncalibratedAbsolutePose::focal_length)
  ;

  py::class_<theia::UncalibratedRelativePose>(m, "UncalibratedRelativePose")
    .def(py::init<>())
    .def_readwrite("fundamental_matrix", &theia::UncalibratedRelativePose::fundamental_matrix)
    .def_readwrite("focal_length1", &theia::UncalibratedRelativePose::focal_length1)
    .def_readwrite("focal_length2", &theia::UncalibratedRelativePose::focal_length2)
    .def_readwrite("rotation", &theia::UncalibratedRelativePose::rotation)
    .def_readwrite("position", &theia::UncalibratedRelativePose::position)

  ;

  py::class_<theia::RelativePose>(m, "RelativePose")
    .def(py::init<>())
    .def_readwrite("essential_matrix", &theia::RelativePose::essential_matrix)
    .def_readwrite("rotation", &theia::RelativePose::rotation)
    .def_readwrite("position", &theia::RelativePose::position)
  ;

  py::class_<theia::Plane>(m, "Plane")
    .def(py::init<>())
    .def_readwrite("point", &theia::Plane::point)
    .def_readwrite("unit_normal", &theia::Plane::unit_normal)
  ;

  py::class_<theia::RadialDistortionFeatureCorrespondence>(m, "RadialDistortionFeatureCorrespondence")
    .def(py::init<>())
    .def_readwrite("feature_left", &theia::RadialDistortionFeatureCorrespondence::feature_left)
    .def_readwrite("feature_right", &theia::RadialDistortionFeatureCorrespondence::feature_right)
    .def_readwrite("normalized_feature_left", &theia::RadialDistortionFeatureCorrespondence::normalized_feature_left)
    .def_readwrite("normalized_feature_right", &theia::RadialDistortionFeatureCorrespondence::normalized_feature_right)
    .def_readwrite("focal_length_estimate_left", &theia::RadialDistortionFeatureCorrespondence::focal_length_estimate_left)
    .def_readwrite("focal_length_estimate_right", &theia::RadialDistortionFeatureCorrespondence::focal_length_estimate_right)
    .def_readwrite("min_radial_distortion", &theia::RadialDistortionFeatureCorrespondence::min_radial_distortion)
    .def_readwrite("max_radial_distortion", &theia::RadialDistortionFeatureCorrespondence::max_radial_distortion)

  ;



  // estimator ransac




  py::enum_<theia::RansacType>(m, "RansacType")
    .value("RANSAC", theia::RansacType::RANSAC)
    .value("PROSAC", theia::RansacType::PROSAC)
    .value("LMED", theia::RansacType::LMED)
    .value("EXHAUSTIVE", theia::RansacType::EXHAUSTIVE)
    .export_values()
  ;

  m.def("EstimateAbsolutePoseWithKnownOrientation", theia::EstimateAbsolutePoseWithKnownOrientationWrapper);
  m.def("EstimateCalibratedAbsolutePose", theia::EstimateCalibratedAbsolutePoseWrapper);
  m.def("EstimateDominantPlaneFromPoints", theia::EstimateDominantPlaneFromPointsWrapper);
  m.def("EstimateEssentialMatrix", theia::EstimateEssentialMatrixWrapper);
  m.def("EstimateFundamentalMatrix", theia::EstimateFundamentalMatrixWrapper);
  m.def("EstimateHomography", theia::EstimateHomographyWrapper);
  m.def("EstimateRadialHomographyMatrix", theia::EstimateRadialHomographyMatrixWrapper);
  m.def("EstimateRelativePose", theia::EstimateRelativePoseWrapper);
  m.def("EstimateRelativePoseWithKnownOrientation", theia::EstimateRelativePoseWithKnownOrientationWrapper);
  m.def("EstimateRigidTransformation2D3D", theia::EstimateRigidTransformation2D3DWrapper);
  m.def("EstimateRigidTransformation2D3DNormalized", theia::EstimateRigidTransformation2D3DNormalizedWrapper);
  m.def("EstimateTriangulation", theia::EstimateTriangulationWrapper);
  m.def("EstimateUncalibratedAbsolutePose", theia::EstimateUncalibratedAbsolutePoseWrapper);
  m.def("EstimateUncalibratedRelativePose", theia::EstimateUncalibratedRelativePoseWrapper);





  //triangulation
  m.def("Triangulate", theia::TriangulateWrapper);
  m.def("TriangulateMidpoint", theia::TriangulateMidpointWrapper);
  m.def("TriangulateDLT", theia::TriangulateDLTWrapper);
  m.def("TriangulateNViewSVD", theia::TriangulateNViewSVDWrapper);
  m.def("TriangulateNView", theia::TriangulateNViewWrapper);
  m.def("IsTriangulatedPointInFrontOfCameras", theia::IsTriangulatedPointInFrontOfCameras);
  m.def("SufficientTriangulationAngle", theia::SufficientTriangulationAngle);

  // function in the sfm folder

  py::class_<theia::EstimateTwoViewInfoOptions>(m, "EstimateTwoViewInfoOptions")
    .def(py::init<>())
    .def_readwrite("ransac_type", &theia::EstimateTwoViewInfoOptions::ransac_type)
    .def_readwrite("max_sampson_error_pixels", &theia::EstimateTwoViewInfoOptions::max_sampson_error_pixels)
    .def_readwrite("expected_ransac_confidence", &theia::EstimateTwoViewInfoOptions::expected_ransac_confidence)
    .def_readwrite("min_ransac_iterations", &theia::EstimateTwoViewInfoOptions::min_ransac_iterations)
    .def_readwrite("max_ransac_iterations", &theia::EstimateTwoViewInfoOptions::max_ransac_iterations)
    .def_readwrite("use_mle", &theia::EstimateTwoViewInfoOptions::use_mle)
  ;

  py::class_<theia::FilterViewPairsFromRelativeTranslationOptions>(m, "FilterViewPairsFromRelativeTranslationOptions")
    .def(py::init<>())
    .def_readwrite("num_threads", &theia::FilterViewPairsFromRelativeTranslationOptions::num_threads)
    .def_readwrite("num_iterations", &theia::FilterViewPairsFromRelativeTranslationOptions::num_iterations)
    .def_readwrite("translation_projection_tolerance", &theia::FilterViewPairsFromRelativeTranslationOptions::translation_projection_tolerance)
  ;

  py::class_<theia::LocalizeViewToReconstructionOptions>(m, "LocalizeViewToReconstructionOptions")
    .def(py::init<>())
    .def_readwrite("reprojection_error_threshold_pixels", &theia::LocalizeViewToReconstructionOptions::reprojection_error_threshold_pixels)
    .def_readwrite("assume_known_orientation", &theia::LocalizeViewToReconstructionOptions::assume_known_orientation)
    .def_readwrite("ransac_params", &theia::LocalizeViewToReconstructionOptions::ransac_params)
    .def_readwrite("bundle_adjust_view", &theia::LocalizeViewToReconstructionOptions::bundle_adjust_view)
    .def_readwrite("ba_options", &theia::LocalizeViewToReconstructionOptions::ba_options)
    .def_readwrite("min_num_inliers", &theia::LocalizeViewToReconstructionOptions::min_num_inliers)
  ;


  m.def("EstimateTwoViewInfo", theia::EstimateTwoViewInfoWrapper);
  m.def("ColorizeReconstruction", theia::ColorizeReconstructionWrapper);
  m.def("ExtractMaximallyParallelRigidSubgraph", theia::ExtractMaximallyParallelRigidSubgraphWrapper);
  m.def("FilterViewGraphCyclesByRotation", theia::FilterViewGraphCyclesByRotationWrapper);
  m.def("FilterViewPairsFromOrientation", theia::FilterViewPairsFromOrientationWrapper);
  m.def("FilterViewPairsFromRelativeTranslation", theia::FilterViewPairsFromRelativeTranslationWrapper);
  m.def("LocalizeViewToReconstruction", theia::LocalizeViewToReconstructionWrapper);
  m.def("SelectGoodTracksForBundleAdjustmentAll", theia::SelectGoodTracksForBundleAdjustmentAllWrapper);
  m.def("SelectGoodTracksForBundleAdjustment", theia::SelectGoodTracksForBundleAdjustmentWrapper);
  m.def("SetCameraIntrinsicsFromPriors", theia::SetCameraIntrinsicsFromPriorsWrapper);
  m.def("SetOutlierTracksToUnestimated", theia::SetOutlierTracksToUnestimatedWrapper);
  m.def("SetOutlierTracksToUnestimatedAll", theia::SetOutlierTracksToUnestimatedAllWrapper);
  //m.def("UndistortImage", theia::UndistortImageWrapper);
  //m.def("UndistortCamera", theia::UndistortCameraWrapper);
  //m.def("UndistortReconstruction", theia::UndistortReconstructionWrapper);
  m.def("FindCommonViewsByName", theia::FindCommonViewsByName);
  m.def("FindCommonTracksInViews", theia::FindCommonTracksInViews);


  // View class
  py::class_<theia::View>(m, "View")
    .def(py::init<>())
    .def(py::init<std::string>())
    .def_property_readonly("Name", &theia::View::Name)
    .def_property("IsEstimated", &theia::View::IsEstimated, &theia::View::SetEstimated)
    .def("NumFeatures", &theia::View::NumFeatures)
    .def("AddFeature", &theia::View::AddFeature)
    .def("RemoveFeature", &theia::View::RemoveFeature)
    .def_property_readonly("TrackIds", &theia::View::TrackIds)
    //.def_readwrite("focal_length", &theia::View::Track)

    .def("GetFeature", &theia::View::GetFeature, py::return_value_policy::reference)
    .def("Camera", &theia::View::Camera, "Camera class object")
    .def("CameraIntrinsicsPrior", &theia::View::CameraIntrinsicsPrior)
    .def("SetCameraIntrinsicsPrior", &theia::View::SetCameraIntrinsicsPrior)
    .def("MutableCameraIntrinsicsPrior", &theia::View::MutableCameraIntrinsicsPrior, py::return_value_policy::reference)
    .def("MutableCamera", &theia::View::MutableCamera, py::return_value_policy::reference)

  ;
  // Visibility pyramid
  py::class_<theia::VisibilityPyramid>(m, "VisibilityPyramid")
    .def(py::init<int, int, int>())
    .def("AddPoint", &theia::VisibilityPyramid::AddPoint)
    .def("ComputeScore", &theia::VisibilityPyramid::ComputeScore)
  ;



  // TwoViewMatchGeometricVerification Options
  py::class_<theia::TwoViewMatchGeometricVerification::Options>(m, "TwoViewMatchGeometricVerificationOptions")
    .def(py::init<>())
    .def_readwrite("estimate_twoview_info_options", &theia::TwoViewMatchGeometricVerification::Options::estimate_twoview_info_options)
    .def_readwrite("min_num_inlier_matches", &theia::TwoViewMatchGeometricVerification::Options::min_num_inlier_matches)
    .def_readwrite("guided_matching", &theia::TwoViewMatchGeometricVerification::Options::guided_matching)
    .def_readwrite("guided_matching_max_distance_pixels", &theia::TwoViewMatchGeometricVerification::Options::guided_matching_max_distance_pixels)
    .def_readwrite("guided_matching_lowes_ratio", &theia::TwoViewMatchGeometricVerification::Options::guided_matching_lowes_ratio)
    .def_readwrite("bundle_adjustment", &theia::TwoViewMatchGeometricVerification::Options::bundle_adjustment)
    .def_readwrite("triangulation_max_reprojection_error", &theia::TwoViewMatchGeometricVerification::Options::triangulation_max_reprojection_error)
    .def_readwrite("min_triangulation_angle_degrees", &theia::TwoViewMatchGeometricVerification::Options::min_triangulation_angle_degrees)
    .def_readwrite("final_max_reprojection_error", &theia::TwoViewMatchGeometricVerification::Options::final_max_reprojection_error)

  ;

/*
  // KeypointsAndDescriptors
  py::class_<theia::KeypointsAndDescriptors>(m, "KeypointsAndDescriptors")
    .def(py::init<>())
    .def_readwrite("image_name", &theia::KeypointsAndDescriptors::image_name)
    .def_readwrite("keypoints", &theia::KeypointsAndDescriptors::keypoints)
    .def_readwrite("descriptors", &theia::KeypointsAndDescriptors::descriptors)
  ;

  */

  // TwoViewMatchGeometricVerification
  py::class_<theia::TwoViewMatchGeometricVerification>(m, "TwoViewMatchGeometricVerification")
    .def(py::init<theia::TwoViewMatchGeometricVerification::Options, theia::CameraIntrinsicsPrior, theia::CameraIntrinsicsPrior, theia::KeypointsAndDescriptors, theia::KeypointsAndDescriptors, std::vector<theia::IndexedFeatureMatch>>())
    .def("VerifyMatches", &theia::TwoViewMatchGeometricVerification::VerifyMatches)
  ;


  // Track class
  py::class_<theia::Track>(m, "Track")
    .def(py::init<>())

    //.def_property_readonly("Name", &theia::View::Name)
    .def_property("IsEstimated", &theia::Track::IsEstimated, &theia::Track::SetEstimated)
    .def("NumViews", &theia::Track::NumViews)
    .def("AddView", &theia::Track::AddView)
    .def("RemoveView", &theia::Track::RemoveView)
    .def_property_readonly("ViewIds", &theia::Track::ViewIds)
    //.def_readwrite("focal_length", &theia::View::Track)

    //.def("GetFeature", &theia::View::GetFeature, py::return_value_policy::reference)
    .def("Point", &theia::Track::Point)
    .def("MutablePoint", &theia::Track::MutablePoint, py::return_value_policy::reference)
    .def("Color", &theia::Track::Color)
    .def("MutableColor", &theia::Track::MutableColor, py::return_value_policy::reference)
  ;

  // Track builder class
  py::class_<theia::TrackBuilder>(m, "TrackBuilder")
    .def(py::init<int, int>())
    .def("AddFeatureCorrespondence", &theia::TrackBuilder::AddFeatureCorrespondence)
    .def("BuildTracks", &theia::TrackBuilder::BuildTracks)
  ;


  py::class_<theia::BundleAdjustmentOptions>(m, "BundleAdjustmentOptions")
    .def(py::init<>())
    .def_readwrite("loss_function_type", &theia::BundleAdjustmentOptions::loss_function_type)
    .def_readwrite("robust_loss_width", &theia::BundleAdjustmentOptions::robust_loss_width)
    .def_readwrite("verbose", &theia::BundleAdjustmentOptions::verbose)
    .def_readwrite("constant_camera_orientation", &theia::BundleAdjustmentOptions::constant_camera_orientation)
    .def_readwrite("constant_camera_position", &theia::BundleAdjustmentOptions::constant_camera_position)
    .def_readwrite("intrinsics_to_optimize", &theia::BundleAdjustmentOptions::intrinsics_to_optimize)
    .def_readwrite("num_threads", &theia::BundleAdjustmentOptions::num_threads)
    .def_readwrite("max_num_iterations", &theia::BundleAdjustmentOptions::max_num_iterations)
    .def_readwrite("max_solver_time_in_seconds", &theia::BundleAdjustmentOptions::max_solver_time_in_seconds)
    .def_readwrite("use_inner_iterations", &theia::BundleAdjustmentOptions::use_inner_iterations)
    .def_readwrite("function_tolerance", &theia::BundleAdjustmentOptions::function_tolerance)
    .def_readwrite("gradient_tolerance", &theia::BundleAdjustmentOptions::gradient_tolerance)
    .def_readwrite("parameter_tolerance", &theia::BundleAdjustmentOptions::parameter_tolerance)
    .def_readwrite("max_trust_region_radius", &theia::BundleAdjustmentOptions::max_trust_region_radius)
  ;


  // Track Estimator Options
  py::class_<theia::TrackEstimator::Options>(m, "TrackEstimatorOptions")
    .def_readwrite("num_threads", &theia::TrackEstimator::Options::num_threads)
    .def_readwrite("max_acceptable_reprojection_error_pixels", &theia::TrackEstimator::Options::max_acceptable_reprojection_error_pixels)
    .def_readwrite("min_triangulation_angle_degrees", &theia::TrackEstimator::Options::min_triangulation_angle_degrees)
    .def_readwrite("bundle_adjustment", &theia::TrackEstimator::Options::bundle_adjustment)
    //.def_readwrite("BundleAdjustmentOptions", &theia::BundleAdjustmentOptions)
    .def_readwrite("multithreaded_step_size", &theia::TrackEstimator::Options::multithreaded_step_size)
  ;

  // Track Estimator Summary
  py::class_<theia::TrackEstimator::Summary>(m, "TrackEstimatorSummary")
    .def_readwrite("input_num_estimated_tracks", &theia::TrackEstimator::Summary::input_num_estimated_tracks)
    .def_readwrite("num_triangulation_attempts", &theia::TrackEstimator::Summary::num_triangulation_attempts)
    .def_readwrite("estimated_tracks", &theia::TrackEstimator::Summary::estimated_tracks)

  ;

  // Track Estimator class
  py::class_<theia::TrackEstimator>(m, "TrackEstimator")
    //.def(py::init<theia::TrackEstimator::Options, theia::Reconstruction>())
    .def("EstimateAllTracks", &theia::TrackEstimator::EstimateAllTracks)
    .def("EstimateTracks", &theia::TrackEstimator::EstimateTracks)
  ;

  // ReconstructionEstimatorSummary
  py::class_<theia::ReconstructionEstimatorSummary>(m, "ReconstructionEstimatorSummary")
    .def_readwrite("success", &theia::ReconstructionEstimatorSummary::success)
    .def_readwrite("estimated_tracks", &theia::ReconstructionEstimatorSummary::estimated_tracks)
    .def_readwrite("estimated_views", &theia::ReconstructionEstimatorSummary::estimated_views)
    .def_readwrite("camera_intrinsics_calibration_time", &theia::ReconstructionEstimatorSummary::camera_intrinsics_calibration_time)
    .def_readwrite("pose_estimation_time", &theia::ReconstructionEstimatorSummary::pose_estimation_time)
    .def_readwrite("triangulation_time", &theia::ReconstructionEstimatorSummary::triangulation_time)
    .def_readwrite("bundle_adjustment_time", &theia::ReconstructionEstimatorSummary::bundle_adjustment_time)
    .def_readwrite("total_time", &theia::ReconstructionEstimatorSummary::total_time)
    .def_readwrite("message", &theia::ReconstructionEstimatorSummary::message)


  ;


  // Reconstruction Estimator class
  py::class_<theia::ReconstructionEstimator>(m, "ReconstructionEstimator")
    .def_static("Create", &theia::ReconstructionEstimator::Create, py::return_value_policy::reference)
  ;

  // not sure about pointer  GlobalReconstructionEstimator
  py::class_<theia::GlobalReconstructionEstimator, theia::ReconstructionEstimator>(m, "GlobalReconstructionEstimator")
    .def(py::init<theia::ReconstructionEstimatorOptions>())
    .def("Estimate", &theia::GlobalReconstructionEstimator::Estimate)
  ;

  // not sure about pointer  IncrementalReconstructionEstimator
  py::class_<theia::IncrementalReconstructionEstimator, theia::ReconstructionEstimator>(m, "IncrementalReconstructionEstimator")
    .def(py::init<theia::ReconstructionEstimatorOptions>())
    .def("Estimate", &theia::IncrementalReconstructionEstimator::Estimate)
  ;

  // not sure about pointer  HybridReconstructionEstimator
  py::class_<theia::HybridReconstructionEstimator, theia::ReconstructionEstimator>(m, "HybridReconstructionEstimator")
    .def(py::init<theia::ReconstructionEstimatorOptions>())
    .def("Estimate", &theia::HybridReconstructionEstimator::Estimate)
  ;

  // Reconstruction Builder Options
  py::class_<theia::ReconstructionBuilderOptions>(m, "ReconstructionBuilderOptions")
    .def(py::init<>())
    .def_readwrite("reconstruct_largest_connected_component", &theia::ReconstructionBuilderOptions::reconstruct_largest_connected_component)
    .def_readwrite("only_calibrated_views", &theia::ReconstructionBuilderOptions::only_calibrated_views)
    .def_readwrite("min_track_length", &theia::ReconstructionBuilderOptions::min_track_length)
    .def_readwrite("max_track_length", &theia::ReconstructionBuilderOptions::max_track_length)
    .def_readwrite("min_num_inlier_matches", &theia::ReconstructionBuilderOptions::min_num_inlier_matches)
    .def_readwrite("descriptor_type", &theia::ReconstructionBuilderOptions::descriptor_type)
    .def_readwrite("feature_density", &theia::ReconstructionBuilderOptions::feature_density)
    .def_readwrite("matching_strategy", &theia::ReconstructionBuilderOptions::matching_strategy)
    .def_readwrite("matching_options", &theia::ReconstructionBuilderOptions::matching_options)
    .def_readwrite("reconstruction_estimator_options", &theia::ReconstructionBuilderOptions::reconstruction_estimator_options)

    .def_readwrite("features_and_matches_database_directory", &theia::ReconstructionBuilderOptions::features_and_matches_database_directory)
    .def_readwrite("select_image_pairs_with_global_image_descriptor_matching", &theia::ReconstructionBuilderOptions::select_image_pairs_with_global_image_descriptor_matching)
    .def_readwrite("num_nearest_neighbors_for_global_descriptor_matching", &theia::ReconstructionBuilderOptions::num_nearest_neighbors_for_global_descriptor_matching)
    .def_readwrite("num_gmm_clusters_for_fisher_vector", &theia::ReconstructionBuilderOptions::num_gmm_clusters_for_fisher_vector)
    .def_readwrite("max_num_features_for_fisher_vector_training", &theia::ReconstructionBuilderOptions::max_num_features_for_fisher_vector_training)
    .def_readwrite("reconstruction_estimator_options", &theia::ReconstructionBuilderOptions::reconstruction_estimator_options)
  ;

  // Reconstruction Builder
  py::class_<theia::ReconstructionBuilder>(m, "ReconstructionBuilder")
    //.def(py::init<>())
    .def(py::init<theia::ReconstructionBuilderOptions, theia::FeaturesAndMatchesDatabase*>())
    .def("AddImage", (bool (theia::ReconstructionBuilder::*)(const std::string& )) &theia::ReconstructionBuilder::AddImage)
    .def("AddImage", (bool (theia::ReconstructionBuilder::*)(const std::string&,  unsigned int )) &theia::ReconstructionBuilder::AddImage)
    .def("AddImageWithCameraIntrinsicsPrior", (bool (theia::ReconstructionBuilder::*)(const std::string&, const theia::CameraIntrinsicsPrior& )) &theia::ReconstructionBuilder::AddImageWithCameraIntrinsicsPrior)
    .def("AddImageWithCameraIntrinsicsPrior", (bool (theia::ReconstructionBuilder::*)(const std::string&, const theia::CameraIntrinsicsPrior&, unsigned int )) &theia::ReconstructionBuilder::AddImageWithCameraIntrinsicsPrior)
    //.def("AddTwoViewMatch", &theia::ReconstructionBuilder::AddTwoViewMatch)
    .def("AddMaskForFeaturesExtraction", &theia::ReconstructionBuilder::AddMaskForFeaturesExtraction)
    .def("ExtractAndMatchFeatures", &theia::ReconstructionBuilder::ExtractAndMatchFeatures)

  ;



  // Reconstruction Options

  py::enum_<theia::ReconstructionEstimatorType>(m, "ReconstructionEstimatorType")
    .value("GLOBAL", theia::ReconstructionEstimatorType::GLOBAL)
    .value("INCREMENTAL", theia::ReconstructionEstimatorType::INCREMENTAL)
    .value("HYBRID", theia::ReconstructionEstimatorType::HYBRID)
    .export_values()
  ;

  py::enum_<theia::GlobalPositionEstimatorType>(m, "GlobalPositionEstimatorType")
    .value("NONLINEAR", theia::GlobalPositionEstimatorType::NONLINEAR)
    .value("LINEAR_TRIPLET", theia::GlobalPositionEstimatorType::LINEAR_TRIPLET)
    .value("LEAST_UNSQUARED_DEVIATION", theia::GlobalPositionEstimatorType::LEAST_UNSQUARED_DEVIATION)
    .export_values()
  ;

  py::enum_<theia::GlobalRotationEstimatorType>(m, "GlobalRotationEstimatorType")
    .value("ROBUST_L1L2", theia::GlobalRotationEstimatorType::ROBUST_L1L2)
    .value("NONLINEAR", theia::GlobalRotationEstimatorType::NONLINEAR)
    .value("LINEAR", theia::GlobalRotationEstimatorType::LINEAR)
    .export_values()
  ;

  // ReconstructionEstimatorOptions
  py::class_<theia::ReconstructionEstimatorOptions>(m, "ReconstructionEstimatorOptions")
    .def(py::init<>())

    .def_readwrite("reconstruction_estimator_type", &theia::ReconstructionEstimatorOptions::reconstruction_estimator_type)
    .def_readwrite("global_position_estimator_type", &theia::ReconstructionEstimatorOptions::global_position_estimator_type)
    .def_readwrite("global_rotation_estimator_type", &theia::ReconstructionEstimatorOptions::global_rotation_estimator_type)
    .def_readwrite("num_threads", &theia::ReconstructionEstimatorOptions::num_threads)
    .def_readwrite("max_reprojection_error_in_pixels", &theia::ReconstructionEstimatorOptions::max_reprojection_error_in_pixels)
    .def_readwrite("min_num_two_view_inliers", &theia::ReconstructionEstimatorOptions::min_num_two_view_inliers)
    .def_readwrite("ransac_confidence", &theia::ReconstructionEstimatorOptions::ransac_confidence)
    .def_readwrite("ransac_min_iterations", &theia::ReconstructionEstimatorOptions::ransac_min_iterations)
    .def_readwrite("ransac_max_iterations", &theia::ReconstructionEstimatorOptions::ransac_max_iterations)
    .def_readwrite("ransac_use_mle", &theia::ReconstructionEstimatorOptions::ransac_use_mle)
    .def_readwrite("rotation_filtering_max_difference_degrees", &theia::ReconstructionEstimatorOptions::rotation_filtering_max_difference_degrees)
    .def_readwrite("refine_relative_translations_after_rotation_estimation", &theia::ReconstructionEstimatorOptions::refine_relative_translations_after_rotation_estimation)
    .def_readwrite("extract_maximal_rigid_subgraph", &theia::ReconstructionEstimatorOptions::extract_maximal_rigid_subgraph)
    .def_readwrite("filter_relative_translations_with_1dsfm", &theia::ReconstructionEstimatorOptions::filter_relative_translations_with_1dsfm)
    .def_readwrite("translation_filtering_num_iterations", &theia::ReconstructionEstimatorOptions::translation_filtering_num_iterations)
    .def_readwrite("translation_filtering_projection_tolerance", &theia::ReconstructionEstimatorOptions::translation_filtering_projection_tolerance)
    .def_readwrite("rotation_estimation_robust_loss_scale", &theia::ReconstructionEstimatorOptions::rotation_estimation_robust_loss_scale)
    .def_readwrite("nonlinear_position_estimator_options", &theia::ReconstructionEstimatorOptions::nonlinear_position_estimator_options)
    .def_readwrite("linear_triplet_position_estimator_options", &theia::ReconstructionEstimatorOptions::linear_triplet_position_estimator_options)
    .def_readwrite("least_unsquared_deviation_position_estimator_options", &theia::ReconstructionEstimatorOptions::least_unsquared_deviation_position_estimator_options)
    .def_readwrite("refine_camera_positions_and_points_after_position_estimation", &theia::ReconstructionEstimatorOptions::refine_camera_positions_and_points_after_position_estimation)
    .def_readwrite("multiple_view_localization_ratio", &theia::ReconstructionEstimatorOptions::multiple_view_localization_ratio)
    .def_readwrite("absolute_pose_reprojection_error_threshold", &theia::ReconstructionEstimatorOptions::absolute_pose_reprojection_error_threshold)
    .def_readwrite("min_num_absolute_pose_inliers", &theia::ReconstructionEstimatorOptions::min_num_absolute_pose_inliers)
    .def_readwrite("full_bundle_adjustment_growth_percent", &theia::ReconstructionEstimatorOptions::full_bundle_adjustment_growth_percent)
    .def_readwrite("partial_bundle_adjustment_num_views", &theia::ReconstructionEstimatorOptions::partial_bundle_adjustment_num_views)
    .def_readwrite("relative_position_estimation_max_sampson_error_pixels", &theia::ReconstructionEstimatorOptions::relative_position_estimation_max_sampson_error_pixels)
    .def_readwrite("min_triangulation_angle_degrees", &theia::ReconstructionEstimatorOptions::min_triangulation_angle_degrees)
    .def_readwrite("bundle_adjust_tracks", &theia::ReconstructionEstimatorOptions::bundle_adjust_tracks)
    .def_readwrite("num_retriangulation_iterations", &theia::ReconstructionEstimatorOptions::num_retriangulation_iterations)
    .def_readwrite("bundle_adjustment_loss_function_type", &theia::ReconstructionEstimatorOptions::bundle_adjustment_loss_function_type)
    .def_readwrite("bundle_adjustment_robust_loss_width", &theia::ReconstructionEstimatorOptions::bundle_adjustment_robust_loss_width)
    .def_readwrite("min_cameras_for_iterative_solver", &theia::ReconstructionEstimatorOptions::min_cameras_for_iterative_solver)
    .def_readwrite("intrinsics_to_optimize", &theia::ReconstructionEstimatorOptions::intrinsics_to_optimize)
    .def_readwrite("subsample_tracks_for_bundle_adjustment", &theia::ReconstructionEstimatorOptions::subsample_tracks_for_bundle_adjustment)
    .def_readwrite("track_subset_selection_long_track_length_threshold", &theia::ReconstructionEstimatorOptions::track_subset_selection_long_track_length_threshold)
    .def_readwrite("track_selection_image_grid_cell_size_pixels", &theia::ReconstructionEstimatorOptions::track_selection_image_grid_cell_size_pixels)
    .def_readwrite("min_num_optimized_tracks_per_view", &theia::ReconstructionEstimatorOptions::min_num_optimized_tracks_per_view)
  ;

  // Reconstruction class
  py::class_<theia::Reconstruction>(m, "Reconstruction")
    .def(py::init<>())

    //.def_property("IsEstimated", &theia::Reconstruction::IsEstimated, &theia::Track::SetEstimated)
    .def("NumViews", &theia::Reconstruction::NumViews)
    .def("ViewIdFromName", &theia::Reconstruction::ViewIdFromName)
    //.def("set", static_cast<void (Pet::*)(int)>(&Pet::set), "Set the pet's age")
    //.def("AddView", static_cast<theia::ViewId (theia::Reconstruction*)(const std::string&)>(&theia::Reconstruction::AddView))
    .def("AddView", (theia::ViewId (theia::Reconstruction::*)(const std::string&, const double)) &theia::Reconstruction::AddView, py::return_value_policy::reference_internal)
    .def("AddView", (theia::ViewId (theia::Reconstruction::*)(const std::string&, const theia::CameraIntrinsicsGroupId, const double)) &theia::Reconstruction::AddView, py::return_value_policy::reference_internal)
    .def("RemoveView", &theia::Reconstruction::RemoveView)
    .def_property_readonly("ViewIds", &theia::Reconstruction::ViewIds)

    .def_property_readonly("NumTracks", &theia::Reconstruction::NumTracks)
    .def("AddTrack", (theia::TrackId (theia::Reconstruction::*)()) &theia::Reconstruction::AddTrack, py::return_value_policy::reference_internal)
    .def("AddTrack", (theia::TrackId (theia::Reconstruction::*)(const std::vector<std::pair<theia::ViewId, theia::Feature>>&)) &theia::Reconstruction::AddTrack, py::return_value_policy::reference_internal)

          //.def("AddTrack", &theia::Reconstruction::AddTrack)
    .def("RemoveTrack", &theia::Reconstruction::RemoveTrack)
    .def_property_readonly("TrackIds", &theia::Reconstruction::TrackIds)

    .def("AddObservation", &theia::Reconstruction::AddObservation)
    .def_property_readonly("NumCameraIntrinsicGroups", &theia::Reconstruction::NumCameraIntrinsicGroups)
    .def("Normalize", &theia::Reconstruction::Normalize)
    .def("CameraIntrinsicsGroupIdFromViewId", &theia::Reconstruction::CameraIntrinsicsGroupIdFromViewId)
    .def("CameraIntrinsicsGroupIds", &theia::Reconstruction::CameraIntrinsicsGroupIds)


    .def("View", &theia::Reconstruction::View, py::return_value_policy::reference)
    .def("MutableView", &theia::Reconstruction::MutableView, py::return_value_policy::reference)

    .def("Track", &theia::Reconstruction::Track, py::return_value_policy::reference)
    .def("MutableTrack", &theia::Reconstruction::MutableTrack, py::return_value_policy::reference)

    .def("GetViewsInCameraIntrinsicGroup", &theia::Reconstruction::GetViewsInCameraIntrinsicGroup)
    //.def("GetSubReconstruction", &theia::Reconstruction::GetSubReconstructionWrapper)
  ;

  // Reconstruction Estimator

  // ExifReader
  // py::class_<theia::ExifReader>(m, "ExifReader")
  //   .def(py::init<>())
  //   .def("ExtractEXIFMetadataWrapper", &theia::ExifReader::ExtractEXIFMetadataWrapper)
  // ;






  // TwoViewInfo
  py::class_<theia::TwoViewInfo>(m, "TwoViewInfo")
    .def(py::init<>())
    .def_readwrite("focal_length_1", &theia::TwoViewInfo::focal_length_1)
    .def_readwrite("focal_length_2", &theia::TwoViewInfo::focal_length_2)
    .def_readwrite("position_2", &theia::TwoViewInfo::position_2)
    .def_readwrite("rotation_2", &theia::TwoViewInfo::rotation_2)
    .def_readwrite("num_verified_matches", &theia::TwoViewInfo::num_verified_matches)
    .def_readwrite("num_homography_inliers", &theia::TwoViewInfo::num_homography_inliers)
    .def_readwrite("visibility_score", &theia::TwoViewInfo::visibility_score)
  ;

  m.def("SwapCameras", &theia::SwapCameras);


  // ViewGraph
  py::class_<theia::ViewGraph>(m, "ViewGraph")
    .def(py::init<>())
    //.def_property_readonly("Name", &theia::View::Name)
    .def("ReadFromDisk", &theia::ViewGraph::ReadFromDisk)
    .def("WriteToDisk", &theia::ViewGraph::WriteToDisk)
    .def("HasView", &theia::ViewGraph::HasView)
    .def("RemoveView", &theia::ViewGraph::RemoveView)
    .def("HasEdge", &theia::ViewGraph::HasEdge)
    .def("AddEdge", &theia::ViewGraph::AddEdge)
    .def("RemoveEdge", &theia::ViewGraph::RemoveEdge)
    .def_property_readonly("NumViews", &theia::ViewGraph::NumViews)
    .def_property_readonly("NumEdges", &theia::ViewGraph::NumEdges)
    .def("GetNeighborIdsForView", &theia::ViewGraph::GetNeighborIdsForView, py::return_value_policy::reference)
    .def("GetEdge", &theia::ViewGraph::GetEdge, py::return_value_policy::reference)
    .def("GetAllEdges", &theia::ViewGraph::GetAllEdges)

    // not sure pointer as input
    //.def("ExtractSubgraph", &theia::ViewGraph::ExtractSubgraph)
    //.def("GetLargestConnectedComponentIds", &theia::ViewGraph::GetLargestConnectedComponentIds)

  ;




  // GPS converter
  py::class_<theia::GPSConverter>(m, "GPSConverter")
    .def(py::init<>())
    .def_static("ECEFToLLA", theia::GPSConverter::ECEFToLLA)
    .def_static("LLAToECEF", theia::GPSConverter::LLAToECEF)
  ;

  // Bundle Adjustment
  py::enum_<theia::OptimizeIntrinsicsType>(m, "OptimizeIntrinsicsType")
    .value("NONE", theia::OptimizeIntrinsicsType::NONE)
    .value("FOCAL_LENGTH", theia::OptimizeIntrinsicsType::FOCAL_LENGTH)
    .value("ASPECT_RATIO", theia::OptimizeIntrinsicsType::ASPECT_RATIO)
    .value("SKEW", theia::OptimizeIntrinsicsType::SKEW)
    .value("PRINCIPAL_POINTS", theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS)
    .value("RADIAL_DISTORTION", theia::OptimizeIntrinsicsType::RADIAL_DISTORTION)
    .value("TANGENTIAL_DISTORTION", theia::OptimizeIntrinsicsType::TANGENTIAL_DISTORTION)
    .value("ALL", theia::OptimizeIntrinsicsType::ALL)
    .export_values()
  ;

  py::enum_<theia::LossFunctionType>(m, "LossFunctionType")
    .value("TRIVIAL", theia::LossFunctionType::TRIVIAL)
    .value("HUBER", theia::LossFunctionType::HUBER)
    .value("SOFTLONE", theia::LossFunctionType::SOFTLONE)
    .value("CAUCHY", theia::LossFunctionType::CAUCHY)
    .value("ARCTAN", theia::LossFunctionType::ARCTAN)
    .value("TUKEY", theia::LossFunctionType::TUKEY)
    .export_values()
  ;

  //TwoViewBundleAdjustmentOptions
  py::class_<theia::TwoViewBundleAdjustmentOptions>(m, "TwoViewBundleAdjustmentOptions")
    .def(py::init<>())
    .def_readwrite("ba_options", &theia::TwoViewBundleAdjustmentOptions::ba_options)
    .def_readwrite("constant_camera1_intrinsics", &theia::TwoViewBundleAdjustmentOptions::constant_camera1_intrinsics)
    .def_readwrite("constant_camera2_intrinsics", &theia::TwoViewBundleAdjustmentOptions::constant_camera2_intrinsics)
  ;

  py::class_<theia::BundleAdjustmentSummary>(m, "BundleAdjustmentSummary")
    .def(py::init<>())
    .def_readwrite("success", &theia::BundleAdjustmentSummary::success)
    .def_readwrite("initial_cost", &theia::BundleAdjustmentSummary::initial_cost)
    .def_readwrite("final_cost", &theia::BundleAdjustmentSummary::final_cost)
    .def_readwrite("setup_time_in_seconds", &theia::BundleAdjustmentSummary::setup_time_in_seconds)
    .def_readwrite("solve_time_in_seconds", &theia::BundleAdjustmentSummary::solve_time_in_seconds)
  ;

  m.def("BundleAdjustPartialReconstruction", theia::BundleAdjustPartialReconstructionWrapper);
  m.def("BundleAdjustReconstruction", theia::BundleAdjustReconstructionWrapper);
  m.def("BundleAdjustView", theia::BundleAdjustViewWrapper);
  m.def("BundleAdjustTrack", theia::BundleAdjustTrackWrapper);
  m.def("BundleAdjustViewWithCov", theia::BundleAdjustViewWithCovWrapper);
  m.def("BundleAdjustTrackWithCov", theia::BundleAdjustTrackWithCovWrapper);

  m.def("BundleAdjustTwoViews", theia::BundleAdjustTwoViewsWrapper);
  m.def("BundleAdjustTwoViewsAngular", theia::BundleAdjustTwoViewsAngularWrapper);
  m.def("OptimizeRelativePositionWithKnownRotation", theia::OptimizeRelativePositionWithKnownRotationWrapper);

  // Bundle Adjuster
  py::class_<theia::BundleAdjuster>(m, "BundleAdjuster")
    // constructor uses pointer of an object as input
    //.def(py::init<theia::BundleAdjustmentOptions, theia::Reconstruction>())

    .def("AddView", &theia::BundleAdjuster::AddView)
    .def("AddTrack", &theia::BundleAdjuster::AddTrack)
    .def("Optimize", &theia::BundleAdjuster::Optimize)
    //.def("SetCameraExtrinsicsParameterization", &theia::BundleAdjuster::SetCameraExtrinsicsParameterization)
    //.def("SetCameraIntrinsicsParameterization", &theia::BundleAdjuster::SetCameraIntrinsicsParameterization)
    //.def("SetCameraExtrinsicsConstant", &theia::BundleAdjuster::SetCameraExtrinsicsConstant)
    //.def("SetCameraPositionConstant", &theia::BundleAdjuster::SetCameraPositionConstant)
    //.def("SetCameraOrientationConstant", &theia::BundleAdjuster::SetCameraOrientationConstant)
    //.def("SetTrackConstant", &theia::BundleAdjuster::SetTrackConstant)
    //.def("SetTrackVariable", &theia::BundleAdjuster::SetTrackVariable)
    //.def("SetCameraSchurGroups", &theia::BundleAdjuster::SetCameraSchurGroups)
    //.def("SetTrackSchurGroup", &theia::BundleAdjuster::SetTrackSchurGroup)
    //.def("AddReprojectionErrorResidual", &theia::BundleAdjuster::AddReprojectionErrorResidual)
  ;


  // Global SfM
  m.def("ComputeTripletBaselineRatios", theia::ComputeTripletBaselineRatiosWrapper);

  // Position Estimator
  py::class_<theia::PositionEstimator>(m, "PositionEstimator")
  ;

  py::class_<theia::LinearPositionEstimator, theia::PositionEstimator>(m, "LinearPositionEstimator")
    .def(py::init<theia::LinearPositionEstimator::Options, theia::Reconstruction>())
    .def("EstimatePositions", &theia::LinearPositionEstimator::EstimatePositions)

  ;

  py::class_<theia::NonlinearPositionEstimator, theia::PositionEstimator>(m, "NonlinearPositionEstimator")
    .def(py::init<theia::NonlinearPositionEstimator::Options, theia::Reconstruction>())
    .def("EstimatePositions", &theia::NonlinearPositionEstimator::EstimatePositions)

  ;

  py::class_<theia::LeastUnsquaredDeviationPositionEstimator, theia::PositionEstimator>(m, "LeastUnsquaredDeviationPositionEstimator")
    .def(py::init<theia::LeastUnsquaredDeviationPositionEstimator::Options>())
    .def("EstimatePositions", &theia::LeastUnsquaredDeviationPositionEstimator::EstimatePositions)

  ;


  py::class_<theia::RotationEstimator>(m, "RotationEstimator")
  ;

  py::class_<theia::RobustRotationEstimator, theia::RotationEstimator>(m, "RobustRotationEstimator")
    .def(py::init<theia::RobustRotationEstimator::Options>())
    //.def("EstimateRotations", &theia::RobustRotationEstimator::EstimateRotations)
    .def("AddRelativeRotationConstraint", &theia::RobustRotationEstimator::AddRelativeRotationConstraint)
    //.def("EstimateRotations", &theia::RobustRotationEstimator::EstimateRotations)

  ;

  py::class_<theia::NonlinearRotationEstimator, theia::RotationEstimator>(m, "NonlinearRotationEstimator")
    .def(py::init<>())
    .def(py::init<double>())
    .def("EstimateRotations", &theia::NonlinearRotationEstimator::EstimateRotations)

  ;

  py::class_<theia::LinearRotationEstimator, theia::RotationEstimator>(m, "LinearRotationEstimator")
    .def(py::init<>())
    .def("AddRelativeRotationConstraint", &theia::LinearRotationEstimator::AddRelativeRotationConstraint)
    //.def("EstimateRotations", &theia::RobustRotationEstimator::EstimateRotations)
    //.def("EstimateRotations", &theia::RobustRotationEstimator::EstimateRotations)

  ;
}

void pytheia_sfm(py::module &m) {
    py::module m_submodule = m.def_submodule("sfm");
    pytheia_sfm_classes(m_submodule);
}

}
}