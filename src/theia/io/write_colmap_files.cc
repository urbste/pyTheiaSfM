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
// Author: Aleksander Holynski (holynski@cs.washington.edu)

#include "theia/io/write_colmap_files.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model.h"
#include "theia/sfm/camera/full_opencv_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/sfm/camera/thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/track.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"

namespace theia {
namespace {

// A COLMAP camera model with its binary model id and parameters in COLMAP order.
struct ColmapModelSpec {
  int binary_id;
  std::string name;
  std::vector<double> params;
};

// COLMAP model ids for the models that are not pinhole models
// (colmap/src/colmap/sensor/models.h).
enum ColmapExtendedModelId : int {
  kColmapRadialFisheye = 9,
  kColmapFullOpenCv = 6,
  kColmapThinPrismFisheye = 10,
  kColmapRadTanThinPrismFisheye = 11,
};

// Returns true if the camera uses a model that COLMAP has an equivalent for
// and that is not a pinhole model.
bool IsExtendedColmapCamera(const Camera& camera) {
  switch (camera.GetCameraIntrinsicsModelType()) {
    case CameraIntrinsicsModelType::FULL_OPENCV:
    case CameraIntrinsicsModelType::RADIAL_FISHEYE:
    case CameraIntrinsicsModelType::THIN_PRISM_FISHEYE:
    case CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE:
      return true;
    default:
      return false;
  }
}

// Converts a camera with an extended model to the COLMAP model. Parameters are
// written in COLMAP order: fx, fy, cx, cy followed by the distortion terms, which
// are stored at the same indices in the pyTheia models. Returns false (and logs
// an error) if COLMAP cannot represent the camera.
bool ExtendedColmapModel(const Camera& camera,
                         const std::string& view_name,
                         ColmapModelSpec* spec) {
  const CameraIntrinsicsModel& intrinsics = *camera.CameraIntrinsics();
  Eigen::Matrix3d K;
  camera.GetCalibrationMatrix(&K);
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);

  // Distortion parameters start after the focal length, aspect ratio and
  // principal point (indices 0..3).
  const int kFirstDistortionIndex = 4;
  std::vector<double> distortion;
  for (int i = kFirstDistortionIndex; i < intrinsics.NumParameters(); ++i) {
    distortion.push_back(intrinsics.GetParameter(i));
  }

  switch (camera.GetCameraIntrinsicsModelType()) {
    case CameraIntrinsicsModelType::FULL_OPENCV:
      spec->binary_id = kColmapFullOpenCv;
      spec->name = "FULL_OPENCV";
      spec->params = {fx, fy, cx, cy};
      break;
    case CameraIntrinsicsModelType::THIN_PRISM_FISHEYE:
      spec->binary_id = kColmapThinPrismFisheye;
      spec->name = "THIN_PRISM_FISHEYE";
      spec->params = {fx, fy, cx, cy};
      break;
    case CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE:
      spec->binary_id = kColmapRadTanThinPrismFisheye;
      spec->name = "RAD_TAN_THIN_PRISM_FISHEYE";
      spec->params = {fx, fy, cx, cy};
      break;
    case CameraIntrinsicsModelType::RADIAL_FISHEYE:
      // COLMAP's RADIAL_FISHEYE has a single focal length.
      if (std::abs(fx - fy) > 1e-9 * std::max(1.0, fx)) {
        LOG(ERROR) << "Could not add camera " << view_name
                   << " to the COLMAP output file because COLMAP's "
                      "RADIAL_FISHEYE model has one focal length and this "
                      "camera has a non-unit aspect ratio.";
        return false;
      }
      spec->binary_id = kColmapRadialFisheye;
      spec->name = "RADIAL_FISHEYE";
      spec->params = {fx, cx, cy};
      break;
    default:
      LOG(ERROR) << "Camera " << view_name << " uses an unsupported model.";
      return false;
  }
  spec->params.insert(spec->params.end(), distortion.begin(), distortion.end());
  return true;
}

bool WriteCamerasFile(const Reconstruction& reconstruction,
                      const std::string& cameras_file) {
  std::ofstream ofs_cameras(cameras_file);
  if (!ofs_cameras.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << cameras_file << " for writing.";
    return false;
  }

  const auto& group_ids = reconstruction.CameraIntrinsicsGroupIds();

  for (auto group_id : group_ids) {
    const View* view = reconstruction.View(
        *reconstruction.GetViewsInCameraIntrinsicGroup(group_id).begin());
    const Camera& camera = view->Camera();
    if (IsExtendedColmapCamera(camera)) {
      ColmapModelSpec spec;
      if (!ExtendedColmapModel(camera, view->Name(), &spec)) {
        return false;
      }
      const std::streamsize precision =
          ofs_cameras.precision(std::numeric_limits<double>::max_digits10);
      ofs_cameras << group_id << " " << spec.name << " " << camera.ImageWidth()
                  << " " << camera.ImageHeight();
      for (const double param : spec.params) {
        ofs_cameras << " " << param;
      }
      ofs_cameras << std::endl;
      ofs_cameras.precision(precision);
      continue;
    }
    if (camera.GetCameraIntrinsicsModelType() !=
        CameraIntrinsicsModelType::PINHOLE) {
      LOG(ERROR) << "Could not add camera " << view->Name()
                 << " to the COLMAP output file because COLMAP doesn't support "
                    "FOV or fisheye cameras.";
      return false;
    }

    const CameraIntrinsicsModel& intrinsics = *camera.CameraIntrinsics();
    ofs_cameras
        << group_id << " RADIAL " << camera.ImageWidth() << " "
        << camera.ImageHeight() << " " << camera.FocalLength() << " "
        << camera.PrincipalPointX() << " " << camera.PrincipalPointY() << " "
        << intrinsics.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_1)
        << " "
        << intrinsics.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_2)
        << std::endl;
  }
  ofs_cameras.close();

  return true;
}

bool WriteImagesFile(const Reconstruction& reconstruction,
                     const std::string& images_file) {
  std::ofstream ofs_images(images_file);

  if (!ofs_images.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << images_file << " for writing.";
    return false;
  }

  for (auto view_id : reconstruction.ViewIds()) {
    const View* view = reconstruction.View(view_id);
    const Camera& camera = view->Camera();
    const Eigen::Vector3d translation =
        -camera.GetOrientationAsRotationMatrix() * camera.GetPosition();
    const Eigen::Quaterniond orientation(
        camera.GetOrientationAsRotationMatrix());
    ofs_images << view_id << " " << orientation.w() << " " << orientation.x()
               << " " << orientation.y() << " " << orientation.z() << " "
               << translation.x() << " " << translation.y() << " "
               << translation.z() << " "
               << reconstruction.CameraIntrinsicsGroupIdFromViewId(view_id)
               << " " << view->Name() << std::endl;
    const auto& track_ids = view->TrackIds();
    for (auto track_id : track_ids) {
      auto feature = view->GetFeature(track_id);
      ofs_images << feature->point_.x() << " " << feature->point_.y() << " "
                 << track_id << " ";
    }
    ofs_images << std::endl;
  }
  ofs_images.close();
  return true;
}

bool WritePointsFile(const Reconstruction& reconstruction,
                     const std::string& points_file) {
  std::ofstream ofs_points(points_file);
  if (!ofs_points.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << points_file << " for writing.";
    return false;
  }

  for (auto track_id : reconstruction.TrackIds()) {
    const Track* track = reconstruction.Track(track_id);
    const Eigen::Vector3d point = track->Point().hnormalized();
    const auto& color = track->Color();
    ofs_points << track_id << " " << point.x() << " " << point.y() << " "
               << point.z() << " " << static_cast<int>(color[0]) << " "
               << static_cast<int>(color[1]) << " "
               << static_cast<int>(color[2]) << " " << 0.0 << " ";
    for (auto view_id : track->ViewIds()) {
      const View* view = reconstruction.View(view_id);
      const auto& track_ids = view->TrackIds();
      const int point_index =
          std::find(track_ids.begin(), track_ids.end(), track_id) -
          track_ids.begin();
      ofs_points << view_id << " " << point_index << " ";
    }
    ofs_points << std::endl;
  }

  ofs_points.close();
  return true;
}

// ----------------------------------------------------------------------------
// Binary COLMAP format helpers. All values are little-endian as specified by
// the COLMAP binary model format (cameras.bin, images.bin, points3D.bin).
// ----------------------------------------------------------------------------

// COLMAP camera model ids (colmap/src/base/camera_models.h).
enum ColmapCameraModelId : int {
  kColmapSimplePinhole = 0,  // f, cx, cy
  kColmapPinhole = 1,        // fx, fy, cx, cy
  kColmapOpenCv = 4          // fx, fy, cx, cy, k1, k2, p1, p2
};

bool IsLittleEndianHost() {
  const uint16_t one = 1;
  unsigned char first_byte;
  std::memcpy(&first_byte, &one, 1);
  return first_byte == 1;
}

template <typename T>
void WriteLittleEndian(std::ofstream* ofs, const T value) {
  static_assert(std::is_arithmetic<T>::value,
                "WriteLittleEndian only supports arithmetic types.");
  unsigned char bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  if (!IsLittleEndianHost()) {
    std::reverse(bytes, bytes + sizeof(T));
  }
  ofs->write(reinterpret_cast<const char*>(bytes), sizeof(T));
}

bool WriteCamerasBinaryFile(const Reconstruction& reconstruction,
                            const std::string& cameras_file) {
  std::ofstream ofs(cameras_file, std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << cameras_file << " for writing.";
    return false;
  }

  const auto& group_ids = reconstruction.CameraIntrinsicsGroupIds();
  WriteLittleEndian<uint64_t>(&ofs, group_ids.size());
  for (auto group_id : group_ids) {
    const View* view = reconstruction.View(
        *reconstruction.GetViewsInCameraIntrinsicGroup(group_id).begin());
    const Camera& camera = view->Camera();
    if (IsExtendedColmapCamera(camera)) {
      ColmapModelSpec spec;
      if (!ExtendedColmapModel(camera, view->Name(), &spec)) {
        return false;
      }
      WriteLittleEndian<uint32_t>(&ofs, group_id);
      WriteLittleEndian<int32_t>(&ofs, spec.binary_id);
      WriteLittleEndian<uint64_t>(&ofs,
                                  static_cast<uint64_t>(camera.ImageWidth()));
      WriteLittleEndian<uint64_t>(&ofs,
                                  static_cast<uint64_t>(camera.ImageHeight()));
      for (double param : spec.params) {
        WriteLittleEndian<double>(&ofs, param);
      }
      continue;
    }
    if (camera.GetCameraIntrinsicsModelType() !=
        CameraIntrinsicsModelType::PINHOLE) {
      LOG(ERROR) << "Could not add camera " << view->Name()
                 << " to the COLMAP output file because COLMAP doesn't support "
                    "FOV or fisheye cameras.";
      return false;
    }

    const CameraIntrinsicsModel& intrinsics = *camera.CameraIntrinsics();
    Eigen::Matrix3d K;
    camera.GetCalibrationMatrix(&K);
    const double fx = K(0, 0);
    const double fy = K(1, 1);
    const double cx = K(0, 2);
    const double cy = K(1, 2);
    const double k1 =
        intrinsics.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_1);
    const double k2 =
        intrinsics.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_2);
    if (K(0, 1) != 0.0) {
      LOG(WARNING) << "Camera group " << group_id
                   << " has skew, which COLMAP camera models cannot "
                      "represent. Skew is ignored.";
    }

    // Use the simplest COLMAP model that represents the camera exactly.
    const bool square_pixels = std::abs(fx - fy) <= 1e-9 * std::max(1.0, fx);
    const bool has_distortion = (k1 != 0.0) || (k2 != 0.0);
    int model_id;
    std::vector<double> params;
    if (!has_distortion && square_pixels) {
      model_id = kColmapSimplePinhole;
      params = {fx, cx, cy};
    } else if (!has_distortion) {
      model_id = kColmapPinhole;
      params = {fx, fy, cx, cy};
    } else {
      model_id = kColmapOpenCv;
      params = {fx, fy, cx, cy, k1, k2, 0.0, 0.0};
    }

    WriteLittleEndian<uint32_t>(&ofs, group_id);
    WriteLittleEndian<int32_t>(&ofs, model_id);
    WriteLittleEndian<uint64_t>(&ofs,
                                static_cast<uint64_t>(camera.ImageWidth()));
    WriteLittleEndian<uint64_t>(&ofs,
                                static_cast<uint64_t>(camera.ImageHeight()));
    for (double param : params) {
      WriteLittleEndian<double>(&ofs, param);
    }
  }
  ofs.close();
  return true;
}

bool WriteImagesBinaryFile(const Reconstruction& reconstruction,
                           const std::string& images_file) {
  std::ofstream ofs(images_file, std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << images_file << " for writing.";
    return false;
  }

  const auto& view_ids = reconstruction.ViewIds();
  WriteLittleEndian<uint64_t>(&ofs, view_ids.size());
  for (auto view_id : view_ids) {
    const View* view = reconstruction.View(view_id);
    const Camera& camera = view->Camera();
    const Eigen::Vector3d translation =
        -camera.GetOrientationAsRotationMatrix() * camera.GetPosition();
    Eigen::Quaterniond orientation(camera.GetOrientationAsRotationMatrix());
    orientation.normalize();
    if (orientation.w() < 0.0) {
      orientation.coeffs() *= -1.0;
    }

    WriteLittleEndian<uint32_t>(&ofs, view_id);
    WriteLittleEndian<double>(&ofs, orientation.w());
    WriteLittleEndian<double>(&ofs, orientation.x());
    WriteLittleEndian<double>(&ofs, orientation.y());
    WriteLittleEndian<double>(&ofs, orientation.z());
    WriteLittleEndian<double>(&ofs, translation.x());
    WriteLittleEndian<double>(&ofs, translation.y());
    WriteLittleEndian<double>(&ofs, translation.z());
    WriteLittleEndian<uint32_t>(
        &ofs, reconstruction.CameraIntrinsicsGroupIdFromViewId(view_id));
    // Name is null-terminated.
    ofs.write(view->Name().c_str(), view->Name().size() + 1);

    const auto& track_ids = view->TrackIds();
    WriteLittleEndian<uint64_t>(&ofs, track_ids.size());
    for (auto track_id : track_ids) {
      const auto feature = view->GetFeature(track_id);
      WriteLittleEndian<double>(&ofs, feature->point_.x());
      WriteLittleEndian<double>(&ofs, feature->point_.y());
      WriteLittleEndian<int64_t>(&ofs, static_cast<int64_t>(track_id));
    }
  }
  ofs.close();
  return true;
}

bool WritePointsBinaryFile(const Reconstruction& reconstruction,
                           const std::string& points_file) {
  std::ofstream ofs(points_file, std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Cannot open the file: " << points_file << " for writing.";
    return false;
  }

  const auto& track_ids = reconstruction.TrackIds();
  WriteLittleEndian<uint64_t>(&ofs, track_ids.size());
  for (auto track_id : track_ids) {
    const Track* track = reconstruction.Track(track_id);
    const Eigen::Vector3d point = track->Point().hnormalized();
    const auto& color = track->Color();
    const auto& view_ids = track->ViewIds();

    // Mean reprojection error (pixels) over all observations of the track.
    double mean_error = 0.0;
    int num_observations = 0;
    for (auto view_id : view_ids) {
      const View* view = reconstruction.View(view_id);
      const auto feature = view->GetFeature(track_id);
      if (feature == nullptr) {
        continue;
      }
      Eigen::Vector2d projected;
      view->Camera().ProjectPoint(track->Point(), &projected);
      mean_error += (projected - feature->point_).norm();
      ++num_observations;
    }
    if (num_observations > 0) {
      mean_error /= num_observations;
    }

    WriteLittleEndian<uint64_t>(&ofs, track_id);
    WriteLittleEndian<double>(&ofs, point.x());
    WriteLittleEndian<double>(&ofs, point.y());
    WriteLittleEndian<double>(&ofs, point.z());
    WriteLittleEndian<uint8_t>(&ofs, static_cast<uint8_t>(color[0]));
    WriteLittleEndian<uint8_t>(&ofs, static_cast<uint8_t>(color[1]));
    WriteLittleEndian<uint8_t>(&ofs, static_cast<uint8_t>(color[2]));
    WriteLittleEndian<double>(&ofs, mean_error);
    WriteLittleEndian<uint64_t>(&ofs, view_ids.size());
    for (auto view_id : view_ids) {
      const View* view = reconstruction.View(view_id);
      const auto& view_track_ids = view->TrackIds();
      const uint32_t point2d_index = static_cast<uint32_t>(
          std::find(view_track_ids.begin(), view_track_ids.end(), track_id) -
          view_track_ids.begin());
      WriteLittleEndian<uint32_t>(&ofs, view_id);
      WriteLittleEndian<uint32_t>(&ofs, point2d_index);
    }
  }
  ofs.close();
  return true;
}

}  // namespace

bool WriteColmapFiles(const Reconstruction& reconstruction,
                      const std::string& output_directory,
                      const bool binary) {
  Reconstruction estimated_reconstruction;
  CreateEstimatedSubreconstruction(reconstruction, &estimated_reconstruction);

  if (binary) {
    const std::string cameras_file = output_directory + "/cameras.bin";
    const std::string images_file = output_directory + "/images.bin";
    const std::string points_file = output_directory + "/points3D.bin";
    return WriteCamerasBinaryFile(estimated_reconstruction, cameras_file) &&
           WriteImagesBinaryFile(estimated_reconstruction, images_file) &&
           WritePointsBinaryFile(estimated_reconstruction, points_file);
  }

  const std::string& cameras_file = output_directory + "/cameras.txt";
  const std::string& images_file = output_directory + "/images.txt";
  const std::string& points_file = output_directory + "/points3D.txt";

  if (!WriteCamerasFile(estimated_reconstruction, cameras_file)) {
    return false;
  }
  if (!WriteImagesFile(estimated_reconstruction, images_file)) {
    return false;
  }
  if (!WritePointsFile(estimated_reconstruction, points_file)) {
    return false;
  }
  return true;
}

}  // namespace theia
