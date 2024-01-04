// Copyright (C) 2023, Steffen Urban
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

#include "theia/io/write_nerfstudio.h"

#include <Eigen/Core>
#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/prettywriter.h>
#include <cereal/external/rapidjson/rapidjson.h>
#include <cereal/external/rapidjson/stringbuffer.h>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <string>
#include <vector>

#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/reconstruction.h"
#include "theia/util/filesystem.h"

using namespace cereal;

namespace theia {

void AddDoubleKey(const std::string& key,
                  const double value,
                  rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
  writer.Key(key.c_str());
  writer.Double(value);
}

void AddIntKey(const std::string& key,
               const int value,
               rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
  writer.Key(key.c_str());
  writer.Int(value);
}

bool WriteNerfStudio(const std::string& path_to_images,
                     const Reconstruction& reconstruction,
                     const int aabb_scale,
                     const std::string& out_json_nerfstudio_file) {
  CHECK_GT(out_json_nerfstudio_file.length(), 0);
  CHECK_GT(path_to_images.length(), 0);

  rapidjson::Value frames_array(rapidjson::kObjectType);

  const auto view_ids = reconstruction.ViewIds();
  if (view_ids.size() <= 0) {
    LOG(INFO) << "Reconstruction seems to be empty.";
    return false;
  }

  rapidjson::StringBuffer strbuf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);

  writer.StartObject();

  writer.Key("frames");
  writer.StartArray();
  for (const auto& vid : reconstruction.ViewIds()) {
    const auto view = reconstruction.View(vid);
    if (!view->IsEstimated()) {
      continue;
    }
    writer.StartObject();

    writer.Key("file_path");
    const std::string img_path = JoinPath(path_to_images, view->Name());
    writer.String(img_path.c_str());

    rapidjson::Value file_path;
    // name should be filename, e.g. image001.png

    // https://github.com/nerfstudio-project/nerfstudio/blob/6486c00d4d717ee437198d08930a86f5cced5359/nerfstudio/process_data/colmap_utils.py#L422
    // w2c = np.concatenate([rotation, translation], 1)
    // w2c = np.concatenate([w2c, np.array([[0, 0, 0, 1]])], 0)
    // c2w = np.linalg.inv(w2c)
    // # Convert from COLMAP's camera coordinate system (OpenCV) to ours
    // (OpenGL) c2w[0:3, 1:3] *= -1 c2w = c2w[np.array([1, 0, 2, 3]), :] c2w[2,
    // :] *= -1
    const auto cam = view->Camera();
    Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
    c2w.block<3, 3>(0, 0) = cam.GetOrientationAsRotationMatrix().transpose();
    c2w.block<3, 1>(0, 3) = view->Camera().GetPosition();
    c2w.block<3, 2>(0, 1) *= -1;

    Eigen::Matrix4d c2w_ns = Eigen::Matrix4d::Identity();
    c2w_ns.block<1, 4>(0, 0) = c2w.row(1);
    c2w_ns.block<1, 4>(1, 0) = c2w.row(0);
    c2w_ns.block<1, 4>(2, 0) = c2w.row(2);
    c2w_ns.block<1, 4>(3, 0) = c2w.row(3);
    c2w_ns.block<1, 4>(2, 0) *= -1;

    writer.Key("transform_matrix");
    writer.StartArray();
    // create transform matrix
    for (int r = 0; r < 4; ++r) {
      writer.StartArray();
      for (int c = 0; c < 4; ++c) {
        writer.Double(c2w_ns(r, c));
      }
      writer.EndArray();
    }
    writer.EndArray();

    // now get the intrinsics
    Eigen::Matrix3d K;
    cam.GetCalibrationMatrix(&K);
    AddDoubleKey("fl_x", K(0, 0), writer);
    AddDoubleKey("fl_y", K(1, 1), writer);
    AddDoubleKey("cx", K(0, 2), writer);
    AddDoubleKey("cy", K(1, 2), writer);

    AddIntKey("w", cam.ImageWidth(), writer);
    AddIntKey("h", cam.ImageHeight(), writer);

    const CameraIntrinsicsModel& intrinsics = *cam.CameraIntrinsics();
    if (cam.GetCameraIntrinsicsModelType() ==
        theia::CameraIntrinsicsModelType::PINHOLE) {
      AddDoubleKey("k1", intrinsics.GetParameter(
                  theia::PinholeCameraModel::RADIAL_DISTORTION_1),
                   writer);
      AddDoubleKey("k2", intrinsics.GetParameter(
                  theia::PinholeCameraModel::RADIAL_DISTORTION_2),
                   writer);
      AddDoubleKey("p1", 0.0, writer);
      AddDoubleKey("p2", 0.0, writer);
    } else if (cam.GetCameraIntrinsicsModelType() ==
               theia::CameraIntrinsicsModelType::FISHEYE) {
      AddDoubleKey("k1", intrinsics.GetParameter(
                  theia::FisheyeCameraModel::RADIAL_DISTORTION_1),
                   writer);
      AddDoubleKey("k2", intrinsics.GetParameter(
                  theia::FisheyeCameraModel::RADIAL_DISTORTION_2),
                   writer);
      AddDoubleKey("k3", intrinsics.GetParameter(
                  theia::FisheyeCameraModel::RADIAL_DISTORTION_3),
                   writer);
      AddDoubleKey("k4", intrinsics.GetParameter(
                  theia::FisheyeCameraModel::RADIAL_DISTORTION_4),
                   writer);
    } else {
      LOG(ERROR) << "Camera Model not supported. Currently only PINHOLE and "
                    "FISHEYE are supported models.";
      return false;
      exit(-1);
    }
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();

  // Return false if the file cannot be opened for writing.
  std::ofstream json_writer(out_json_nerfstudio_file, std::ofstream::out);
  if (!json_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << out_json_nerfstudio_file
               << " for writing a nerfstudio file.";
    return false;
  }
  json_writer << strbuf.GetString();
  json_writer.close();
  return true;
}

}  // namespace theia
