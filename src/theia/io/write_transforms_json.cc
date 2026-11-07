// Copyright (C) 2025, Steffen Urban
// All rights reserved.
//
// See write_transforms_json.h for the license and the output convention.

#include "theia/io/write_transforms_json.h"

#include <Eigen/Core>
#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/prettywriter.h>
#include <cereal/external/rapidjson/rapidjson.h>
#include <cereal/external/rapidjson/stringbuffer.h>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <string>
#include <vector>

#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/reconstruction.h"
#include "theia/util/filesystem.h"

using namespace cereal;

namespace theia {
namespace {

void AddDouble(const std::string& key,
               const double value,
               rapidjson::PrettyWriter<rapidjson::StringBuffer>* writer) {
  writer->Key(key.c_str());
  writer->Double(value);
}

void AddInt(const std::string& key,
            const int value,
            rapidjson::PrettyWriter<rapidjson::StringBuffer>* writer) {
  writer->Key(key.c_str());
  writer->Int(value);
}

// Intrinsics of one camera in the form written to transforms.json.
struct TransformsIntrinsics {
  int width;
  int height;
  double fl_x;
  double fl_y;
  double cx;
  double cy;
  double k1;
  double k2;
  bool has_distortion;
};

bool GetIntrinsics(const Camera& camera, TransformsIntrinsics* intrinsics) {
  if (camera.GetCameraIntrinsicsModelType() !=
      CameraIntrinsicsModelType::PINHOLE) {
    LOG(ERROR) << "WriteTransformsJson only supports PINHOLE cameras. Found "
               << "an unsupported camera model.";
    return false;
  }

  Eigen::Matrix3d K;
  camera.GetCalibrationMatrix(&K);
  const CameraIntrinsicsModel& model = *camera.CameraIntrinsics();
  intrinsics->width = camera.ImageWidth();
  intrinsics->height = camera.ImageHeight();
  intrinsics->fl_x = K(0, 0);
  intrinsics->fl_y = K(1, 1);
  intrinsics->cx = K(0, 2);
  intrinsics->cy = K(1, 2);
  intrinsics->k1 =
      model.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_1);
  intrinsics->k2 =
      model.GetParameter(PinholeCameraModel::RADIAL_DISTORTION_2);
  intrinsics->has_distortion = (intrinsics->k1 != 0.0) ||
                               (intrinsics->k2 != 0.0);
  if (K(0, 1) != 0.0) {
    LOG(WARNING) << "Skew is not representable in transforms.json and is "
                    "ignored.";
  }
  return true;
}

void WriteIntrinsics(const TransformsIntrinsics& intrinsics,
                     rapidjson::PrettyWriter<rapidjson::StringBuffer>* writer) {
  AddInt("w", intrinsics.width, writer);
  AddInt("h", intrinsics.height, writer);
  AddDouble("fl_x", intrinsics.fl_x, writer);
  AddDouble("fl_y", intrinsics.fl_y, writer);
  AddDouble("cx", intrinsics.cx, writer);
  AddDouble("cy", intrinsics.cy, writer);
  AddDouble("k1", intrinsics.k1, writer);
  AddDouble("k2", intrinsics.k2, writer);
  AddDouble("p1", 0.0, writer);
  AddDouble("p2", 0.0, writer);
}

}  // namespace

bool WriteTransformsJson(const std::string& path_to_images,
                         const Reconstruction& reconstruction,
                         const std::string& out_json_file) {
  CHECK_GT(out_json_file.length(), 0);

  std::vector<ViewId> view_ids;
  for (const ViewId view_id : reconstruction.ViewIds()) {
    if (reconstruction.View(view_id)->IsEstimated()) {
      view_ids.push_back(view_id);
    }
  }
  if (view_ids.empty()) {
    LOG(ERROR) << "Reconstruction has no estimated views. Nothing to write.";
    return false;
  }

  rapidjson::StringBuffer strbuf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
  writer.StartObject();

  // Top-level intrinsics when there is a single intrinsics group.
  if (reconstruction.NumCameraIntrinsicGroups() == 1) {
    TransformsIntrinsics intrinsics;
    const Camera& camera = reconstruction.View(view_ids.front())->Camera();
    if (!GetIntrinsics(camera, &intrinsics)) {
      return false;
    }
    writer.Key("camera_model");
    writer.String(intrinsics.has_distortion ? "OPENCV" : "PINHOLE");
    WriteIntrinsics(intrinsics, &writer);
  }

  writer.Key("frames");
  writer.StartArray();
  for (const ViewId view_id : view_ids) {
    const View* view = reconstruction.View(view_id);
    const Camera& camera = view->Camera();

    TransformsIntrinsics intrinsics;
    if (!GetIntrinsics(camera, &intrinsics)) {
      return false;
    }

    // OpenCV camera-to-world: R_c2w = R^T, C = camera center.
    Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
    c2w.block<3, 3>(0, 0) = camera.GetOrientationAsRotationMatrix().transpose();
    c2w.block<3, 1>(0, 3) = camera.GetPosition();
    // OpenCV -> OpenGL camera axes: flip y and z.
    c2w.block<3, 2>(0, 1) *= -1.0;

    writer.StartObject();
    writer.Key("file_path");
    const std::string file_path = JoinPath(path_to_images, view->Name());
    writer.String(file_path.c_str());
    writer.Key("transform_matrix");
    writer.StartArray();
    for (int r = 0; r < 4; ++r) {
      writer.StartArray();
      for (int c = 0; c < 4; ++c) {
        writer.Double(c2w(r, c));
      }
      writer.EndArray();
    }
    writer.EndArray();

    writer.Key("camera_model");
    writer.String(intrinsics.has_distortion ? "OPENCV" : "PINHOLE");
    WriteIntrinsics(intrinsics, &writer);
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();

  std::ofstream json_writer(out_json_file, std::ofstream::out);
  if (!json_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << out_json_file
               << " for writing a transforms.json file.";
    return false;
  }
  json_writer << strbuf.GetString();
  json_writer.close();
  return true;
}

}  // namespace theia
