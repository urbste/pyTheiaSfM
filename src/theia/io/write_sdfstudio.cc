
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

#include "theia/io/write_sdfstudio.h"

#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <string>
#include <vector>
#include <cereal/external/rapidjson/rapidjson.h>
#include <cereal/external/rapidjson/prettywriter.h>
#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/stringbuffer.h>
#include <Eigen/Core>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/util/filesystem.h"

using namespace cereal;

namespace theia {

namespace {

void AddDoubleKey(
  const std::string& key, const double value, 
  rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
  writer.Key(key.c_str());
  writer.Double(value);
}

void AddIntKey(
  const std::string& key, const int value, 
  rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
  writer.Key(key.c_str());
  writer.Int(value);
}

void AddStrKey(
  const std::string& key, const std::string value, 
  rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
  writer.Key(key.c_str());
  writer.String(value.c_str());
}

void AddMat4Key(
  const std::string& key,
  const Eigen::Matrix4d& matrix, 
  rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
   writer.Key(key.c_str());
   writer.StartArray();
   // create transform matrix
   for (int r = 0; r < 4; ++r) {
   writer.StartArray();
   for (int i = 0; i < 4; ++i) {
      writer.Double(matrix(r,i));
    }
    writer.EndArray();
  }
  writer.EndArray();
}


void WriteAllSfmPoints(
    const Reconstruction& recon,
    const std::string& out_path) {  
  std::ofstream file;
  file.open(out_path);
  for (const auto& track_id : recon.TrackIds()) {
    const auto track = recon.Track(track_id);
    const auto& point = track->Point().hnormalized();
    file << point(0) << " " << point(1) << " " << point(2) << "\n";
  }
  file.close();
}

void WriteViewSfmPoints(const Reconstruction& recon,
                    const ViewId& view_id,
                    const std::string& out_path) {  
  std::ofstream file;
  file.open(out_path);
  for (const auto& track_id : recon.View(view_id)->TrackIds()) {
    const auto track = recon.Track(track_id);
    const auto& point = track->Point().hnormalized();
    file << point(0) << " " << point(1) << " " << point(2) << "\n";
  }
  file.close();
}

} // namespace

bool WriteSdfStudio(const std::string& path_to_images,
                  const Reconstruction& reconstruction,
                  const std::pair<double, double>& nearfar,
                  const double radius) {
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

  AddStrKey("camera_model", "OPENCV", writer);
  const auto cam = reconstruction.View(0)->Camera();
  AddIntKey("height", cam.ImageHeight(), writer);
  AddIntKey("width", cam.ImageWidth(), writer);

  const std::string sparse_sfm_points_path = 
    JoinPath(path_to_images, "sparse_sfm_points.txt");
  WriteAllSfmPoints(reconstruction, sparse_sfm_points_path);
  AddStrKey("sparse_sfm_points", "sparse_sfm_points.txt", writer);
  AddStrKey("has_mono_prior", "false", writer);
  AddStrKey("has_foreground_mask", "false", writer);
  AddStrKey("has_sparse_sfm_points", "true", writer);

  Eigen::Matrix4d worldtogt = Eigen::Matrix4d::Identity();
  AddMat4Key("worldtogt", worldtogt, writer);

  writer.Key("scene_box");
  writer.StartObject();
  writer.Key("aabb");
  writer.StartArray();

  writer.StartArray();
  writer.Int(-1);
  writer.Int(-1);
  writer.Int(-1);
  writer.EndArray();

  writer.StartArray();
  writer.Int(1);
  writer.Int(1);
  writer.Int(1);
  writer.EndArray();

  writer.EndArray();
  AddDoubleKey("near", nearfar.first, writer);
  AddDoubleKey("far", nearfar.second, writer);
  AddDoubleKey("radius", radius, writer);
  AddStrKey("collider_type", "near_far", writer);
  writer.EndObject();

  writer.Key("frames");
  writer.StartArray();
  for (const auto& vid : reconstruction.ViewIds()) {
      const auto view = reconstruction.View(vid);
      if (!view->IsEstimated()) {
          continue;
      }
    
      std::string sfm_pt_name = view->Name() + "_sfm_points.txt";
      std::string out_sfm_points_path = 
        JoinPath(path_to_images, sfm_pt_name);
      WriteViewSfmPoints(reconstruction, vid, out_sfm_points_path);

      writer.StartObject();
      
      AddStrKey("rgb_path", view->Name().c_str(), writer);
      AddStrKey("sfm_sparse_points_view", sfm_pt_name, writer);

      // name should be filename, e.g. image001.png

      const auto cam = view->Camera();
      Eigen::Matrix4d c2w = Eigen::Matrix4d::Identity();
      c2w.block<3,3>(0,0) = cam.GetOrientationAsRotationMatrix().transpose();
      c2w.block<3,1>(0,3) = view->Camera().GetPosition();

      AddMat4Key("camtoworld", c2w, writer);

      Eigen::Matrix4d K = Eigen::Matrix4d::Identity();
      K(0,0) = cam.FocalLength();
      K(1,1) = cam.FocalLength();
      K(0,2) = cam.PrincipalPointX();
      K(1,2) = cam.PrincipalPointY();
  
      AddMat4Key("intrinsics", K, writer);

      writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();

  // Return false if the file cannot be opened for writing.
  const std::string out_json = JoinPath(path_to_images, "meta_data.json");
  std::ofstream json_writer(out_json, std::ofstream::out);
  if (!json_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << out_json
               << " for writing a nerfstudio file.";
    return false;
  }
  json_writer << strbuf.GetString();
  json_writer.close();
  return true;
}

}  // namespace theia
