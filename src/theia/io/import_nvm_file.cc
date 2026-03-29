// Copyright (C) 2016 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/io/import_nvm_file.h"

#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"
#include "theia/util/filesystem.h"
#include "theia/util/map_util.h"

namespace theia {
namespace {

// Parses NVM_V3 text models (VisualSfM-style layout) without third-party deps.
// Supports quaternion cameras and NVM_V3_R9T 3x3 rotation lines.
bool ReadNvmModel(
    std::istream* in,
    std::vector<std::string>* names,
    std::vector<double>* focal,
    std::vector<Eigen::Matrix3d>* world_to_cam,
    std::vector<Eigen::Vector3d>* camera_center,
    std::vector<Eigen::Vector3d>* points_xyz,
    std::vector<Eigen::Vector3i>* point_rgb,
    std::vector<int>* camidx,
    std::vector<int>* ptidx,
    std::vector<Eigen::Vector2d>* measurements) {
  int rotation_parameter_num = 4;
  if (in->peek() == static_cast<int>('N')) {
    std::string header_token;
    *in >> header_token;
    if (header_token.find("R9T") != std::string::npos) {
      rotation_parameter_num = 9;
    }
    std::string dummy;
    std::getline(*in, dummy);
  }

  int ncam = 0;
  *in >> ncam;
  if (ncam <= 1) {
    LOG(ERROR) << "NVM import failed: need at least 2 cameras, got " << ncam;
    return false;
  }

  names->resize(static_cast<size_t>(ncam));
  focal->resize(static_cast<size_t>(ncam));
  world_to_cam->resize(static_cast<size_t>(ncam));
  camera_center->resize(static_cast<size_t>(ncam));

  for (int i = 0; i < ncam; ++i) {
    std::string image_token;
    double f = 0;
    double q[9] = {};
    double c[3] = {};
    double d[2] = {};
    *in >> image_token >> f;
    for (int j = 0; j < rotation_parameter_num; ++j) {
      *in >> q[j];
    }
    *in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];
    (void)d;

    (*names)[static_cast<size_t>(i)] = image_token;
    (*focal)[static_cast<size_t>(i)] = f;

    if (rotation_parameter_num == 9) {
      Eigen::Matrix3d R;
      for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
          R(r, col) = q[3 * r + col];
        }
      }
      const Eigen::Vector3d T(c[0], c[1], c[2]);
      const Eigen::Vector3d C = -(R.transpose() * T);
      (*world_to_cam)[static_cast<size_t>(i)] = R;
      (*camera_center)[static_cast<size_t>(i)] = C;
    } else {
      Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
      quat.normalize();
      const Eigen::Matrix3d R = quat.toRotationMatrix();
      const Eigen::Vector3d C(c[0], c[1], c[2]);
      (*world_to_cam)[static_cast<size_t>(i)] = R;
      (*camera_center)[static_cast<size_t>(i)] = C;
    }
  }

  int npoint = 0;
  *in >> npoint;
  if (npoint <= 0) {
    LOG(ERROR) << "NVM import failed: no 3D points.";
    return false;
  }

  points_xyz->resize(static_cast<size_t>(npoint));
  point_rgb->resize(static_cast<size_t>(npoint));

  for (int i = 0; i < npoint; ++i) {
    float pt[3];
    int cc[3];
    int npj = 0;
    *in >> pt[0] >> pt[1] >> pt[2] >> cc[0] >> cc[1] >> cc[2] >> npj;
    (*points_xyz)[static_cast<size_t>(i)] =
        Eigen::Vector3d(pt[0], pt[1], pt[2]);
    (*point_rgb)[static_cast<size_t>(i)] = Eigen::Vector3i(cc[0], cc[1], cc[2]);
    for (int j = 0; j < npj; ++j) {
      int cidx = 0;
      int fidx = 0;
      float imx = 0.f;
      float imy = 0.f;
      *in >> cidx >> fidx >> imx >> imy;
      (void)fidx;
      if (cidx < 0 || cidx >= ncam) {
        LOG(ERROR) << "Invalid camera index " << cidx << " in NVM observations.";
        return false;
      }
      camidx->push_back(cidx);
      ptidx->push_back(i);
      measurements->emplace_back(static_cast<double>(imx),
                                 static_cast<double>(imy));
    }
  }

  return !in->fail();
}

}  // namespace

bool ImportNVMFile(const std::string& nvm_filepath,
                   Reconstruction* reconstruction) {
  CHECK_GT(nvm_filepath.length(), 0);
  CHECK_NOTNULL(reconstruction);

  if (nvm_filepath.find(".nvm") == std::string::npos) {
    LOG(ERROR) << "ImportNVMFile expected a filepath containing '.nvm'.";
    return false;
  }

  std::ifstream file(nvm_filepath.c_str());
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open NVM file: " << nvm_filepath;
    return false;
  }

  std::vector<std::string> names;
  std::vector<double> focals;
  std::vector<Eigen::Matrix3d> world_to_cam;
  std::vector<Eigen::Vector3d> centers;
  std::vector<Eigen::Vector3d> points_xyz;
  std::vector<Eigen::Vector3i> point_rgb;
  std::vector<int> camidx;
  std::vector<int> ptidx;
  std::vector<Eigen::Vector2d> measurements;

  if (!ReadNvmModel(&file, &names, &focals, &world_to_cam, &centers,
                    &points_xyz, &point_rgb, &camidx, &ptidx, &measurements)) {
    LOG(ERROR) << "Failed to parse NVM file: " << nvm_filepath;
    return false;
  }

  CHECK_EQ(names.size(), world_to_cam.size());
  CHECK_EQ(names.size(), centers.size());
  CHECK_EQ(measurements.size(), ptidx.size());
  CHECK_EQ(measurements.size(), camidx.size());

  std::vector<ViewId> nvm_index_to_view_id(names.size());
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    std::string view_name;
    CHECK(GetFilenameFromFilepath(names[static_cast<size_t>(i)], true,
                                  &view_name));
    LOG(INFO) << "Adding view " << view_name << " to the reconstruction.";
    const ViewId view_id =
        reconstruction->AddView(view_name, static_cast<double>(i));
    CHECK_NE(view_id, kInvalidViewId);
    nvm_index_to_view_id[static_cast<size_t>(i)] = view_id;
    View* view = reconstruction->MutableView(view_id);
    view->SetEstimated(true);

    const Eigen::Matrix3d& R = world_to_cam[static_cast<size_t>(i)];
    Camera* camera = view->MutableCamera();
    camera->SetCameraIntrinsicsModelType(CameraIntrinsicsModelType::PINHOLE);
    camera->SetFocalLength(focals[static_cast<size_t>(i)]);
    // Match legacy VisualSfM bridge: world-to-camera R from NVM, Theia stores
    // rotation consistent with transpose used previously.
    camera->SetOrientationFromRotationMatrix(R.transpose());
    camera->SetPosition(centers[static_cast<size_t>(i)]);
  }

  std::unordered_map<int, std::vector<std::pair<ViewId, Feature>>> tracks;
  for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
    const Feature feature(measurements[static_cast<size_t>(i)]);
    const ViewId view_id =
        nvm_index_to_view_id[static_cast<size_t>(camidx[static_cast<size_t>(i)])];
    tracks[ptidx[static_cast<size_t>(i)]].emplace_back(view_id, feature);
  }

  for (int i = 0; i < static_cast<int>(points_xyz.size()); ++i) {
    const Eigen::Vector3d& p = points_xyz[static_cast<size_t>(i)];
    Eigen::Vector4d point(p.x(), p.y(), p.z(), 1.0);
    Eigen::Vector3i color = point_rgb[static_cast<size_t>(i)];
    const auto& features = FindOrDie(tracks, i);
    const TrackId track_id = reconstruction->AddTrack(features);
    CHECK_NE(track_id, kInvalidTrackId);
    Track* track = reconstruction->MutableTrack(track_id);
    track->SetEstimated(true);
    *track->MutablePoint() = point;
    *track->MutableColor() = color.cast<uint8_t>();
  }

  return true;
}

}  // namespace theia
