// Copyright (C) 2026, Steffen Urban
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

#ifndef THEIA_IO_IMPORTED_RECONSTRUCTION_H_
#define THEIA_IO_IMPORTED_RECONSTRUCTION_H_

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "theia/sfm/camera/camera_intrinsics_model_type.h"
#include "theia/sfm/types.h"

namespace theia {

class Reconstruction;

// Messages of the reconstruction importers (ReadColmapFiles,
// ReadOpenMVGSfMData). error is set when an importer returns false, and
// file_not_found tells a missing input apart from a malformed one. warnings
// lists recoverable problems, e.g. skipped observations. Every message is also
// logged.
struct ImportMessages {
  std::string error;
  bool file_not_found = false;
  std::vector<std::string> warnings;
};

namespace internal {

// Camera intrinsics read from another tool, in the pixel convention of the
// returned reconstruction.
struct ImportedIntrinsics {
  CameraIntrinsicsModelType model_type = CameraIntrinsicsModelType::PINHOLE;
  int width = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  // Distortion of PINHOLE (k1, k2) and PINHOLE_RADIAL_TANGENTIAL (k1, k2, k3,
  // p1, p2).
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  // Distortion of the other models in COLMAP order, e.g. FISHEYE: k1..k4,
  // FOV: omega, FULL_OPENCV: k1, k2, p1, p2, k3, k4, k5, k6.
  std::vector<double> extra;
};

struct ImportedView {
  std::string name;
  CameraIntrinsicsGroupId group_id = 0;
  ImportedIntrinsics intrinsics;
  // World-to-camera rotation and camera center. Views without a pose are
  // added as unestimated.
  bool has_pose = false;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
};

struct ImportedTrack {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  bool has_color = false;
  Eigen::Matrix<uint8_t, 3, 1> color = Eigen::Matrix<uint8_t, 3, 1>::Zero();
  // (index into the view list, x, y).
  std::vector<std::tuple<int, double, double>> observations;
};

// Builds the reconstruction from the imported views and tracks. Views are
// added in order with their index as timestamp. A track keeps only the first
// observation per view. Returns false if a view cannot be added (duplicate or
// empty name) or a camera cannot be configured.
bool BuildImportedReconstruction(const std::vector<ImportedView>& views,
                                 const std::vector<ImportedTrack>& tracks,
                                 Reconstruction* reconstruction,
                                 ImportMessages* messages);

// Records an error (returns false) or a warning in messages, which may be
// nullptr, and logs it.
bool ImportError(const std::string& error, ImportMessages* messages,
                 bool file_not_found = false);
void ImportWarning(const std::string& warning, ImportMessages* messages);

}  // namespace internal
}  // namespace theia

#endif  // THEIA_IO_IMPORTED_RECONSTRUCTION_H_
