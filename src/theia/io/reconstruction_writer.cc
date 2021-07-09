// Copyright (C) 2014 The Regents of the University of California (Regents).
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

#include "theia/io/reconstruction_writer.h"

#include <cereal/archives/portable_binary.hpp>
#include <Eigen/Core>
#include <glog/logging.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>   // NOLINT
#include <iostream>  // NOLINT
#include <string>

#include "theia/sfm/reconstruction_estimator_utils.h"
#include "theia/sfm/reconstruction.h"

#include "theia/util/json.h"

namespace theia {

bool WriteReconstruction(const Reconstruction& reconstruction,
                         const std::string& output_file) {
  std::ofstream output_writer(output_file, std::ios::out | std::ios::binary);
  if (!output_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << output_file << " for writing.";
    return false;
  }

  Reconstruction estimated_reconstruction;
  CreateEstimatedSubreconstruction(reconstruction, &estimated_reconstruction);

  // Make sure that Cereal is able to finish executing before returning.
  {
    cereal::PortableBinaryOutputArchive output_archive(output_writer);
    output_archive(estimated_reconstruction);
  }

  return true;
}

bool WriteReconstructionJson(const Reconstruction& reconstruction,
                             const std::string& output_json_file) {


  nlohmann::json calib_out_json;

  nlohmann::json views_json;
  // iterate views 
  const auto view_ids = reconstruction.ViewIds();
  for (const theia::ViewId view_id : view_ids) {
    const theia::View &view = *reconstruction.View(view_id);
    if (!view.IsEstimated()) {
      continue;
    }
    const theia::Camera &camera = view.Camera();

    nlohmann::json current_view;
    current_view["timestamp"] = view.GetTimestamp();
    current_view["numFeatures"] = view.NumFeatures();
    current_view["trackIds"] = view.TrackIds();
    // current_view["covariance"] = 
    // save features and tracks ids (view graph)

    // we save camera to world transformation so transpose rotation
    nlohmann::json extrinsics;
    const Eigen::Matrix3d rotation(
        camera.GetOrientationAsRotationMatrix().transpose());
    const Eigen::Vector3d position(camera.GetPosition());
    extrinsics[0][0] = rotation(0, 0);
    extrinsics[0][1] = rotation(0, 1);
    extrinsics[0][2] = rotation(0, 2);
    extrinsics[1][0] = rotation(1, 0);
    extrinsics[1][1] = rotation(1, 1);
    extrinsics[1][2] = rotation(1, 2);
    extrinsics[2][0] = rotation(2, 0);
    extrinsics[2][1] = rotation(2, 1);
    extrinsics[2][2] = rotation(2, 2);

    extrinsics[0][3] = position(0);
    extrinsics[1][3] = position(1);
    extrinsics[2][3] = position(2);
    extrinsics[3] = {0.0, 0.0, 0.0, 1.0};

    current_view["cameraExtrinsics"] = extrinsics;
  
    // check camera model and write parameters depending on model
    // nlohmann::json intrinsics;
    // intrinsics["focalLengthX"] = camera.FocalLength();
    // intrinsics["focalLengthY"] = camera.CameraIntrinsics()->GetParameter(
    //                                  theia::PinholeCameraModel::ASPECT_RATIO) *
    //                              camera.FocalLength();
    // intrinsics["principalPointX"] = camera.CameraIntrinsics()->GetParameter(
    //     theia::PinholeCameraModel::PRINCIPAL_POINT_X);
    // intrinsics["principalPointY"] = camera.CameraIntrinsics()->GetParameter(
    //     theia::PinholeCameraModel::PRINCIPAL_POINT_Y);

    // current_view["cameraIntrinsics"] = intrinsics;

    views_json[view.Name()] = current_view;
  }
  calib_out_json["views"] = views_json;

  nlohmann::json tracks_json;
  const auto track_ids = reconstruction.TrackIds();
  for (const auto& t_id : track_ids) {
    const theia::Track* track = reconstruction.Track(t_id);
    if (!track->IsEstimated()) {
      continue;
    }
    nlohmann::json current_track;
    current_track["referenceViewId"] = track->ReferenceViewId();
    const Eigen::Vector3d pt = track->Point().hnormalized();
    current_track["point"] = {pt[0],pt[1],pt[2]};
    //current_track["covariance"] =
    tracks_json[std::to_string(t_id)] = current_track;
  }
  calib_out_json["tracks"] = tracks_json;

  std::ofstream out_stream(output_json_file);
  CHECK(out_stream.is_open())
      << "Could not open '" + output_json_file + "' for writing.";
  out_stream << calib_out_json;

  return true;
}

}  // namespace theia
