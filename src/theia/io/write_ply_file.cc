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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/io/write_ply_file.h"

#include <algorithm>
#include <fstream>  // NOLINT
#include <glog/logging.h>
#include <string>
#include <vector>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/view.h"

namespace theia {

// Gather points from tracks.
void GatherTracks(const Reconstruction& reconstruction,
                  std::vector<Eigen::Vector3d>* points_to_write,
                  std::vector<Eigen::Vector3i>* colors_to_write) {
  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track& track = *reconstruction.Track(track_id);
    points_to_write->emplace_back(track.Point().hnormalized());
    colors_to_write->emplace_back(
        track.Color()[0], track.Color()[1], track.Color()[2]);
  }
}

// Gather camera positions.
void GatherCameras(const Reconstruction& reconstruction,
                   const Eigen::Vector3i& camera_color,
                   std::vector<Eigen::Vector3d>* points_to_write,
                   std::vector<Eigen::Vector3i>* colors_to_write) {
  for (const ViewId view_id : reconstruction.ViewIds()) {
    const View& view = *reconstruction.View(view_id);
    if (!view.IsEstimated()) {
      continue;
    }
    points_to_write->emplace_back(view.Camera().GetPosition());
    colors_to_write->emplace_back(camera_color);
  }
}

// Writes a PLY file for viewing in software such as MeshLab.
bool WritePlyFile(const std::string& ply_file,
                  const Reconstruction& const_reconstruction,
                  const Eigen::Vector3i& camera_color,
                  const int min_num_observations_per_point) {
  CHECK_GT(ply_file.length(), 0);

  // Return false if the file cannot be opened for writing.
  std::ofstream ply_writer(ply_file, std::ofstream::out);
  if (!ply_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << ply_file
               << " for writing a PLY file.";
    return false;
  }

  // First, remove any points that are unestimated or do not have enough 3D
  // points.
  Reconstruction reconstruction = const_reconstruction;
  const auto& track_ids = reconstruction.TrackIds();
  for (const TrackId track_id : track_ids) {
    const Track& track = *reconstruction.Track(track_id);
    if (!track.IsEstimated() ||
        track.NumViews() < min_num_observations_per_point) {
      reconstruction.RemoveTrack(track_id);
    }
  }

  // Extract points that we will write to the PLY file.
  std::vector<Eigen::Vector3d> points_to_write;
  std::vector<Eigen::Vector3i> colors_to_write;
  GatherTracks(reconstruction, &points_to_write, &colors_to_write);
  GatherCameras(
      reconstruction, camera_color, &points_to_write, &colors_to_write);

  ply_writer << "ply" << '\n'
             << "format ascii 1.0" << '\n'
             << "element vertex " << points_to_write.size() << '\n'
             << "property float x" << '\n'
             << "property float y" << '\n'
             << "property float z" << '\n'
             << "property uchar red" << '\n'
             << "property uchar green" << '\n'
             << "property uchar blue" << '\n'
             << "end_header" << std::endl;

  for (int i = 0; i < points_to_write.size(); i++) {
    ply_writer << points_to_write[i].transpose() << " "
               << colors_to_write[i].transpose() << "\n";
  }

  return true;
}

namespace {

void AppendSegmentSamples(const Eigen::Vector3d& a,
                          const Eigen::Vector3d& b,
                          const Eigen::Vector3i& color,
                          const int edge_samples,
                          std::vector<Eigen::Vector3d>* points,
                          std::vector<Eigen::Vector3i>* colors) {
  const int samples = std::max(1, edge_samples);
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    points->emplace_back((1.0 - t) * a + t * b);
    colors->emplace_back(color);
  }
}

}  // namespace

bool WriteRigPlyFile(const std::string& ply_file,
                     const Reconstruction& const_reconstruction,
                     const Eigen::Vector3i& sensor_color,
                     const int min_num_observations_per_point,
                     const Eigen::Vector3i& capture_color,
                     const Eigen::Vector3i& trajectory_color,
                     const Eigen::Vector3i& baseline_color,
                     const int edge_samples) {
  CHECK_GT(ply_file.length(), 0);

  std::ofstream ply_writer(ply_file, std::ofstream::out);
  if (!ply_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << ply_file
               << " for writing a PLY file.";
    return false;
  }

  Reconstruction reconstruction = const_reconstruction;
  const auto& track_ids = reconstruction.TrackIds();
  for (const TrackId track_id : track_ids) {
    const Track& track = *reconstruction.Track(track_id);
    if (!track.IsEstimated() ||
        track.NumViews() < min_num_observations_per_point) {
      reconstruction.RemoveTrack(track_id);
    }
  }

  std::vector<Eigen::Vector3d> points_to_write;
  std::vector<Eigen::Vector3i> colors_to_write;
  GatherTracks(reconstruction, &points_to_write, &colors_to_write);

  // Estimated captures sorted by timestamp (stable trajectory order).
  std::vector<CaptureId> capture_ids = reconstruction.CaptureIds();
  std::sort(capture_ids.begin(),
            capture_ids.end(),
            [&](const CaptureId a, const CaptureId b) {
              const RigCapture* ca = reconstruction.GetRigCapture(a);
              const RigCapture* cb = reconstruction.GetRigCapture(b);
              if (ca == nullptr || cb == nullptr) {
                return a < b;
              }
              if (ca->GetTimestamp() == cb->GetTimestamp()) {
                return a < b;
              }
              return ca->GetTimestamp() < cb->GetTimestamp();
            });

  std::vector<Eigen::Vector3d> capture_centers;
  capture_centers.reserve(capture_ids.size());
  std::vector<std::vector<Eigen::Vector3d>> sensors_per_capture;
  sensors_per_capture.reserve(capture_ids.size());

  for (const CaptureId capture_id : capture_ids) {
    const RigCapture* capture = reconstruction.GetRigCapture(capture_id);
    if (capture == nullptr || !capture->IsEstimated()) {
      continue;
    }

    const Eigen::Vector3d body = capture->GetPosition();
    capture_centers.push_back(body);
    points_to_write.push_back(body);
    colors_to_write.push_back(capture_color);

    std::vector<Eigen::Vector3d> sensor_centers;
    for (const ViewId view_id : capture->GetViewIds()) {
      const View* view = reconstruction.View(view_id);
      if (view == nullptr || !view->IsEstimated()) {
        continue;
      }
      const Eigen::Vector3d cam_c = view->Camera().GetPosition();
      sensor_centers.push_back(cam_c);
      points_to_write.push_back(cam_c);
      colors_to_write.push_back(sensor_color);
    }
    sensors_per_capture.push_back(sensor_centers);
  }

  // Trajectory polyline: consecutive capture body centers.
  for (size_t i = 1; i < capture_centers.size(); ++i) {
    AppendSegmentSamples(capture_centers[i - 1],
                         capture_centers[i],
                         trajectory_color,
                         edge_samples,
                         &points_to_write,
                         &colors_to_write);
  }

  // Intra-capture baselines: connect every pair of sensors in a capture
  // (stereo = one segment; multi-camera = full clique for visibility).
  for (const auto& sensors : sensors_per_capture) {
    for (size_t i = 0; i < sensors.size(); ++i) {
      for (size_t j = i + 1; j < sensors.size(); ++j) {
        AppendSegmentSamples(sensors[i],
                             sensors[j],
                             baseline_color,
                             edge_samples,
                             &points_to_write,
                             &colors_to_write);
      }
    }
  }

  LOG(INFO) << "Writing rig PLY with " << points_to_write.size()
            << " vertices (" << capture_centers.size()
            << " captures).";

  ply_writer << "ply" << '\n'
             << "format ascii 1.0" << '\n'
             << "element vertex " << points_to_write.size() << '\n'
             << "property float x" << '\n'
             << "property float y" << '\n'
             << "property float z" << '\n'
             << "property uchar red" << '\n'
             << "property uchar green" << '\n'
             << "property uchar blue" << '\n'
             << "end_header" << std::endl;

  for (size_t i = 0; i < points_to_write.size(); ++i) {
    ply_writer << points_to_write[i].transpose() << " "
               << colors_to_write[i].transpose() << "\n";
  }

  return true;
}

}  // namespace theia
