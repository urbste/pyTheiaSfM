// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_OSE_PROBLEM_H_
#define THEIA_SFM_INIT_FREE_OSE_PROBLEM_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <string>
#include <vector>

#include "theia/sfm/init_free/init_free_options.h"
#include "theia/sfm/types.h"

namespace theia {

class Reconstruction;
class ViewGraph;

// A projective camera P = [A | t], 3x4, stored as a fixed-size matrix. The
// camera parameter vector used by the solvers is vec(P) in row-major order:
// [P(0,0..3), P(1,0..3), P(2,0..3)].
using OseCamera = Eigen::Matrix<double, 3, 4>;
using OseCameraVector =
    std::vector<OseCamera, Eigen::aligned_allocator<OseCamera>>;

// A camera-only term ||sqrt(w) * (A_a A_b^T - R_ab)||_F^2, where A_k is the
// left 3x3 block of camera k. For a == b this is an orthogonality term with
// R_ab = I.
struct OseRotationTerm {
  int camera_a = -1;
  int camera_b = -1;
  Eigen::Matrix3d relative_rotation = Eigen::Matrix3d::Identity();
  double weight = 1.0;
};

// The data of an OSE problem in flat, index-based arrays. Cameras and points
// are numbered 0..n-1 and 0..m-1. Observations are sorted by point so that the
// observations of point j are [point_offsets[j], point_offsets[j + 1]). This
// layout is chosen so that a GPU backend can upload it without reshuffling.
struct OseProblem {
  std::vector<ViewId> view_ids;    // camera index -> ViewId
  std::vector<TrackId> track_ids;  // point index  -> TrackId

  std::vector<int> observation_camera;  // per observation
  std::vector<int> observation_point;   // per observation
  Eigen::Matrix2Xd observations;        // normalized measurements, 2 x N
  std::vector<int> point_offsets;       // size num_points + 1

  // Maps a pixel x of camera i to the coordinates used in `observations` as
  // m ~ normalizations[i] * [x; 1]. Only meaningful for PIXEL_ISOTROPIC; for
  // CALIBRATED_RAYS the mapping is the (nonlinear) camera undistortion and
  // this is the identity.
  std::vector<Eigen::Matrix3d> normalizations;
  OseObservationNormalization normalization =
      OseObservationNormalization::CALIBRATED_RAYS;

  std::vector<OseRotationTerm> rotation_terms;

  int NumCameras() const { return static_cast<int>(view_ids.size()); }
  int NumPoints() const { return static_cast<int>(track_ids.size()); }
  int NumObservations() const {
    return static_cast<int>(observation_camera.size());
  }
};

// Builds an OseProblem from the tracks of `reconstruction`. Views must have
// camera intrinsics set when CALIBRATED_RAYS is used. If `view_graph` is not
// null, only views in the view graph are used, and its edges provide the
// relative rotations for POSE_ROT. Returns false and sets `error` if the
// resulting problem is empty or the options are inconsistent.
bool BuildOseProblem(const Reconstruction& reconstruction,
                     const ViewGraph* view_graph,
                     const InitFreeOptions& options,
                     OseProblem* problem,
                     std::string* error);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_OSE_PROBLEM_H_
