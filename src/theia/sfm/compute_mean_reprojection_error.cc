// Copyright (C) 2026 Steffen Urban

#include "theia/sfm/compute_mean_reprojection_error.h"

#include <cmath>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"

namespace theia {

double ComputeMeanReprojectionError(const Reconstruction& reconstruction) {
  double error_sum = 0.0;
  int num_observations = 0;

  for (const ViewId view_id : reconstruction.ViewIds()) {
    const View* view = reconstruction.View(view_id);
    if (view == nullptr || !view->IsEstimated()) {
      continue;
    }
    const Camera& camera = view->Camera();
    for (const TrackId track_id : view->TrackIds()) {
      const Track* track = reconstruction.Track(track_id);
      if (track == nullptr || !track->IsEstimated()) {
        continue;
      }
      const Feature* feature = view->GetFeature(track_id);
      if (feature == nullptr) {
        continue;
      }
      Eigen::Vector2d projection;
      camera.ProjectPoint(track->Point(), &projection);
      error_sum += (projection - feature->point_).norm();
      ++num_observations;
    }
  }

  if (num_observations == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return error_sum / static_cast<double>(num_observations);
}

}  // namespace theia
