// Copyright (C) 2026, Steffen Urban
// All rights reserved.
//
// See imported_reconstruction.h for the license.

#include "theia/io/imported_reconstruction.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <string>
#include <unordered_set>
#include <vector>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/full_opencv_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/sfm/camera/thin_prism_fisheye_camera_model.h"
#include "theia/sfm/feature.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"

namespace theia {
namespace internal {
namespace {

template <class Model>
Model* MutableModel(Camera* camera) {
  return static_cast<Model*>(camera->MutableCameraIntrinsics().get());
}

// Sets the intrinsics of camera. The model type is set first, because changing
// it creates a new model with default parameters.
bool ConfigureCamera(const ImportedIntrinsics& intrinsics,
                     const std::string& view_name,
                     Camera* camera,
                     ImportMessages* messages) {
  camera->SetCameraIntrinsicsModelType(intrinsics.model_type);
  camera->SetImageSize(intrinsics.width, intrinsics.height);
  camera->SetFocalLength(intrinsics.fx);
  camera->SetPrincipalPoint(intrinsics.cx, intrinsics.cy);
  const double aspect_ratio = intrinsics.fy / intrinsics.fx;
  const std::vector<double>& d = intrinsics.extra;

  // Number of distortion terms in intrinsics.extra for each model.
  auto require_extra = [&](const size_t size) {
    if (d.size() == size) {
      return true;
    }
    return ImportError("View " + view_name + ": expected " +
                           std::to_string(size) + " distortion terms, got " +
                           std::to_string(d.size()) + ".",
                       messages);
  };

  switch (intrinsics.model_type) {
    case CameraIntrinsicsModelType::PINHOLE: {
      auto* model = MutableModel<PinholeCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetSkew(0.0);
      model->SetRadialDistortion(intrinsics.k1, intrinsics.k2);
      return true;
    }
    case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL: {
      auto* model = MutableModel<PinholeRadialTangentialCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetSkew(0.0);
      model->SetRadialDistortion(intrinsics.k1, intrinsics.k2, intrinsics.k3);
      model->SetTangentialDistortion(intrinsics.p1, intrinsics.p2);
      return true;
    }
    case CameraIntrinsicsModelType::FISHEYE: {
      if (!require_extra(4)) return false;
      auto* model = MutableModel<FisheyeCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetSkew(0.0);
      model->SetRadialDistortion(d[0], d[1], d[2], d[3]);
      return true;
    }
    case CameraIntrinsicsModelType::FOV: {
      if (!require_extra(1)) return false;
      auto* model = MutableModel<FOVCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetRadialDistortion(d[0]);
      return true;
    }
    case CameraIntrinsicsModelType::FULL_OPENCV: {
      // COLMAP order: k1, k2, p1, p2, k3, k4, k5, k6.
      if (!require_extra(8)) return false;
      auto* model = MutableModel<FullOpenCVCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetRadialDistortion(d[0], d[1], d[4], d[5], d[6], d[7]);
      model->SetTangentialDistortion(d[2], d[3]);
      return true;
    }
    case CameraIntrinsicsModelType::RADIAL_FISHEYE: {
      if (!require_extra(2)) return false;
      auto* model = MutableModel<RadialFisheyeCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetRadialDistortion(d[0], d[1]);
      return true;
    }
    case CameraIntrinsicsModelType::THIN_PRISM_FISHEYE: {
      // COLMAP order: k1, k2, p1, p2, k3, k4, sx1, sy1.
      if (!require_extra(8)) return false;
      auto* model = MutableModel<ThinPrismFisheyeCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetRadialDistortion(d[0], d[1], d[4], d[5]);
      model->SetTangentialDistortion(d[2], d[3]);
      model->SetThinPrismDistortion(d[6], d[7]);
      return true;
    }
    case CameraIntrinsicsModelType::RAD_TAN_THIN_PRISM_FISHEYE: {
      // COLMAP order: k0..k5, p0, p1, s0..s3.
      if (!require_extra(12)) return false;
      auto* model = MutableModel<RadTanThinPrismFisheyeCameraModel>(camera);
      model->SetAspectRatio(aspect_ratio);
      model->SetRadialDistortion(d[0], d[1], d[2], d[3], d[4], d[5]);
      model->SetTangentialDistortion(d[6], d[7]);
      model->SetThinPrismDistortion(d[8], d[9], d[10], d[11]);
      return true;
    }
    default:
      return ImportError(
          "View " + view_name + ": no distortion mapping for camera model " +
              std::to_string(static_cast<int>(intrinsics.model_type)) + ".",
          messages);
  }
}

}  // namespace

bool ImportError(const std::string& error,
                 ImportMessages* messages,
                 const bool file_not_found) {
  LOG(ERROR) << error;
  if (messages != nullptr) {
    messages->error = error;
    messages->file_not_found = file_not_found;
  }
  return false;
}

void ImportWarning(const std::string& warning, ImportMessages* messages) {
  LOG(WARNING) << warning;
  if (messages != nullptr) {
    messages->warnings.push_back(warning);
  }
}

bool BuildImportedReconstruction(const std::vector<ImportedView>& views,
                                 const std::vector<ImportedTrack>& tracks,
                                 Reconstruction* reconstruction,
                                 ImportMessages* messages) {
  CHECK_NOTNULL(reconstruction);
  std::vector<ViewId> view_ids;
  view_ids.reserve(views.size());
  for (int i = 0; i < static_cast<int>(views.size()); ++i) {
    const ImportedView& spec = views[i];
    // Timestamps must be unique: use the view index.
    const ViewId view_id =
        reconstruction->AddView(spec.name, spec.group_id, static_cast<double>(i));
    if (view_id == kInvalidViewId) {
      return ImportError("Could not add view '" + spec.name +
                             "' (duplicate or empty image name).",
                         messages);
    }
    View* view = reconstruction->MutableView(view_id);
    Camera* camera = view->MutableCamera();
    if (!ConfigureCamera(spec.intrinsics, spec.name, camera, messages)) {
      return false;
    }
    if (spec.has_pose) {
      camera->SetOrientationFromRotationMatrix(spec.rotation);
      camera->SetPosition(spec.center);
    }
    view->SetEstimated(spec.has_pose);
    view_ids.push_back(view_id);
  }

  int duplicates = 0;
  for (const ImportedTrack& spec : tracks) {
    const TrackId track_id = reconstruction->AddTrack();
    Track* track = reconstruction->MutableTrack(track_id);
    track->SetPoint(spec.point.homogeneous());
    if (spec.has_color) {
      track->SetColor(spec.color);
    }
    track->SetEstimated(true);
    // A track holds at most one observation per view. Some tools (COLMAP can,
    // for duplicated features) write more; keep the first.
    std::unordered_set<int> seen_views;
    for (const auto& [view_index, x, y] : spec.observations) {
      if (!seen_views.insert(view_index).second) {
        ++duplicates;
        continue;
      }
      reconstruction->AddObservation(
          view_ids[view_index], track_id, Feature(x, y));
    }
  }
  if (duplicates > 0) {
    ImportWarning(std::to_string(duplicates) +
                      " observations repeat a view already observing the same "
                      "track; only the first was kept.",
                  messages);
  }
  return true;
}

}  // namespace internal
}  // namespace theia
