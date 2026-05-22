// Copyright (C) 2026 Steffen Urban
// All rights reserved.

#include "theia/sfm/transformation/sim3_pose_from_view.h"

#include <glog/logging.h>

namespace theia {

Sophus::Sim3d GetSim3PoseFromView(const View& view) {
  const Eigen::Matrix3d R_c_w = view.Camera().GetOrientationAsRotationMatrix();
  const Eigen::Vector3d t_c_w =
      -R_c_w * view.Camera().GetPosition();
  return Sophus::Sim3d(Sophus::RxSO3d(1.0, R_c_w), t_c_w);
}

Eigen::Matrix<double, 7, 1> GetSim3LieFromView(const View& view) {
  return GetSim3PoseFromView(view).log();
}

void SetViewCameraFromSim3Lie(View* view,
                              const Eigen::Matrix<double, 7, 1>& lie) {
  const Sophus::Sim3d sim3 = Sophus::Sim3d::exp(lie);
  const Eigen::Matrix3d R_c_w = sim3.rotationMatrix();
  const Eigen::Vector3d t_c_w = sim3.translation();
  const Eigen::Vector3d position = -R_c_w.transpose() * t_c_w;
  view->MutableCamera()->SetOrientationFromRotationMatrix(R_c_w);
  view->MutableCamera()->SetPosition(position);
}

void GetSim3LiesFromReconstruction(
    const Reconstruction& reconstruction,
    const std::vector<ViewId>& view_ids,
    Sim3LieMap* lies) {
  CHECK_NOTNULL(lies)->clear();
  for (const ViewId view_id : view_ids) {
    const View* view = reconstruction.View(view_id);
    CHECK(view != nullptr) << "Invalid view id " << view_id;
    (*lies)[view_id] = GetSim3LieFromView(*view);
  }
}

}  // namespace theia
