// Copyright (C) 2026 The pyTheiaSfM Authors.

#include "theia/sfm/rig/capture_view_graph.h"

#include <algorithm>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <glog/logging.h>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/rig/camera_rig.h"
#include "theia/sfm/rig/rig_capture.h"
#include "theia/sfm/twoview_info.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/map_util.h"

namespace theia {
namespace {

Eigen::Matrix3d AngleAxisToMatrix(const Eigen::Vector3d& aa) {
  Eigen::Matrix3d R;
  ceres::AngleAxisToRotationMatrix(aa.data(),
                                   ceres::ColumnMajorAdapter3x3(R.data()));
  return R;
}

Eigen::Vector3d MatrixToAngleAxis(const Eigen::Matrix3d& R) {
  Eigen::Vector3d aa;
  ceres::RotationMatrixToAngleAxis(ceres::ColumnMajorAdapter3x3(R.data()),
                                   aa.data());
  return aa;
}

// TwoViewInfo stores cam1 at identity; convert to capture-relative TwoViewInfo
// using calibrated sensor poses in the abstract rig frame.
bool ViewEdgeToCaptureEdge(const RigSensor& sensor1,
                           const RigSensor& sensor2,
                           const TwoViewInfo& view_info,
                           TwoViewInfo* capture_info) {
  CHECK_NOTNULL(capture_info);
  const Eigen::Matrix3d R_s1 = sensor1.GetOrientationAsRotationMatrix();
  const Eigen::Matrix3d R_s2 = sensor2.GetOrientationAsRotationMatrix();

  const Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d c1 = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d R2 = AngleAxisToMatrix(view_info.rotation_2);
  const Eigen::Vector3d c2 = view_info.position_2;

  // Recover capture poses implied by the measured cameras.
  const Eigen::Matrix3d R_g1 = R_s1.transpose() * R1;
  const Eigen::Vector3d c_g1 = c1 - R_g1.transpose() * sensor1.position;
  const Eigen::Matrix3d R_g2 = R_s2.transpose() * R2;
  const Eigen::Vector3d c_g2 = c2 - R_g2.transpose() * sensor2.position;

  // Put capture 1 at identity for the edge payload.
  const Eigen::Matrix3d R_g2_rel = R_g2 * R_g1.transpose();
  const Eigen::Vector3d c_g2_rel = R_g1 * (c_g2 - c_g1);

  *capture_info = view_info;
  capture_info->rotation_2 = MatrixToAngleAxis(R_g2_rel);
  capture_info->position_2 = c_g2_rel;
  return true;
}

}  // namespace

bool BuildCaptureViewGraph(const Reconstruction& reconstruction,
                           const ViewGraph& view_graph,
                           ViewGraph* capture_view_graph) {
  CHECK_NOTNULL(capture_view_graph);
  *capture_view_graph = ViewGraph();

  const auto& edges = view_graph.GetAllEdges();
  int num_added = 0;
  for (const auto& edge : edges) {
    const ViewId view_id1 = edge.first.first;
    const ViewId view_id2 = edge.first.second;
    const TwoViewInfo& view_info = edge.second;

    const ViewRigMembership* m1 = reconstruction.GetViewRigMembership(view_id1);
    const ViewRigMembership* m2 = reconstruction.GetViewRigMembership(view_id2);
    if (m1 == nullptr || m2 == nullptr) {
      continue;
    }
    if (m1->capture_id == m2->capture_id) {
      continue;  // Intra-capture: calibrated extrinsics, not a motion edge.
    }
    if (m1->rig_id != m2->rig_id) {
      LOG(WARNING) << "Skipping edge between views on different rigs.";
      continue;
    }

    const CameraRig* rig = reconstruction.GetCameraRig(m1->rig_id);
    if (rig == nullptr) {
      continue;
    }
    const RigSensor* s1 = rig->GetSensor(m1->rig_camera_id);
    const RigSensor* s2 = rig->GetSensor(m2->rig_camera_id);
    if (s1 == nullptr || s2 == nullptr) {
      continue;
    }

    TwoViewInfo capture_info;
    if (!ViewEdgeToCaptureEdge(*s1, *s2, view_info, &capture_info)) {
      continue;
    }

    const CaptureId c1 = m1->capture_id;
    const CaptureId c2 = m2->capture_id;
    // ViewGraph stores undirected edges with min,max ordering.
    const CaptureId a = std::min(c1, c2);
    const CaptureId b = std::max(c1, c2);
    TwoViewInfo stored = capture_info;
    if (c1 != a) {
      // Edge was computed as c2 relative to c1; swap to a=min at identity.
      SwapCameras(&stored);
    }

    const TwoViewInfo* existing = capture_view_graph->GetEdge(a, b);
    if (existing == nullptr ||
        stored.num_verified_matches > existing->num_verified_matches) {
      capture_view_graph->AddEdge(a, b, stored);
      if (existing == nullptr) {
        ++num_added;
      }
    }
  }

  VLOG(1) << "Built capture view graph with " << num_added << " edges from "
          << edges.size() << " view edges.";
  return num_added > 0;
}

}  // namespace theia
