// Copyright (C) 2023 Steffen Urban
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
// Author: Steffen Urban (urbste@googlemail.com)

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <glog/logging.h>
#include <string>
#include <utility>
#include <vector>
#include <Sophus/sophus/se3.hpp>
#include <Sophus/sophus/sim3.hpp>
#include "gtest/gtest.h"

#include <ceres/local_parameterization.h>
#include <ceres/solver.h>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/transformation/align_reconstructions.h"
#include "theia/sfm/transformation/transform_reconstruction.h"
#include "theia/sfm/types.h"
#include "theia/util/random.h"
#include "theia/util/stringprintf.h"
#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/io/write_ply_file.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/sfm/view_graph/view_graph_from_reconstruction.h"
#include "theia/sfm/transformation/align_reconstructions_pose_graph_optim.h"
#include "theia/sfm/camera/create_reprojection_error_cost_function.h"
#include "theia/sfm/transformation/align_reconstructions.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/set_outlier_tracks_to_unestimated.h"
#include "theia/sfm/find_common_tracks_by_feature_in_reconstructions.h"

namespace theia {



void GetSim3s(const Reconstruction& recon, std::map<ViewId, Eigen::Matrix<double, 7, 1>>& sim3s) {
  sim3s.clear();
  for (const auto& view_id : recon.ViewIds()) {
    Eigen::Matrix3d R_c_w = recon.View(view_id)->Camera().GetOrientationAsRotationMatrix();
    Eigen::Vector3d t_c_w = -R_c_w*recon.View(view_id)->Camera().GetPosition();
    Sophus::Sim3d sim3(Sophus::RxSO3d(1.0, R_c_w), t_c_w);
    sim3s[view_id] = sim3.log();
  }
}


void GetSE3s(const Reconstruction& recon, std::map<ViewId, Eigen::Matrix<double, 6, 1>>& se3s) {
  se3s.clear();
  for (const auto& view_id : recon.ViewIds()) {
    Eigen::Matrix3d R_c_w = recon.View(view_id)->Camera().GetOrientationAsRotationMatrix();
    Eigen::Vector3d t_c_w = -R_c_w*recon.View(view_id)->Camera().GetPosition();
    Sophus::SE3d se3(R_c_w, t_c_w);
    se3s[view_id] = se3.log();
  }
}

double GetReconReprojectionError(const theia::Reconstruction& recon) {

  const auto track_ids = recon.TrackIds();
  int num_projected_points = 0;
  double reproj_error = 0.0;
  for (const auto& track_id : track_ids) {
    const Track* track = recon.Track(track_id);
    const auto& view_ids = track->ViewIds();
    for (const auto& view_id : view_ids) {
      const View* view = recon.View(view_id);
      const Feature* feature = view->GetFeature(track_id);
      const Camera& camera = view->Camera();

      Eigen::Vector2d reprojected_point;
      camera.ProjectPoint(track->Point(), &reprojected_point);

      const double reproj_error_i = (reprojected_point - feature->point_).squaredNorm();
      reproj_error += reproj_error_i;
      num_projected_points++;
    }
  } 

  return reproj_error / num_projected_points;
}



TEST(AlignReconstructionPoseGraphOptim, Test) {
  Reconstruction recon_qry;
  Reconstruction recon_ref;
  
  std::string base_path = "/home/steffen/Data/GPStrava/TAAWN_TEST_DATA/1/Reference/run1/undist_reduced_result/debug_reconstructions/";
  theia::ReadReconstruction(base_path+"/chunk_000000_01_original_ref.sfm", &recon_ref);  
  theia::ReadReconstruction(base_path+"/chunk_000001_01_original_qry.sfm", &recon_qry);  
  

  theia::BundleAdjustReconstruction(theia::BundleAdjustmentOptions(), &recon_qry);
  theia::BundleAdjustReconstruction(theia::BundleAdjustmentOptions(), &recon_ref);

  // just write out view ids and names
  for (const auto& view_id : recon_qry.ViewIds()) {
    std::cout<<"Query View id: "<<view_id<<" name: "<<recon_qry.View(view_id)->Name()<<std::endl;
  }
  for (const auto& view_id : recon_ref.ViewIds()) {
    std::cout<<"Reference View id: "<<view_id<<" name: "<<recon_ref.View(view_id)->Name()<<std::endl;
  }

  double reproj_error_qry = GetReconReprojectionError(recon_qry);
  double reproj_error_ref = GetReconReprojectionError(recon_ref);
  std::cout<<"Reprojection error before optimization: "<<reproj_error_qry<<std::endl;
  std::cout<<"Reprojection error before optimization: "<<reproj_error_ref<<std::endl;

  // define view graph matches that need to align
  int chunk_size = 20;
  int overlap_size = 5;

  // we need a function so that id_ref=chunk_size-overlap_size+0  equals id_qry= 0
  // and id_ref=chunk_size-overlap_size+1  equals id_qry=1
  std::vector<std::pair<ViewId, ViewId>> view_graph_matches_ref_qry;
  for (int i = 0; i < overlap_size; i += 1) {
    int idx_ref = chunk_size - overlap_size + i;
    int idx_qry = i;
    std::cout<<"idx_ref: "<<idx_ref<<" idx_qry: "<<idx_qry<<std::endl;
    view_graph_matches_ref_qry.push_back({idx_ref, idx_qry});
  }


  // now we get all corresponding tracks in the query and reference reconstruction
  std::vector<Eigen::Vector3d> points_ref;
  std::vector<Eigen::Vector3d> points_qry;
  std::vector<std::pair<TrackId, TrackId>> track_id_pairs;
  theia::FindCommonTracksByFeatureInReconstructions(
    recon_ref, recon_qry, view_graph_matches_ref_qry, &points_ref, &points_qry, &track_id_pairs);

  std::cout<<"Number of common tracks: "<<points_ref.size()<<std::endl;


  theia::Sim3AlignmentOptions options;
  options.alignment_type = theia::Sim3AlignmentType::POINT_TO_POINT;
  options.max_num_iterations = 100;
  theia::Sim3AlignmentSummary summary = theia::OptimizeAlignmentSim3(points_qry, points_ref, options);

  std::cout<<"Summary: "<<summary.success<<" "<<summary.final_cost<<" "<<summary.num_iterations<<" "<<summary.alignment_error<<std::endl;
  theia::TransformReconstruction(summary.sim3_params, &recon_qry);


  theia::WritePlyFile(base_path+"/chunk_000001_trafo.ply", recon_qry, Eigen::Vector3i(0, 255,255), 0);

  for (const auto& pair : view_graph_matches_ref_qry) {
    const ViewId view_id_ref = pair.first;
    const ViewId view_id_qry = pair.second;
    View* view_ref = recon_ref.MutableView(view_id_ref);
    View* view_qry = recon_qry.MutableView(view_id_qry);
    view_qry->SetOrientationPrior(view_ref->Camera().GetOrientationAsAngleAxis(), 5*Eigen::Matrix3d::Identity());
    view_qry->SetPositionPrior(view_ref->Camera().GetPosition(), 100*Eigen::Matrix3d::Identity());
  }


  theia::BundleAdjustmentOptions ba_options;
  ba_options.use_orientation_priors = true;
  ba_options.use_position_priors = true;
  ba_options.max_num_iterations = 50;
  ba_options.verbose = true;

  theia::BundleAdjustReconstruction(ba_options, &recon_qry);

  //int removed_tracks = theia::SetOutlierTracksToUnestimated(5, 0.15, &recon_qry);

  // plot pose error between ref and qry
  for (const auto& view_id : view_graph_matches_ref_qry) {
    const View* view_ref = recon_ref.View(view_id.first);
    const View* view_qry = recon_qry.View(view_id.second);

    Eigen::Matrix3d R_ref = view_ref->Camera().GetOrientationAsRotationMatrix();
    Eigen::Matrix3d R_qry = view_qry->Camera().GetOrientationAsRotationMatrix();
    Eigen::Matrix3d R_error = R_ref.transpose() * R_qry;
    std::cout<<"Pose error: "<<R_error.transpose()<<std::endl;

    // position error
    Eigen::Vector3d t_ref = view_ref->Camera().GetPosition();
    Eigen::Vector3d t_qry = view_qry->Camera().GetPosition();
    Eigen::Vector3d t_error = t_ref - t_qry;
    std::cout<<"Position error: "<<t_error.transpose()<<std::endl;
  }

  // Write the optimized reconstruction
  theia::WritePlyFile(base_path+"/chunk_000001_opt.ply", recon_qry, Eigen::Vector3i(255, 0,255), 0);

}

}  // namespace theia
