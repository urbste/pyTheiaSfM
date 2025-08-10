// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#ifndef THEIA_SFM_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
#define THEIA_SFM_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_

#include <Eigen/Core>
#include <vector>
#include <ceres/ceres.h>
#include <sophus/sim3.hpp>
#include <sophus/types.hpp>

namespace theia {

// Computes the orientation, position, and scale factor for the transformation
// between two corresponding 3D point sets A and B such as they are related by:
//
//     B = s * R * A + t
//
// where A is "left" and B is "right". Implementation is based on the paper by
// Umeyama "Least-squares estimation of transformation parameters between two
// point patterns".
void AlignPointCloudsUmeyama(const std::vector<Eigen::Vector3d>& left,
                             const std::vector<Eigen::Vector3d>& right,
                             Eigen::Matrix3d* rotation,
                             Eigen::Vector3d* translation,
                             double* scale);

// Adds weights for each match
// The previous objective function E = Sum(|| yi - (c.R.xi + T) ||^2) becomes
// Sum(wi * || yi - (c.R.xi + T) ||^2)
// The weights must be positive
void AlignPointCloudsUmeyamaWithWeights(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
    const std::vector<double>& weights,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double* scale);

// SIM3 Alignment Types
enum class Sim3AlignmentType {
  POINT_TO_POINT,
  ROBUST_POINT_TO_POINT,
  POINT_TO_PLANE
};

// Options for SIM3 alignment
struct Sim3AlignmentOptions {
  // Alignment type
  Sim3AlignmentType alignment_type = Sim3AlignmentType::POINT_TO_POINT;
  
  // Initial guess for SIM3 parameters [rx, ry, rz, tx, ty, tz, scale]
  const Sophus::Vector7d* initial_sim3_params = nullptr;
  
  // Point weights and thresholds
  double point_weight = 1.0;
  double huber_threshold = 0.1;
  double outlier_threshold = 1.0;
  
  // Target normals for point-to-plane alignment
  const std::vector<Eigen::Vector3d>* target_normals = nullptr;
  const std::vector<double>* point_weights = nullptr;
  
  // Solver options
  ceres::LinearSolverType linear_solver_type = ceres::SPARSE_SCHUR;
  ceres::MinimizerType minimizer_type = ceres::TRUST_REGION;
  int max_iterations = 100;
  bool verbose = false;

  // Whether to perform optimization or just use the initial guess
  bool perform_optimization = true;
  
  Sim3AlignmentOptions() = default;
  
  // Setter functions for Python bindings
  void SetInitialSim3Params(const Sophus::Vector7d& params) {
    initial_sim3_params_storage_ = params;
    initial_sim3_params = &initial_sim3_params_storage_;
  }
  
  void SetTargetNormals(const std::vector<Eigen::Vector3d>& normals) {
    target_normals_storage_ = normals;
    target_normals = &target_normals_storage_;
  }
  
  void SetPointWeights(const std::vector<double>& weights) {
    point_weights_storage_ = weights;
    point_weights = &point_weights_storage_;
  }
  
  void ClearInitialSim3Params() {
    initial_sim3_params = nullptr;
  }
  
  void ClearTargetNormals() {
    target_normals = nullptr;
  }
  
  void ClearPointWeights() {
    point_weights = nullptr;
  }

 private:
  // Storage for the pointer members to avoid dangling pointers
  Sophus::Vector7d initial_sim3_params_storage_;
  std::vector<Eigen::Vector3d> target_normals_storage_;
  std::vector<double> point_weights_storage_;
};

// Summary of SIM3 alignment results
struct Sim3AlignmentSummary {
  bool success = false;
  double final_cost = 0.0;
  int num_iterations = 0;
  double alignment_error = 0.0;
  Sophus::Vector7d sim3_params;
  
  Sim3AlignmentSummary() {
    sim3_params.setZero();
    sim3_params[6] = 1.0;  // scale = 1.0
  }
};

// Main SIM3 alignment function using initial Umeyama + Ceres optimization
Sim3AlignmentSummary OptimizeAlignmentSim3(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points,
    const Sim3AlignmentOptions& options = Sim3AlignmentOptions());

// Convert rotation matrix, translation, and scale to SIM3 parameters
Sophus::Vector7d Sim3FromRotationTranslationScale(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation,
    double scale);

// Convert SIM3 parameters to rotation matrix, translation, and scale
void Sim3ToRotationTranslationScale(
    const Sophus::Vector7d& sim3_params,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double* scale);

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
