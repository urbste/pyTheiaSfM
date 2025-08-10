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

#include "theia/sfm/transformation/align_point_clouds.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <Sophus/sophus/sim3.hpp>

#include "theia/sfm/transformation/alignment_error.h"

namespace theia {

void AlignPointCloudsUmeyama(const std::vector<Eigen::Vector3d>& left,
                             const std::vector<Eigen::Vector3d>& right,
                             Eigen::Matrix3d* rotation,
                             Eigen::Vector3d* translation,
                             double* scale) {
  std::vector<double> weights(left.size(), 1.0);
  AlignPointCloudsUmeyamaWithWeights(
      left, right, weights, rotation, translation, scale);
}

void AlignPointCloudsUmeyamaWithWeights(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
    const std::vector<double>& weights,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double* scale) {
  CHECK_EQ(left.size(), right.size());
  CHECK_EQ(left.size(), weights.size());
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(translation);
  CHECK_NOTNULL(scale);

  // Fill outputs (useful when it fails)
  *scale = 1.0;
  *translation = Eigen::Vector3d::Zero();
  *rotation = Eigen::Matrix3d::Identity();

  const size_t num_points = left.size();
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > left_points(
      left[0].data(), 3, num_points);
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > right_points(
      right[0].data(), 3, num_points);

  Eigen::Vector3d left_centroid, right_centroid;
  left_centroid.setZero();
  right_centroid.setZero();
  double weights_sum = 0.0;
  for (size_t i = 0; i < num_points; i++) {
    CHECK_GE(weights[i], 0)
        << "The point weight must be greater or equal to zero.";
    weights_sum += weights[i];
    left_centroid += left[i] * weights[i];
    right_centroid += right[i] * weights[i];
  }
  // Check if the sum is valid
  CHECK_GT(weights_sum, 0) << "The sum of weights must be greater than zero.";

  left_centroid /= weights_sum;
  right_centroid /= weights_sum;

  double sigma = 0.0;
  for (size_t i = 0; i < num_points; i++) {
    sigma += (left[i] - left_centroid).squaredNorm() * weights[i];
  }
  sigma /= weights_sum;

  // Calculate cross correlation matrix based on the points shifted about the
  // centroid.
  Eigen::Matrix3d cross_correlation = Eigen::Matrix3d::Zero();
  for (int i = 0; i < num_points; i++) {
    cross_correlation += weights[i] * (left_points.col(i) - left_centroid) *
                         (right_points.col(i) - right_centroid).transpose();
  }
  cross_correlation /= weights_sum;

  // Compute SVD decomposition of the cross correlation.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      cross_correlation.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Matrix3d& umatrix = svd.matrixU();
  const Eigen::Matrix3d& vtmatrix = svd.matrixV().transpose();
  const Eigen::Vector3d& singular_values = svd.singularValues();

  // Check for matrix singularity (degenerate point clouds)
  const double min_singular_value = singular_values.minCoeff();
  const double max_singular_value = singular_values.maxCoeff();
  const double condition_number = max_singular_value / (min_singular_value + 1e-12);
  
  // If the condition number is too large or singular values are too small, 
  // the point clouds are degenerate (identical, collinear, or coplanar)
  const double singularity_threshold = 1e6;  // Condition number threshold
  const double min_singular_threshold = 1e-8;  // Minimum singular value threshold
  
  if (condition_number > singularity_threshold || min_singular_value < min_singular_threshold) {
    LOG(WARNING) << "Degenerate point clouds detected (condition number: " 
                 << condition_number << ", min singular value: " << min_singular_value 
                 << "). Returning identity transformation.";
    // Return identity transformation for degenerate cases
    *scale = 1.0;
    *rotation = Eigen::Matrix3d::Identity();
    *translation = Eigen::Vector3d::Zero();
    return;
  }

  const double det = umatrix.determinant() * vtmatrix.determinant();
  Eigen::Matrix3d s = Eigen::Matrix3d::Identity();
  s(2, 2) = det > 0 ? 1 : -1;

  *scale =
      (singular_values(0) + singular_values(1) + s(2, 2) * singular_values(2)) /
      sigma;
  *rotation = umatrix * s * vtmatrix;
  *translation = right_centroid - (*scale) * (*rotation) * left_centroid;
}

// SIM3 Alignment with Initial Umeyama + Ceres Optimization
// 
// This approach uses a two-stage alignment:
// 1. Initial alignment using robust Umeyama algorithm for coarse alignment
// 2. Fine optimization using Ceres Solver for precise alignment
// 
// This is more robust than using Ceres alone, especially for large transformations
// or when the initial guess is poor.

Sim3AlignmentSummary OptimizeAlignmentSim3(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points,
    const Sim3AlignmentOptions& options) {
  
  Sim3AlignmentSummary summary;
  
  if (source_points.size() != target_points.size()) {
    LOG(ERROR) << "Source and target point clouds must have the same size.";
    return summary;
  }
  if (source_points.size() == 0) {
    LOG(ERROR) << "Point clouds cannot be empty.";
    return summary;
  }
  
  // Initialize SIM3 parameters (identity transformation)
  Sophus::Vector7d sim3_params = Sophus::Vector7d::Zero();
  sim3_params[6] = 1.0;  // scale = 1.0 (last parameter)
      
  // If initial guess is provided, use it
  if (options.initial_sim3_params) {
    summary.sim3_params = *options.initial_sim3_params;
  } else {
    // Use robust Umeyama for initial alignment
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double scale;
    
    // Use weighted Umeyama with robust weights
    std::vector<double> weights(source_points.size(), 1.0);
    AlignPointCloudsUmeyamaWithWeights(source_points, target_points, weights, 
                                      &rotation, &translation, &scale);
    
    // Convert to SIM3 parameters [translation, rotation, scale]
    summary.sim3_params = Sim3FromRotationTranslationScale(rotation, translation, scale);
  }
  summary.success = true;
  
  if (options.perform_optimization) {
    // Create optimization problem
    ceres::Problem problem;
    
    // Add point-to-point residuals
    for (size_t i = 0; i < source_points.size(); ++i) {
      ceres::CostFunction* cost_function = nullptr;
      double point_weight = 1.0;  // Default weight
      
      // Use individual point weights if provided (higher weight = higher confidence)
      if (options.point_weights && i < options.point_weights->size()) {
        point_weight = (*options.point_weights)[i];
      } else {
        // Fall back to global point weight if no individual weights provided
        point_weight = options.point_weight;
      }
      
      switch (options.alignment_type) {
        case Sim3AlignmentType::POINT_TO_POINT:
          cost_function = Sim3PointToPointError::Create(
              source_points[i], target_points[i], point_weight);
          break;
          
        case Sim3AlignmentType::ROBUST_POINT_TO_POINT:
          cost_function = Sim3PointToPointError::Create(
              source_points[i], target_points[i], point_weight);
          break;
          
        case Sim3AlignmentType::POINT_TO_PLANE:
          if (options.target_normals && i < options.target_normals->size()) {
            cost_function = Sim3PointToPlaneError::Create(
                source_points[i], target_points[i], 
                (*options.target_normals)[i], point_weight);
          } else {
            LOG(WARNING) << "Target normals not provided for point-to-plane alignment. "
                        << "Falling back to point-to-point.";
            cost_function = Sim3PointToPointError::Create(
                source_points[i], target_points[i], point_weight);
          }
          break;
          
        default:
          LOG(FATAL) << "Unknown alignment type.";
          break;
      }
      
      if (cost_function) {
        // Use Huber loss function for robust alignment
        ceres::LossFunction* loss_function = nullptr;
        if (options.alignment_type == Sim3AlignmentType::ROBUST_POINT_TO_POINT) {
          loss_function = new ceres::HuberLoss(options.huber_threshold);
        }
        
        problem.AddResidualBlock(cost_function, loss_function, sim3_params.data());
      }
    }
      
    // Set up solver options
    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = options.linear_solver_type;
    solver_options.minimizer_type = options.minimizer_type;
    solver_options.max_num_iterations = options.max_iterations;
    solver_options.minimizer_progress_to_stdout = options.verbose;
    
    // Solve the problem
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    
    // Fill summary
    summary.success = solver_summary.termination_type == ceres::CONVERGENCE;
    summary.final_cost = solver_summary.final_cost;
    summary.num_iterations = solver_summary.iterations.size();
    summary.sim3_params = sim3_params;
    
    // Compute final alignment error
    Sophus::Sim3d final_transformation = Sophus::Sim3d::exp(sim3_params);
    double total_error = 0.0;
    for (size_t i = 0; i < source_points.size(); ++i) {
      Eigen::Vector3d transformed_point = final_transformation * source_points[i];
      total_error += (transformed_point - target_points[i]).norm();
    }
    summary.alignment_error = total_error / source_points.size();
  }
  return summary;
}

// Utility functions for converting between different representations

Sophus::Vector7d Sim3FromRotationTranslationScale(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation,
    double scale) {
  
  // Create SIM3 using the correct constructor
  Sophus::Sim3d sim3(Sophus::RxSO3d(scale, rotation), translation);
  
  // Return the 7-parameter representation
  return sim3.log();
}

void Sim3ToRotationTranslationScale(
    const Sophus::Vector7d& sim3_params,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double* scale) {
  
  CHECK_NOTNULL(rotation);
  CHECK_NOTNULL(translation);
  CHECK_NOTNULL(scale);
  
  // Create SIM3 from the 7-parameter representation
  Sophus::Sim3d sim3 = Sophus::Sim3d::exp(sim3_params);
  
  // Extract components
  *rotation = sim3.rotationMatrix();
  *translation = sim3.translation();
  *scale = sim3.scale();
}
}  // namespace theia
