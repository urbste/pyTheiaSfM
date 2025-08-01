// Copyright (C) 2025 Steffen Urban
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

#ifndef THEIA_SFM_TRANSFORMATION_ALIGNMENT_ERROR_H_
#define THEIA_SFM_TRANSFORMATION_ALIGNMENT_ERROR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <ceres/ceres.h>

#include <Sophus/sophus/sim3.hpp>

namespace theia {

/**
 * @brief SIM3 point-to-point alignment error for Ceres Solver.
 * 
 * This cost function computes the error between a source point and its
 * corresponding target point after applying an SIM3 transformation.
 * 
 * Parameters:
 * - sim3_pose: 7-dimensional SIM3 pose [rx, ry, rz, tx, ty, tz, scale]
 * - source_point: 3D source point coordinates
 * - target_point: 3D target point coordinates
 * 
 * Residual: 3-dimensional error vector (target - transformed_source)
 */
struct Sim3PointToPointError {
  Sim3PointToPointError(const Eigen::Vector3d& source_point,
                        const Eigen::Vector3d& target_point,
                        double weight = 1.0)
      : source_point_(source_point), target_point_(target_point), weight_(weight) {}

  template <typename T>
  bool operator()(const T* sim3_pose, T* residuals) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Sophus::Sim3<T> Sim3T;

    // Extract SIM3 pose parameters
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params(sim3_pose);
    Sim3T sim3_transformation = Sim3T::exp(sim3_params);

    // Transform source point
    Vector3T source_point_t = source_point_.cast<T>();
    Vector3T transformed_point = sim3_transformation * source_point_t;

    // Compute residual: target - transformed_source
    Vector3T target_point_t = target_point_.cast<T>();
    residuals[0] = weight_ * (target_point_t[0] - transformed_point[0]);
    residuals[1] = weight_ * (target_point_t[1] - transformed_point[1]);
    residuals[2] = weight_ * (target_point_t[2] - transformed_point[2]);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& source_point,
                                    const Eigen::Vector3d& target_point,
                                    double weight = 1.0) {
    return new ceres::AutoDiffCostFunction<Sim3PointToPointError, 3, 7>(
        new Sim3PointToPointError(source_point, target_point, weight));
  }

 private:
  const Eigen::Vector3d source_point_;
  const Eigen::Vector3d target_point_;
  const double weight_;
};

/**
 * @brief SIM3 point-to-plane alignment error for Ceres Solver.
 * 
 * This cost function computes the point-to-plane distance between a source point
 * and its corresponding target plane after applying an SIM3 transformation.
 * 
 * Parameters:
 * - sim3_pose: 7-dimensional SIM3 pose [rx, ry, rz, tx, ty, tz, scale]
 * - source_point: 3D source point coordinates
 * - target_point: 3D point on the target plane
 * - target_normal: 3D normal vector of the target plane
 * 
 * Residual: 1-dimensional point-to-plane distance
 */
struct Sim3PointToPlaneError {
  Sim3PointToPlaneError(const Eigen::Vector3d& source_point,
                        const Eigen::Vector3d& target_point,
                        const Eigen::Vector3d& target_normal,
                        double weight = 1.0)
      : source_point_(source_point),
        target_point_(target_point),
        target_normal_(target_normal.normalized()),
        weight_(weight) {}

  template <typename T>
  bool operator()(const T* sim3_pose, T* residuals) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Sophus::Sim3<T> Sim3T;

    // Extract SIM3 pose parameters
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params(sim3_pose);
    Sim3T sim3_transformation = Sim3T::exp(sim3_params);

    // Transform source point
    Vector3T source_point_t = source_point_.cast<T>();
    Vector3T transformed_point = sim3_transformation * source_point_t;

    // Compute point-to-plane distance: n^T * (p_target - p_transformed)
    Vector3T target_point_t = target_point_.cast<T>();
    Vector3T target_normal_t = target_normal_.cast<T>();
    
    Vector3T diff = target_point_t - transformed_point;
    residuals[0] = weight_ * target_normal_t.dot(diff);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& source_point,
                                    const Eigen::Vector3d& target_point,
                                    const Eigen::Vector3d& target_normal,
                                    double weight = 1.0) {
    return new ceres::AutoDiffCostFunction<Sim3PointToPlaneError, 1, 7>(
        new Sim3PointToPlaneError(source_point, target_point, target_normal, weight));
  }

 private:
  const Eigen::Vector3d source_point_;
  const Eigen::Vector3d target_point_;
  const Eigen::Vector3d target_normal_;
  const double weight_;
};

/**
 * @brief SIM3 point cloud alignment error for Ceres Solver.
 * 
 * This cost function computes the alignment error between two point clouds
 * by summing up individual point-to-point errors.
 * 
 * Parameters:
 * - sim3_pose: 7-dimensional SIM3 pose [rx, ry, rz, tx, ty, tz, scale]
 * - source_points: Vector of 3D source point coordinates
 * - target_points: Vector of 3D target point coordinates
 * 
 * Residual: 3N-dimensional error vector (N point pairs)
 */
struct Sim3PointCloudAlignmentError {
  Sim3PointCloudAlignmentError(const std::vector<Eigen::Vector3d>& source_points,
                               const std::vector<Eigen::Vector3d>& target_points,
                               double weight = 1.0)
      : source_points_(source_points), target_points_(target_points), weight_(weight) {
    CHECK_EQ(source_points_.size(), target_points_.size())
        << "Source and target point clouds must have the same size.";
  }

  template <typename T>
  bool operator()(const T* sim3_pose, T* residuals) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Sophus::Sim3<T> Sim3T;

    // Extract SIM3 pose parameters
    Eigen::Map<const Eigen::Matrix<T, 7, 1>> sim3_params(sim3_pose);
    Sim3T sim3_transformation = Sim3T::exp(sim3_params);

    // Compute residuals for each point pair
    for (size_t i = 0; i < source_points_.size(); ++i) {
      Vector3T source_point_t = source_points_[i].cast<T>();
      Vector3T transformed_point = sim3_transformation * source_point_t;
      Vector3T target_point_t = target_points_[i].cast<T>();

      residuals[3 * i + 0] = weight_ * (target_point_t[0] - transformed_point[0]);
      residuals[3 * i + 1] = weight_ * (target_point_t[1] - transformed_point[1]);
      residuals[3 * i + 2] = weight_ * (target_point_t[2] - transformed_point[2]);
    }

    return true;
  }

  static ceres::CostFunction* Create(const std::vector<Eigen::Vector3d>& source_points,
                                    const std::vector<Eigen::Vector3d>& target_points,
                                    double weight = 1.0) {
    return new ceres::AutoDiffCostFunction<Sim3PointCloudAlignmentError, 
                                          ceres::DYNAMIC, 7>(
        new Sim3PointCloudAlignmentError(source_points, target_points, weight),
        3 * source_points.size());
  }

 private:
  const std::vector<Eigen::Vector3d> source_points_;
  const std::vector<Eigen::Vector3d> target_points_;
  const double weight_;
};



/**
 * @brief SIM3 scale-constrained alignment error.
 * 
 * This cost function adds a scale constraint to the SIM3 alignment,
 * useful when the scale should be close to a known value.
 * 
 * Parameters:
 * - sim3_pose: 7-dimensional SIM3 pose [rx, ry, rz, tx, ty, tz, scale]
 * - target_scale: Target scale value
 * - scale_weight: Weight for the scale constraint
 * 
 * Residual: 1-dimensional scale difference
 */
struct Sim3ScaleConstraintError {
  Sim3ScaleConstraintError(double target_scale, double scale_weight = 1.0)
      : target_scale_(target_scale), scale_weight_(scale_weight) {}

  template <typename T>
  bool operator()(const T* sim3_pose, T* residuals) const {
    // Extract scale from SIM3 pose (7th parameter)
    T current_scale = sim3_pose[6];
    T target_scale_t = T(target_scale_);
    T scale_weight_t = T(scale_weight_);

    // Compute scale difference
    residuals[0] = scale_weight_t * (current_scale - target_scale_t);

    return true;
  }

  static ceres::CostFunction* Create(double target_scale, double scale_weight = 1.0) {
    return new ceres::AutoDiffCostFunction<Sim3ScaleConstraintError, 1, 7>(
        new Sim3ScaleConstraintError(target_scale, scale_weight));
  }

 private:
  const double target_scale_;
  const double scale_weight_;
};

}  // namespace theia

#endif  // THEIA_SFM_TRANSFORMATION_ALIGNMENT_ERROR_H_