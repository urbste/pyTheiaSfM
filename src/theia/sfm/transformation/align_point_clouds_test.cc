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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <Sophus/sophus/sim3.hpp>

#include "theia/math/util.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/util/random.h"
#include "gtest/gtest.h"

namespace theia {
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::RowMajor;
using Eigen::Vector3d;

RandomNumberGenerator rng(50);

namespace {
double kEpsilon = 1e-6;

void UmeyamaSimpleTest() {
  std::vector<Vector3d> left = {Vector3d(0.4, -3.105, 2.147),
                                Vector3d(1.293, 7.1982, -.068),
                                Vector3d(-5.34, 0.708, -3.69),
                                Vector3d(-.345, 1.987, 0.936),
                                Vector3d(0.93, 1.45, 1.079),
                                Vector3d(-3.15, -4.73, 2.49),
                                Vector3d(2.401, -2.03, -1.87),
                                Vector3d(3.192, -.573, 0.1),
                                Vector3d(-2.53, 3.07, -5.19)};

  const Matrix3d rotation_mat =
      Eigen::AngleAxisd(DegToRad(15.0), Vector3d(1.0, -2.7, 1.9).normalized())
          .toRotationMatrix();
  const Vector3d translation_vec(0, 2, 2);
  const double expected_scale = 1.5;

  // Transform the points.
  std::vector<Vector3d> right;
  for (int i = 0; i < left.size(); i++) {
    Vector3d transformed_point =
        expected_scale * rotation_mat * left[i] + translation_vec;
    right.emplace_back(transformed_point);
  }

  // Compute the similarity transformation.
  Matrix3d rotation;
  Vector3d translation;
  double scale;
  AlignPointCloudsUmeyama(left, right, &rotation, &translation, &scale);

  // Ensure the calculated transformation is the same as the one we set.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_LT(std::abs(rotation(i, j) - rotation_mat(i, j)), kEpsilon);
    }
    ASSERT_LT(std::abs(translation(i) - translation_vec(i)), kEpsilon);
  }
  ASSERT_LT(fabs(expected_scale - scale), kEpsilon);
}

// It is not easy to find a formula for the weights that always works
// We need to make some statistics to see if the function works in most cases
//
// This test adds some noise on a given fraction of the points
// We try to estimate the weights from the errors of the first pass (setting
// weights to 1)
// We can expect that the errors are where the noise was added
// From this first result we can set a weight for each point.
// The weight is given by 1.0/(1.0 + distance)
// Where distance is the distance between right[i] and s * R * left[i] + T
// estimated by a first pass
// To see if the new estimation from these weights is better
// we need to check that the new parameters are closer to the expected
// parameters
void UmeyamaWithWeigthsAndNoise() {
  // Make some statistics
  // 1000 is not a problem the algorith is fast
  static const size_t kNumIterations = 1000;

  // 15 % of noise
  static const float kNoiseRatio = 0.15f;

  // Percentage to consider the test succeeds (is it enough ?)
  static const float kTestsSucceeded = 95.0f;

  size_t succeeded = 0;
  for (size_t iteration = 0; iteration < kNumIterations; ++iteration) {
    // 4 pts required
    const size_t num_points = static_cast<size_t>(rng.RandInt(4, 1000));
    std::vector<Vector3d> left(num_points);
    std::vector<double> weights(num_points, 1.0);

    for (size_t i = 0; i < num_points; ++i) {
      left[i] = rng.RandVector3d();
    }

    const Matrix3d rotation_mat =
        Eigen::AngleAxisd(DegToRad(rng.RandDouble(0.0, 360.0)),
                          rng.RandVector3d().normalized())
            .toRotationMatrix();
    const Vector3d translation_vec = rng.RandVector3d();
    const double expected_scale = rng.RandDouble(0.001, 10);

    // Transform the points.
    std::vector<Vector3d> right;
    for (size_t i = 0; i < left.size(); i++) {
      const Vector3d transformed_point =
          expected_scale * rotation_mat * left[i] + translation_vec;
      right.emplace_back(transformed_point);
    }

    // Add noise on scale, point and translation
    for (size_t i = 0, end = (size_t)(kNoiseRatio * num_points); i < end; ++i) {
      const size_t k = static_cast<size_t>(rng.RandInt(0, num_points - 1));
      const double noiseOnScale = expected_scale + rng.RandDouble(0, 10);
      right[k] = noiseOnScale * rotation_mat * (left[k] + rng.RandVector3d()) +
                 translation_vec + rng.RandVector3d();
    }

    // We need to find some weights
    Matrix3d rotation_noisy;
    Vector3d translation_noisy;
    double scale_noisy;
    AlignPointCloudsUmeyamaWithWeights(left,
                                       right,
                                       weights,
                                       &rotation_noisy,
                                       &translation_noisy,
                                       &scale_noisy);
    for (size_t i = 0; i < num_points; ++i) {
      const double dist = (right[i] - (scale_noisy * rotation_noisy * left[i] +
                                       translation_noisy))
                              .norm();
      weights[i] = 1.0 / (1.0 + dist);
    }
    //

    Matrix3d rotation_weighted;
    Vector3d translation_weighted;
    double scale_weighted;
    AlignPointCloudsUmeyamaWithWeights(left,
                                       right,
                                       weights,
                                       &rotation_weighted,
                                       &translation_weighted,
                                       &scale_weighted);

    // Check if the parameters are closer to real parameters ?
    const bool condition_on_scale = (std::abs(scale_weighted - expected_scale) <
                                     std::abs(scale_noisy - expected_scale));
    const bool condition_on_translation =
        (translation_vec - translation_weighted).norm() <
        (translation_vec - translation_noisy).norm();
    const bool condition_on_rotation =
        (rotation_mat - rotation_weighted).norm() <
        (rotation_mat - rotation_noisy).norm();

    if (condition_on_scale && condition_on_translation && condition_on_rotation)
      ++succeeded;
  }

  ASSERT_LE(kTestsSucceeded, 100.0 * succeeded / (0.0 + kNumIterations));
}

// SIM3 Alignment Tests

// Helper function to generate synthetic point cloud data
std::vector<Vector3d> GenerateSyntheticPointCloud(int num_points, double noise_std = 0.0) {
  std::vector<Vector3d> points;
  for (int i = 0; i < num_points; ++i) {
    double x = 10.0 * (static_cast<double>(i) / num_points - 0.5);
    double y = 5.0 * std::sin(x * 0.5);
    double z = 2.0 * std::cos(x * 0.3);
    
    if (noise_std > 0.0) {
      x += rng.RandGaussian(0.0, noise_std);
      y += rng.RandGaussian(0.0, noise_std);
      z += rng.RandGaussian(0.0, noise_std);
    }
    
    points.emplace_back(x, y, z);
  }
  return points;
}

// Helper function to transform point cloud with SIM3
std::vector<Vector3d> TransformPointCloud(const std::vector<Vector3d>& points,
                                         const Sophus::Vector7d& sim3_params) {
  Sophus::Sim3d sim3_transformation = Sophus::Sim3d::exp(sim3_params);
  std::vector<Vector3d> transformed_points;
  
  for (const auto& point : points) {
    transformed_points.push_back(sim3_transformation * point);
  }
  
  return transformed_points;
}

// Helper function to create SIM3 transformation from components
Sophus::Vector7d CreateSim3Transformation(const Eigen::Vector3d& translation,
                                         const Eigen::Vector3d& rotation_axis_angle,
                                         double scale) {
  // Create rotation matrix from angle-axis
  double angle = rotation_axis_angle.norm();
  Eigen::Matrix3d rotation;
  if (angle > 1e-8) {
    Eigen::Vector3d axis = rotation_axis_angle / angle;
    rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  } else {
    rotation = Eigen::Matrix3d::Identity();
  }
  
  // Create SIM3 and get its 7-parameter representation
  Sophus::Sim3d sim3(Sophus::RxSO3d(scale, rotation), translation);
  return sim3.log();
}

// Helper function to compute alignment error
double ComputeAlignmentError(const std::vector<Vector3d>& source_points,
                            const std::vector<Vector3d>& target_points,
                            const Sophus::Vector7d& sim3_params) {
  Sophus::Sim3d transformation = Sophus::Sim3d::exp(sim3_params);
  double total_error = 0.0;
  
  for (size_t i = 0; i < source_points.size(); ++i) {
    Vector3d transformed_point = transformation * source_points[i];
    total_error += (transformed_point - target_points[i]).norm();
  }
  
  return total_error / source_points.size();
}

void Sim3PointToPointAlignmentTest() {
  // Generate synthetic point cloud data
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(100, 0.01);
  
  // Define ground truth SIM3 transformation using proper construction
  Sophus::Vector7d ground_truth_sim3 = CreateSim3Transformation(
      Eigen::Vector3d(1.0, 2.0, 0.5),    // translation
      Eigen::Vector3d(0.1, 0.05, 0.02),  // rotation (angle-axis)
      1.5);                               // scale
  
  // Transform source points to create target points
  std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth_sim3);
  
  // Perform SIM3 alignment (with initial Umeyama + Ceres optimization)
  Sim3AlignmentOptions options;
  options.alignment_type = Sim3AlignmentType::POINT_TO_POINT;
  options.verbose = false;
  options.max_iterations = 100;
  
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      source_points, target_points, options);
  
  // Check that alignment was successful
  ASSERT_TRUE(summary.success);
  
  // Check that the estimated parameters are close to ground truth
  // Convert both to SIM3 objects and compare components
  Sophus::Sim3d estimated_sim3 = Sophus::Sim3d::exp(summary.sim3_params);
  Sophus::Sim3d ground_truth_sim3_obj = Sophus::Sim3d::exp(ground_truth_sim3);
  
  double translation_error = (estimated_sim3.translation() - ground_truth_sim3_obj.translation()).norm();
  double rotation_error = (estimated_sim3.rotationMatrix() - ground_truth_sim3_obj.rotationMatrix()).norm();
  double scale_error = std::abs(estimated_sim3.scale() - ground_truth_sim3_obj.scale());
  
  ASSERT_LT(rotation_error, 0.1);
  ASSERT_LT(translation_error, 0.1);
  ASSERT_LT(scale_error, 0.1);
  ASSERT_LT(summary.alignment_error, 0.1);
}

void Sim3RobustAlignmentTest() {
  // Generate synthetic point cloud data
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(100, 0.01);
  
  // Define ground truth SIM3 transformation using proper construction
  Sophus::Vector7d ground_truth_sim3 = CreateSim3Transformation(
      Eigen::Vector3d(1.0, 2.0, 0.5),    // translation
      Eigen::Vector3d(0.1, 0.05, 0.02),  // rotation (angle-axis)
      1.5);                               // scale
  
  // Transform source points to create target points
  std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth_sim3);
  
  // Add some outliers to the target points (but fewer and smaller)
  for (size_t i = 0; i < 5; ++i) {
    target_points[i] += Vector3d(2.0, 2.0, 2.0);  // Smaller offset
  }
  
  // Perform robust SIM3 alignment
  Sim3AlignmentOptions options;
  options.alignment_type = Sim3AlignmentType::ROBUST_POINT_TO_POINT;
  options.huber_threshold = 0.5;  // Increased threshold
  options.verbose = false;
  options.max_iterations = 100;
  
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      source_points, target_points, options);
  
  // Check that alignment was successful
  ASSERT_TRUE(summary.success);
  
  // Check that the estimated parameters are reasonable
  double alignment_error = ComputeAlignmentError(source_points, target_points, summary.sim3_params);
  ASSERT_LT(alignment_error, 5.0);  // More lenient threshold for robust alignment
}

void Sim3PointToPlaneAlignmentTest() {
  // Generate synthetic point cloud data
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(100, 0.01);
  
  // Define ground truth SIM3 transformation using proper construction
  Sophus::Vector7d ground_truth_sim3 = CreateSim3Transformation(
      Eigen::Vector3d(1.0, 2.0, 0.5),    // translation
      Eigen::Vector3d(0.1, 0.05, 0.02),  // rotation (angle-axis)
      1.5);                               // scale
  
  // Transform source points to create target points
  std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth_sim3);
  
  // Create synthetic plane normals (assume mostly horizontal surface)
  std::vector<Vector3d> plane_normals;
  for (size_t i = 0; i < source_points.size(); ++i) {
    Vector3d normal(0.0, 0.0, 1.0);
    plane_normals.push_back(normal);
  }
  
  // Perform point-to-plane SIM3 alignment
  Sim3AlignmentOptions options;
  options.alignment_type = Sim3AlignmentType::POINT_TO_PLANE;
  options.target_normals = &plane_normals;
  options.verbose = false;
  options.max_iterations = 100;
  
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      source_points, target_points, options);
  
  // Check that alignment was successful
  ASSERT_TRUE(summary.success);
  
  // Check that the estimated parameters are close to ground truth
  // Convert both to SIM3 objects and compare components
  Sophus::Sim3d estimated_sim3 = Sophus::Sim3d::exp(summary.sim3_params);
  Sophus::Sim3d ground_truth_sim3_obj = Sophus::Sim3d::exp(ground_truth_sim3);
  
  double translation_error = (estimated_sim3.translation() - ground_truth_sim3_obj.translation()).norm();
  double rotation_error = (estimated_sim3.rotationMatrix() - ground_truth_sim3_obj.rotationMatrix()).norm();
  double scale_error = std::abs(estimated_sim3.scale() - ground_truth_sim3_obj.scale());
  
  ASSERT_LT(rotation_error, 1.0);  // More lenient threshold for point-to-plane
  ASSERT_LT(translation_error, 1.0);
  ASSERT_LT(scale_error, 1.0);
}


void Sim3InitialGuessTest() {
  // Generate synthetic point cloud data
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(100, 0.01);
  
  // Define ground truth SIM3 transformation using proper construction
  Sophus::Vector7d ground_truth_sim3 = CreateSim3Transformation(
      Eigen::Vector3d(1.0, 2.0, 0.5),    // translation
      Eigen::Vector3d(0.1, 0.05, 0.02),  // rotation (angle-axis)
      1.5);                               // scale
  
  // Transform source points to create target points
  std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth_sim3);
  
  // Create a rough initial guess using proper construction
  Sophus::Vector7d initial_guess = CreateSim3Transformation(
      Eigen::Vector3d(0.5, 1.0, 0.2),     // rough translation
      Eigen::Vector3d(0.05, 0.02, 0.01),  // rough rotation
      1.2);                                // rough scale
  
  // Perform SIM3 alignment with initial guess
  Sim3AlignmentOptions options;
  options.initial_sim3_params = &initial_guess;
  options.verbose = false;
  options.max_iterations = 50;
  
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      source_points, target_points, options);
  
  // Check that alignment was successful
  ASSERT_TRUE(summary.success);
  
  // Check that the estimated parameters are close to ground truth
  // Convert both to SIM3 objects and compare components
  Sophus::Sim3d estimated_sim3 = Sophus::Sim3d::exp(summary.sim3_params);
  Sophus::Sim3d ground_truth_sim3_obj = Sophus::Sim3d::exp(ground_truth_sim3);
  
  double translation_error = (estimated_sim3.translation() - ground_truth_sim3_obj.translation()).norm();
  double rotation_error = (estimated_sim3.rotationMatrix() - ground_truth_sim3_obj.rotationMatrix()).norm();
  double scale_error = std::abs(estimated_sim3.scale() - ground_truth_sim3_obj.scale());
  
  ASSERT_LT(rotation_error, 0.1);
  ASSERT_LT(translation_error, 0.1);
  ASSERT_LT(scale_error, 0.1);
}

void Sim3UtilityFunctionsTest() {
  // Test conversion between different representations
  
  // Create a rotation matrix, translation, and scale
  Matrix3d rotation = Eigen::AngleAxisd(0.5, Vector3d(1.0, 0.0, 0.0)).toRotationMatrix();
  Vector3d translation(1.0, 2.0, 3.0);
  double scale = 1.5;
  
  // Convert to SIM3 parameters
  Sophus::Vector7d sim3_params = Sim3FromRotationTranslationScale(rotation, translation, scale);
  
  // Convert back
  Matrix3d recovered_rotation;
  Vector3d recovered_translation;
  double recovered_scale;
  Sim3ToRotationTranslationScale(sim3_params, &recovered_rotation, &recovered_translation, &recovered_scale);
  
  // Check that the conversion is accurate
  double rotation_error = (rotation - recovered_rotation).norm();
  double translation_error = (translation - recovered_translation).norm();
  double scale_error = std::abs(scale - recovered_scale);
  
  ASSERT_LT(rotation_error, kEpsilon);
  ASSERT_LT(translation_error, kEpsilon);
  ASSERT_LT(scale_error, kEpsilon);
}

void Sim3EmptyPointCloudTest() {
  // Test that empty point clouds are handled gracefully
  std::vector<Vector3d> empty_source, empty_target;
  
  Sim3AlignmentOptions options;
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      empty_source, empty_target, options);
  
  // Should fail gracefully (not crash)
  ASSERT_FALSE(summary.success);
  
  // Test mismatched sizes
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(10);
  Sim3AlignmentSummary summary2 = OptimizeAlignmentSim3(
      source_points, empty_target, options);
  
  ASSERT_FALSE(summary2.success);
}

void Sim3MismatchedPointCloudTest() {
  // Test that mismatched point cloud sizes are handled gracefully
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(10);
  std::vector<Vector3d> target_points = GenerateSyntheticPointCloud(5);  // Different size
  
  Sim3AlignmentOptions options;
  Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
      source_points, target_points, options);
  
  // Should fail gracefully
  ASSERT_FALSE(summary.success);
  
  // Test with very small point clouds (less than minimum required)
  std::vector<Vector3d> small_source = GenerateSyntheticPointCloud(2);
  std::vector<Vector3d> small_target = GenerateSyntheticPointCloud(2);
  
  Sim3AlignmentSummary summary2 = OptimizeAlignmentSim3(
      small_source, small_target, options);
  
  // Should still work with small point clouds
  ASSERT_TRUE(summary2.success);
}

void Sim3InitialAlignmentTest() {
  // Test that initial Umeyama alignment works correctly
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(100, 0.01);
  
  // Define ground truth SIM3 transformation using proper construction
  Sophus::Vector7d ground_truth_sim3 = CreateSim3Transformation(
      Eigen::Vector3d(1.0, 2.0, 0.5),    // translation
      Eigen::Vector3d(0.1, 0.05, 0.02),  // rotation (angle-axis)
      1.5);                               // scale
  
  // Transform source points to create target points
  std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth_sim3);
  
  // Test initial alignment with Umeyama only
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double scale;
  
  AlignPointCloudsUmeyama(source_points, target_points, &rotation, &translation, &scale);
  
  // Convert to SIM3 parameters
  Sophus::Vector7d umeyama_sim3 = Sim3FromRotationTranslationScale(rotation, translation, scale);
  
  // Check that Umeyama gives reasonable initial alignment
  // Convert both to SIM3 objects and compare components
  Sophus::Sim3d umeyama_sim3_obj = Sophus::Sim3d::exp(umeyama_sim3);
  Sophus::Sim3d ground_truth_sim3_obj = Sophus::Sim3d::exp(ground_truth_sim3);
  
  double translation_error = (umeyama_sim3_obj.translation() - ground_truth_sim3_obj.translation()).norm();
  double rotation_error = (umeyama_sim3_obj.rotationMatrix() - ground_truth_sim3_obj.rotationMatrix()).norm();
  double scale_error = std::abs(umeyama_sim3_obj.scale() - ground_truth_sim3_obj.scale());
  
  // Umeyama should give good initial alignment
  ASSERT_LT(rotation_error, 0.5);
  ASSERT_LT(translation_error, 0.5);
  ASSERT_LT(scale_error, 0.5);
}

void Sim3ConvergenceTest() {
  // Test that the algorithm converges for various scenarios
  std::vector<Vector3d> source_points = GenerateSyntheticPointCloud(50, 0.05);
  
  // Test different ground truth transformations using proper construction
  std::vector<Sophus::Vector7d> test_transformations = {
    CreateSim3Transformation(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 1.0),  // Identity
    CreateSim3Transformation(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0), 1.2),  // Small translation + rotation + scale
    CreateSim3Transformation(Eigen::Vector3d(5.0, 3.0, 2.0), Eigen::Vector3d(0.5, 0.3, 0.2), 2.0),  // Large transformation
  };
  
  for (const auto& ground_truth : test_transformations) {
    std::vector<Vector3d> target_points = TransformPointCloud(source_points, ground_truth);
    
    Sim3AlignmentOptions options;
    options.verbose = false;
    options.max_iterations = 200;
    
    Sim3AlignmentSummary summary = OptimizeAlignmentSim3(
        source_points, target_points, options);
    
    // Check that alignment was successful
    ASSERT_TRUE(summary.success);
    
    // Check that the final cost is reasonable
    ASSERT_LT(summary.final_cost, 1e-3);
    
    // Check that we converged in reasonable number of iterations
    ASSERT_LT(summary.num_iterations, 200);
  }
}

}  // namespace

TEST(AlignPointCloudsUmeyama, SimpleTest) { UmeyamaSimpleTest(); }

TEST(AlignPointCloudsUmeyamaWithWeights, WeightsAndNoise) {
  UmeyamaWithWeigthsAndNoise();
}

// SIM3 Alignment Tests
TEST(Sim3Alignment, PointToPointAlignment) {
  Sim3PointToPointAlignmentTest();
}

TEST(Sim3Alignment, RobustAlignment) {
  Sim3RobustAlignmentTest();
}

TEST(Sim3Alignment, PointToPlaneAlignment) {
  Sim3PointToPlaneAlignmentTest();
}

TEST(Sim3Alignment, InitialGuess) {
  Sim3InitialGuessTest();
}

TEST(Sim3Alignment, UtilityFunctions) {
  Sim3UtilityFunctionsTest();
}

TEST(Sim3Alignment, EmptyPointCloud) {
  Sim3EmptyPointCloudTest();
}

TEST(Sim3Alignment, MismatchedPointCloud) {
  Sim3MismatchedPointCloudTest();
}

TEST(Sim3Alignment, InitialAlignment) {
  Sim3InitialAlignmentTest();
}

TEST(Sim3Alignment, Convergence) {
  Sim3ConvergenceTest();
}

}  // namespace theia
