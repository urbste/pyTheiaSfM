#pragma once

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/transformation/align_point_clouds.h"
#include "theia/sfm/transformation/cross_reconstruction_pose_graph_types.h"
#include "theia/sfm/transformation/cross_reconstruction_sim3_pose_graph_optimizer.h"
#include "theia/sfm/view.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

namespace theia {

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
AlignPointCloudsUmeyamaWrapper(const std::vector<Eigen::Vector3d>& left,
                               const std::vector<Eigen::Vector3d>& right);

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
AlignPointCloudsUmeyamaWithWeightsWrapper(
    const std::vector<Eigen::Vector3d>& left,
    const std::vector<Eigen::Vector3d>& right,
    const std::vector<double>& weights);

std::tuple<std::vector<Eigen::Vector4d>,
           std::vector<Eigen::Vector3d>,
           std::vector<double>>
GdlsSimilarityTransformWrapper(
    const std::vector<Eigen::Vector3d>& ray_origin,
    const std::vector<Eigen::Vector3d>& ray_direction,
    const std::vector<Eigen::Vector3d>& world_point);

std::vector<Eigen::Vector3d> AlignRotationsWrapper(
    const std::vector<Eigen::Vector3d>& gt_rotation);

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
AlignReconstructionsWrapper(const Reconstruction& fixed_reconstruction,
                            Reconstruction& variable_reconstruction);

std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>
AlignReconstructionsRobustWrapper(const double robust_error_threshold,
                                  const Reconstruction& fixed_reconstruction,
                                  Reconstruction& variable_reconstruction);

void TransformReconstructionWrapper(Reconstruction& reconstruction,
                                    const Eigen::Matrix3d& rotation,
                                    const Eigen::Vector3d& translation,
                                    const double scale);

void TransformReconstructionWrapper4(Reconstruction& reconstruction,
                                    const Eigen::Matrix4d& transformation);

// SIM3 Point Cloud Alignment Wrappers
Sim3AlignmentSummary OptimizeAlignmentSim3Wrapper(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points,
    const Sim3AlignmentOptions& options = Sim3AlignmentOptions());

Sophus::Vector7d Sim3FromRotationTranslationScaleWrapper(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation,
    double scale);

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double> Sim3ToRotationTranslationScaleWrapper(
    const Sophus::Vector7d& sim3_params);

// Wrapper that returns homogeneous transformation matrix
Eigen::Matrix4d Sim3ToHomogeneousMatrixWrapper(
    const Sophus::Vector7d& sim3_params);

// Cross-reconstruction Sim(3) pose graph alignment
std::pair<bool, CrossReconstructionPoseGraphSummary>
AlignReconstructionsWithPoseGraphWrapper(
    const Reconstruction& fixed_reconstruction,
    Reconstruction& variable_reconstruction,
    const CrossReconstructionConstraints& constraints,
    const CrossReconstructionPoseGraphOptions& options,
    bool apply_to_variable_reconstruction);

Sophus::Vector7d GetSim3LieFromViewWrapper(const View& view);

Sophus::Vector7d RelativeSim3BetweenViewsWrapper(const View& view_i,
                                                 const View& view_j);

std::pair<bool, CrossReconstructionPoseGraphSummary>
CrossReconstructionSim3PoseGraphOptimizerOptimizeWrapper(
    CrossReconstructionSim3PoseGraphOptimizer& optimizer);

}  // namespace theia
