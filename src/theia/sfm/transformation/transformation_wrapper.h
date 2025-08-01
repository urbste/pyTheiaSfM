#pragma once

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/transformation/align_point_clouds.h"
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

// SIM3 Point Cloud Alignment Wrappers
Sim3AlignmentSummary OptimizeAlignmentSim3Wrapper(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points,
    const Sim3AlignmentOptions& options = Sim3AlignmentOptions());

Sophus::Vector7d Sim3FromRotationTranslationScaleWrapper(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation,
    double scale);

void Sim3ToRotationTranslationScaleWrapper(
    const Sophus::Vector7d& sim3_params,
    Eigen::Matrix3d* rotation,
    Eigen::Vector3d* translation,
    double* scale);

// Wrapper that returns homogeneous transformation matrix
Eigen::Matrix4d Sim3ToHomogeneousMatrixWrapper(
    const Sophus::Vector7d& sim3_params);

}  // namespace theia
