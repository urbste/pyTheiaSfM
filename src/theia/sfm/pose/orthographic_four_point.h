// Copyright (C) 2022
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com
#ifndef THEIA_SFM_POSE_ORTHOGRAPHIC_FOUR_POINT_H_
#define THEIA_SFM_POSE_ORTHOGRAPHIC_FOUR_POINT_H_

#include <Eigen/Core>
#include <vector>
#include "theia/sfm/estimators/feature_correspondence_2d_3d.h"

namespace theia {

bool PlanarUncalibratedOrthographicPose(
        const std::vector<FeatureCorrespondence2D3D>& correspondeces,
        const Eigen::Vector2d &principal_point,
        std::vector<Eigen::Matrix3d>* solution_rotations,
        std::vector<Eigen::Vector3d>* solution_translations,
        double* magnification);

bool PlanarUncalibratedOrthographicPose(
        const std::vector<Eigen::Vector2d>& feature_point,
        const std::vector<Eigen::Vector3d>& world_point,
        const Eigen::Vector2d &principal_point,
        std::vector<Eigen::Matrix3d>* solution_rotations,
        std::vector<Eigen::Vector3d>* solution_translations,
        double* magnification);

}  // namespace theia

#endif  // THEIA_SFM_POSE_ORTHOGRAPHIC_FOUR_POINT_H_
