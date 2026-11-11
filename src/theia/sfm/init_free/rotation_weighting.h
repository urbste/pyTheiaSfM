// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_ROTATION_WEIGHTING_H_
#define THEIA_SFM_INIT_FREE_ROTATION_WEIGHTING_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

namespace theia {

struct TwoViewRotationWeight {
  // Square root L of the 9x9 weight W (L = L^T, L L = W), acting on the
  // row-major vec of A_a A_b^T - R_ab.
  Eigen::Matrix<double, 9, 9> sqrt_weight = Eigen::Matrix<double, 9, 9>::Zero();
  // Reprojection information of the relative rotation (3x3, translation and
  // points marginalized), in the units of the observations.
  Eigen::Matrix3d rotation_information = Eigen::Matrix3d::Zero();
  // Unit translation used (estimated from the epipolar constraint if none was
  // given) and the number of correspondences in front of both cameras.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  int num_points = 0;
};

// The rotation weight of Olsson & Nilsson ("Towards Initialization-free
// Calibrated Bundle Adjustment", arXiv:2506.23808, Eq. 4) for one view pair.
//
// Two-view setup: camera b is [I | 0] and camera a is [R_ab | t_ab], so a point
// X_b maps to X_a = R_ab X_b + t_ab. m_a / m_b are normalized observations of
// the same points. If t_ab is (near) zero, its direction is estimated from the
// epipolar constraint m_a^T [t]x R_ab m_b = 0 and the cheirality of the
// triangulated points. Points are triangulated with the midpoint method.
//
// With J the reprojection Jacobian with respect to a left rotation
// perturbation R_ab <- exp([w]x) R_ab and K the Jacobian with respect to the
// translation and the points, the rotation information is
// H = J^T (I - K K^+) J. In the orthonormal basis V of R^9 made of the tangent
// directions vec([e_i]x R_ab) / sqrt(2) and six normal directions vec(S R_ab)
// (S symmetric),
//
//   W = V diag(H / 2, normal_weight * I_6) V^T,
//
// so that for a tangent step vec(A_a A_b^T - R_ab) ~ vec([w]x R_ab) the term
// equals w^T H w, the Gauss-Newton increase of the two-view reprojection
// error. Unlike InitFreeBA (pixel units, an extra factor 1/3), H is in the
// units of the observations, i.e. those of the OSE residuals.
//
// Returns false if fewer than min_points correspondences are in front of both
// cameras.
bool ComputeTwoViewRotationWeight(
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>&
        m_a,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>&
        m_b,
    const Eigen::Matrix3d& R_ab,
    const Eigen::Vector3d& t_ab,
    double normal_weight,
    int min_points,
    TwoViewRotationWeight* weight);

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_ROTATION_WEIGHTING_H_
