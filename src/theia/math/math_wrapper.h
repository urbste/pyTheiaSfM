#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <complex>

#include "theia/math/rotation.h"

namespace theia {

std::unordered_map<ViewId, Eigen::Vector3d> AlignOrientationsWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& rotations);

// Wrapper for SolveQuadraticReals that takes an Eigen vector output argument
int SolveQuadraticRealsWrapper(const double a, const double b, const double c, 
                             const double tolerance, Eigen::VectorXd& roots);

// Wrapper for SolveCubicReals that takes an Eigen vector output argument
int SolveCubicRealsWrapper(const double a, const double b, const double c, const double d,
                         const double tolerance, Eigen::VectorXd& roots);

// Wrapper for SolveQuarticReals that takes an Eigen vector output argument
int SolveQuarticRealsWrapper(const long double a, const long double b, const long double c, 
                           const long double d, const long double e,
                           const long double tolerance, Eigen::VectorXd& roots);

// Wrapper for SolveQuadratic that takes an Eigen vector output argument
int SolveQuadraticWrapper(const double a, const double b, const double c, 
                        Eigen::VectorXcd& roots);

// Wrapper for DividePolynomial that takes Eigen vectors
void DividePolynomialWrapper(const Eigen::VectorXd& dividend,
                           const Eigen::VectorXd& divisor,
                           Eigen::VectorXd& quotient,
                           Eigen::VectorXd& remainder);

// Wrapper for MinimizePolynomial that takes Eigen vectors
void MinimizePolynomialWrapper(const Eigen::VectorXd& polynomial,
                             const double min_x,
                             const double max_x,
                             double& optimal_x,
                             double& optimal_value);

}  // namespace theia