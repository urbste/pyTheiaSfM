#include "theia/math/math_wrapper.h"
#include "theia/math/closed_form_polynomial_solver.h"
#include "theia/math/polynomial.h"

namespace theia {

std::unordered_map<ViewId, Eigen::Vector3d> AlignOrientationsWrapper(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    const std::unordered_map<ViewId, Eigen::Vector3d>& rotations) {
  std::unordered_map<ViewId, Eigen::Vector3d> rot_aligned = rotations;
  theia::AlignOrientations(gt_rotations, &rot_aligned);

  return rot_aligned;
}

// Implementation for SolveQuadraticRealsWrapper
int SolveQuadraticRealsWrapper(const double a, const double b, const double c, 
                             const double tolerance, Eigen::VectorXd& roots) {
  int num_real_roots = 0;
  double real_roots[2];
  num_real_roots = SolveQuadraticReals(a, b, c, tolerance, real_roots);
  
  // Copy output to Eigen vector
  roots.resize(num_real_roots);
  for (int i = 0; i < num_real_roots; ++i) {
    roots(i) = real_roots[i];
  }
  
  return num_real_roots;
}

// Implementation for SolveCubicRealsWrapper
int SolveCubicRealsWrapper(const double a, const double b, const double c, const double d,
                         const double tolerance, Eigen::VectorXd& roots) {
  int num_real_roots = 0;
  double real_roots[3];
  num_real_roots = SolveCubicReals(a, b, c, d, tolerance, real_roots);
  
  // Copy output to Eigen vector
  roots.resize(num_real_roots);
  for (int i = 0; i < num_real_roots; ++i) {
    roots(i) = real_roots[i];
  }
  
  return num_real_roots;
}

// Implementation for SolveQuarticRealsWrapper
int SolveQuarticRealsWrapper(const long double a, const long double b, const long double c, 
                           const long double d, const long double e,
                           const long double tolerance, Eigen::VectorXd& roots) {
  int num_real_roots = 0;
  long double real_roots[4];
  num_real_roots = SolveQuarticReals(a, b, c, d, e, tolerance, real_roots);
  
  // Copy output to Eigen vector
  roots.resize(num_real_roots);
  for (int i = 0; i < num_real_roots; ++i) {
    roots(i) = real_roots[i];
  }
  
  return num_real_roots;
}

// Implementation for SolveQuadraticWrapper
int SolveQuadraticWrapper(const double a, const double b, const double c, 
                        Eigen::VectorXcd& roots) {
  std::complex<double> complex_roots[2];
  int num_roots = SolveQuadratic(a, b, c, complex_roots);
  
  // Copy output to Eigen vector
  roots.resize(num_roots);
  for (int i = 0; i < num_roots; ++i) {
    roots(i) = complex_roots[i];
  }
  
  return num_roots;
}

// Implementation for DividePolynomialWrapper
void DividePolynomialWrapper(const Eigen::VectorXd& dividend,
                           const Eigen::VectorXd& divisor,
                           Eigen::VectorXd& quotient,
                           Eigen::VectorXd& remainder) {
  DividePolynomial(dividend, divisor, &quotient, &remainder);
}

// Implementation for MinimizePolynomialWrapper
void MinimizePolynomialWrapper(const Eigen::VectorXd& polynomial,
                             const double min_x,
                             const double max_x,
                             double& optimal_x,
                             double& optimal_value) {
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);
}

}  // namespace theia