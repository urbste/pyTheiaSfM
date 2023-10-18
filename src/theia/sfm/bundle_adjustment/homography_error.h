// Please contact the author of this library if you have any questions.
// Author: Steffen Urban

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_HOMOGRAPHY_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_HOMOGRAPHY_ERROR_H_

#include <ceres/ceres.h>

namespace theia {

// A parameterization of the 2D homography matrix that uses 8 parameters so
// that the matrix is normalized (H(2,2) == 1).
// The homography matrix H is built from a list of 8 parameters (a, b,...g, h)
// as follows
//
//         |a b c|
//     H = |d e f|
//         |g h 1|
//
template <typename T = double>
class Homography2DNormalizedParameterization {
 public:
  using Parameters = Eigen::Matrix<T, 8, 1>;     // a, b, ... g, h
  using Parameterized = Eigen::Matrix<T, 3, 3>;  // H

  // Convert from the 8 parameters to a H matrix.
  static void To(const Parameters& p, Parameterized* h) {
    // clang-format off
    *h << p(0), p(1), p(2),
          p(3), p(4), p(5),
          p(6), p(7), 1.0;
    // clang-format on
  }

  // Convert from a H matrix to the 8 parameters.
  static void From(const Parameterized& h, Parameters* p) {
    // clang-format off
    *p << h(0, 0), h(0, 1), h(0, 2),
          h(1, 0), h(1, 1), h(1, 2),
          h(2, 0), h(2, 1);
    // clang-format on
  }
};

// Calculate symmetric geometric cost terms:
//
// forward_error = D(H * x1, x2)
// backward_error = D(H^-1 * x2, x1)
//
// Templated to be used with autodifferentiation.
template <typename T>
void SymmetricGeometricDistanceTerms(const Eigen::Matrix<T, 3, 3>& H,
                                     const Eigen::Matrix<T, 2, 1>& x1,
                                     const Eigen::Matrix<T, 2, 1>& x2,
                                     T forward_error[2],
                                     T backward_error[2]) {
  using Vec3 = Eigen::Matrix<T, 3, 1>;
  Vec3 x(x1(0), x1(1), T(1.0));
  Vec3 y(x2(0), x2(1), T(1.0));

  Vec3 H_x = H * x;
  Vec3 Hinv_y = H.inverse() * y;

  H_x /= H_x(2);
  Hinv_y /= Hinv_y(2);

  forward_error[0] = H_x(0) - y(0);
  forward_error[1] = H_x(1) - y(1);
  backward_error[0] = Hinv_y(0) - x(0);
  backward_error[1] = Hinv_y(1) - x(1);
}

// Cost functor which computes symmetric geometric distance
// used for homography matrix refinement.
class HomographySymmetricGeometricCostFunctor {
 public:
  HomographySymmetricGeometricCostFunctor(
    const Eigen::Vector2d& x, const Eigen::Vector2d& y)
      : x_(std::move(x)), y_(std::move(y)) {}

  template <typename T>
  bool operator()(const T* homography_parameters, T* residuals) const {
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Vec2 = Eigen::Matrix<T, 2, 1>;

    Mat3 H(homography_parameters);
    Vec2 x(T(x_(0)), T(x_(1)));
    Vec2 y(T(y_(0)), T(y_(1)));

    SymmetricGeometricDistanceTerms<T>(H, x, y, &residuals[0], &residuals[2]);
    return true;
  }

  const Eigen::Vector2d x_;
  const Eigen::Vector2d y_;
};

}

#endif // THEIA_SFM_BUNDLE_ADJUSTMENT_HOMOGRAPHY_ERROR_H_