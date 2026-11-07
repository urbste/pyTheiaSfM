// Copyright (C) 2026
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@googlemail.com)

#ifndef THEIA_SFM_CAMERA_DISTORTION_INVERSION_H_
#define THEIA_SFM_CAMERA_DISTORTION_INVERSION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/jet.h>
#include <cmath>
#include <limits>

namespace theia {
namespace internal {

// Returns the scalar value of a double or of a ceres::Jet (its real part).
inline double ScalarValue(const double x) { return x; }

template <typename S, int N>
inline double ScalarValue(const ceres::Jet<S, N>& x) {
  return ScalarValue(x.a);
}

// Fisheye (equidistant) projection used by the COLMAP fisheye models: a
// normalized point (u, v) = r * (cos phi, sin phi) with r = |(u, v)| is mapped
// to the angle-scaled point theta * (cos phi, sin phi) with theta = atan(r).
template <typename T>
inline void FisheyeFromNormal(const T u, const T v, T* uu, T* vv) {
  const T r_sq = u * u + v * v;
  if (ScalarValue(r_sq) > std::numeric_limits<double>::epsilon()) {
    const T r = ceres::sqrt(r_sq);
    const T scale = ceres::atan(r) / r;
    *uu = u * scale;
    *vv = v * scale;
  } else {
    *uu = u;
    *vv = v;
  }
}

// Inverse of FisheyeFromNormal: maps an angle-scaled point to a normalized
// point using u = tan(theta) * dir, i.e. the factor tan(theta) / theta.
template <typename T>
inline void NormalFromFisheye(const T uu, const T vv, T* u, T* v) {
  const T theta_sq = uu * uu + vv * vv;
  if (ScalarValue(theta_sq) > std::numeric_limits<double>::epsilon()) {
    const T theta = ceres::sqrt(theta_sq);
    const T scale = ceres::tan(theta) / theta;
    *u = uu * scale;
    *v = vv * scale;
  } else {
    *u = uu;
    *v = vv;
  }
}

// Inverts a 2D distortion map. Model must provide a static, templated function
//
//   void Model::ApplyDistortion(const T* extra_params, T u, T v, T* x, T* y);
//
// that maps the undistorted coordinates (u, v) to the distorted coordinates
// (x, y). This function solves ApplyDistortion(extra_params, source) = target
// with Newton's method. The Jacobian is evaluated with ceres::Jet in double
// precision. Once converged, one further Newton step is taken in T so that the
// returned value carries the exact implicit-function derivatives with respect
// to both the distortion parameters and the target point (this is what allows
// autodiff through the inverse).
//
// Returns false if the Jacobian is singular or if Newton does not converge.
template <typename Model, int kNumExtraParams, typename T>
bool InvertDistortion(const T* extra_params, const T* target, T* source) {
  constexpr int kMaxIterations = 100;
  constexpr double kResidualTolerance = 1e-12;
  constexpr double kConvergenceTolerance = 1e-8;
  constexpr double kSingularTolerance = 1e-14;

  double params[kNumExtraParams];
  for (int i = 0; i < kNumExtraParams; ++i) {
    params[i] = ScalarValue(extra_params[i]);
  }
  const Eigen::Vector2d target_value(ScalarValue(target[0]),
                                     ScalarValue(target[1]));

  // Residual r(x) = D(x) - target and its 2x2 Jacobian at x.
  auto evaluate = [&params, &target_value](const Eigen::Vector2d& x,
                                           Eigen::Vector2d* residual,
                                           Eigen::Matrix2d* jacobian) {
    ceres::Jet<double, 2> jet_params[kNumExtraParams];
    for (int i = 0; i < kNumExtraParams; ++i) {
      jet_params[i] = ceres::Jet<double, 2>(params[i]);
    }
    const ceres::Jet<double, 2> u(x.x(), 0);
    const ceres::Jet<double, 2> v(x.y(), 1);
    ceres::Jet<double, 2> dx, dy;
    Model::ApplyDistortion(jet_params, u, v, &dx, &dy);
    *residual << dx.a - target_value.x(), dy.a - target_value.y();
    (*jacobian)(0, 0) = dx.v[0];
    (*jacobian)(0, 1) = dx.v[1];
    (*jacobian)(1, 0) = dy.v[0];
    (*jacobian)(1, 1) = dy.v[1];
  };

  // Distortion is close to the identity for realistic coefficients, so the
  // target is a good initial guess.
  Eigen::Vector2d x = target_value;
  Eigen::Vector2d residual;
  Eigen::Matrix2d jacobian;
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    evaluate(x, &residual, &jacobian);
    if (residual.norm() < kResidualTolerance) {
      break;
    }
    if (!(std::abs(jacobian.determinant()) > kSingularTolerance)) {
      return false;
    }
    x -= jacobian.partialPivLu().solve(residual);
  }
  evaluate(x, &residual, &jacobian);
  if (!(residual.norm() < kConvergenceTolerance)) {
    return false;
  }
  if (!(std::abs(jacobian.determinant()) > kSingularTolerance)) {
    return false;
  }

  // One Newton step in T: the value change is ~0 but the derivatives are the
  // implicit-function derivatives of the inverse.
  const Eigen::Matrix2d inverse = jacobian.inverse();
  const T ux = T(x.x());
  const T uy = T(x.y());
  T dx, dy;
  Model::ApplyDistortion(extra_params, ux, uy, &dx, &dy);
  const T rx = dx - target[0];
  const T ry = dy - target[1];
  source[0] = ux - (inverse(0, 0) * rx + inverse(0, 1) * ry);
  source[1] = uy - (inverse(1, 0) * rx + inverse(1, 1) * ry);
  return true;
}

}  // namespace internal
}  // namespace theia

#endif  // THEIA_SFM_CAMERA_DISTORTION_INVERSION_H_
