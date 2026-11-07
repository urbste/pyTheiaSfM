// Copyright (C) 2026 pyTheia contributors.
// Released under the same BSD-style license as TheiaSfM (see LICENSE).

// Prior on the 3D position of a track (e.g. a ground control point or a known
// LiDAR point). The residual is

//   r = sqrt_information * (X - prior),

// where X = (x, y, z) is the Euclidean point obtained from the homogeneous
// 4-vector parameter (x, y, z, w) as X = (x, y, z) / w. The sqrt information S
// satisfies S^T S = Sigma^-1 for a position covariance Sigma.

#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_POINT_PRIOR_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_POINT_PRIOR_ERROR_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace theia {

struct PointPriorError {
 public:
  PointPriorError(const Eigen::Vector3d& position,
                  const Eigen::Matrix3d& sqrt_information)
      : position_(position), sqrt_information_(sqrt_information) {}

  // point is the homogeneous 4-vector (x, y, z, w) of the track.
  template <typename T>
  bool operator()(const T* const point, T* residual) const {
    static const double kVerySmallNumber = 1e-12;
    const T w = point[3];
    // A point at (or very near) infinity has no finite Euclidean position to
    // compare with the prior. Failing the evaluation lets Ceres reject the step.
    if (ceres::abs(w) < T(kVerySmallNumber)) {
      return false;
    }

    T diff[3];
    for (int i = 0; i < 3; ++i) {
      diff[i] = point[i] / w - T(position_(i));
    }

    for (int i = 0; i < 3; ++i) {
      T weighted = T(0.0);
      for (int j = 0; j < 3; ++j) {
        weighted += T(sqrt_information_(i, j)) * diff[j];
      }
      residual[i] = weighted;
    }
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& position,
                                     const Eigen::Matrix3d& sqrt_information) {
    static const int kPointSize = 4;
    static const int kNumResiduals = 3;
    return new ceres::AutoDiffCostFunction<PointPriorError,
                                           kNumResiduals,
                                           kPointSize>(
        new PointPriorError(position, sqrt_information));
  }

 private:
  const Eigen::Vector3d position_;
  const Eigen::Matrix3d sqrt_information_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_POINT_PRIOR_ERROR_H_
