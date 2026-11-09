// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/camera_sampler.h"

#include <Eigen/Geometry>
#include <cmath>

#include "theia/util/random.h"

namespace theia {

OseCameraVector SampleInitialCameras(const int num_cameras,
                                     const InitFreeCameraInitType type,
                                     const unsigned int seed) {
  RandomNumberGenerator rng(seed);
  OseCameraVector cameras(num_cameras);
  for (int i = 0; i < num_cameras; ++i) {
    OseCamera& P = cameras[i];
    switch (type) {
      case InitFreeCameraInitType::NORMAL:
        for (int k = 0; k < 12; ++k) {
          P(k / 4, k % 4) = rng.RandGaussian(0.0, 1.0);
        }
        break;
      case InitFreeCameraInitType::UNIFORM:
        for (int k = 0; k < 12; ++k) {
          P(k / 4, k % 4) = rng.RandDouble(-1.0, 1.0);
        }
        break;
      case InitFreeCameraInitType::UNIT_CIRCLE: {
        const double angle =
            2.0 * M_PI * i / std::max(1, num_cameras) +
            rng.RandGaussian(0.0, 0.05);
        const Eigen::Vector3d center(std::cos(angle), 0.0, std::sin(angle));
        // Optical axis toward the origin, y axis as the up direction.
        const Eigen::Vector3d z_axis = -center.normalized();
        const Eigen::Vector3d x_axis =
            Eigen::Vector3d::UnitY().cross(z_axis).normalized();
        const Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        Eigen::Matrix3d R;
        R.row(0) = x_axis.transpose();
        R.row(1) = y_axis.transpose();
        R.row(2) = z_axis.transpose();
        P.leftCols<3>() = R;
        P.col(3) = -R * center;
        break;
      }
    }
  }
  return cameras;
}

}  // namespace theia
