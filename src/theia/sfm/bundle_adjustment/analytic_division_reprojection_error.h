#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_DIVISION_REPROJECTION_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_DIVISION_REPROJECTION_ERROR_H_

// EXPERIMENTAL: hand-derived Jacobians for the homogeneous-point reprojection
// residual with the DIVISION_UNDISTORTION camera model. Residual and parameter
// layout are identical to ReprojectionError<DivisionUndistortionCameraModel>:
//   block 0: extrinsics [position(3), angle-axis(3)]
//   block 1: intrinsics [f, aspect, cx, cy, k]
//   block 2: homogeneous point [X, Y, Z, w]  (ambient Jacobian; Ceres applies
//            the SphereManifold<4> PlusJacobian itself).

#include <Eigen/Core>
#include <ceres/rotation.h>
#include <ceres/sized_cost_function.h>

#include <cmath>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/feature.h"

namespace theia {

class AnalyticDivisionReprojectionError
    : public ceres::SizedCostFunction<2, 6, 5, 4> {
 public:
  explicit AnalyticDivisionReprojectionError(const Feature& feature)
      : feature_(feature) {}
  ~AnalyticDivisionReprojectionError() override {}

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    typedef Eigen::Matrix<double, 3, 3> M3;
    typedef Eigen::Matrix<double, 3, 1> V3;
    const double* ext = parameters[0];
    const double* K = parameters[1];
    const double* P = parameters[2];

    const V3 center(ext[Camera::POSITION], ext[Camera::POSITION + 1],
                    ext[Camera::POSITION + 2]);
    const double w = P[3];
    const V3 p = V3(P[0], P[1], P[2]) - w * center;
    if (p.squaredNorm() < 1e-8) return false;

    const double* aa = ext + Camera::ORIENTATION;
    M3 R;  // column-major, as ceres::AngleAxisToRotationMatrix writes.
    ceres::AngleAxisToRotationMatrix(aa, R.data());
    const V3 q = R * p;

    const double f = K[0], a = K[1], cx = K[2], cy = K[3], k = K[4];
    const double inv_z = 1.0 / q[2];
    const double nx = q[0] * inv_z, ny = q[1] * inv_z;
    const double ux = f * nx, uy = f * a * ny;
    const double r2 = ux * ux + uy * uy;
    const double x = 4.0 * k * r2;
    const double inner = 1.0 - x;

    // s(x) = 2 (1 - sqrt(1 - x)) / x ; the model returns the identity when
    // the discriminant is negative (same convention as the autodiff path).
    double s, ds;
    bool identity = false;
    if (inner < 0.0 || std::abs(2.0 * k * r2) < 1e-300) {
      identity = true;
      s = 1.0;
      ds = 0.0;
    } else if (std::abs(x) < 1e-3) {
      s = 1.0 + x * (0.25 + x * (0.125 + x * (5.0 / 64.0 + x * 7.0 / 128.0)));
      ds = 0.25 + x * (0.25 + x * (15.0 / 64.0 + x * (7.0 / 32.0)));
    } else {
      const double S = std::sqrt(inner);
      s = 2.0 * (1.0 - S) / x;
      ds = (x / S - 2.0 * (1.0 - S)) / (x * x);
    }

    const double dx_obs_x = s * ux + cx - feature_.point_.x();
    const double dy_obs_y = s * uy + cy - feature_.point_.y();
    const Eigen::Matrix2d si = feature_.SqrtInformation();
    residuals[0] = si(0, 0) * dx_obs_x + si(0, 1) * dy_obs_y;
    residuals[1] = si(1, 0) * dx_obs_x + si(1, 1) * dy_obs_y;

    if (jacobians == nullptr) return true;

    // d pixel / d q (2x3).
    Eigen::Matrix<double, 2, 2> dd_du;  // d distorted / d undistorted
    if (identity) {
      dd_du.setIdentity();
    } else {
      const double c = ds * 8.0 * k;
      dd_du(0, 0) = s + c * ux * ux;
      dd_du(0, 1) = c * ux * uy;
      dd_du(1, 0) = dd_du(0, 1);
      dd_du(1, 1) = s + c * uy * uy;
    }
    Eigen::Matrix<double, 2, 3> du_dq;
    du_dq << f * inv_z, 0.0, -f * nx * inv_z,
        0.0, f * a * inv_z, -f * a * ny * inv_z;
    // Whitened pixel Jacobian wrt camera-frame point.
    const Eigen::Matrix<double, 2, 3> Jq = si * (dd_du * du_dq);

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      // position: dq/dc = -w R
      J.block<2, 3>(0, Camera::POSITION) = (-w) * (Jq * R);
      // orientation: dq/dw = -R [p]x Jr(w)
      const double th2 = aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2];
      double A, B;
      if (th2 < 1e-8) {
        A = 0.5 - th2 / 24.0;
        B = 1.0 / 6.0 - th2 / 120.0;
      } else {
        const double th = std::sqrt(th2);
        A = (1.0 - std::cos(th)) / th2;
        B = (th - std::sin(th)) / (th2 * th);
      }
      M3 W;
      W << 0, -aa[2], aa[1], aa[2], 0, -aa[0], -aa[1], aa[0], 0;
      M3 Jr = M3::Identity() - A * W + B * (W * W);
      M3 Px;
      Px << 0, -p[2], p[1], p[2], 0, -p[0], -p[1], p[0], 0;
      J.block<2, 3>(0, Camera::ORIENTATION) = -(Jq * R) * (Px * Jr);
    }
    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J(jacobians[1]);
      Eigen::Vector2d du_df(nx, a * ny), du_da(0.0, f * ny);
      J.col(0) = si * (dd_du * du_df);
      J.col(1) = si * (dd_du * du_da);
      J.col(2) = si.col(0);
      J.col(3) = si.col(1);
      // d distorted / d k = u * ds * 4 r^2  (identity branch: 0)
      const double dk = identity ? 0.0 : ds * 4.0 * r2;
      J.col(4) = si * Eigen::Vector2d(ux * dk, uy * dk);
    }
    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[2]);
      const Eigen::Matrix<double, 2, 3> JqR = Jq * R;
      J.block<2, 3>(0, 0) = JqR;
      J.col(3) = -(JqR * center);
    }
    return true;
  }

 private:
  const Feature feature_;
};

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_DIVISION_REPROJECTION_ERROR_H_
