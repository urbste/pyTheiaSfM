#ifndef THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_REPROJECTION_ERROR_H_
#define THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_REPROJECTION_ERROR_H_

// Hand-derived Jacobians for the reprojection residuals of ReprojectionError<>
// and InvReprojectionError<> (theia/sfm/camera/reprojection_error.h). The
// residuals, parameter layouts and invalid-point handling are identical to the
// autodiff versions; only the derivatives are computed analytically.
//
// Structure:
//   * AnalyticModel<CameraModel>::Project maps a camera-frame point q and the
//     intrinsics K to the (unwhitened) pixel together with d pixel / d q (2x3)
//     and d pixel / d K (2xN). One specialization per camera model; most are
//     built from a small "distortion policy" (distortion of the normalized
//     point, of the fisheye angle-scaled point, or of the full 3D ray) plus the
//     shared calibration step (focal, aspect, skew, principal point).
//   * AnalyticReprojectionError<CameraModel>: homogeneous point (XYZ) residual
//     SizedCostFunction<2, 6, N, 4>. The point Jacobian is the ambient 2x4
//     one; Ceres multiplies it with the SphereManifold<4> PlusJacobian.
//   * AnalyticInvReprojectionError<CameraModel> / ...PoseError<CameraModel>:
//     inverse-depth residuals (see InvReprojectionError).
//
// Models without an analytic implementation (ORTHOGRAPHIC) are not handled
// here: the Create* functions return nullptr and callers fall back to
// autodiff.

#include <Eigen/Core>
#include <ceres/rotation.h>
#include <ceres/sized_cost_function.h>

#include <cmath>
#include <limits>

#include "theia/sfm/camera/camera.h"
#include "theia/sfm/camera/camera_intrinsics_model_type.h"
#include "theia/sfm/camera/division_undistortion_camera_model.h"
#include "theia/sfm/camera/double_sphere_camera_model.h"
#include "theia/sfm/camera/extended_unified_camera_model.h"
#include "theia/sfm/camera/fisheye_camera_model.h"
#include "theia/sfm/camera/fov_camera_model.h"
#include "theia/sfm/camera/full_opencv_camera_model.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/camera/rad_tan_thin_prism_fisheye_camera_model.h"
#include "theia/sfm/camera/radial_fisheye_camera_model.h"
#include "theia/sfm/camera/thin_prism_fisheye_camera_model.h"
#include "theia/sfm/feature.h"

namespace theia {
namespace analytic {

typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 2, 2> Mat2;
typedef Eigen::Matrix<double, 2, 3> Mat23;
template <int N>
using Mat2N = Eigen::Matrix<double, 2, N>;

// Right Jacobian of SO(3) for the angle-axis vector aa:
// R(aa + d) ~= R(aa) Exp(Jr d).
inline Mat3 RightJacobian(const double* aa) {
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
  Mat3 W;
  W << 0, -aa[2], aa[1], aa[2], 0, -aa[0], -aa[1], aa[0], 0;
  return Mat3::Identity() - A * W + B * (W * W);
}

inline Mat3 Skew(const Vec3& p) {
  Mat3 Px;
  Px << 0, -p[2], p[1], p[2], 0, -p[0], -p[1], p[0], 0;
  return Px;
}

// ---------------------------------------------------------------------------
// Distortion policies. Distort(e, in, d, Din, Dp):
//   e   : pointer to the model specific parameters (K + kExtra)
//   in  : input of the distortion (normalized point (u,v), angle scaled point
//         or the 3D ray, depending on kInput)
//   d   : distorted point (before the calibration)
//   Din : d d / d in (may be null), Dp : d d / d e (null iff Din is null).
// ---------------------------------------------------------------------------
enum InputKind { kRay = 0, kNormalized = 1, kFisheyeNormalized = 2 };

struct PinholePolicy {
  static constexpr int N = 7, M = 2, kSkew = 2, kCx = 3, kCy = 4, kExtra = 5;
  static constexpr InputKind kInput = kNormalized;
  static constexpr int InDim = 2;
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1], k1 = e[0], k2 = e[1];
    const double r2 = u * u + v * v;
    const double dd = 1.0 + r2 * (k1 + k2 * r2);
    d << u * dd, v * dd;
    if (Din) {
      const double dr = k1 + 2.0 * k2 * r2;
      (*Din)(0, 0) = dd + 2.0 * u * u * dr;
      (*Din)(0, 1) = 2.0 * u * v * dr;
      (*Din)(1, 0) = (*Din)(0, 1);
      (*Din)(1, 1) = dd + 2.0 * v * v * dr;
      (*Dp) << u * r2, u * r2 * r2, v * r2, v * r2 * r2;
    }
    return true;
  }
};

struct RadTanPolicy {
  static constexpr int N = 10, M = 5, kSkew = 2, kCx = 3, kCy = 4, kExtra = 5;
  static constexpr InputKind kInput = kNormalized;
  static constexpr int InDim = 2;
  // e = k1, k2, k3, t1, t2
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1];
    const double k1 = e[0], k2 = e[1], k3 = e[2], t1 = e[3], t2 = e[4];
    const double r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    const double rd = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    d << u * rd + t2 * (r2 + 2.0 * u * u) + 2.0 * t1 * u * v,
        v * rd + t1 * (r2 + 2.0 * v * v) + 2.0 * t2 * u * v;
    if (Din) {
      const double dr = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4;
      (*Din)(0, 0) = rd + 2.0 * u * u * dr + 6.0 * t2 * u + 2.0 * t1 * v;
      (*Din)(0, 1) = 2.0 * u * v * dr + 2.0 * t2 * v + 2.0 * t1 * u;
      (*Din)(1, 0) = 2.0 * u * v * dr + 2.0 * t1 * u + 2.0 * t2 * v;
      (*Din)(1, 1) = rd + 2.0 * v * v * dr + 6.0 * t1 * v + 2.0 * t2 * u;
      (*Dp) << u * r2, u * r4, u * r6, 2.0 * u * v, r2 + 2.0 * u * u,
          v * r2, v * r4, v * r6, r2 + 2.0 * v * v, 2.0 * u * v;
    }
    return true;
  }
};

struct FullOpenCVPolicy {
  static constexpr int N = 12, M = 8, kSkew = -1, kCx = 2, kCy = 3, kExtra = 4;
  static constexpr InputKind kInput = kNormalized;
  static constexpr int InDim = 2;
  // e = k1, k2, p1, p2, k3, k4, k5, k6
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1];
    const double k1 = e[0], k2 = e[1], p1 = e[2], p2 = e[3], k3 = e[4],
                 k4 = e[5], k5 = e[6], k6 = e[7];
    const double r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2;
    const double num = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double den = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
    const double rad = num / den;
    d << u * rad + 2.0 * p1 * u * v + p2 * (r2 + 2.0 * u * u),
        v * rad + 2.0 * p2 * u * v + p1 * (r2 + 2.0 * v * v);
    if (Din) {
      const double dnum = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4;
      const double dden = k4 + 2.0 * k5 * r2 + 3.0 * k6 * r4;
      const double drad = (dnum * den - num * dden) / (den * den);
      (*Din)(0, 0) = rad + 2.0 * u * u * drad + 2.0 * p1 * v + 6.0 * p2 * u;
      (*Din)(0, 1) = 2.0 * u * v * drad + 2.0 * p1 * u + 2.0 * p2 * v;
      (*Din)(1, 0) = 2.0 * u * v * drad + 2.0 * p2 * v + 2.0 * p1 * u;
      (*Din)(1, 1) = rad + 2.0 * v * v * drad + 2.0 * p2 * u + 6.0 * p1 * v;
      const double i1 = 1.0 / den, i2 = -num / (den * den);
      (*Dp) << u * r2 * i1, u * r4 * i1, 2.0 * u * v, r2 + 2.0 * u * u,
          u * r6 * i1, u * r2 * i2, u * r4 * i2, u * r6 * i2,
          v * r2 * i1, v * r4 * i1, r2 + 2.0 * v * v, 2.0 * u * v,
          v * r6 * i1, v * r2 * i2, v * r4 * i2, v * r6 * i2;
    }
    return true;
  }
};

struct FovPolicy {
  static constexpr int N = 5, M = 1, kSkew = -1, kCx = 2, kCy = 3, kExtra = 4;
  static constexpr InputKind kInput = kNormalized;
  static constexpr int InDim = 2;
  // e = omega. Differentiates the branch that FOVCameraModel::DistortPoint
  // takes (including its Taylor approximations).
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1], w = e[0];
    const double r2 = u * u + v * v;
    double rd, drd_dr2, drd_dw;
    if (w < 1e-3) {
      rd = (w * w * r2) / 3.0 - w * w / 12.0 + 1.0;
      drd_dr2 = w * w / 3.0;
      drd_dw = 2.0 * w * r2 / 3.0 - w / 6.0;
    } else if (r2 < 1e-3) {
      const double t = std::tan(w / 2.0);
      rd = (-2.0 * t * (4.0 * r2 * t * t - 3.0)) / (3.0 * w);
      drd_dr2 = -(8.0 / 3.0) * t * t * t / w;
      drd_dw = (1.0 + t * t) / w - 2.0 * t / (w * w) -
               (8.0 / 3.0) * r2 *
                   (1.5 * t * t * (1.0 + t * t) / w - t * t * t / (w * w));
    } else {
      const double r = std::sqrt(r2);
      const double t = std::tan(w / 2.0);
      const double A = 2.0 * r * t;
      const double at = std::atan(A);
      rd = at / (r * w);
      const double drd_dr = (2.0 * t / (1.0 + A * A) * r - at) / (r * r * w);
      drd_dr2 = drd_dr / (2.0 * r);
      drd_dw = (1.0 + t * t) / ((1.0 + A * A) * w) - at / (r * w * w);
    }
    d << rd * u, rd * v;
    if (Din) {
      (*Din)(0, 0) = rd + 2.0 * u * u * drd_dr2;
      (*Din)(0, 1) = 2.0 * u * v * drd_dr2;
      (*Din)(1, 0) = (*Din)(0, 1);
      (*Din)(1, 1) = rd + 2.0 * v * v * drd_dr2;
      (*Dp) << u * drd_dw, v * drd_dw;
    }
    return true;
  }
};

// Radial distortion of the angle scaled point (RADIAL_FISHEYE).
struct RadialFisheyePolicy {
  static constexpr int N = 6, M = 2, kSkew = -1, kCx = 2, kCy = 3, kExtra = 4;
  static constexpr InputKind kInput = kFisheyeNormalized;
  static constexpr int InDim = 2;
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1], k1 = e[0], k2 = e[1];
    const double t2 = u * u + v * v, t4 = t2 * t2;
    const double radial = k1 * t2 + k2 * t4;
    d << u + u * radial, v + v * radial;
    if (Din) {
      const double dr = k1 + 2.0 * k2 * t2;
      (*Din)(0, 0) = 1.0 + radial + 2.0 * u * u * dr;
      (*Din)(0, 1) = 2.0 * u * v * dr;
      (*Din)(1, 0) = (*Din)(0, 1);
      (*Din)(1, 1) = 1.0 + radial + 2.0 * v * v * dr;
      (*Dp) << u * t2, u * t4, v * t2, v * t4;
    }
    return true;
  }
};

struct ThinPrismFisheyePolicy {
  static constexpr int N = 12, M = 8, kSkew = -1, kCx = 2, kCy = 3, kExtra = 4;
  static constexpr InputKind kInput = kFisheyeNormalized;
  static constexpr int InDim = 2;
  // e = k1, k2, p1, p2, k3, k4, sx1, sy1
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1];
    const double k1 = e[0], k2 = e[1], p1 = e[2], p2 = e[3], k3 = e[4],
                 k4 = e[5], sx1 = e[6], sy1 = e[7];
    const double r2 = u * u + v * v, r4 = r2 * r2, r6 = r4 * r2, r8 = r6 * r2;
    const double radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
    d << u + u * radial + 2.0 * p1 * u * v + p2 * (r2 + 2.0 * u * u) +
             sx1 * r2,
        v + v * radial + 2.0 * p2 * u * v + p1 * (r2 + 2.0 * v * v) +
            sy1 * r2;
    if (Din) {
      const double dr = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4 + 4.0 * k4 * r6;
      (*Din)(0, 0) = 1.0 + radial + 2.0 * u * u * dr + 2.0 * p1 * v +
                     6.0 * p2 * u + 2.0 * sx1 * u;
      (*Din)(0, 1) = 2.0 * u * v * dr + 2.0 * p1 * u + 2.0 * p2 * v +
                     2.0 * sx1 * v;
      (*Din)(1, 0) = 2.0 * u * v * dr + 2.0 * p2 * v + 2.0 * p1 * u +
                     2.0 * sy1 * u;
      (*Din)(1, 1) = 1.0 + radial + 2.0 * v * v * dr + 2.0 * p2 * u +
                     6.0 * p1 * v + 2.0 * sy1 * v;
      (*Dp) << u * r2, u * r4, 2.0 * u * v, r2 + 2.0 * u * u, u * r6, u * r8,
          r2, 0.0,
          v * r2, v * r4, r2 + 2.0 * v * v, 2.0 * u * v, v * r6, v * r8,
          0.0, r2;
    }
    return true;
  }
};

struct RadTanThinPrismFisheyePolicy {
  static constexpr int N = 16, M = 12, kSkew = -1, kCx = 2, kCy = 3,
                       kExtra = 4;
  static constexpr InputKind kInput = kFisheyeNormalized;
  static constexpr int InDim = 2;
  // e = k0..k5, p0, p1, s0..s3
  static bool Distort(const double* e, const double* in, Vec2& d,
                      Mat2* Din, Mat2N<M>* Dp) {
    const double u = in[0], v = in[1];
    const double th2 = u * u + v * v;
    double th_pow[7];  // th2^i
    th_pow[0] = 1.0;
    double th = 1.0, dth = 0.0;
    for (int i = 0; i < 6; ++i) {
      th_pow[i + 1] = th_pow[i] * th2;
      th += e[i] * th_pow[i + 1];
      dth += (i + 1) * e[i] * th_pow[i];
    }
    const double p0 = e[6], p1 = e[7], s0 = e[8], s1 = e[9], s2 = e[10],
                 s3 = e[11];
    const double X = th * u, Y = th * v;
    const double X2 = X * X, Y2 = Y * Y, XY = X * Y, R = X2 + Y2;
    d << X + 2.0 * p1 * XY + p0 * (R + 2.0 * X2) + s0 * R + s1 * R * R,
        Y + 2.0 * p0 * XY + p1 * (R + 2.0 * Y2) + s2 * R + s3 * R * R;
    if (Din) {
      Mat2 dF;  // d d / d (X, Y)
      dF(0, 0) = 1.0 + 2.0 * p1 * Y + 6.0 * p0 * X + 2.0 * s0 * X +
                 4.0 * s1 * X * R;
      dF(0, 1) = 2.0 * p1 * X + 2.0 * p0 * Y + 2.0 * s0 * Y + 4.0 * s1 * Y * R;
      dF(1, 0) = 2.0 * p0 * Y + 2.0 * p1 * X + 2.0 * s2 * X + 4.0 * s3 * X * R;
      dF(1, 1) = 1.0 + 2.0 * p0 * X + 6.0 * p1 * Y + 2.0 * s2 * Y +
                 4.0 * s3 * Y * R;
      Mat2 dXY;  // d (X, Y) / d (u, v)
      dXY(0, 0) = th + 2.0 * u * u * dth;
      dXY(0, 1) = 2.0 * u * v * dth;
      dXY(1, 0) = dXY(0, 1);
      dXY(1, 1) = th + 2.0 * v * v * dth;
      *Din = dF * dXY;
      for (int i = 0; i < 6; ++i) {
        const Vec2 dk = dF * Vec2(u * th_pow[i + 1], v * th_pow[i + 1]);
        Dp->col(i) = dk;
      }
      Dp->col(6) << 3.0 * X2 + Y2, 2.0 * XY;
      Dp->col(7) << 2.0 * XY, X2 + 3.0 * Y2;
      Dp->col(8) << R, 0.0;
      Dp->col(9) << R * R, 0.0;
      Dp->col(10) << 0.0, R;
      Dp->col(11) << 0.0, R * R;
    }
    return true;
  }
};

// Theia FISHEYE: distortion of the full 3D ray (angle theta from the optical
// axis, sign flip for z < 0), see FisheyeCameraModel::DistortPoint.
struct FisheyePolicy {
  static constexpr int N = 9, M = 4, kSkew = 2, kCx = 3, kCy = 4, kExtra = 5;
  static constexpr InputKind kInput = kRay;
  static constexpr int InDim = 3;
  static bool Distort(const double* e, const double* q, Vec2& d, Mat23* Dq,
                      Mat2N<M>* Dp) {
    const double x = q[0], y = q[1], z = q[2];
    const double r2 = x * x + y * y;
    if (r2 < 1e-8) {
      d << x, y;
      if (Dq) {
        Dq->setZero();
        (*Dq)(0, 0) = 1.0;
        (*Dq)(1, 1) = 1.0;
        Dp->setZero();
      }
      return true;
    }
    const double r = std::sqrt(r2);
    const double az = std::abs(z);
    const double theta = std::atan2(r, az);
    const double th2 = theta * theta;
    const double poly =
        1.0 + th2 * (e[0] + th2 * (e[1] + th2 * (e[2] + th2 * e[3])));
    const double thd = theta * poly;
    const double sgn = z < 0.0 ? -1.0 : 1.0;
    const double g = sgn * thd / r;
    d << g * x, g * y;
    if (Dq) {
      const double denom = r2 + z * z;
      const double dth_dr = az / denom;
      const double dth_dz = -sgn * r / denom;
      const double P1 =
          1.0 +
          th2 * (3.0 * e[0] +
                 th2 * (5.0 * e[1] + th2 * (7.0 * e[2] + th2 * 9.0 * e[3])));
      const double dg_dr = sgn * (P1 * dth_dr / r - thd / r2);
      const double dg_dz = sgn * P1 * dth_dz / r;
      (*Dq)(0, 0) = g + x * dg_dr * x / r;
      (*Dq)(0, 1) = x * dg_dr * y / r;
      (*Dq)(0, 2) = x * dg_dz;
      (*Dq)(1, 0) = y * dg_dr * x / r;
      (*Dq)(1, 1) = g + y * dg_dr * y / r;
      (*Dq)(1, 2) = y * dg_dz;
      double tp = theta * th2;  // theta^3
      for (int j = 0; j < 4; ++j) {
        (*Dp)(0, j) = sgn * x / r * tp;
        (*Dp)(1, j) = sgn * y / r * tp;
        tp *= th2;
      }
    }
    return true;
  }
};

struct DoubleSpherePolicy {
  static constexpr int N = 7, M = 2, kSkew = 2, kCx = 3, kCy = 4, kExtra = 5;
  static constexpr InputKind kInput = kRay;
  static constexpr int InDim = 3;
  // e = xi, alpha
  static bool Distort(const double* e, const double* q, Vec2& d, Mat23* Dq,
                      Mat2N<M>* Dp) {
    const double xi = e[0], alpha = e[1];
    const double x = q[0], y = q[1], z = q[2];
    const double r2 = x * x + y * y;
    const double d1 = std::sqrt(r2 + z * z);
    const double w1 =
        alpha > 0.5 ? (1.0 - alpha) / alpha : alpha / (1.0 - alpha);
    const double w2 = (w1 + xi) / std::sqrt(2.0 * w1 * xi + xi * xi + 1.0);
    if (z <= -w2 * d1) return false;
    const double k = xi * d1 + z;
    const double d2 = std::sqrt(r2 + k * k);
    const double N_ = alpha * d2 + (1.0 - alpha) * k;
    const double iN = 1.0 / N_;
    d << x * iN, y * iN;
    if (Dq) {
      const Vec3 dd1(x / d1, y / d1, z / d1);
      const Vec3 dk = xi * dd1 + Vec3(0.0, 0.0, 1.0);
      const Vec3 dd2(
          (x + k * dk[0]) / d2, (y + k * dk[1]) / d2, (k * dk[2]) / d2);
      const Vec3 dN = alpha * dd2 + (1.0 - alpha) * dk;
      const double iN2 = iN * iN;
      (*Dq)(0, 0) = iN - x * iN2 * dN[0];
      (*Dq)(0, 1) = -x * iN2 * dN[1];
      (*Dq)(0, 2) = -x * iN2 * dN[2];
      (*Dq)(1, 0) = -y * iN2 * dN[0];
      (*Dq)(1, 1) = iN - y * iN2 * dN[1];
      (*Dq)(1, 2) = -y * iN2 * dN[2];
      const double dN_dxi = alpha * k * d1 / d2 + (1.0 - alpha) * d1;
      const double dN_dal = d2 - k;
      (*Dp)(0, 0) = -x * iN2 * dN_dxi;
      (*Dp)(0, 1) = -x * iN2 * dN_dal;
      (*Dp)(1, 0) = -y * iN2 * dN_dxi;
      (*Dp)(1, 1) = -y * iN2 * dN_dal;
    }
    return true;
  }
};

struct ExtendedUnifiedPolicy {
  static constexpr int N = 7, M = 2, kSkew = 2, kCx = 3, kCy = 4, kExtra = 5;
  static constexpr InputKind kInput = kRay;
  static constexpr int InDim = 3;
  // e = alpha, beta
  static bool Distort(const double* e, const double* q, Vec2& d, Mat23* Dq,
                      Mat2N<M>* Dp) {
    const double alpha = e[0], beta = e[1];
    const double x = q[0], y = q[1], z = q[2];
    const double r2 = x * x + y * y;
    const double rho = std::sqrt(beta * r2 + z * z);
    const double N_ = alpha * rho + (1.0 - alpha) * z;
    d.setZero();
    if (Dq) {
      Dq->setZero();
      Dp->setZero();
    }
    // The autodiff path returns a constant (0, 0) in these two cases.
    if (N_ < 1e-3) return true;
    if (alpha > 0.5) {
      const double zn = z / N_;
      const double C = (alpha - 1.0) / (alpha + alpha - 1.0);
      if (zn < C) return true;
    }
    const double iN = 1.0 / N_;
    d << x * iN, y * iN;
    if (Dq) {
      const double iN2 = iN * iN;
      const double dN0 = alpha * beta * x / rho;
      const double dN1 = alpha * beta * y / rho;
      const double dN2 = alpha * z / rho + (1.0 - alpha);
      (*Dq)(0, 0) = iN - x * iN2 * dN0;
      (*Dq)(0, 1) = -x * iN2 * dN1;
      (*Dq)(0, 2) = -x * iN2 * dN2;
      (*Dq)(1, 0) = -y * iN2 * dN0;
      (*Dq)(1, 1) = iN - y * iN2 * dN1;
      (*Dq)(1, 2) = -y * iN2 * dN2;
      const double dN_dal = rho - z;
      const double dN_dbe = alpha * r2 / (2.0 * rho);
      (*Dp)(0, 0) = -x * iN2 * dN_dal;
      (*Dp)(0, 1) = -x * iN2 * dN_dbe;
      (*Dp)(1, 0) = -y * iN2 * dN_dal;
      (*Dp)(1, 1) = -y * iN2 * dN_dbe;
    }
    return true;
  }
};

// ---------------------------------------------------------------------------
// Generic projector built from a policy: q -> (normalized point | angle scaled
// point | ray) -> distortion -> calibration [[f, s], [0, f a]] + (cx, cy).
// ---------------------------------------------------------------------------
template <class P>
struct Projector {
  static constexpr int N = P::N;

  // Jq / JK may be null (residual only); either both or none.
  static bool Project(const double* K, const double* q, double* pix, Mat23* Jq,
                      Mat2N<N>* JK) {
    const double* e = K + P::kExtra;
    const bool jac = (Jq != nullptr);
    Vec2 d;
    Mat23 Dq;
    Eigen::Matrix<double, 2, P::M> Dp;
    if constexpr (P::kInput == kRay) {
      if (!P::Distort(e, q, d, jac ? &Dq : nullptr, jac ? &Dp : nullptr))
        return false;
    } else {
      const double inv_z = 1.0 / q[2];
      double n[2] = {q[0] * inv_z, q[1] * inv_z};
      Mat2 Din;
      double a[2] = {n[0], n[1]};
      [[maybe_unused]] bool scaled = false;
      [[maybe_unused]] double r2 = 0.0, scale = 1.0;
      if constexpr (P::kInput == kFisheyeNormalized) {
        r2 = n[0] * n[0] + n[1] * n[1];
        if (r2 > std::numeric_limits<double>::epsilon()) {
          const double r = std::sqrt(r2);
          scale = std::atan(r) / r;
          a[0] = n[0] * scale;
          a[1] = n[1] * scale;
          scaled = true;
        }
      }
      P::Distort(e, a, d, jac ? &Din : nullptr, jac ? &Dp : nullptr);
      if (jac) {
        // d a / d n
        Mat2 Da = Mat2::Identity();
        if (scaled) {
          double c;  // (1 / (1 + r2) - scale) / r2
          if (r2 < 1e-3) {
            c = -2.0 / 3.0 +
                r2 * (4.0 / 5.0 + r2 * (-6.0 / 7.0 + r2 * 8.0 / 9.0));
          } else {
            c = (1.0 / (1.0 + r2) - scale) / r2;
          }
          Da = scale * Mat2::Identity();
          Da(0, 0) += c * n[0] * n[0];
          Da(0, 1) += c * n[0] * n[1];
          Da(1, 0) += c * n[0] * n[1];
          Da(1, 1) += c * n[1] * n[1];
        }
        Mat23 dn_dq;
        dn_dq << inv_z, 0.0, -n[0] * inv_z, 0.0, inv_z, -n[1] * inv_z;
        Dq = (Din * Da) * dn_dq;
      }
    }
    const double f = K[0], asp = K[1];
    const double s = P::kSkew >= 0 ? K[P::kSkew] : 0.0;
    pix[0] = f * d[0] + s * d[1] + K[P::kCx];
    pix[1] = f * asp * d[1] + K[P::kCy];
    if (jac) {
      Mat2 A;
      A << f, s, 0.0, f * asp;
      *Jq = A * Dq;
      JK->setZero();
      JK->col(0) << d[0], asp * d[1];
      JK->col(1) << 0.0, f * d[1];
      if (P::kSkew >= 0) JK->col(P::kSkew) << d[1], 0.0;
      JK->col(P::kCx) << 1.0, 0.0;
      JK->col(P::kCy) << 0.0, 1.0;
      JK->template block<2, P::M>(0, P::kExtra) = A * Dp;
    }
    return true;
  }
};

// DIVISION_UNDISTORTION: the scaling acts on the calibrated point, so it does
// not fit the generic projector.
struct DivisionProjector {
  static constexpr int N = 5;
  static bool Project(const double* K, const double* q, double* pix, Mat23* Jq,
                      Mat2N<5>* JK) {
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
    pix[0] = s * ux + cx;
    pix[1] = s * uy + cy;
    if (Jq == nullptr) return true;

    Mat2 dd_du;  // d distorted / d undistorted
    if (identity) {
      dd_du.setIdentity();
    } else {
      const double c = ds * 8.0 * k;
      dd_du(0, 0) = s + c * ux * ux;
      dd_du(0, 1) = c * ux * uy;
      dd_du(1, 0) = dd_du(0, 1);
      dd_du(1, 1) = s + c * uy * uy;
    }
    Mat23 du_dq;
    du_dq << f * inv_z, 0.0, -f * nx * inv_z, 0.0, f * a * inv_z,
        -f * a * ny * inv_z;
    *Jq = dd_du * du_dq;
    JK->col(0) = dd_du * Vec2(nx, a * ny);
    JK->col(1) = dd_du * Vec2(0.0, f * ny);
    JK->col(2) << 1.0, 0.0;
    JK->col(3) << 0.0, 1.0;
    // d distorted / d k = u * ds * 4 r^2  (identity branch: 0)
    const double dk = identity ? 0.0 : ds * 4.0 * r2;
    JK->col(4) << ux * dk, uy * dk;
    return true;
  }
};

template <class CameraModel>
struct AnalyticModel;
template <>
struct AnalyticModel<PinholeCameraModel> : Projector<PinholePolicy> {};
template <>
struct AnalyticModel<PinholeRadialTangentialCameraModel>
    : Projector<RadTanPolicy> {};
template <>
struct AnalyticModel<FisheyeCameraModel> : Projector<FisheyePolicy> {};
template <>
struct AnalyticModel<DoubleSphereCameraModel> : Projector<DoubleSpherePolicy> {
};
template <>
struct AnalyticModel<ExtendedUnifiedCameraModel>
    : Projector<ExtendedUnifiedPolicy> {};
template <>
struct AnalyticModel<FOVCameraModel> : Projector<FovPolicy> {};
template <>
struct AnalyticModel<FullOpenCVCameraModel> : Projector<FullOpenCVPolicy> {};
template <>
struct AnalyticModel<RadialFisheyeCameraModel>
    : Projector<RadialFisheyePolicy> {};
template <>
struct AnalyticModel<ThinPrismFisheyeCameraModel>
    : Projector<ThinPrismFisheyePolicy> {};
template <>
struct AnalyticModel<RadTanThinPrismFisheyeCameraModel>
    : Projector<RadTanThinPrismFisheyePolicy> {};
template <>
struct AnalyticModel<DivisionUndistortionCameraModel> : DivisionProjector {};

}  // namespace analytic

// Residual of a homogeneous point [X, Y, Z, w]:
//   block 0: extrinsics [position(3), angle-axis(3)]
//   block 1: intrinsics
//   block 2: homogeneous point (ambient Jacobian).
template <class CameraModel>
class AnalyticReprojectionError
    : public ceres::SizedCostFunction<2, 6, CameraModel::kIntrinsicsSize, 4> {
 public:
  static constexpr int N = CameraModel::kIntrinsicsSize;

  explicit AnalyticReprojectionError(const Feature& feature)
      : feature_(feature) {}
  ~AnalyticReprojectionError() override {}

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    using namespace analytic;
    const double* ext = parameters[0];
    const double* K = parameters[1];
    const double* P = parameters[2];

    const Vec3 center(ext[Camera::POSITION], ext[Camera::POSITION + 1],
                      ext[Camera::POSITION + 2]);
    const double w = P[3];
    const Vec3 p = Vec3(P[0], P[1], P[2]) - w * center;
    // Same guard as ReprojectionError: point (almost) at the camera center.
    if (p.squaredNorm() < 1e-8) return false;

    const double* aa = ext + Camera::ORIENTATION;
    Mat3 R;  // column-major, as ceres::AngleAxisToRotationMatrix writes.
    ceres::AngleAxisToRotationMatrix(aa, R.data());
    const Vec3 q = R * p;

    const bool want_jac = jacobians != nullptr;
    double pix[2];
    Mat23 Jq;
    Mat2N<N> JK;
    if (!AnalyticModel<CameraModel>::Project(
            K, q.data(), pix, want_jac ? &Jq : nullptr,
            want_jac ? &JK : nullptr)) {
      return false;
    }

    const double dx = pix[0] - feature_.point_.x();
    const double dy = pix[1] - feature_.point_.y();
    const Eigen::Matrix2d si = feature_.SqrtInformation();
    residuals[0] = si(0, 0) * dx + si(0, 1) * dy;
    residuals[1] = si(1, 0) * dx + si(1, 1) * dy;
    if (!want_jac) return true;

    const Mat23 JqW = si * Jq;  // whitened
    const Mat23 JqR = JqW * R;
    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      // dq/dc = -w R ; dq/d(angle axis) = -R [p]x Jr
      J.block<2, 3>(0, Camera::POSITION) = (-w) * JqR;
      J.block<2, 3>(0, Camera::ORIENTATION) =
          -JqR * (Skew(p) * RightJacobian(aa));
    }
    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, N, Eigen::RowMajor>> J(jacobians[1]);
      J = si * JK;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[2]);
      J.block<2, 3>(0, 0) = JqR;
      J.col(3) = -(JqR * center);
    }
    return true;
  }

 private:
  const Feature feature_;
};

// Inverse depth residual (see InvReprojectionError):
//   q = R_o * (R_ref^T b + rho (c_ref - c_o))
//   block 0: reference extrinsics, 1: other extrinsics, 2: intrinsics of the
//   other camera, 3: inverse depth rho.
template <class CameraModel>
class AnalyticInvReprojectionError
    : public ceres::SizedCostFunction<2, 6, 6, CameraModel::kIntrinsicsSize,
                                      1> {
 public:
  static constexpr int N = CameraModel::kIntrinsicsSize;

  AnalyticInvReprojectionError(const Feature& feature,
                               const Eigen::Vector3d& bearing_vector_ref)
      : feature_(feature), bearing_(bearing_vector_ref.normalized()) {}
  ~AnalyticInvReprojectionError() override {}

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    using namespace analytic;
    const double* ext_ref = parameters[0];
    const double* ext_o = parameters[1];
    const double* K = parameters[2];
    const double rho = parameters[3][0];

    Mat3 R_ref, R_o;
    ceres::AngleAxisToRotationMatrix(ext_ref + Camera::ORIENTATION,
                                     R_ref.data());
    ceres::AngleAxisToRotationMatrix(ext_o + Camera::ORIENTATION, R_o.data());
    const Vec3 c_ref(ext_ref + Camera::POSITION);
    const Vec3 c_o(ext_o + Camera::POSITION);
    const Vec3 dc = c_ref - c_o;
    const Vec3 wv = R_ref.transpose() * bearing_;
    const Vec3 v = wv + rho * dc;
    const Vec3 q = R_o * v;

    const bool want_jac = jacobians != nullptr;
    double pix[2];
    Mat23 Jq;
    Mat2N<N> JK;
    if (!AnalyticModel<CameraModel>::Project(
            K, q.data(), pix, want_jac ? &Jq : nullptr,
            want_jac ? &JK : nullptr)) {
      return false;
    }
    const double dx = pix[0] - feature_.point_.x();
    const double dy = pix[1] - feature_.point_.y();
    const Eigen::Matrix2d si = feature_.SqrtInformation();
    residuals[0] = si(0, 0) * dx + si(0, 1) * dy;
    residuals[1] = si(1, 0) * dx + si(1, 1) * dy;
    if (!want_jac) return true;

    const Mat23 JqR = (si * Jq) * R_o;  // d res / d v
    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.block<2, 3>(0, Camera::POSITION) = rho * JqR;
      J.block<2, 3>(0, Camera::ORIENTATION) =
          JqR * (Skew(wv) * RightJacobian(ext_ref + Camera::ORIENTATION));
    }
    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[1]);
      J.block<2, 3>(0, Camera::POSITION) = (-rho) * JqR;
      J.block<2, 3>(0, Camera::ORIENTATION) =
          -JqR * (Skew(v) * RightJacobian(ext_o + Camera::ORIENTATION));
    }
    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, N, Eigen::RowMajor>> J(jacobians[2]);
      J = si * JK;
    }
    if (jacobians[3] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 1>> J(jacobians[3]);
      J = JqR * dc;
    }
    return true;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_;
};

// Residual of the reference observation itself (see InvReprojectionPoseError):
// only the intrinsics enter. Blocks: ref extrinsics, intrinsics, inverse depth.
template <class CameraModel>
class AnalyticInvReprojectionPoseError
    : public ceres::SizedCostFunction<2, 6, CameraModel::kIntrinsicsSize, 1> {
 public:
  static constexpr int N = CameraModel::kIntrinsicsSize;

  AnalyticInvReprojectionPoseError(const Feature& feature,
                                   const Eigen::Vector3d& bearing_vector_ref)
      : feature_(feature), bearing_(bearing_vector_ref.normalized()) {}
  ~AnalyticInvReprojectionPoseError() override {}

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    using namespace analytic;
    const double* K = parameters[1];
    const bool want_jac = jacobians != nullptr;
    double pix[2];
    Mat23 Jq;
    Mat2N<N> JK;
    if (!AnalyticModel<CameraModel>::Project(
            K, bearing_.data(), pix, want_jac ? &Jq : nullptr,
            want_jac ? &JK : nullptr)) {
      return false;
    }
    const double dx = pix[0] - feature_.point_.x();
    const double dy = pix[1] - feature_.point_.y();
    const Eigen::Matrix2d si = feature_.SqrtInformation();
    residuals[0] = si(0, 0) * dx + si(0, 1) * dy;
    residuals[1] = si(1, 0) * dx + si(1, 1) * dy;
    if (!want_jac) return true;
    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>(jacobians[0])
          .setZero();
    }
    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, N, Eigen::RowMajor>> J(jacobians[1]);
      J = si * JK;
    }
    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 1>>(jacobians[2]).setZero();
    }
    return true;
  }

 private:
  const Feature feature_;
  const Eigen::Vector3d bearing_;
};

// Factories. Return nullptr for camera models without an analytic
// implementation (ORTHOGRAPHIC); the caller then uses the autodiff version.
#define THEIA_ANALYTIC_MODEL_LIST(X)                                  \
  X(PINHOLE, PinholeCameraModel)                                      \
  X(PINHOLE_RADIAL_TANGENTIAL, PinholeRadialTangentialCameraModel)    \
  X(FISHEYE, FisheyeCameraModel)                                      \
  X(DOUBLE_SPHERE, DoubleSphereCameraModel)                           \
  X(EXTENDED_UNIFIED, ExtendedUnifiedCameraModel)                     \
  X(FOV, FOVCameraModel)                                              \
  X(DIVISION_UNDISTORTION, DivisionUndistortionCameraModel)           \
  X(FULL_OPENCV, FullOpenCVCameraModel)                               \
  X(RADIAL_FISHEYE, RadialFisheyeCameraModel)                         \
  X(THIN_PRISM_FISHEYE, ThinPrismFisheyeCameraModel)                  \
  X(RAD_TAN_THIN_PRISM_FISHEYE, RadTanThinPrismFisheyeCameraModel)

inline ceres::CostFunction* CreateAnalyticReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& type, const Feature& feature) {
  switch (type) {
#define X(NAME, MODEL)                      \
  case CameraIntrinsicsModelType::NAME:     \
    return new AnalyticReprojectionError<MODEL>(feature);
    THEIA_ANALYTIC_MODEL_LIST(X)
#undef X
    default:
      return nullptr;
  }
}

inline ceres::CostFunction* CreateAnalyticInvReprojectionErrorCostFunction(
    const CameraIntrinsicsModelType& type,
    const Feature& feature,
    const Eigen::Vector3d& ref_bearing) {
  switch (type) {
#define X(NAME, MODEL)                                                \
  case CameraIntrinsicsModelType::NAME:                               \
    return new AnalyticInvReprojectionError<MODEL>(feature, ref_bearing);
    THEIA_ANALYTIC_MODEL_LIST(X)
#undef X
    default:
      return nullptr;
  }
}

inline ceres::CostFunction* CreateAnalyticInvReprojectionPoseErrorCostFunction(
    const CameraIntrinsicsModelType& type,
    const Feature& feature,
    const Eigen::Vector3d& ref_bearing) {
  switch (type) {
#define X(NAME, MODEL)                                                    \
  case CameraIntrinsicsModelType::NAME:                                   \
    return new AnalyticInvReprojectionPoseError<MODEL>(feature, ref_bearing);
    THEIA_ANALYTIC_MODEL_LIST(X)
#undef X
    default:
      return nullptr;
  }
}

#undef THEIA_ANALYTIC_MODEL_LIST

}  // namespace theia

#endif  // THEIA_SFM_BUNDLE_ADJUSTMENT_ANALYTIC_REPROJECTION_ERROR_H_
