// Copyright (c) 2020, Viktor Larsson (PoseLib authors). All rights reserved.
// Copyright (C) 2024, The pyTheiaSfM authors. All rights reserved.
//
// This file is adapted from PoseLib (https://github.com/PoseLib/PoseLib),
// solvers/relpose_monodepth_3pt{,_shared_focal,_varying_focal}.{h,cc},
// commit fa7280fee27f97aff31ae7f98bab7f583fac7d08, and is distributed under
// the BSD-3-Clause license. See docs/licenses/POSELIB_LICENSE.txt for the
// full PoseLib license text. Math kept as close as possible to upstream to
// keep future diffs against PoseLib reviewable; the only substantive changes
// are: quartic-root finding is done via theia::SolveQuarticReals instead of
// PoseLib's univariate::solve_quartic_real, and results are written into
// theia-native MonoDepthRelativePose structs instead of PoseLib's
// CameraPose-based MonoDepthTwoViewGeometry/MonoDepthImagePair/Camera
// (theia does not need a Camera object here since callers already track
// focal length via theia's own camera-model system).
//
// Reference: Y. Ding, V. Larsson, et al., "RePoseD: Efficient Relative Pose
// Estimation With Known Depth Information", ICCV 2025.

#include "theia/sfm/pose/relative_pose_monodepth_3pt.h"

#include <Eigen/Dense>
#include <cmath>
#include <utility>

#include "theia/math/closed_form_polynomial_solver.h"

namespace theia {
namespace {

// Solves the quartic x^4 + b*x^3 + c*x^2 + d*x + e = 0 for its real roots
// using theia's general (non-monic, long double) quartic solver with a
// leading coefficient of 1. Equivalent to PoseLib's
// univariate::solve_quartic_real(b, c, d, e, roots).
int SolveMonicQuarticReal(double b, double c, double d, double e,
                          double roots[4]) {
  long double roots_ld[4];
  const int n_roots = SolveQuarticReals(1.0L,
                                        static_cast<long double>(b),
                                        static_cast<long double>(c),
                                        static_cast<long double>(d),
                                        static_cast<long double>(e),
                                        roots_ld);
  for (int i = 0; i < n_roots; i++) {
    roots[i] = static_cast<double>(roots_ld[i]);
  }
  return n_roots;
}

// --- relpose_monodepth_3pt (calibrated) -------------------------------------

std::pair<Eigen::MatrixXd, Eigen::VectorXd> SolverP3PMono3D(
    const Eigen::VectorXd& data) {
  const double* d = data.data();
  Eigen::VectorXd coeffs(18);
  coeffs[0] = std::pow(d[6], 2) - 2 * d[6] * d[7] + std::pow(d[7], 2) + std::pow(d[9], 2) - 2 * d[9] * d[10] +
              std::pow(d[10], 2);
  coeffs[1] = -std::pow(d[0], 2) + 2 * d[0] * d[1] - std::pow(d[1], 2) - std::pow(d[3], 2) + 2 * d[3] * d[4] -
              std::pow(d[4], 2);
  coeffs[2] = 2 * std::pow(d[6], 2) * d[15] - 2 * d[6] * d[7] * d[15] + 2 * std::pow(d[9], 2) * d[15] -
              2 * d[9] * d[10] * d[15] - 2 * d[6] * d[7] * d[16] + 2 * std::pow(d[7], 2) * d[16] -
              2 * d[9] * d[10] * d[16] + 2 * std::pow(d[10], 2) * d[16];
  coeffs[3] = std::pow(d[6], 2) * std::pow(d[15], 2) + std::pow(d[9], 2) * std::pow(d[15], 2) -
              2 * d[6] * d[7] * d[15] * d[16] - 2 * d[9] * d[10] * d[15] * d[16] +
              std::pow(d[7], 2) * std::pow(d[16], 2) + std::pow(d[10], 2) * std::pow(d[16], 2) + std::pow(d[15], 2) -
              2 * d[15] * d[16] + std::pow(d[16], 2);
  coeffs[4] = -2 * std::pow(d[0], 2) * d[12] + 2 * d[0] * d[1] * d[12] - 2 * std::pow(d[3], 2) * d[12] +
              2 * d[3] * d[4] * d[12] + 2 * d[0] * d[1] * d[13] - 2 * std::pow(d[1], 2) * d[13] +
              2 * d[3] * d[4] * d[13] - 2 * std::pow(d[4], 2) * d[13];
  coeffs[5] = -std::pow(d[0], 2) * std::pow(d[12], 2) - std::pow(d[3], 2) * std::pow(d[12], 2) +
              2 * d[0] * d[1] * d[12] * d[13] + 2 * d[3] * d[4] * d[12] * d[13] -
              std::pow(d[1], 2) * std::pow(d[13], 2) - std::pow(d[4], 2) * std::pow(d[13], 2) - std::pow(d[12], 2) +
              2 * d[12] * d[13] - std::pow(d[13], 2);
  coeffs[6] = std::pow(d[6], 2) - 2 * d[6] * d[8] + std::pow(d[8], 2) + std::pow(d[9], 2) - 2 * d[9] * d[11] +
              std::pow(d[11], 2);
  coeffs[7] = -std::pow(d[0], 2) + 2 * d[0] * d[2] - std::pow(d[2], 2) - std::pow(d[3], 2) + 2 * d[3] * d[5] -
              std::pow(d[5], 2);
  coeffs[8] = 2 * std::pow(d[6], 2) * d[15] - 2 * d[6] * d[8] * d[15] + 2 * std::pow(d[9], 2) * d[15] -
              2 * d[9] * d[11] * d[15] - 2 * d[6] * d[8] * d[17] + 2 * std::pow(d[8], 2) * d[17] -
              2 * d[9] * d[11] * d[17] + 2 * std::pow(d[11], 2) * d[17];
  coeffs[9] = std::pow(d[6], 2) * std::pow(d[15], 2) + std::pow(d[9], 2) * std::pow(d[15], 2) -
              2 * d[6] * d[8] * d[15] * d[17] - 2 * d[9] * d[11] * d[15] * d[17] +
              std::pow(d[8], 2) * std::pow(d[17], 2) + std::pow(d[11], 2) * std::pow(d[17], 2) + std::pow(d[15], 2) -
              2 * d[15] * d[17] + std::pow(d[17], 2);
  coeffs[10] = -2 * std::pow(d[0], 2) * d[12] + 2 * d[0] * d[2] * d[12] - 2 * std::pow(d[3], 2) * d[12] +
               2 * d[3] * d[5] * d[12] + 2 * d[0] * d[2] * d[14] - 2 * std::pow(d[2], 2) * d[14] +
               2 * d[3] * d[5] * d[14] - 2 * std::pow(d[5], 2) * d[14];
  coeffs[11] = -std::pow(d[0], 2) * std::pow(d[12], 2) - std::pow(d[3], 2) * std::pow(d[12], 2) +
               2 * d[0] * d[2] * d[12] * d[14] + 2 * d[3] * d[5] * d[12] * d[14] -
               std::pow(d[2], 2) * std::pow(d[14], 2) - std::pow(d[5], 2) * std::pow(d[14], 2) - std::pow(d[12], 2) +
               2 * d[12] * d[14] - std::pow(d[14], 2);
  coeffs[12] = std::pow(d[7], 2) - 2 * d[7] * d[8] + std::pow(d[8], 2) + std::pow(d[10], 2) - 2 * d[10] * d[11] +
               std::pow(d[11], 2);
  coeffs[13] = -std::pow(d[1], 2) + 2 * d[1] * d[2] - std::pow(d[2], 2) - std::pow(d[4], 2) + 2 * d[4] * d[5] -
               std::pow(d[5], 2);
  coeffs[14] = 2 * std::pow(d[7], 2) * d[16] - 2 * d[7] * d[8] * d[16] + 2 * std::pow(d[10], 2) * d[16] -
               2 * d[10] * d[11] * d[16] - 2 * d[7] * d[8] * d[17] + 2 * std::pow(d[8], 2) * d[17] -
               2 * d[10] * d[11] * d[17] + 2 * std::pow(d[11], 2) * d[17];
  coeffs[15] = std::pow(d[7], 2) * std::pow(d[16], 2) + std::pow(d[10], 2) * std::pow(d[16], 2) -
               2 * d[7] * d[8] * d[16] * d[17] - 2 * d[10] * d[11] * d[16] * d[17] +
               std::pow(d[8], 2) * std::pow(d[17], 2) + std::pow(d[11], 2) * std::pow(d[17], 2) + std::pow(d[16], 2) -
               2 * d[16] * d[17] + std::pow(d[17], 2);
  coeffs[16] = -2 * std::pow(d[1], 2) * d[13] + 2 * d[1] * d[2] * d[13] - 2 * std::pow(d[4], 2) * d[13] +
               2 * d[4] * d[5] * d[13] + 2 * d[1] * d[2] * d[14] - 2 * std::pow(d[2], 2) * d[14] +
               2 * d[4] * d[5] * d[14] - 2 * std::pow(d[5], 2) * d[14];
  coeffs[17] = -std::pow(d[1], 2) * std::pow(d[13], 2) - std::pow(d[4], 2) * std::pow(d[13], 2) +
               2 * d[1] * d[2] * d[13] * d[14] + 2 * d[4] * d[5] * d[13] * d[14] -
               std::pow(d[2], 2) * std::pow(d[14], 2) - std::pow(d[5], 2) * std::pow(d[14], 2) - std::pow(d[13], 2) +
               2 * d[13] * d[14] - std::pow(d[14], 2);

  Eigen::MatrixXd C0(3, 3);
  C0 << coeffs[0], coeffs[2], coeffs[3], coeffs[6], coeffs[8], coeffs[9], coeffs[12], coeffs[14], coeffs[15];

  Eigen::MatrixXd C1(3, 3);
  C1 << coeffs[1], coeffs[4], coeffs[5], coeffs[7], coeffs[10], coeffs[11], coeffs[13], coeffs[16], coeffs[17];

  Eigen::MatrixXd C2 = -C0.fullPivLu().solve(C1);

  double k0 = C2(0, 0);
  double k1 = C2(0, 1);
  double k2 = C2(0, 2);
  double k3 = C2(1, 0);
  double k4 = C2(1, 1);
  double k5 = C2(1, 2);
  double k6 = C2(2, 0);
  double k7 = C2(2, 1);
  double k8 = C2(2, 2);

  double c4 = 1.0 / (k3 * k3 - k0 * k6);
  double c3 = c4 * (2 * k3 * k4 - k1 * k6 - k0 * k7);
  double c2 = c4 * (k4 * k4 - k0 * k8 - k1 * k7 - k2 * k6 + 2 * k3 * k5);
  double c1 = c4 * (2 * k4 * k5 - k2 * k7 - k1 * k8);
  double c0 = c4 * (k5 * k5 - k2 * k8);

  double roots[4];
  int n_roots = SolveMonicQuarticReal(c3, c2, c1, c0, roots);
  int m = 0;
  Eigen::MatrixXd sols(3, n_roots);
  for (int ii = 0; ii < n_roots; ii++) {
    double ss = k6 * roots[ii] * roots[ii] + k7 * roots[ii] + k8;
    if (ss < 0.001) continue;
    sols(1, ii) = roots[ii];
    sols(0, ii) = std::sqrt(ss);
    sols(2, ii) = (k3 * roots[ii] * roots[ii] + k4 * roots[ii] + k5) / ss;
    ++m;
  }
  sols.conservativeResize(3, m);
  return {sols, coeffs};
}

inline void RefineSUV(double& s, double& u, double& v, const Eigen::VectorXd& c) {
  for (int iter = 0; iter < 5; ++iter) {
    Eigen::Vector3d r;
    r(0) = c(0) * s * v * v + c(1) * u * u + c(2) * s * v + c(3) * s + c(4) * u + c(5);
    r(1) = c(6) * s * v * v + c(7) * u * u + c(8) * s * v + c(9) * s + c(10) * u + c(11);
    r(2) = c(12) * s * v * v + c(13) * u * u + c(14) * s * v + c(15) * s + c(16) * u + c(17);
    if (std::abs(r(0)) + std::abs(r(1)) + std::abs(r(2)) < 1e-10) return;
    Eigen::Matrix3d J;
    J(0, 0) = c(0) * v * v + c(2) * v + c(3);
    J(0, 1) = 2.0 * c(1) * u + c(4);
    J(0, 2) = 2.0 * c(0) * s * v + c(2) * s;

    J(1, 0) = c(6) * v * v + c(8) * v + c(9);
    J(1, 1) = 2.0 * c(7) * u + c(10);
    J(1, 2) = 2.0 * c(6) * s * v + c(8) * s;

    J(2, 0) = c(12) * v * v + c(14) * v + c(15);
    J(2, 1) = 2.0 * c(13) * u + c(16);
    J(2, 2) = 2.0 * c(12) * s * v + c(14) * s;

    Eigen::Vector3d delta_lambda = (J.transpose() * J).ldlt().solve(-J.transpose() * r);

    s += delta_lambda(0);
    u += delta_lambda(1);
    v += delta_lambda(2);
  }
}

}  // namespace

int MonoDepthRelativePose3pt(const std::vector<Eigen::Vector3d>& x1h,
                             const std::vector<Eigen::Vector3d>& x2h,
                             const std::vector<double>& depth1,
                             const std::vector<double>& depth2,
                             std::vector<MonoDepthRelativePose>* poses) {
  poses->clear();
  poses->reserve(4);

  Eigen::VectorXd datain(18);
  datain << x1h[0][0], x1h[1][0], x1h[2][0], x1h[0][1], x1h[1][1], x1h[2][1], x2h[0][0], x2h[1][0], x2h[2][0],
      x2h[0][1], x2h[1][1], x2h[2][1], depth1[0], depth1[1], depth1[2], depth2[0], depth2[1], depth2[2];

  auto [sols, cc] = SolverP3PMono3D(datain);
  for (int k = 0; k < sols.cols(); ++k) {
    double s = sols(0, k);
    double u = sols(1, k);
    double v = sols(2, k);

    if (depth2[0] + v <= 0 || depth2[1] + v <= 0 || depth2[2] + v <= 0 || depth1[0] + u <= 0 ||
        depth1[1] + u <= 0 || depth1[2] + u <= 0)
      continue;

    double s2 = s * s;
    RefineSUV(s2, u, v, cc);
    s = std::sqrt(s2);

    Eigen::Vector3d v1 = s * (depth2[0] + v) * x2h[0] - s * (depth2[1] + v) * x2h[1];
    Eigen::Vector3d v2 = s * (depth2[0] + v) * x2h[0] - s * (depth2[2] + v) * x2h[2];

    Eigen::Matrix3d Y;
    Y << v1, v2, v1.cross(v2);

    Eigen::Vector3d u1 = (depth1[0] + u) * x1h[0] - (depth1[1] + u) * x1h[1];
    Eigen::Vector3d u2 = (depth1[0] + u) * x1h[0] - (depth1[2] + u) * x1h[2];

    Eigen::Matrix3d X;
    X << u1, u2, u1.cross(u2);
    X = X.inverse().eval();

    Eigen::Matrix3d rot = Y * X;
    Eigen::Vector3d t = s * (depth2[0] + v) * x2h[0] - (depth1[0] + u) * rot * x1h[0];

    MonoDepthRelativePose pose;
    pose.rotation = rot;
    pose.translation = t;
    pose.scale = s;
    pose.shift1 = u;
    pose.shift2 = v;

    poses->push_back(pose);
  }

  return poses->size();
}

// --- relpose_monodepth_3pt_shared_focal -------------------------------------

int MonoDepthRelativePose3ptSharedFocal(
    const std::vector<Eigen::Vector3d>& x1h,
    const std::vector<Eigen::Vector3d>& x2h,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    std::vector<MonoDepthRelativePose>* poses) {
  poses->clear();
  poses->reserve(4);

  Eigen::Matrix3d X1;
  X1.col(0) = depth1[0] * x1h[0];
  X1.col(1) = depth1[1] * x1h[1];
  X1.col(2) = depth1[2] * x1h[2];

  Eigen::Matrix3d X2;
  X2.col(0) = depth2[0] * x2h[0];
  X2.col(1) = depth2[1] * x2h[1];
  X2.col(2) = x2h[2];

  double a[17];

  a[0] = X1(0, 0);
  a[1] = X1(0, 1);
  a[2] = X1(0, 2);
  a[3] = X1(1, 0);
  a[4] = X1(1, 1);
  a[5] = X1(1, 2);
  a[6] = X1(2, 0);
  a[7] = X1(2, 1);
  a[8] = X1(2, 2);

  a[9] = X2(0, 0);
  a[10] = X2(0, 1);
  a[11] = X2(0, 2);
  a[12] = X2(1, 0);
  a[13] = X2(1, 1);
  a[14] = X2(1, 2);
  a[15] = X2(2, 0);
  a[16] = X2(2, 1);

  double b[12];
  b[0] = a[0] - a[1];
  b[1] = a[3] - a[4];
  b[2] = a[6] - a[7];
  b[3] = a[0] - a[2];
  b[4] = a[3] - a[5];
  b[5] = a[6] - a[8];
  b[6] = a[1] - a[2];
  b[7] = a[4] - a[5];
  b[8] = a[7] - a[8];
  b[9] = a[9] - a[10];
  b[10] = a[12] - a[13];
  b[11] = a[15] - a[16];

  double c[18];
  c[0] = -std::pow(b[11], 2);
  c[1] = std::pow(b[2], 2);
  c[2] = -std::pow(b[9], 2) - std::pow(b[10], 2);
  c[3] = std::pow(b[0], 2) + std::pow(b[1], 2);

  c[4] = -1.0;
  c[5] = 2 * a[15];
  c[6] = -std::pow(a[15], 2);
  c[7] = std::pow(b[5], 2);
  c[8] = -std::pow(a[11], 2) - std::pow(a[14], 2);
  c[9] = 2 * a[9] * a[11] + 2 * a[12] * a[14];
  c[10] = -std::pow(a[9], 2) - std::pow(a[12], 2);
  c[11] = std::pow(b[3], 2) + std::pow(b[4], 2);

  c[12] = 2 * a[16] - 2 * a[15];
  c[13] = std::pow(a[15], 2) - std::pow(a[16], 2);
  c[14] = std::pow(b[8], 2) - std::pow(b[5], 2);
  c[15] = 2 * a[10] * a[11] - 2 * a[9] * a[11] - 2 * a[12] * a[14] + 2 * a[13] * a[14];
  c[16] = std::pow(a[9], 2) - std::pow(a[10], 2) + std::pow(a[12], 2) - std::pow(a[13], 2);
  c[17] = -std::pow(b[3], 2) - std::pow(b[4], 2) + std::pow(b[6], 2) + std::pow(b[7], 2);

  double d[21];

  d[6] = 1 / (a[6] - a[7]);
  d[0] = (-c[3] * c[8]) * d[6];
  d[1] = (-c[3] * c[9]) * d[6];
  d[2] = (c[2] * c[11] - c[3] * c[10]) * d[6];
  d[3] = (-c[3] * c[4] - c[1] * c[8]) * d[6];
  d[4] = (-c[3] * c[5] - c[1] * c[9]) * d[6];
  d[5] = (c[2] * c[7] - c[3] * c[6] + c[0] * c[11] - c[1] * c[10]) * d[6];
  d[7] = (a[6] * a[16] - 2 * a[6] * a[15] + a[7] * a[15] + a[8] * a[15] - a[8] * a[16]) * d[6];

  d[8] = 1 / (2 * (a[6] - a[7]) * (a[15] - a[16]));
  d[9] = (-c[3] * c[15]) * d[8];
  d[10] = (c[2] * c[17] - c[3] * c[16]) * d[8];
  d[11] = (-c[3] * c[12] - c[1] * c[15]) * d[8];
  d[12] = (c[2] * c[14] - c[3] * c[13] + c[0] * c[17] - c[1] * c[16]) * d[8];

  d[13] = 1 / (a[6] + a[7] - 2 * a[8]);
  d[14] = (a[8] * a[15] - a[7] * a[15] - a[6] * a[16] + a[8] * a[16]) * d[13];
  d[15] = (c[8] * c[17]) * d[13];
  d[16] = (c[9] * c[17] - c[11] * c[15]) * d[13];
  d[17] = (c[10] * c[17] - c[11] * c[16]) * d[13];
  d[18] = (c[4] * c[17] + c[8] * c[14]) * d[13];
  d[19] = (c[5] * c[17] - c[7] * c[15] + c[9] * c[14] - c[11] * c[12]) * d[13];
  d[20] = (c[6] * c[17] - c[7] * c[16] + c[10] * c[14] - c[11] * c[13]) * d[13];

  Eigen::MatrixXd C0(3, 3);
  C0 << d[2], d[5], d[7], d[10], d[12], 1.0, d[17], d[20], d[14];

  Eigen::MatrixXd C1(3, 4);
  C1 << d[0] - d[9], d[3] - d[11], d[1] - d[10], d[4] - d[12], 0, 0, d[9], d[11], d[15] - d[9], d[18] - d[11],
      d[16] - d[10], d[19] - d[12];

  Eigen::MatrixXd C2 = -C0.partialPivLu().solve(C1);

  Eigen::MatrixXd AM(4, 4);
  AM << 0, 0, 1.0, 0, 0, 0, 0, 1.0, C2(0, 0), C2(0, 1), C2(0, 2), C2(0, 3), C2(1, 0), C2(1, 1), C2(1, 2), C2(1, 3);

  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(AM, false);
  Eigen::ArrayXcd D = es.eigenvalues();

  for (int k = 0; k < 4; ++k) {
    if (std::abs(D(k).imag()) > 0.001 || D(k).real() < 0.0) continue;

    double d3 = 1.0 / D(k).real();

    Eigen::MatrixXd A0(2, 2);
    A0 << (d[3] - d[11]) * d3 * d3 + (d[4] - d[12]) * d3 + d[5], d[7], d[12] + d[11] * d3, 1.0;

    Eigen::VectorXd A1(2);
    A1 << (d[0] - d[9]) * d3 * d3 + (d[1] - d[10]) * d3 + d[2], d[10] + d[9] * d3;
    Eigen::VectorXd A2 = -A0.partialPivLu().solve(A1);

    if (A2(0) < 0.0) continue;

    double s2 = -(c[1] * A2(0) + c[3]) / (c[0] * A2(0) + c[2]);
    if (s2 < 0.001) continue;

    double s = std::sqrt(s2);
    double f = std::sqrt(A2(0));

    Eigen::Matrix3d Kinv;
    Kinv << 1.0 / f, 0, 0, 0, 1.0 / f, 0, 0, 0, 1;

    Eigen::Vector3d v1 = s * (depth2[0]) * Kinv * x2h[0] - s * (depth2[1]) * Kinv * x2h[1];
    Eigen::Vector3d v2 = s * (depth2[0]) * Kinv * x2h[0] - s * (d3)*Kinv * x2h[2];
    Eigen::Matrix3d Y;
    Y << v1, v2, v1.cross(v2);

    Eigen::Vector3d u1 = (depth1[0]) * Kinv * x1h[0] - (depth1[1]) * Kinv * x1h[1];
    Eigen::Vector3d u2 = (depth1[0]) * Kinv * x1h[0] - (depth1[2]) * Kinv * x1h[2];
    Eigen::Matrix3d X;
    X << u1, u2, u1.cross(u2);
    X = X.inverse().eval();

    Eigen::Matrix3d rot = Y * X;

    Eigen::Vector3d trans1 = (depth1[0]) * rot * Kinv * x1h[0];
    Eigen::Vector3d trans2 = s * (depth2[0]) * Kinv * x2h[0];
    Eigen::Vector3d trans = trans2 - trans1;

    MonoDepthRelativePose pose;
    pose.rotation = rot;
    pose.translation = trans;
    pose.scale = s;
    pose.focal_length1 = f;
    pose.focal_length2 = f;

    poses->push_back(pose);
  }

  return poses->size();
}

// --- relpose_monodepth_3pt_varying_focal ------------------------------------

int MonoDepthRelativePose3ptVaryingFocal(
    const std::vector<Eigen::Vector3d>& x1h,
    const std::vector<Eigen::Vector3d>& x2h,
    const std::vector<double>& depth1,
    const std::vector<double>& depth2,
    std::vector<MonoDepthRelativePose>* poses) {
  poses->clear();
  poses->reserve(1);

  double a[18];
  a[0] = x1h[0][0] * depth1[0];
  a[1] = x1h[1][0] * depth1[1];
  a[2] = x1h[2][0] * depth1[2];
  a[3] = x1h[0][1] * depth1[0];
  a[4] = x1h[1][1] * depth1[1];
  a[5] = x1h[2][1] * depth1[2];
  a[6] = depth1[0];
  a[7] = depth1[1];
  a[8] = depth1[2];

  a[9] = x2h[0][0] * depth2[0];
  a[10] = x2h[1][0] * depth2[1];
  a[11] = x2h[2][0] * depth2[2];
  a[12] = x2h[0][1] * depth2[0];
  a[13] = x2h[1][1] * depth2[1];
  a[14] = x2h[2][1] * depth2[2];
  a[15] = depth2[0];
  a[16] = depth2[1];
  a[17] = depth2[2];

  double b[18];
  b[0] = a[0] - a[1];
  b[1] = a[3] - a[4];
  b[2] = a[6] - a[7];
  b[3] = a[0] - a[2];
  b[4] = a[3] - a[5];
  b[5] = a[6] - a[8];
  b[6] = a[1] - a[2];
  b[7] = a[4] - a[5];
  b[8] = a[7] - a[8];
  b[9] = a[9] - a[10];
  b[10] = a[12] - a[13];
  b[11] = a[15] - a[16];
  b[12] = a[9] - a[11];
  b[13] = a[12] - a[14];
  b[14] = a[15] - a[17];
  b[15] = a[10] - a[11];
  b[16] = a[13] - a[14];
  b[17] = a[16] - a[17];

  Eigen::Matrix3d A;
  A << std::pow(b[0], 2) + std::pow(b[1], 2), -std::pow(b[9], 2) - std::pow(b[10], 2), -std::pow(b[11], 2),
      std::pow(b[3], 2) + std::pow(b[4], 2), -std::pow(b[12], 2) - std::pow(b[13], 2), -std::pow(b[14], 2),
      std::pow(b[6], 2) + std::pow(b[7], 2), -std::pow(b[15], 2) - std::pow(b[16], 2), -std::pow(b[17], 2);
  Eigen::Vector3d B;
  B << b[2] * b[2], b[5] * b[5], b[8] * b[8];
  Eigen::Vector3d sol = -A.partialPivLu().solve(B);

  if (sol(0) > 0 && sol(1) > 0 && sol(2) > 0) {
    double f = std::sqrt(sol(0));
    double s = std::sqrt(sol(2));
    double w = std::sqrt(sol(1) / sol(2));

    Eigen::Matrix3d K1inv;
    K1inv << f, 0, 0, 0, f, 0, 0, 0, 1;

    Eigen::Matrix3d K2inv;
    K2inv << w, 0, 0, 0, w, 0, 0, 0, 1;

    Eigen::Vector3d v1 = s * ((depth2[0]) * K2inv * x2h[0] - (depth2[1]) * K2inv * x2h[1]);
    Eigen::Vector3d v2 = s * ((depth2[0]) * K2inv * x2h[0] - (depth2[2]) * K2inv * x2h[2]);
    Eigen::Matrix3d Y;
    Y << v1, v2, v1.cross(v2);

    Eigen::Vector3d u1 = (depth1[0]) * K1inv * x1h[0] - (depth1[1]) * K1inv * x1h[1];
    Eigen::Vector3d u2 = (depth1[0]) * K1inv * x1h[0] - (depth1[2]) * K1inv * x1h[2];
    Eigen::Matrix3d X;
    X << u1, u2, u1.cross(u2);
    X = X.inverse().eval();

    Eigen::Matrix3d rot = Y * X;

    Eigen::Vector3d trans1 = (depth1[0]) * rot * K1inv * x1h[0];
    Eigen::Vector3d trans2 = s * (depth2[0]) * K2inv * x2h[0];
    Eigen::Vector3d trans = trans2 - trans1;

    double focal1 = 1.0 / f;
    double focal2 = 1.0 / w;

    MonoDepthRelativePose pose;
    pose.rotation = rot;
    pose.translation = trans;
    pose.scale = s;
    pose.focal_length1 = focal1;
    pose.focal_length2 = focal2;

    poses->push_back(pose);
  }

  return poses->size();
}

}  // namespace theia
