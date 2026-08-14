// Copyright (C) 2026, The pyTheiaSfM authors. All rights reserved.
//
// Fast Iterative Five point Relative Pose Estimation based on Powell's Dogleg
// method, as described in:
//
//   J. Hedborg and M. Felsberg, "Fast Iterative Five point Relative Pose
//   Estimation", Computer Vision Laboratory, Linköping University.

#include "theia/sfm/pose/fast_iterative_five_point.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "theia/sfm/estimators/estimate_relative_pose.h"
#include "theia/sfm/pose/util.h"

namespace theia {
namespace {

using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;

// Clamp helper for numerical stability.
inline double Clamp(const double val, const double min_val, const double max_val) {
  return std::max(min_val, std::min(max_val, val));
}

// Compute Rotation matrix from Cardan Euler angles R = Rx(alpha) * Ry(beta) * Rz(gamma)
// and its partial derivatives w.r.t alpha, beta, gamma.
void ComputeRotationAndDerivatives(
    const double alpha,
    const double beta,
    const double gamma,
    Matrix3d* R,
    Matrix3d* dR_dalpha,
    Matrix3d* dR_dbeta,
    Matrix3d* dR_dgamma) {
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);
  const double cg = std::cos(gamma);
  const double sg = std::sin(gamma);

  Matrix3d Rx, Ry, Rz;
  Rx << 1.0, 0.0, 0.0,
        0.0,  ca, -sa,
        0.0,  sa,  ca;

  Ry <<  cb, 0.0,  sb,
        0.0, 1.0, 0.0,
        -sb, 0.0,  cb;

  Rz <<  cg, -sg, 0.0,
         sg,  cg, 0.0,
        0.0, 0.0, 1.0;

  *R = Rx * Ry * Rz;

  if (dR_dalpha) {
    Matrix3d dRx;
    dRx << 0.0, 0.0, 0.0,
           0.0, -sa, -ca,
           0.0,  ca, -sa;
    *dR_dalpha = dRx * Ry * Rz;
  }

  if (dR_dbeta) {
    Matrix3d dRy;
    dRy << -sb, 0.0,  cb,
           0.0, 0.0, 0.0,
           -cb, 0.0, -sb;
    *dR_dbeta = Rx * dRy * Rz;
  }

  if (dR_dgamma) {
    Matrix3d dRz;
    dRz << -sg, -cg, 0.0,
            cg, -sg, 0.0,
           0.0, 0.0, 0.0;
    *dR_dgamma = Rx * Ry * dRz;
  }
}

// Compute unit translation vector from spherical angles t(theta, phi)
// and its partial derivatives w.r.t theta, phi.
void ComputeTranslationAndDerivatives(
    const double theta,
    const double phi,
    Vector3d* t,
    Vector3d* dt_dtheta,
    Vector3d* dt_dphi) {
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double sp = std::sin(phi);
  const double cp = std::cos(phi);

  *t = Vector3d(st * cp, st * sp, ct);

  if (dt_dtheta) {
    *dt_dtheta = Vector3d(ct * cp, ct * sp, -st);
  }

  if (dt_dphi) {
    *dt_dphi = Vector3d(-st * sp, st * cp, 0.0);
  }
}

// Helper to evaluate residual vector r(w) and analytical Jacobian J(w).
void EvaluateResidualsAndJacobian(
    const std::vector<Vector3d>& x1h,
    const std::vector<Vector3d>& x2h,
    const Matrix<double, 5, 1>& w,
    Eigen::VectorXd* residuals,
    Eigen::MatrixXd* jacobian) {
  const size_t num_points = x1h.size();
  residuals->resize(num_points);
  if (jacobian) {
    jacobian->resize(num_points, 5);
  }

  Matrix3d R, dR_dalpha, dR_dbeta, dR_dgamma;
  ComputeRotationAndDerivatives(
      w(0), w(1), w(2), &R,
      jacobian ? &dR_dalpha : nullptr,
      jacobian ? &dR_dbeta : nullptr,
      jacobian ? &dR_dgamma : nullptr);

  Vector3d t, dt_dtheta, dt_dphi;
  ComputeTranslationAndDerivatives(
      w(3), w(4), &t,
      jacobian ? &dt_dtheta : nullptr,
      jacobian ? &dt_dphi : nullptr);

  const Matrix3d tx = CrossProductMatrix(t);
  const Matrix3d E = tx * R;

  // Flattened row-major E: [e11, e12, e13, e21, e22, e23, e31, e32, e33]
  Matrix<double, 1, 9> E_tilde;
  E_tilde << E(0, 0), E(0, 1), E(0, 2),
             E(1, 0), E(1, 1), E(1, 2),
             E(2, 0), E(2, 1), E(2, 2);

  // Partial derivatives of E w.r.t w_j
  Matrix<double, 1, 9> dE_tilde[5];
  if (jacobian) {
    const Matrix3d dE_0 = tx * dR_dalpha;
    const Matrix3d dE_1 = tx * dR_dbeta;
    const Matrix3d dE_2 = tx * dR_dgamma;
    const Matrix3d dE_3 = CrossProductMatrix(dt_dtheta) * R;
    const Matrix3d dE_4 = CrossProductMatrix(dt_dphi) * R;

    dE_tilde[0] << dE_0(0, 0), dE_0(0, 1), dE_0(0, 2),
                   dE_0(1, 0), dE_0(1, 1), dE_0(1, 2),
                   dE_0(2, 0), dE_0(2, 1), dE_0(2, 2);
    dE_tilde[1] << dE_1(0, 0), dE_1(0, 1), dE_1(0, 2),
                   dE_1(1, 0), dE_1(1, 1), dE_1(1, 2),
                   dE_1(2, 0), dE_1(2, 1), dE_1(2, 2);
    dE_tilde[2] << dE_2(0, 0), dE_2(0, 1), dE_2(0, 2),
                   dE_2(1, 0), dE_2(1, 1), dE_2(1, 2),
                   dE_2(2, 0), dE_2(2, 1), dE_2(2, 2);
    dE_tilde[3] << dE_3(0, 0), dE_3(0, 1), dE_3(0, 2),
                   dE_3(1, 0), dE_3(1, 1), dE_3(1, 2),
                   dE_3(2, 0), dE_3(2, 1), dE_3(2, 2);
    dE_tilde[4] << dE_4(0, 0), dE_4(0, 1), dE_4(0, 2),
                   dE_4(1, 0), dE_4(1, 1), dE_4(1, 2),
                   dE_4(2, 0), dE_4(2, 1), dE_4(2, 2);
  }

  for (size_t i = 0; i < num_points; ++i) {
    const double u1 = x1h[i].x() / x1h[i].z();
    const double v1 = x1h[i].y() / x1h[i].z();
    const double u2 = x2h[i].x() / x2h[i].z();
    const double v2 = x2h[i].y() / x2h[i].z();

    Matrix<double, 9, 1> q;
    q << u2 * u1, u2 * v1, u2,
         v2 * u1, v2 * v1, v2,
         u1,      v1,      1.0;

    const double l1_x = u2 * E(0, 0) + v2 * E(1, 0) + E(2, 0);
    const double l1_y = u2 * E(0, 1) + v2 * E(1, 1) + E(2, 1);
    const double s = l1_x * l1_x + l1_y * l1_y;
    const double sqrt_s = std::sqrt(std::max(s, 1e-16));
    const double eq = E_tilde * q;

    (*residuals)(i) = eq / sqrt_s;

    if (jacobian) {
      const double inv_sqrt_s = 1.0 / sqrt_s;
      const double inv_2s = 1.0 / (2.0 * std::max(s, 1e-16));

      for (int j = 0; j < 5; ++j) {
        const double dl1_x = u2 * dE_tilde[j](0) + v2 * dE_tilde[j](3) + dE_tilde[j](6);
        const double dl1_y = u2 * dE_tilde[j](1) + v2 * dE_tilde[j](4) + dE_tilde[j](7);
        const double ds = 2.0 * l1_x * dl1_x + 2.0 * l1_y * dl1_y;

        const Matrix<double, 1, 9> d_row =
            inv_sqrt_s * (dE_tilde[j] - inv_2s * ds * E_tilde);
        (*jacobian)(i, j) = d_row * q;
      }
    }
  }
}

}  // namespace

void FastIterativePoseParametersToRotationAndTranslation(
    const Matrix<double, 5, 1>& w,
    Matrix3d* rotation,
    Vector3d* translation) {
  ComputeRotationAndDerivatives(w(0), w(1), w(2), rotation, nullptr, nullptr, nullptr);
  ComputeTranslationAndDerivatives(w(3), w(4), translation, nullptr, nullptr);
}

Matrix<double, 5, 1>
FastIterativeRotationAndTranslationToPoseParameters(
    const Matrix3d& rotation,
    const Vector3d& translation) {
  Matrix<double, 5, 1> w;

  // Extract Euler Cardan angles from R = Rx(alpha) * Ry(beta) * Rz(gamma)
  // R(0, 2) = sin(beta)
  const double sin_beta = Clamp(rotation(0, 2), -1.0, 1.0);
  w(1) = std::asin(sin_beta);
  const double cos_beta = std::cos(w(1));

  if (std::abs(cos_beta) > 1e-7) {
    w(0) = std::atan2(-rotation(1, 2), rotation(2, 2));
    w(2) = std::atan2(-rotation(0, 1), rotation(0, 0));
  } else {
    // Gimbal lock
    w(0) = std::atan2(rotation(2, 1), rotation(1, 1));
    w(2) = 0.0;
  }

  // Extract spherical coordinates for unit translation vector t
  Vector3d t_unit = translation;
  const double norm = t_unit.norm();
  if (norm > 1e-12) {
    t_unit /= norm;
  } else {
    t_unit = Vector3d(0.0, 0.0, 1.0);
  }

  const double cos_theta = Clamp(t_unit.z(), -1.0, 1.0);
  w(3) = std::acos(cos_theta);
  const double sin_theta = std::sin(w(3));

  if (std::abs(sin_theta) > 1e-7) {
    w(4) = std::atan2(t_unit.y(), t_unit.x());
  } else {
    w(4) = 0.0;
  }

  return w;
}

int FastIterativeFivePoint(
    const std::vector<Vector3d>& x1h,
    const std::vector<Vector3d>& x2h,
    const FastIterativeFivePointOptions& options,
    std::vector<Matrix3d>* essential_matrices,
    std::vector<RelativePose>* relative_poses) {
  essential_matrices->clear();
  if (relative_poses) {
    relative_poses->clear();
  }

  if (x1h.size() < 5 || x2h.size() < 5 || x1h.size() != x2h.size()) {
    return 0;
  }

  // Initialize parameter vector w from prior (default: zero angles -> forward motion).
  Matrix<double, 5, 1> w =
      FastIterativeRotationAndTranslationToPoseParameters(
          options.prior_rotation, options.prior_translation);

  double delta = options.initial_trust_region_radius;
  Eigen::VectorXd r;
  Eigen::MatrixXd J;

  EvaluateResidualsAndJacobian(x1h, x2h, w, &r, &J);
  double cost = 0.5 * r.squaredNorm();

  bool converged = false;

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    const Matrix<double, 5, 1> g = J.transpose() * r;

    if (g.cwiseAbs().maxCoeff() < options.gradient_tolerance ||
        r.cwiseAbs().maxCoeff() < options.residual_tolerance) {
      converged = true;
      break;
    }

    // 1. Newton-Raphson step: solve J * h_nr = -r (or normal eq for N > 5)
    Matrix<double, 5, 1> h_nr;
    if (x1h.size() == 5) {
      const Matrix<double, 5, 5> J5 = J.block<5, 5>(0, 0);
      Eigen::PartialPivLU<Matrix<double, 5, 5>> lu(J5);
      if (lu.rcond() < 1e-12) {
        // Near-singular Jacobian: fall back to damped normal solve
        const Matrix<double, 5, 5> H = J5.transpose() * J5 + 1e-6 * Matrix<double, 5, 5>::Identity();
        h_nr = H.ldlt().solve(-g);
      } else {
        h_nr = lu.solve(-r.head<5>());
      }
    } else {
      const Matrix<double, 5, 5> H = J.transpose() * J;
      h_nr = (H + 1e-8 * Matrix<double, 5, 5>::Identity()).ldlt().solve(-g);
    }

    // 2. Steepest descent Cauchy step: h_sd = -alpha * g
    const Eigen::VectorXd Jg = J * g;
    const double Jg_sqnorm = Jg.squaredNorm();
    double alpha = 0.0;
    if (Jg_sqnorm > 1e-16) {
      alpha = g.squaredNorm() / Jg_sqnorm;
    }
    const Matrix<double, 5, 1> h_sd = -alpha * g;

    // 3. Compute Dogleg step h_dl based on trust region delta
    Matrix<double, 5, 1> h_dl;
    const double h_nr_norm = h_nr.norm();
    const double h_sd_norm = h_sd.norm();

    if (h_nr_norm <= delta) {
      h_dl = h_nr;
    } else if (h_sd_norm >= delta) {
      h_dl = -(delta / std::max(g.norm(), 1e-16)) * g;
    } else {
      // Find beta in [0, 1] such that ||h_sd + beta * (h_nr - h_sd)||^2 = delta^2
      const Matrix<double, 5, 1> d = h_nr - h_sd;
      const double c = h_sd.dot(d);
      const double d_sqnorm = d.squaredNorm();
      const double sqrt_term = std::sqrt(std::max(0.0, c * c + d_sqnorm * (delta * delta - h_sd_norm * h_sd_norm)));
      const double beta = (-c + sqrt_term) / std::max(d_sqnorm, 1e-16);
      h_dl = h_sd + beta * d;
    }

    const double h_norm = h_dl.norm();
    if (h_norm < options.step_tolerance || delta < options.trust_region_tolerance) {
      converged = true;
      break;
    }

    // 4. Evaluate new point and gain factor rho
    const Matrix<double, 5, 1> w_new = w + h_dl;
    Eigen::VectorXd r_new;
    EvaluateResidualsAndJacobian(x1h, x2h, w_new, &r_new, nullptr);
    const double cost_new = 0.5 * r_new.squaredNorm();

    const double actual_reduction = cost - cost_new;
    const Eigen::VectorXd Jh_dl = J * h_dl;
    const double predicted_reduction = -g.dot(h_dl) - 0.5 * Jh_dl.squaredNorm();

    double rho = 0.0;
    if (std::abs(predicted_reduction) > 1e-16) {
      rho = actual_reduction / predicted_reduction;
    }

    // Update trust region radius
    if (rho > 0.75) {
      delta = std::max(delta, 3.0 * h_norm);
    } else if (rho < 0.25) {
      delta = 0.5 * delta;
    }

    // Accept or reject step
    if (rho > 0.0) {
      w = w_new;
      r = r_new;
      cost = cost_new;
      EvaluateResidualsAndJacobian(x1h, x2h, w, &r, &J);
    }
  }

  // Final check: Cost or max residual must be sufficiently small to accept the solution.
  // For 5 correspondences without extreme noise, residual < 0.1 rad / normalized epipolar units.
  if (r.cwiseAbs().maxCoeff() > 0.5) {
    return 0;
  }

  Matrix3d R;
  Vector3d t;
  FastIterativePoseParametersToRotationAndTranslation(w, &R, &t);
  const Matrix3d E = CrossProductMatrix(t) * R;

  essential_matrices->push_back(E);

  if (relative_poses) {
    RelativePose pose;
    pose.essential_matrix = E;
    pose.rotation = R;
    pose.position = -R.transpose() * t;
    relative_poses->push_back(pose);
  }

  return 1;
}

int FastIterativeFivePoint(
    const std::vector<Vector2d>& image1_points,
    const std::vector<Vector2d>& image2_points,
    const FastIterativeFivePointOptions& options,
    std::vector<Matrix3d>* essential_matrices,
    std::vector<RelativePose>* relative_poses) {
  std::vector<Vector3d> x1h, x2h;
  x1h.reserve(image1_points.size());
  x2h.reserve(image2_points.size());
  for (size_t i = 0; i < image1_points.size(); ++i) {
    x1h.emplace_back(image1_points[i].homogeneous());
    x2h.emplace_back(image2_points[i].homogeneous());
  }
  return FastIterativeFivePoint(x1h, x2h, options, essential_matrices, relative_poses);
}

}  // namespace theia
