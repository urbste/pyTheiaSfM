// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/rotation_weighting.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace theia {

namespace {

using Vector2dList =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

Eigen::Matrix<double, 9, 1> RowMajorVec(const Eigen::Matrix3d& M) {
  Eigen::Matrix<double, 9, 1> v;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      v[3 * r + c] = M(r, c);
    }
  }
  return v;
}

// d pi(p) / d p for pi(p) = p_{1:2} / p_3.
Eigen::Matrix<double, 2, 3> ProjectionJacobian(const Eigen::Vector3d& p) {
  const double iz = 1.0 / p.z();
  Eigen::Matrix<double, 2, 3> J;
  J << iz, 0, -p.x() * iz * iz, 0, iz, -p.y() * iz * iz;
  return J;
}

// Midpoint triangulation in the frame of camera b. Returns false if the point
// is not in front of both cameras.
bool Triangulate(const Eigen::Vector2d& m_a,
                 const Eigen::Vector2d& m_b,
                 const Eigen::Matrix3d& R,
                 const Eigen::Vector3d& t,
                 Eigen::Vector3d* X_b) {
  const Eigen::Vector3d f_b = m_b.homogeneous().normalized();
  const Eigen::Vector3d f_a = m_a.homogeneous().normalized();
  // lambda_b R f_b - lambda_a f_a = -t.
  Eigen::Matrix<double, 3, 2> A;
  A.col(0) = R * f_b;
  A.col(1) = -f_a;
  const Eigen::Matrix2d AtA = A.transpose() * A;
  if (std::abs(AtA.determinant()) < 1e-14) {
    return false;  // parallel rays
  }
  const Eigen::Vector2d lambda = AtA.ldlt().solve(-A.transpose() * t);
  if (!(lambda[0] > 0.0 && lambda[1] > 0.0)) {
    return false;
  }
  *X_b = lambda[0] * f_b;
  return true;
}

int CountInFront(const Vector2dList& m_a,
                 const Vector2dList& m_b,
                 const Eigen::Matrix3d& R,
                 const Eigen::Vector3d& t) {
  int count = 0;
  Eigen::Vector3d X;
  for (size_t k = 0; k < m_a.size(); ++k) {
    count += Triangulate(m_a[k], m_b[k], R, t, &X);
  }
  return count;
}

// Symmetric square root of a PSD matrix (negative eigenvalues clamped).
template <int N>
Eigen::Matrix<double, N, N> PsdSqrt(const Eigen::Matrix<double, N, N>& M) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> eig(
      0.5 * (M + M.transpose()));
  return eig.eigenvectors() *
         eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
         eig.eigenvectors().transpose();
}

}  // namespace

bool ComputeTwoViewRotationWeight(const Vector2dList& m_a,
                                  const Vector2dList& m_b,
                                  const Eigen::Matrix3d& R_ab,
                                  const Eigen::Vector3d& t_ab,
                                  const double normal_weight,
                                  const int min_points,
                                  TwoViewRotationWeight* weight) {
  *weight = TwoViewRotationWeight();
  const int n = static_cast<int>(std::min(m_a.size(), m_b.size()));
  if (n < std::max(min_points, 2)) {
    return false;
  }

  // Translation direction.
  Eigen::Vector3d t;
  if (t_ab.norm() > 1e-12) {
    t = t_ab.normalized();
  } else {
    Eigen::MatrixXd Q(n, 3);
    for (int k = 0; k < n; ++k) {
      Q.row(k) = (R_ab * m_b[k].homogeneous()).cross(m_a[k].homogeneous());
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeFullV);
    t = svd.matrixV().col(2);
  }
  if (CountInFront(m_a, m_b, R_ab, -t) > CountInFront(m_a, m_b, R_ab, t)) {
    t = -t;
  }

  // Rotation and translation information with the points eliminated.
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  int num_points = 0;
  for (int k = 0; k < n; ++k) {
    Eigen::Vector3d X;
    if (!Triangulate(m_a[k], m_b[k], R_ab, t, &X)) {
      continue;
    }
    const Eigen::Vector3d RX = R_ab * X;
    const Eigen::Matrix<double, 2, 3> D_a = ProjectionJacobian(RX + t);
    Eigen::Matrix<double, 4, 3> Jx;
    Jx.topRows<2>() = ProjectionJacobian(X);
    Jx.bottomRows<2>() = D_a * R_ab;
    Eigen::Matrix<double, 4, 6> Jq = Eigen::Matrix<double, 4, 6>::Zero();
    Jq.block<2, 3>(2, 0) = -D_a * Skew(RX);  // left rotation perturbation
    Jq.block<2, 3>(2, 3) = D_a;              // translation
    const Eigen::Matrix3d Hxx = Jx.transpose() * Jx;
    const Eigen::Matrix<double, 3, 6> Hxq = Jx.transpose() * Jq;
    Eigen::LDLT<Eigen::Matrix3d> ldlt(Hxx);
    H.noalias() += Jq.transpose() * Jq - Hxq.transpose() * ldlt.solve(Hxq);
    ++num_points;
  }
  if (num_points < std::max(min_points, 2)) {
    return false;
  }

  // Marginalize the translation with a pseudo-inverse: the direction of t
  // itself is a gauge (scale) direction once the points are eliminated.
  const Eigen::Matrix3d Htt = 0.5 * (H.block<3, 3>(3, 3) +
                                     H.block<3, 3>(3, 3).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(Htt);
  const double max_eig = std::max(eig.eigenvalues().maxCoeff(), 0.0);
  Eigen::Vector3d inv = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    if (eig.eigenvalues()[i] > 1e-9 * max_eig) {
      inv[i] = 1.0 / eig.eigenvalues()[i];
    }
  }
  const Eigen::Matrix3d Htt_pinv =
      eig.eigenvectors() * inv.asDiagonal() * eig.eigenvectors().transpose();
  Eigen::Matrix3d Hrr =
      H.block<3, 3>(0, 0) -
      H.block<3, 3>(0, 3) * Htt_pinv * H.block<3, 3>(0, 3).transpose();
  Hrr = 0.5 * (Hrr + Hrr.transpose());

  // Orthonormal basis of R^9 (row-major vec): tangent then normal directions.
  Matrix9d V;
  for (int i = 0; i < 3; ++i) {
    V.col(i) =
        RowMajorVec(Skew(Eigen::Vector3d::Unit(i)) * R_ab) / std::sqrt(2.0);
  }
  int col = 3;
  for (int r = 0; r < 3; ++r) {
    for (int c = r; c < 3; ++c) {
      Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
      S(r, c) = 1.0;
      S(c, r) = 1.0;
      S /= S.norm();
      V.col(col++) = RowMajorVec(S * R_ab);
    }
  }
  // The symmetric basis is already orthonormal; V is orthogonal because
  // right-multiplication by R_ab preserves the Frobenius inner product and
  // skew and symmetric matrices are orthogonal.

  Matrix9d sqrt_A = Matrix9d::Zero();
  sqrt_A.block<3, 3>(0, 0) = PsdSqrt<3>(0.5 * Hrr);
  sqrt_A.block<6, 6>(3, 3) =
      std::sqrt(std::max(normal_weight, 0.0)) *
      Eigen::Matrix<double, 6, 6>::Identity();

  weight->sqrt_weight = V * sqrt_A * V.transpose();
  weight->rotation_information = Hrr;
  weight->translation = t;
  weight->num_points = num_points;
  return weight->sqrt_weight.allFinite();
}

}  // namespace theia
