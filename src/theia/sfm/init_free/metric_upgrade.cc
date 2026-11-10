// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/metric_upgrade.h"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <cmath>

namespace theia {

namespace {

// Index of the symmetric entry (i, j) in the 10-vector of unknowns.
int SymmetricIndex(int i, int j) {
  if (i > j) std::swap(i, j);
  static const int kIndex[4][4] = {
      {0, 1, 2, 3}, {1, 4, 5, 6}, {2, 5, 7, 8}, {3, 6, 8, 9}};
  return kIndex[i][j];
}

// Coefficients of (P Omega P^T)_{rs} in the 10 unknowns of Omega.
Eigen::Matrix<double, 1, 10> QuadricRow(const OseCamera& P, int r, int s) {
  Eigen::Matrix<double, 1, 10> row = Eigen::Matrix<double, 1, 10>::Zero();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      row[SymmetricIndex(i, j)] += P(r, i) * P(s, j);
    }
  }
  return row;
}

Eigen::Matrix3d ProjectToRotation(const Eigen::Matrix3d& M) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = -1.0;
    R = svd.matrixU() * D * svd.matrixV().transpose();
  }
  return R;
}

double FractionInFront(const OseProblem& problem,
                       const MetricUpgradeResult& result) {
  int in_front = 0;
  int total = 0;
  for (int k = 0; k < problem.NumObservations(); ++k) {
    const int j = problem.observation_point[k];
    if (!result.point_valid[j]) continue;
    const int c = problem.observation_camera[k];
    const double depth =
        (result.rotations[c] * result.points.col(j) + result.translations[c])
            .z();
    in_front += depth > 0.0;
    ++total;
  }
  return total > 0 ? static_cast<double>(in_front) / total : 0.0;
}

}  // namespace

bool UpgradeToMetricCalibrated(const OseProblem& problem,
                               const OseCameraVector& cameras,
                               const Eigen::Matrix4Xd& points,
                               MetricUpgradeResult* result,
                               std::string* error) {
  CHECK_NOTNULL(result);
  CHECK_NOTNULL(error);
  const int n = static_cast<int>(cameras.size());
  if (n < 2) {
    *error = "Metric upgrade needs at least two cameras.";
    return false;
  }

  // Five equations per camera: off-diagonals zero, equal diagonal.
  Eigen::MatrixXd A(5 * n, 10);
  for (int i = 0; i < n; ++i) {
    const OseCamera P = cameras[i] / cameras[i].norm();
    const auto m00 = QuadricRow(P, 0, 0);
    A.row(5 * i + 0) = QuadricRow(P, 0, 1);
    A.row(5 * i + 1) = QuadricRow(P, 0, 2);
    A.row(5 * i + 2) = QuadricRow(P, 1, 2);
    A.row(5 * i + 3) = m00 - QuadricRow(P, 1, 1);
    A.row(5 * i + 4) = m00 - QuadricRow(P, 2, 2);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Matrix<double, 10, 1> omega = svd.matrixV().col(9);
  Eigen::Matrix4d Omega;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Omega(i, j) = omega[SymmetricIndex(i, j)];
    }
  }
  if (Omega.trace() < 0.0) {
    Omega = -Omega;
  }

  // Omega should be positive semidefinite with rank 3.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(Omega);
  const Eigen::Vector4d values = eig.eigenvalues();  // ascending
  const Eigen::Matrix4d vectors = eig.eigenvectors();
  if (values[1] <= 1e-12 * values[3]) {
    *error =
        "Estimated dual quadric is not rank-3 positive semidefinite; the "
        "projective reconstruction is likely degenerate.";
    return false;
  }
  Eigen::Matrix4d H;
  H.col(0) = vectors.col(3) * std::sqrt(values[3]);
  H.col(1) = vectors.col(2) * std::sqrt(values[2]);
  H.col(2) = vectors.col(1) * std::sqrt(values[1]);
  H.col(3) = vectors.col(0);
  const Eigen::Matrix4d H_inv = H.inverse();

  result->H = H;
  result->rotations.resize(n);
  result->translations.resize(n);
  double orthogonality_error = 0.0;
  for (int i = 0; i < n; ++i) {
    const OseCamera Q = cameras[i] * H;
    const Eigen::Matrix3d M = Q.leftCols<3>();
    const double s = std::cbrt(M.determinant());
    if (!std::isfinite(s) || std::abs(s) < 1e-15) {
      *error = "Upgraded camera " + std::to_string(i) + " is degenerate.";
      return false;
    }
    orthogonality_error +=
        (M * M.transpose() / (s * s) - Eigen::Matrix3d::Identity()).norm();
    result->rotations[i] = ProjectToRotation(M / s);
    result->translations[i] = Q.col(3) / s;
  }
  result->mean_orthogonality_error = orthogonality_error / n;

  result->points.resize(3, points.cols());
  result->point_valid.assign(points.cols(), 0);
  for (int j = 0; j < points.cols(); ++j) {
    const Eigen::Vector4d X = H_inv * points.col(j);
    if (std::abs(X[3]) > 1e-12 * X.head<3>().norm()) {
      result->points.col(j) = X.head<3>() / X[3];
      result->point_valid[j] = result->points.col(j).allFinite();
    } else {
      result->points.col(j).setZero();
    }
  }

  // Cheirality: (R, t, X) and (R, -t, -X) give the same projections. Pick the
  // one with most points in front of the cameras.
  result->fraction_in_front = FractionInFront(problem, *result);
  if (result->fraction_in_front < 0.5) {
    for (auto& t : result->translations) t = -t;
    result->points = -result->points;
    Eigen::Matrix4d flip = Eigen::Matrix4d::Identity();
    flip(3, 3) = -1.0;
    result->H = H * flip;
    result->fraction_in_front = FractionInFront(problem, *result);
  }
  return true;
}

}  // namespace theia
