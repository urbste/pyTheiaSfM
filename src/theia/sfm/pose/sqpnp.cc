//
// sqpnp.cpp
//
// George Terzakis (terzakig-at-hotmail-dot-com), September 2020
//
// Implementation of SQPnP as described in the paper:
//
// "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point
// Problem" by G. Terzakis and M. Lourakis
//  	 a) Paper:
//  http://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf
//       b) Supplementary:
//       https://www.ecva.net/papers/eccv_2020/papers_ECCV/papers/123460460.pdf

#include "theia/sfm/pose/sqpnp.h"
#include "theia/sfm/pose/sqpnp_helper.h"
#include <iostream>

namespace theia {

//
// Run sequential quadratic programming on orthogonal matrices
SQPSolution RunSQP(
    const Matrix91& r0,
    const Matrix99& Omega,
    std::function<void(const Matrix91&, Matrix91&)> NearestRotationMatrixFunc) {
  Matrix91 r = r0;

  double delta_squared_norm = std::numeric_limits<double>::max();
  Matrix91 delta;
  int step = 0;

  while (delta_squared_norm > DEFAULT_SQP_SQUARED_TOLERANCE &&
         step++ < DEFAULT_SQP_SQUARED_TOLERANCE) {
    SolveSQPSystem(r, Omega, delta);
    r += delta;
    delta_squared_norm = delta.squaredNorm();
  }

  SQPSolution solution;
  solution.num_iterations = step;
  solution.r = r;
  // clear the estimate and/or flip the matrix sign if necessary
  double det_r = Determinant9x1(solution.r);
  if (det_r < 0) {
    solution.r = -r;
    det_r = -det_r;
  }
  if (det_r > DEFAULT_SQP_DET_THRESHOLD) {
    NearestRotationMatrixFunc(solution.r, solution.r_hat);
  } else {
    solution.r_hat = solution.r;
  }
  return solution;
}

// Solve
bool SQPnP(const std::vector<Eigen::Vector2d>& feature_positions,
           const std::vector<Eigen::Vector3d>& world_points,
           std::vector<Eigen::Quaterniond>* solution_rotation,
           std::vector<Eigen::Vector3d>* solution_translation) {

  size_t N = world_points.size();
  if (N != feature_positions.size() || N < 3 || feature_positions.size() < 3) {
    return false;
  }
  std::vector<double> weights;
  weights.resize(N, 1.0);

  int num_null_vectors =
      -1;  // set to -1 in case we never make it to the decomposition of Omega
  Matrix99 Omega_ = Matrix99::Zero();
  double sum_wx = 0, sum_wy = 0, sum_wx2_plus_wy2 = 0, sum_w = 0;
  double sum_X = 0, sum_Y = 0, sum_Z = 0;

  Matrix39 QA = Matrix39::Zero();  // Sum( Qi*Ai )

  for (size_t i = 0; i < N; i++) {
    const double& w = weights.at(i);

    const double wx = feature_positions[i][0] * w,
                 wy = feature_positions[i][1] * w,
                 wsq_norm_m = w * feature_positions[i].squaredNorm();
    sum_wx += wx;
    sum_wy += wy;
    sum_wx2_plus_wy2 += wsq_norm_m;
    sum_w += w;

    double X = world_points[i][0], Y = world_points[i][1],
           Z = world_points[i][2];
    sum_X += X;
    sum_Y += Y;
    sum_Z += Z;

    // Accumulate Omega by kronecker( Qi, Mi*Mi' ) = A'*Qi*Ai. NOTE: Skipping
    // block (3:5, 3:5) because its same as (0:2, 0:2)
    const double X2 = X * X, XY = X * Y, XZ = X * Z, Y2 = Y * Y, YZ = Y * Z,
                 Z2 = Z * Z;

    // a. Block (0:2, 0:2) populated by Mi*Mi'. NOTE: Only upper triangle
    Omega_(0, 0) += w * X2;
    Omega_(0, 1) += w * XY;
    Omega_(0, 2) += w * XZ;
    Omega_(1, 1) += w * Y2;
    Omega_(1, 2) += w * YZ;
    Omega_(2, 2) += w * Z2;

    // b. Block (0:2, 6:8) populated by -x*Mi*Mi'. NOTE: Only upper triangle
    Omega_(0, 6) += -wx * X2;
    Omega_(0, 7) += -wx * XY;
    Omega_(0, 8) += -wx * XZ;
    Omega_(1, 7) += -wx * Y2;
    Omega_(1, 8) += -wx * YZ;
    Omega_(2, 8) += -wx * Z2;
    // c. Block (3:5, 6:8) populated by -y*Mi*Mi'. NOTE: Only upper triangle
    Omega_(3, 6) += -wy * X2;
    Omega_(3, 7) += -wy * XY;
    Omega_(3, 8) += -wy * XZ;
    Omega_(4, 7) += -wy * Y2;
    Omega_(4, 8) += -wy * YZ;
    Omega_(5, 8) += -wy * Z2;

    // d. Block (6:8, 6:8) populated by (x^2+y^2)*Mi*Mi'. NOTE: Only upper
    // triangle
    Omega_(6, 6) += wsq_norm_m * X2;
    Omega_(6, 7) += wsq_norm_m * XY;
    Omega_(6, 8) += wsq_norm_m * XZ;
    Omega_(7, 7) += wsq_norm_m * Y2;
    Omega_(7, 8) += wsq_norm_m * YZ;
    Omega_(8, 8) += wsq_norm_m * Z2;

    // Accumulating Qi*Ai in QA
    const double wX = w * X, wY = w * Y, wZ = w * Z;
    QA(0, 0) += wX;
    QA(0, 1) += wY;
    QA(0, 2) += wZ;
    QA(0, 6) += -wx * X;
    QA(0, 7) += -wx * Y;
    QA(0, 8) += -wx * Z;
    QA(1, 3) += wX;
    QA(1, 4) += wY;
    QA(1, 5) += wZ;
    QA(1, 6) += -wy * X;
    QA(1, 7) += -wy * Y;
    QA(1, 8) += -wy * Z;

    QA(2, 0) += -wx * X;
    QA(2, 1) += -wx * Y;
    QA(2, 2) += -wx * Z;
    QA(2, 3) += -wy * X;
    QA(2, 4) += -wy * Y;
    QA(2, 5) += -wy * Z;
    QA(2, 6) += wsq_norm_m * X;
    QA(2, 7) += wsq_norm_m * Y;
    QA(2, 8) += wsq_norm_m * Z;
  }

  // Fill-in lower triangles of off-diagonal blocks (0:2, 6:8), (3:5, 6:8) and
  // (6:8, 6:8)
  Omega_(1, 6) = Omega_(0, 7);
  Omega_(2, 6) = Omega_(0, 8);
  Omega_(2, 7) = Omega_(1, 8);
  Omega_(4, 6) = Omega_(3, 7);
  Omega_(5, 6) = Omega_(3, 8);
  Omega_(5, 7) = Omega_(4, 8);
  Omega_(7, 6) = Omega_(6, 7);
  Omega_(8, 6) = Omega_(6, 8);
  Omega_(8, 7) = Omega_(7, 8);

  // Fill-in upper triangle of block (3:5, 3:5)
  Omega_(3, 3) = Omega_(0, 0);
  Omega_(3, 4) = Omega_(0, 1);
  Omega_(3, 5) = Omega_(0, 2);
  Omega_(4, 4) = Omega_(1, 1);
  Omega_(4, 5) = Omega_(1, 2);
  Omega_(5, 5) = Omega_(2, 2);
  // Fill lower triangle of Omega
  Omega_(1, 0) = Omega_(0, 1);
  Omega_(2, 0) = Omega_(0, 2);
  Omega_(2, 1) = Omega_(1, 2);
  Omega_(3, 0) = Omega_(0, 3);
  Omega_(3, 1) = Omega_(1, 3);
  Omega_(3, 2) = Omega_(2, 3);
  Omega_(4, 0) = Omega_(0, 4);
  Omega_(4, 1) = Omega_(1, 4);
  Omega_(4, 2) = Omega_(2, 4);
  Omega_(4, 3) = Omega_(3, 4);
  Omega_(5, 0) = Omega_(0, 5);
  Omega_(5, 1) = Omega_(1, 5);
  Omega_(5, 2) = Omega_(2, 5);
  Omega_(5, 3) = Omega_(3, 5);
  Omega_(5, 4) = Omega_(4, 5);
  Omega_(6, 0) = Omega_(0, 6);
  Omega_(6, 1) = Omega_(1, 6);
  Omega_(6, 2) = Omega_(2, 6);
  Omega_(6, 3) = Omega_(3, 6);
  Omega_(6, 4) = Omega_(4, 6);
  Omega_(6, 5) = Omega_(5, 6);
  Omega_(7, 0) = Omega_(0, 7);
  Omega_(7, 1) = Omega_(1, 7);
  Omega_(7, 2) = Omega_(2, 7);
  Omega_(7, 3) = Omega_(3, 7);
  Omega_(7, 4) = Omega_(4, 7);
  Omega_(7, 5) = Omega_(5, 7);
  Omega_(7, 6) = Omega_(6, 7);
  Omega_(8, 0) = Omega_(0, 8);
  Omega_(8, 1) = Omega_(1, 8);
  Omega_(8, 2) = Omega_(2, 8);
  Omega_(8, 3) = Omega_(3, 8);
  Omega_(8, 4) = Omega_(4, 8);
  Omega_(8, 5) = Omega_(5, 8);
  Omega_(8, 6) = Omega_(6, 8);
  Omega_(8, 7) = Omega_(7, 8);

  // Q = Sum( wi*Qi ) = Sum( [ wi, 0, -wi*xi; 0, 1, -wi*yi; -wi*xi, -wi*yi,
  // wi*(xi^2 + yi^2)] )
  Matrix33 Q;
  Q(0, 0) = sum_w;
  Q(0, 1) = 0;
  Q(0, 2) = -sum_wx;
  Q(1, 0) = 0;
  Q(1, 1) = sum_w;
  Q(1, 2) = -sum_wy;
  Q(2, 0) = -sum_wx;
  Q(2, 1) = -sum_wy;
  Q(2, 2) = sum_wx2_plus_wy2;

  // Qinv = inv( Q ) = inv( Sum( Qi) )
  Matrix33 Qinv;
  InvertSymmetric3x3(Q, Qinv);

  // Compute P = -inv( Sum(wi*Qi) ) * Sum( wi*Qi*Ai ) = -Qinv * QA
  const Matrix39 P = -Qinv * QA;
  // Complete Omega (i.e., Omega = Sum(A'*Qi*A') + Sum(Qi*Ai)'*P = Sum(A'*Qi*A')
  // + Sum(Qi*Ai)'*inv(Sum(Qi))*Sum( Qi*Ai)
  Omega_ += QA.transpose() * P;

  // Finally, decompose Omega
  Eigen::JacobiSVD<Matrix99> svd(Omega_, Eigen::ComputeFullU);

  const Matrix99 U = svd.matrixU();
  const Matrix91 s = svd.singularValues();

  num_null_vectors = 0;
  // Find dimension of null space
  while (s[7 - num_null_vectors] < DEFAULT_RANK_TOLERANCE) {
      num_null_vectors++;
  }
  // Dimension of null space of Omega must be <= 6
  if (++num_null_vectors > 6) {
    return false;
  }

  // 3D point mean (for cheirality checks)
  const double inv_n = 1.0 / N;
  Matrix31 point_mean;
  point_mean << sum_X * inv_n, sum_Y * inv_n, sum_Z * inv_n;

  std::function<void(const Matrix91&, Matrix91&)> NearestRotationMatrixFunc;
  // Assign nearest rotation method
  if (DEFAULT_NEAREST_ROTATION_METHOD == NearestRotationMethod::FOAM) {
    NearestRotationMatrixFunc = NearestRotationMatrix_FOAM;
  } else {
    NearestRotationMatrixFunc = NearestRotationMatrix_SVD;
  }

  double min_sq_error = std::numeric_limits<double>::max();
  int num_eigen_points = num_null_vectors > 0 ? num_null_vectors : 1;
  // clear solutions
  SQPSolution solutions_[18];
  int num_solutions_ = 0;
  for (int i = 9 - num_eigen_points; i < 9; i++) {
    // NOTE: No need to scale by sqrt(3) here, but better be there for other
    // computations (i.e., orthogonality test)
    const Matrix91 e = SQRT3 * U.col(i); // Eigen::Map<Matrix91>(U.block<9, 1>(0, i).data());
    double orthogonality_sq_error = OrthogonalityError(e);
    // Find nearest rotation vector
    SQPSolution solution[2];

    // Avoid SQP if e is orthogonal
    if (orthogonality_sq_error <
        DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD) {
      solution[0].r_hat = Determinant9x1(e) * e;
      solution[0].t = P * solution[0].r_hat;
      solution[0].num_iterations = 0;

      HandleSolution(Omega_,
                     point_mean,
                     solution[0],
                     solutions_,
                     min_sq_error,
                     num_solutions_);
    } else {
      NearestRotationMatrixFunc(e, solution[0].r);
      solution[0] = RunSQP(solution[0].r, Omega_, NearestRotationMatrixFunc);
      solution[0].t = P * solution[0].r_hat;
      HandleSolution(Omega_,
                     point_mean,
                     solution[0],
                     solutions_,
                     min_sq_error,
                     num_solutions_);

      NearestRotationMatrixFunc(-e, solution[1].r);
      solution[1] = RunSQP(solution[1].r, Omega_, NearestRotationMatrixFunc);
      solution[1].t = P * solution[1].r_hat;
      HandleSolution(Omega_,
                     point_mean,
                     solution[1],
                     solutions_,
                     min_sq_error,
                     num_solutions_);
    }
  }

  int c = 1;
  while (min_sq_error > 3 * s[9 - num_eigen_points - c] &&
         9 - num_eigen_points - c > 0) {
    int index = 9 - num_eigen_points - c;

    const Matrix91 e = U.col(index); // Eigen::Map<Matrix91>(U.block<9, 1>(0, index).data());
    SQPSolution solution[2];

    NearestRotationMatrixFunc(e, solution[0].r);
    solution[0] = RunSQP(solution[0].r, Omega_, NearestRotationMatrixFunc);
    solution[0].t = P * solution[0].r_hat;
    HandleSolution(Omega_,
                   point_mean,
                   solution[0],
                   solutions_,
                   min_sq_error,
                   num_solutions_);

    NearestRotationMatrixFunc(-e, solution[1].r);
    solution[1] = RunSQP(solution[1].r, Omega_, NearestRotationMatrixFunc);
    solution[1].t = P * solution[1].r_hat;
    HandleSolution(Omega_,
                   point_mean,
                   solution[1],
                   solutions_,
                   min_sq_error,
                   num_solutions_);

    c++;
  }

  for (int i = 0; i < num_solutions_; i++) {
    Eigen::Map<Matrix33> R(solutions_[i].r_hat.data(),3,3);
    solution_rotation->emplace_back(Eigen::Quaterniond(R.transpose()));
    solution_translation->emplace_back(solutions_[i].t);
  }
  return solution_rotation->size() > 0;
}

}  // namespace theia
