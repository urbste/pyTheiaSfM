// Copyright (C) 2023 Steffen Urban
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Steffen Urban (urbste@gmail.com)

#include "theia/sfm/pose/mlpnp.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <math.h>
#include <limits>

#include "theia/math/nullspace.h"
#include "theia/sfm/pose/mlpnp_helper.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;

bool MLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
           const std::vector<Eigen::Matrix3d>& feature_covariances,
           const std::vector<Eigen::Vector3d>& world_points,
           Eigen::Matrix3d* solution_rotation,
           Eigen::Vector3d* solution_translation,
           bool run_refinement) {
  const size_t num_points = norm_feature_points.size();
  if (num_points < static_cast<size_t>(kMLPnPMinimumPoints) ||
      world_points.size() != num_points) {
    return false;
  }
  if (!feature_covariances.empty() &&
      feature_covariances.size() != num_points) {
    return false;
  }

  bool planar = false;
  const int num_ambiguity_points =
      std::min(static_cast<int>(num_points), 6);
  // compute the nullspace of all vectors
  std::vector<Eigen::Matrix<double, 3, 2>> nullspaces(num_points);
  MatrixXd points3(3, num_points);

  for (int i = 0; i < num_points; i++) {
    points3.col(i) = world_points[i];
    nullS_3x2_templated<double>(
      norm_feature_points[i].homogeneous().normalized(), nullspaces[i]);
  }
  

  //////////////////////////////////////
  // 1. test if we have a planar scene
  //////////////////////////////////////
  const Eigen::VectorXd mean_vector = points3.rowwise().mean();
  const Eigen::MatrixXd deviation_matrix = points3.colwise() - mean_vector;
  const Eigen::Matrix3d covariance_matrix = (deviation_matrix * deviation_matrix.transpose()) / (points3.cols() - 1);

  // now check if any 
  Eigen::FullPivHouseholderQR<Matrix3d> rankTest(covariance_matrix);
  Matrix3d eigenRot;
  eigenRot.setIdentity();

  // if yes -> transform points to new eigen frame
  if (rankTest.rank() == 2) {
    planar = true;
    // self adjoint is faster and more accurate than general eigen solvers
    // in addition this solver sorts the eigenvalues in increasing order
    Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver(covariance_matrix);
    eigenRot = eigen_solver.eigenvectors().real();
    eigenRot.transposeInPlace();
    // The first row is the plane normal; keep eigenRot a proper rotation.
    if (eigenRot.determinant() < 0.0) {
      eigenRot.row(2) *= -1.0;
    }
    // Centered points in the plane frame, so that their first coordinate
    // vanishes also for planes that do not contain the origin.
    for (size_t i = 0; i < num_points; i++) {
      points3.col(i) = eigenRot * (world_points[i] - mean_vector);
    }
  }


  //////////////////////////////////////
  // 2. stochastic model and 3. normal equations
  //////////////////////////////////////
  // Each correspondence contributes the two rows N_i^T [X_i^T (x) I_3, I_3]
  // (in the planar case without the first rotation column) to the design
  // matrix A. Instead of forming A and the 2n x 2n weight matrix P, the normal
  // matrix A^T P A is accumulated directly, with P_i = (N_i^T C_i N_i)^-1.
  const bool use_cov = feature_covariances.size() == num_points;
  Eigen::Matrix<double, 12, 12> AtPA = Eigen::Matrix<double, 12, 12>::Zero();
  Eigen::Matrix<double, 2, 12> A_i;
  for (size_t i = 0; i < num_points; ++i) {
    const Vector3d pt3_current = points3.col(i);
    const Eigen::Matrix<double, 3, 2>& N = nullspaces[i];
    A_i.setZero();
    if (planar) {
      // Columns: r12 r13 r22 r23 r32 r33 t1 t2 t3.
      for (int r = 0; r < 3; ++r) {
        A_i.col(2 * r) = N.row(r).transpose() * pt3_current[1];
        A_i.col(2 * r + 1) = N.row(r).transpose() * pt3_current[2];
      }
      A_i.middleCols<3>(6) = N.transpose();
    } else {
      // Columns: r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3.
      for (int r = 0; r < 3; ++r) {
        A_i.middleCols<3>(3 * r) = N.row(r).transpose() * pt3_current.transpose();
      }
      A_i.middleCols<3>(9) = N.transpose();
    }
    if (use_cov) {
      const Matrix2d W =
          (N.transpose() * feature_covariances[i] * N).inverse();
      AtPA.noalias() += A_i.transpose() * W * A_i;
    } else {
      AtPA.noalias() += A_i.transpose() * A_i;
    }
  }

  //////////////////////////////////////
  // 4. solve least squares
  //////////////////////////////////////
  // The solution is the eigenvector of the smallest eigenvalue of the
  // symmetric normal matrix; the eigenvalues are sorted in increasing order.
  Eigen::VectorXd result1;
  if (planar) {
    const Eigen::Matrix<double, 9, 9> AtPA9 = AtPA.topLeftCorner<9, 9>();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> eig(AtPA9);
    result1 = eig.eigenvectors().col(0);
  } else {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> eig(AtPA);
    result1 = eig.eigenvectors().col(0);
  }
	Matrix3d Rout;
	Vector3d tout;
	////////////////////////////////
	// now we treat the results differently,
	// depending on the scene (planar or not)
	////////////////////////////////
  if (planar) {
    // result1 = s * [r12 r13 r22 r23 r32 r33 t'] for the rotation
    // R' = R * eigenRot^T of the plane frame and the translation
    // t' = t + R * mean of the centered points, up to an unknown scale s whose
    // sign is resolved by the agreement with the observed bearings.
    const Vector3d c1(result1(0), result1(2), result1(4));
    const Vector3d c2(result1(1), result1(3), result1(5));
    const double scale = 1.0 / std::sqrt(c1.norm() * c2.norm());
    const Vector3d t_plane = scale * result1.segment<3>(6);
    double best_error = std::numeric_limits<double>::max();
    for (const double sign : {1.0, -1.0}) {
      Matrix3d M;
      M.col(1) = sign * scale * c1;
      M.col(2) = sign * scale * c2;
      M.col(0) = M.col(1).cross(M.col(2));
      Eigen::JacobiSVD<Matrix3d> svd_R_frob(
          M, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Matrix3d U = svd_R_frob.matrixU();
      if ((U * svd_R_frob.matrixV().transpose()).determinant() < 0.0) {
        U.col(2) *= -1.0;
      }
      const Matrix3d R = U * svd_R_frob.matrixV().transpose() * eigenRot;
      const Vector3d t = sign * t_plane - R * mean_vector;
      double error = 0.0;
      for (int p = 0; p < num_ambiguity_points; ++p) {
        error += 1.0 - (R * world_points[p] + t).normalized().dot(
                           norm_feature_points[p].homogeneous().normalized());
      }
      if (error < best_error) {
        best_error = error;
        Rout = R;
        tout = t;
      }
    }
  }
	else { // non-planar case
		Matrix3d tmp;
		// until now, we only estimated 
		// row one and two of the transposed rotation matrix
		tmp << result1(0, 0), result1(3, 0), result1(6, 0),
			   result1(1, 0), result1(4, 0), result1(7, 0),
			   result1(2, 0), result1(5, 0), result1(8, 0);
		// get the scale
		//double scale = 1.0 / 
		//	std::pow(std::abs(tmp.col(0).norm() * tmp.col(1).norm()* tmp.col(2).norm()), 1.0 / 3.0);
		// find best rotation matrix in frobenius sense
		Eigen::JacobiSVD<MatrixXd> svd_R_frob(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Scale of the estimated rotation block: the mean of its singular values
    // (the smallest one is unstable for noisy minimal samples).
    const double scale = svd_R_frob.singularValues().mean();

		Rout = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
		// test if we found a good rotation matrix
		if (Rout.determinant() < 0) {
			Rout *= -1.0;
    }
		// scale translation
		tout = Rout * (Vector3d(result1(9, 0), result1(10, 0), result1(11, 0)) / scale);
		// find correct direction in terms of reprojection error, just take the first 6 correspondences
		std::vector<double> error(2);
		std::vector<Matrix4d> Ts(2);

		for (int s = 0; s < 2; ++s) {
			error[s] = 0.0;
			Ts[s] = Matrix4d::Identity();
			Ts[s].block<3, 3>(0, 0) = Rout;
			if (s == 0) {
          Ts[s].block<3, 1>(0, 3) = tout;
      }
      else {
          Ts[s].block<3, 1>(0, 3) = -tout;
      }

			Ts[s] = Ts[s].inverse();
			for (int p = 0; p < num_ambiguity_points; ++p) {
				Vector3d v = Ts[s].block<3, 3>(0, 0) * points3.col(p) + Ts[s].block<3, 1>(0, 3);
				v = v / v.norm();
				error[s] += 1.0 - v.transpose() * norm_feature_points[p].homogeneous().normalized();
			}
		}
		if (error[0] < error[1]) {
			tout = Ts[0].block<3, 1>(0, 3);
    } else {
			tout = Ts[1].block<3, 1>(0, 3);
    }
		Rout = Ts[0].block<3, 3>(0, 0);
	}

  *(solution_rotation) = Rout;
  *solution_translation = tout;

  if (run_refinement) {
    MLPnPGaussNewton(norm_feature_points, feature_covariances, world_points, 25,
                     solution_rotation, solution_translation);
  }
  return true;
}
} // namespace theia
