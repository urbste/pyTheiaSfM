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

#include "theia/math/nullspace.h"
#include "theia/sfm/pose/mlpnp_helper.h"

#include <ceres/rotation.h>

#include <iostream>

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;

// Function to check if a matrix is planar
bool IsPlanar(const Matrix3d& covariance_matrix) {
    Eigen::FullPivHouseholderQR<Matrix3d> rankTest(covariance_matrix);
    return rankTest.rank() == 2;
}

// Function to compute the covariance matrix
Matrix3d ComputeCovariance(const MatrixXd& points3) {
    const Eigen::VectorXd mean_vector = points3.rowwise().mean();
    const Eigen::MatrixXd deviation_matrix = points3.colwise() - mean_vector;
    return (deviation_matrix * deviation_matrix.transpose()) / (points3.cols() - 1);
}

Eigen::Matrix3d BearingJac(const Eigen::Vector3d& vec) {
  return (Eigen::Matrix3d::Identity() - vec * vec.transpose()) / vec.norm();
}

bool MLPnP(const std::vector<Eigen::Vector2d>& norm_feature_points,
           const std::vector<Eigen::Matrix3d>& feature_covariances,
           const std::vector<Eigen::Vector3d>& world_points,
           Eigen::Matrix3d* solution_rotation,
           Eigen::Vector3d* solution_translation) {

  const size_t num_points = norm_feature_points.size();
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
  Matrix3d covariance_matrix = ComputeCovariance(points3);
  bool is_planar = IsPlanar(covariance_matrix);
  Matrix3d eigenRot;
  eigenRot.setIdentity();

  // if yes -> transform points to new eigen frame
  if (is_planar) {
    // self adjoint is faster and more accurate than general eigen solvers
    // in addition this solver sorts the eigenvalues in increasing order
    Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver(covariance_matrix);
    eigenRot = eigen_solver.eigenvectors().real().transpose();
    for (size_t i = 0; i < num_points; i++) {
      points3.col(i) = eigenRot * world_points[i];
    }
  }


  //////////////////////////////////////
  // 2. stochastic model
  //////////////////////////////////////
  Eigen::SparseMatrix<double> P(2 * num_points, 2 * num_points);
  const bool use_cov = feature_covariances.size() == num_points;
  P.setIdentity(); // standard

  // if we do have covariance information 
  // -> fill covariance matrix
  if (use_cov) {
    int l = 0;
    for (size_t i = 0; i < num_points; ++i) {
      // MLPnP paper equation (6) and (9)
      // Projecting the singular feature covariance matrix Exx (3x3) to the reduced space (2x2) Evrvr
      const Matrix3d J = BearingJac(norm_feature_points[i].homogeneous());
      const Matrix3d Evv = J * feature_covariances[i] * J.transpose();

      Matrix2d Evrvr = (nullspaces[i].transpose() * Evv * nullspaces[i]).inverse();
      P.coeffRef(l, l) = Evrvr(0, 0);
      P.coeffRef(l, l + 1) = Evrvr(0, 1);
      P.coeffRef(l + 1, l) = Evrvr(1, 0);
      P.coeffRef(l + 1, l + 1) = Evrvr(1, 1);
      l += 2;
    }
  }

  //////////////////////////////////////
  // 3. fill the design matrix A
  //////////////////////////////////////
  const int rowsA = 2 * num_points;
  int colsA = 12;
  MatrixXd A;
  if (is_planar) {
    colsA = 9;
    A = MatrixXd(rowsA, 9);
  }
  else {
	  A = MatrixXd(rowsA, 12);
  }
  A.setZero();

  // fill design matrix
  if (is_planar) {
    for (size_t i = 0; i < num_points; i++) {
        const Vector3d pt3_current = points3.col(i);
        // r12
        A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[1];
        A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[1];
        // r13
        A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[2];
        A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[2];
        // r22
        A(2 * i, 2) = nullspaces[i](1, 0) * pt3_current[1];
        A(2 * i + 1, 2) = nullspaces[i](1, 1)* pt3_current[1];
        // r23
        A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[2];
        A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[2];
        // r32
        A(2 * i, 4) = nullspaces[i](2, 0) * pt3_current[1];
        A(2 * i + 1, 4) = nullspaces[i](2, 1) * pt3_current[1];
        // r33
        A(2 * i, 5) = nullspaces[i](2, 0) * pt3_current[2];
        A(2 * i + 1, 5) = nullspaces[i](2, 1) * pt3_current[2];
        // t1
        A(2 * i, 6) = nullspaces[i](0, 0);
        A(2 * i + 1, 6) = nullspaces[i](0, 1);
        // t2
        A(2 * i, 7) = nullspaces[i](1, 0);
        A(2 * i + 1, 7) = nullspaces[i](1, 1);
        // t3
        A(2 * i, 8) = nullspaces[i](2, 0);
        A(2 * i + 1, 8) = nullspaces[i](2, 1);
    }
  } else {
    for (size_t i = 0; i < num_points; i++) {
        const Vector3d pt3_current = points3.col(i);
        // r11
        A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[0];
        A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[0];
        // r12
        A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[1];
        A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[1];
        // r13
        A(2 * i, 2) = nullspaces[i](0, 0) * pt3_current[2];
        A(2 * i + 1, 2) = nullspaces[i](0, 1) * pt3_current[2];
        // r21
        A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[0];
        A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[0];
        // r22
        A(2 * i, 4) = nullspaces[i](1, 0) * pt3_current[1];
        A(2 * i + 1, 4) = nullspaces[i](1, 1)* pt3_current[1];
        // r23
        A(2 * i, 5) = nullspaces[i](1, 0) * pt3_current[2];
        A(2 * i + 1, 5) = nullspaces[i](1, 1) * pt3_current[2];
        // r31
        A(2 * i, 6) = nullspaces[i](2, 0) * pt3_current[0];
        A(2 * i + 1, 6) = nullspaces[i](2, 1) * pt3_current[0];
        // r32
        A(2 * i, 7) = nullspaces[i](2, 0) * pt3_current[1];
        A(2 * i + 1, 7) = nullspaces[i](2, 1) * pt3_current[1];
        // r33
        A(2 * i, 8) = nullspaces[i](2, 0) * pt3_current[2];
        A(2 * i + 1, 8) = nullspaces[i](2, 1) * pt3_current[2];
        // t1
        A(2 * i, 9) = nullspaces[i](0, 0);
        A(2 * i + 1, 9) = nullspaces[i](0, 1);
        // t2
        A(2 * i, 10) = nullspaces[i](1, 0);
        A(2 * i + 1, 10) = nullspaces[i](1, 1);
        // t3
        A(2 * i, 11) = nullspaces[i](2, 0);
        A(2 * i + 1, 11) = nullspaces[i](2, 1);
    }
  }

	//////////////////////////////////////
	// 4. solve least squares
	//////////////////////////////////////
	MatrixXd AtPA;
	if (use_cov) {
    AtPA = A.transpose() * P * A;
  }	
	else {
    AtPA = A.transpose() * A;
  }

	Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV);
	MatrixXd result1 = svd_A.matrixV().col(colsA - 1);
	Matrix3d Rout;
	Vector3d tout;
	////////////////////////////////
	// now we treat the results differently,
	// depending on the scene (planar or not)
	////////////////////////////////
	if (is_planar) { // planar case
		Matrix3d tmp;
		// until now, we only estimated 
		// row one and two of the transposed rotation matrix
		tmp << 0.0, result1(0, 0), result1(1, 0),
			     0.0, result1(2, 0), result1(3, 0),
			     0.0, result1(4, 0), result1(5, 0);
    
		// row 3
		tmp.col(0) = tmp.col(1).cross(tmp.col(2));
		tmp.transposeInPlace();

		// find best rotation matrix in frobenius sense
		Eigen::JacobiSVD<MatrixXd> svd(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Matrix3d Rout1 = svd.matrixU() * svd.matrixV().transpose();
		// test if we found a good rotation matrix
		if (Rout1.determinant() < 0) {
			Rout1 *= -1.0;
    }
		// rotate this matrix back using the eigen frame
		Rout1 = eigenRot.transpose() * Rout1;

    // scale translation
    const Eigen::VectorXd sv_real = svd.singularValues().real();
		const Vector3d t(result1(6, 0) / sv_real(0), 
         result1(7, 0) / sv_real(1), 
         result1(8, 0) / sv_real(2));

		Rout1.transposeInPlace();
		Rout1 *= -1;
		if (Rout1.determinant() < 0.0) {
			Rout1.col(2) *= -1;
    }

		// now we have to find the best out of 4 combinations
		Eigen::Matrix3d R1, R2;
		R1.col(0) = Rout1.col(0); R1.col(1) = Rout1.col(1); R1.col(2) = Rout1.col(2);
		R2.col(0) = -Rout1.col(0); R2.col(1) = -Rout1.col(1); R2.col(2) = Rout1.col(2);

		std::vector<Eigen::Matrix<double, 3, 4>> Ts(4);
		Ts[0].block<3, 3>(0, 0) = R1; Ts[0].block<3, 1>(0, 3) = t;
		Ts[1].block<3, 3>(0, 0) = R1; Ts[1].block<3, 1>(0, 3) = -t;
		Ts[2].block<3, 3>(0, 0) = R2; Ts[2].block<3, 1>(0, 3) = t;
		Ts[3].block<3, 3>(0, 0) = R2; Ts[3].block<3, 1>(0, 3) = -t;

		std::vector<double> normVal(4);
		for (int i = 0; i < 4; ++i) {
			Vector3d reproPt;
			double norms = 0.0;
			for (int p = 0; p < 6; ++p) {
				reproPt = Ts[i].block<3, 3>(0, 0) * points3.col(p) + Ts[i].block<3, 1>(0, 3);
				reproPt = reproPt / reproPt.norm();
				norms += 1.0 - reproPt.transpose() * norm_feature_points[p].homogeneous().normalized();
			}
			normVal[i] = norms;
		}
		std::vector<double>::iterator
			findMinRepro = std::min_element(std::begin(normVal), std::end(normVal));
		int idx = std::distance(std::begin(normVal), findMinRepro);
		Rout = Ts[idx].block<3, 3>(0, 0);
		tout = Ts[idx].block<3, 1>(0, 3);

	} else { // non-planar case
		Matrix3d tmp;
		// until now, we only estimated 
		// row one and two of the transposed rotation matrix
		tmp << result1(0, 0), result1(3, 0), result1(6, 0),
			   result1(1, 0), result1(4, 0), result1(7, 0),
			   result1(2, 0), result1(5, 0), result1(8, 0);
		// get the scale
		// find best rotation matrix in frobenius sense
		Eigen::JacobiSVD<MatrixXd> svd(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double scale = svd.singularValues()(2);

		Rout = svd.matrixU() * svd.matrixV().transpose();
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
			for (int p = 0; p < 6; ++p) {
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

	//////////////////////////////////////
	// 5. gauss newton refinement
	//////////////////////////////////////
	mlpnp_gn(Rout, tout, world_points, nullspaces, P, use_cov);

  *(solution_rotation) = Rout;
  *(solution_translation) = tout;

  return true;
}
} // namespace theia
