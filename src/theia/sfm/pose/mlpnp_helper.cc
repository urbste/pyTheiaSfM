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

#include "theia/sfm/pose/mlpnp_helper.h"

#include <ceres/rotation.h>

namespace theia {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::VectorXd;

void mlpnp_residuals_and_jacs(
	const Eigen::Matrix3d& x, 
	const Eigen::Vector3d& t,
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	Eigen::VectorXd& r, 
	Eigen::MatrixXd& fjac,
	bool getJacs) {

	int ii = 0;

	Eigen::Matrix<double, 2, 6> jacs;

       err(2*i-1,1) = r(:,i)'*res1(:,i);
        err(2*i,1) = s(:,i)'*res1(:,i);

        J(2*i-1,:) = [-r(:,i)'*skew(res1(:,i)) r(:,i)'];
        J(2*i,:) = [-s(:,i)'*skew(res1(:,i)) s(:,i)'];

	for (int i = 0; i < pts.size(); ++i) {
		const Vector3d ptCam = R*pts[i] + T;

		r[ii] = nullspaces[i].col(0).transpose() * ptCam;
		r[ii + 1] = nullspaces[i].col(1).transpose() * ptCam;
		if (getJacs) {
			// jacs
			fjac.block<1,3>(ii,0) = -nullspaces[i].col(0) * GetSkew(ptCam);
			fjac.block<1,3>(ii+1,0) = -nullspaces[i].col(1) * GetSkew(ptCam);

			fjac.block<1,3>(ii,3) = nullspaces[i].col(0).transpose();
			fjac.block<1,3>(ii+1,3) = nullspaces[i].col(1).transpose(); 
		}
		ii += 2;
	}
}

void mlpnp_gn(
	Eigen::Matrix3d& R,
	Eigen::Vector3d& t,
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	const Eigen::SparseMatrix<double> Kll,
	bool use_cov) {
	const int num_pts = pts.size();
	const int num_unknowns = 6;
	// check redundancy
	assert((2 * num_pts - num_unknowns) > 0);

	// =============
	// set all matrices up
	// =============
	VectorXd r(2 * num_pts);
	VectorXd rd(2 * num_pts);
	MatrixXd Jac(2 * num_pts, num_unknowns);
	VectorXd g(num_unknowns, 1);
	VectorXd dx(num_unknowns, 1); // result vector

	Jac.setZero();
	r.setZero();
	dx.setZero();
	g.setZero();

	int it_cnt = 0;
	bool stop = false;
	const int maxIt = 5;
	double epsP = 1e-5;

	MatrixXd JacTSKll;
	MatrixXd A;
	// solve simple gradient descent
	while (it_cnt < maxIt && !stop) {
		mlpnp_residuals_and_jacs(R, t, pts, 
			nullspaces, r, Jac, true);
		
		if (use_cov) {
			JacTSKll = Jac.transpose() * Kll;
        }
		else {
			JacTSKll = Jac.transpose();
        }

		A = JacTSKll * Jac;
		
		// get system matrix
		g = JacTSKll * r;

		// solve
		Eigen::LDLT<Eigen::MatrixXd> chol(A);
		dx = chol.solve(g);
		//dx = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(g);
		// this is to prevent the solution from falling into a wrong minimum
		// if the linear estimate is spurious
		if (dx.array().abs().maxCoeff() > 5.0 || dx.array().abs().minCoeff() > 1.0) {
			break;
        }
		// observation update
		Eigen::MatrixXd dl = Jac * dx;
		Matrix3d dR;
		ceres::AngleAxisToRotationMatrix(-dx.head<3>().data(), dR);
		if (dl.array().abs().maxCoeff() < epsP) {
        	R = dR * R;
        	t = dR * t - dx.tail<3>().data();
			stop = true;
			break;
		} else {
        	R = dR * R;
        	t = dR * t - dx.tail<3>().data();
        }
		++it_cnt;
	}//while

	return true;
}

} // namespace theia
