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
	const Eigen::VectorXd& x, 
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	Eigen::VectorXd& r, 
	Eigen::MatrixXd& fjac,
	bool getJacs) {
	Vector3d w(x[0], x[1], x[2]);
	Vector3d T(x[3], x[4], x[5]);

	Matrix3d R;
    ceres::AngleAxisToRotationMatrix(w.data(), R.data());
	int ii = 0;

	Eigen::MatrixXd jacs(2, 6);

	for (int i = 0; i < pts.size(); ++i) {
		const Vector3d ptCam = (R*pts[i] + T).normalized();

		r[ii] = nullspaces[i].col(0).transpose() * ptCam;
		r[ii + 1] = nullspaces[i].col(1).transpose() * ptCam;
		if (getJacs) {
			// jacs
			mlpnpJacs(pts[i],
				nullspaces[i].col(0), nullspaces[i].col(1),
				w, T,
				jacs);
			// r
			fjac(ii, 0) = jacs(0, 0);
			fjac(ii, 1) = jacs(0, 1);
			fjac(ii, 2) = jacs(0, 2);

			fjac(ii, 3) = jacs(0, 3);
			fjac(ii, 4) = jacs(0, 4);
			fjac(ii, 5) = jacs(0, 5);
			// s
			fjac(ii + 1, 0) = jacs(1, 0);
			fjac(ii + 1, 1) = jacs(1, 1);
			fjac(ii + 1, 2) = jacs(1, 2);

			fjac(ii + 1, 3) = jacs(1, 3);
			fjac(ii + 1, 4) = jacs(1, 4);
			fjac(ii + 1, 5) = jacs(1, 5);
			
		}
		ii += 2;
	}
}

void mlpnp_gn(
	Eigen::VectorXd& x,
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
	while (it_cnt < maxIt && !stop)
	{
		mlpnp_residuals_and_jacs(x, pts, 
			nullspaces,
			r, Jac, true);
		
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
		if (dl.array().abs().maxCoeff() < epsP) {
			stop = true;
			x = x - dx;
			break;
		} else {
			x = x - dx;
        }

		++it_cnt;
	}//while
	// result
}

void mlpnp_gn(
	Eigen::VectorXd& x,
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	const Eigen::SparseMatrix<double> Kll,
	Eigen::MatrixXd& Qldld,
	Eigen::MatrixXd& Qxx,
	bool use_cov);
{
	const int num_pts = pts.size();
	const int num_unknowns = 6;
	// check redundancy
	assert((2 * num_pts - 6) > 0);

	// =============
	// set all matrices up
	// =============
	
	Eigen::VectorXd r(2 * num_pts);
	Eigen::VectorXd rd(2 * num_pts);
	Eigen::MatrixXd Jac(2 * num_pts, num_unknowns);
	Eigen::VectorXd g(num_unknowns, 1);
	Eigen::VectorXd dx(num_unknowns, 1); // result vector

	Jac.setZero();
	r.setZero();
	dx.setZero();
	g.setZero();

	int it_cnt = 0;
	bool stop = false;
	const int maxIt = 5;
	double epsP = 1e-6;

	MatrixXd JacTSKll;
	MatrixXd A;
	// solve simple gradient descent
	while (it_cnt < maxIt && !stop)
	{
		mlpnp_residuals_and_jacs(x, pts,
			nullspaces,
			r, Jac, true);

		if (use_cov) {
			JacTSKll = Jac.transpose() * Kll;
        } else {
			JacTSKll = Jac.transpose();
        }

		A = JacTSKll * Jac;

		// get system matrix
		g = JacTSKll * r;

		// solve
		Eigen::LDLT<Eigen::MatrixXd> chol(A);
		dx = chol.solve(g);
		// this is to prevent the solution from falling into a wrong minimum
		// if the linear estimate is spurious
		if (dx.array().abs().maxCoeff() > 1.0 || dx.array().abs().minCoeff() > 1.0) {
			break;
        }
		// observation update
		Eigen::MatrixXd dl = Jac * dx;
		if (dl.array().abs().maxCoeff() < epsP) {
			stop = true;
			x = x - dx;
			break;
		} else {
			x = x - dx;
        }

		++it_cnt;
	} //while
	// statistics
	Qxx = A.inverse();
	Qldld = Jac * Qxx * Jac.transpose();
}


void mlpnpJacs(
    const Eigen::Vector3d& pt,
    const Eigen::Vector3d& nullspace_r,
    const Eigen::Vector3d& nullspace_s,
    const Eigen::Vector3d& w,
    const Eigen::Vector3d& t,
    Eigen::MatrixXd& jacs) {
	double r1 = nullspace_r[0];
	double r2 = nullspace_r[1];
	double r3 = nullspace_r[2];

	double s1 = nullspace_s[0];
	double s2 = nullspace_s[1];
	double s3 = nullspace_s[2];

	double X1 = pt[0];
	double Y1 = pt[1];
	double Z1 = pt[2];

	double w1 = w[0];
	double w2 = w[1];
	double w3 = w[2];

	double t1 = t[0];
	double t2 = t[1];
	double t3 = t[2];

	 double t5 = w1*w1;
	 double t6 = w2*w2;
	 double t7 = w3*w3;
	 double t8 = t5+t6+t7;
	 double t9 = sqrt(t8);
	 double t10 = sin(t9);
	 double t11 = 1.0/sqrt(t8);
	 double t12 = cos(t9);
	 double  t13 = t12-1.0;
	 double  t14 = 1.0/t8;
	 double  t16 = t10*t11*w3;
	 double t17 = t13*t14*w1*w2;
	 double t19 = t10*t11*w2;
	 double t20 = t13*t14*w1*w3;
	 double t24 = t6+t7;
	 double t27 = t16+t17;
	 double t28 = Y1*t27;
	 double t29 = t19-t20;
	 double t30 = Z1*t29;
	 double t31 = t13*t14*t24;
	 double t32 = t31+1.0;
	 double t33 = X1*t32;
	 double t15 = t1-t28+t30+t33;
	 double t21 = t10*t11*w1;
	 double t22 = t13*t14*w2*w3;
	 double t45 = t5+t7;
	 double t53 = t16-t17;
	 double t54 = X1*t53;
	 double t55 = t21+t22;
	 double t56 = Z1*t55;
	 double t57 = t13*t14*t45;
	 double t58 = t57+1.0;
	 double t59 = Y1*t58;
	 double t18 = t2+t54-t56+t59;
	 double t34 = t5+t6;
	 double t38 = t19+t20;
	 double t39 = X1*t38;
	 double t40 = t21-t22;
	 double t41 = Y1*t40;
	 double t42 = t13*t14*t34;
	 double t43 = t42+1.0;
	 double t44 = Z1*t43;
	 double t23 = t3-t39+t41+t44;
	 double t25 = 1.0/pow(t8,3.0/2.0);
	 double t26 = 1.0/(t8*t8);
	 double t35 = t12*t14*w1*w2;
	 double t36 = t5*t10*t25*w3;
	 double t37 = t5*t13*t26*w3*2.0;
	 double t46 = t10*t25*w1*w3;
	 double t47 = t5*t10*t25*w2;
	 double t48 = t5*t13*t26*w2*2.0;
	 double t49 = t10*t11;
	 double t50 = t5*t12*t14;
	 double t51 = t13*t26*w1*w2*w3*2.0;
	 double t52 = t10*t25*w1*w2*w3;
	 double t60 = t15*t15;
	 double t61 = t18*t18;
	 double t62 = t23*t23;
	 double t63 = t60+t61+t62;
	 double t64 = t5*t10*t25;
	 double t65 = 1.0/sqrt(t63);
	 double t66 = Y1*r2*t6;
	 double t67 = Z1*r3*t7;
	 double t68 = r1*t1*t5;
	 double t69 = r1*t1*t6;
	 double t70 = r1*t1*t7;
	 double  t71 = r2*t2*t5;
	 double  t72 = r2*t2*t6;
	 double  t73 = r2*t2*t7;
	 double  t74 = r3*t3*t5;
	 double  t75 = r3*t3*t6;
	 double  t76 = r3*t3*t7;
	 double  t77 = X1*r1*t5;
	 double  t78 = X1*r2*w1*w2;
	 double  t79 = X1*r3*w1*w3;
	 double  t80 = Y1*r1*w1*w2;
	 double  t81 = Y1*r3*w2*w3;
	 double  t82 = Z1*r1*w1*w3;
	 double  t83 = Z1*r2*w2*w3;
	 double  t84 = X1*r1*t6*t12;
	 double  t85 = X1*r1*t7*t12;
	 double  t86 = Y1*r2*t5*t12;
	 double  t87 = Y1*r2*t7*t12;
	 double  t88 = Z1*r3*t5*t12;
	 double  t89 = Z1*r3*t6*t12;
	 double  t90 = X1*r2*t9*t10*w3;
	 double  t91 = Y1*r3*t9*t10*w1;
	 double  t92 = Z1*r1*t9*t10*w2;
	 double  t102 = X1*r3*t9*t10*w2;
	 double  t103 = Y1*r1*t9*t10*w3;
	 double  t104 = Z1*r2*t9*t10*w1;
	 double  t105 = X1*r2*t12*w1*w2;
	 double  t106 = X1*r3*t12*w1*w3;
	 double  t107 = Y1*r1*t12*w1*w2;
	 double  t108 = Y1*r3*t12*w2*w3;
	 double  t109 = Z1*r1*t12*w1*w3;
	 double  t110 = Z1*r2*t12*w2*w3;
	 double  t93 = t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88+t89+t90+t91+t92-t102-t103-t104-t105-t106-t107-t108-t109-t110;
	 double  t94 = t10*t25*w1*w2;
	 double  t95 = t6*t10*t25*w3;
	 double  t96 = t6*t13*t26*w3*2.0;
	 double  t97 = t12*t14*w2*w3;
	 double  t98 = t6*t10*t25*w1;
	 double  t99 = t6*t13*t26*w1*2.0;
	 double  t100 = t6*t10*t25;
	 double  t101 = 1.0/pow(t63,3.0/2.0);
	 double  t111 = t6*t12*t14;
	 double  t112 = t10*t25*w2*w3;
	 double  t113 = t12*t14*w1*w3;
	 double  t114 = t7*t10*t25*w2;
	 double  t115 = t7*t13*t26*w2*2.0;
	 double  t116 = t7*t10*t25*w1;
	 double  t117 = t7*t13*t26*w1*2.0;
	 double  t118 = t7*t12*t14;
	 double  t119 = t13*t24*t26*w1*2.0;
	 double  t120 = t10*t24*t25*w1;
	 double  t121 = t119+t120;
	 double  t122 = t13*t26*t34*w1*2.0;
	 double  t123 = t10*t25*t34*w1;
	 double  t131 = t13*t14*w1*2.0;
	 double  t124 = t122+t123-t131;
	 double  t139 = t13*t14*w3;
	 double  t125 = -t35+t36+t37+t94-t139;
	 double  t126 = X1*t125;
	 double  t127 = t49+t50+t51+t52-t64;
	 double  t128 = Y1*t127;
	 double  t129 = t126+t128-Z1*t124;
	 double  t130 = t23*t129*2.0;
	 double  t132 = t13*t26*t45*w1*2.0;
	 double  t133 = t10*t25*t45*w1;
	 double  t138 = t13*t14*w2;
	 double  t134 = -t46+t47+t48+t113-t138;
	 double  t135 = X1*t134;
	 double  t136 = -t49-t50+t51+t52+t64;
	 double  t137 = Z1*t136;
	 double  t140 = X1*s1*t5;
	 double  t141 = Y1*s2*t6;
	 double  t142 = Z1*s3*t7;
	 double  t143 = s1*t1*t5;
	 double  t144 = s1*t1*t6;
	 double  t145 = s1*t1*t7;
	 double  t146 = s2*t2*t5;
	 double  t147 = s2*t2*t6;
	 double  t148 = s2*t2*t7;
	 double  t149 = s3*t3*t5;
	 double  t150 = s3*t3*t6;
	 double  t151 = s3*t3*t7;
	 double  t152 = X1*s2*w1*w2;
	 double  t153 = X1*s3*w1*w3;
	 double  t154 = Y1*s1*w1*w2;
	 double  t155 = Y1*s3*w2*w3;
	 double  t156 = Z1*s1*w1*w3;
	 double  t157 = Z1*s2*w2*w3;
	 double  t158 = X1*s1*t6*t12;
	 double  t159 = X1*s1*t7*t12;
	 double  t160 = Y1*s2*t5*t12;
	 double  t161 = Y1*s2*t7*t12;
	 double  t162 = Z1*s3*t5*t12;
	 double  t163 = Z1*s3*t6*t12;
	 double  t164 = X1*s2*t9*t10*w3;
	 double  t165 = Y1*s3*t9*t10*w1;
	 double  t166 = Z1*s1*t9*t10*w2;
	 double  t183 = X1*s3*t9*t10*w2;
	 double  t184 = Y1*s1*t9*t10*w3;
	 double  t185 = Z1*s2*t9*t10*w1;
	 double  t186 = X1*s2*t12*w1*w2;
	 double  t187 = X1*s3*t12*w1*w3;
	 double  t188 = Y1*s1*t12*w1*w2;
	 double  t189 = Y1*s3*t12*w2*w3;
	 double  t190 = Z1*s1*t12*w1*w3;
	 double  t191 = Z1*s2*t12*w2*w3;
	 double  t167 = t140+t141+t142+t143+t144+t145+t146+t147+t148+t149+t150+t151+t152+t153+t154+t155+t156+t157+t158+t159+t160+t161+t162+t163+t164+t165+t166-t183-t184-t185-t186-t187-t188-t189-t190-t191;
	 double  t168 = t13*t26*t45*w2*2.0;
	 double  t169 = t10*t25*t45*w2;
	 double  t170 = t168+t169;
	 double  t171 = t13*t26*t34*w2*2.0;
	 double  t172 = t10*t25*t34*w2;
	 double  t176 = t13*t14*w2*2.0;
	 double  t173 = t171+t172-t176;
	 double  t174 = -t49+t51+t52+t100-t111;
	 double  t175 = X1*t174;
	 double  t177 = t13*t24*t26*w2*2.0;
	 double  t178 = t10*t24*t25*w2;
	 double  t192 = t13*t14*w1;
	 double  t179 = -t97+t98+t99+t112-t192;
	 double  t180 = Y1*t179;
	 double  t181 = t49+t51+t52-t100+t111;
	 double  t182 = Z1*t181;
	 double  t193 = t13*t26*t34*w3*2.0;
	 double  t194 = t10*t25*t34*w3;
	 double  t195 = t193+t194;
	 double  t196 = t13*t26*t45*w3*2.0;
	 double  t197 = t10*t25*t45*w3;
	 double  t200 = t13*t14*w3*2.0;
	 double  t198 = t196+t197-t200;
	 double  t199 = t7*t10*t25;
	 double  t201 = t13*t24*t26*w3*2.0;
	 double  t202 = t10*t24*t25*w3;
	 double  t203 = -t49+t51+t52-t118+t199;
	 double  t204 = Y1*t203;
	 double  t205 = t1*2.0;
	 double  t206 = Z1*t29*2.0;
	 double  t207 = X1*t32*2.0;
	 double  t208 = t205+t206+t207-Y1*t27*2.0;
	 double  t209 = t2*2.0;
	 double  t210 = X1*t53*2.0;
	 double  t211 = Y1*t58*2.0;
	 double  t212 = t209+t210+t211-Z1*t55*2.0;
	 double  t213 = t3*2.0;
	 double  t214 = Y1*t40*2.0;
	 double  t215 = Z1*t43*2.0;
	 double  t216 = t213+t214+t215-X1*t38*2.0;
	jacs(0, 0) = t14*t65*(X1*r1*w1*2.0+X1*r2*w2+X1*r3*w3+Y1*r1*w2+Z1*r1*w3+r1*t1*w1*2.0+r2*t2*w1*2.0+r3*t3*w1*2.0+Y1*r3*t5*t12+Y1*r3*t9*t10-Z1*r2*t5*t12-Z1*r2*t9*t10-X1*r2*t12*w2-X1*r3*t12*w3-Y1*r1*t12*w2+Y1*r2*t12*w1*2.0-Z1*r1*t12*w3+Z1*r3*t12*w1*2.0+Y1*r3*t5*t10*t11-Z1*r2*t5*t10*t11+X1*r2*t12*w1*w3-X1*r3*t12*w1*w2-Y1*r1*t12*w1*w3+Z1*r1*t12*w1*w2-Y1*r1*t10*t11*w1*w3+Z1*r1*t10*t11*w1*w2-X1*r1*t6*t10*t11*w1-X1*r1*t7*t10*t11*w1+X1*r2*t5*t10*t11*w2+X1*r3*t5*t10*t11*w3+Y1*r1*t5*t10*t11*w2-Y1*r2*t5*t10*t11*w1-Y1*r2*t7*t10*t11*w1+Z1*r1*t5*t10*t11*w3-Z1*r3*t5*t10*t11*w1-Z1*r3*t6*t10*t11*w1+X1*r2*t10*t11*w1*w3-X1*r3*t10*t11*w1*w2+Y1*r3*t10*t11*w1*w2*w3+Z1*r2*t10*t11*w1*w2*w3)-t26*t65*t93*w1*2.0-t14*t93*t101*(t130+t15*(-X1*t121+Y1*(t46+t47+t48-t13*t14*w2-t12*t14*w1*w3)+Z1*(t35+t36+t37-t13*t14*w3-t10*t25*w1*w2))*2.0+t18*(t135+t137-Y1*(t132+t133-t13*t14*w1*2.0))*2.0)*(1.0/2.0);
	jacs(0, 1) = t14*t65*(X1*r2*w1+Y1*r1*w1+Y1*r2*w2*2.0+Y1*r3*w3+Z1*r2*w3+r1*t1*w2*2.0+r2*t2*w2*2.0+r3*t3*w2*2.0-X1*r3*t6*t12-X1*r3*t9*t10+Z1*r1*t6*t12+Z1*r1*t9*t10+X1*r1*t12*w2*2.0-X1*r2*t12*w1-Y1*r1*t12*w1-Y1*r3*t12*w3-Z1*r2*t12*w3+Z1*r3*t12*w2*2.0-X1*r3*t6*t10*t11+Z1*r1*t6*t10*t11+X1*r2*t12*w2*w3-Y1*r1*t12*w2*w3+Y1*r3*t12*w1*w2-Z1*r2*t12*w1*w2-Y1*r1*t10*t11*w2*w3+Y1*r3*t10*t11*w1*w2-Z1*r2*t10*t11*w1*w2-X1*r1*t6*t10*t11*w2+X1*r2*t6*t10*t11*w1-X1*r1*t7*t10*t11*w2+Y1*r1*t6*t10*t11*w1-Y1*r2*t5*t10*t11*w2-Y1*r2*t7*t10*t11*w2+Y1*r3*t6*t10*t11*w3-Z1*r3*t5*t10*t11*w2+Z1*r2*t6*t10*t11*w3-Z1*r3*t6*t10*t11*w2+X1*r2*t10*t11*w2*w3+X1*r3*t10*t11*w1*w2*w3+Z1*r1*t10*t11*w1*w2*w3)-t26*t65*t93*w2*2.0-t14*t93*t101*(t18*(Z1*(-t35+t94+t95+t96-t13*t14*w3)-Y1*t170+X1*(t97+t98+t99-t13*t14*w1-t10*t25*w2*w3))*2.0+t15*(t180+t182-X1*(t177+t178-t13*t14*w2*2.0))*2.0+t23*(t175+Y1*(t35-t94+t95+t96-t13*t14*w3)-Z1*t173)*2.0)*(1.0/2.0);
	jacs(0, 2) = t14*t65*(X1*r3*w1+Y1*r3*w2+Z1*r1*w1+Z1*r2*w2+Z1*r3*w3*2.0+r1*t1*w3*2.0+r2*t2*w3*2.0+r3*t3*w3*2.0+X1*r2*t7*t12+X1*r2*t9*t10-Y1*r1*t7*t12-Y1*r1*t9*t10+X1*r1*t12*w3*2.0-X1*r3*t12*w1+Y1*r2*t12*w3*2.0-Y1*r3*t12*w2-Z1*r1*t12*w1-Z1*r2*t12*w2+X1*r2*t7*t10*t11-Y1*r1*t7*t10*t11-X1*r3*t12*w2*w3+Y1*r3*t12*w1*w3+Z1*r1*t12*w2*w3-Z1*r2*t12*w1*w3+Y1*r3*t10*t11*w1*w3+Z1*r1*t10*t11*w2*w3-Z1*r2*t10*t11*w1*w3-X1*r1*t6*t10*t11*w3-X1*r1*t7*t10*t11*w3+X1*r3*t7*t10*t11*w1-Y1*r2*t5*t10*t11*w3-Y1*r2*t7*t10*t11*w3+Y1*r3*t7*t10*t11*w2+Z1*r1*t7*t10*t11*w1+Z1*r2*t7*t10*t11*w2-Z1*r3*t5*t10*t11*w3-Z1*r3*t6*t10*t11*w3-X1*r3*t10*t11*w2*w3+X1*r2*t10*t11*w1*w2*w3+Y1*r1*t10*t11*w1*w2*w3)-t26*t65*t93*w3*2.0-t14*t93*t101*(t18*(Z1*(t46-t113+t114+t115-t13*t14*w2)-Y1*t198+X1*(t49+t51+t52+t118-t7*t10*t25))*2.0+t23*(X1*(-t97+t112+t116+t117-t13*t14*w1)+Y1*(-t46+t113+t114+t115-t13*t14*w2)-Z1*t195)*2.0+t15*(t204+Z1*(t97-t112+t116+t117-t13*t14*w1)-X1*(t201+t202-t13*t14*w3*2.0))*2.0)*(1.0/2.0);
	jacs(0, 3) = r1*t65-t14*t93*t101*t208*(1.0/2.0);
	jacs(0, 4) = r2*t65-t14*t93*t101*t212*(1.0/2.0);
	jacs(0, 5) = r3*t65-t14*t93*t101*t216*(1.0/2.0);
	jacs(1, 0) = t14*t65*(X1*s1*w1*2.0+X1*s2*w2+X1*s3*w3+Y1*s1*w2+Z1*s1*w3+s1*t1*w1*2.0+s2*t2*w1*2.0+s3*t3*w1*2.0+Y1*s3*t5*t12+Y1*s3*t9*t10-Z1*s2*t5*t12-Z1*s2*t9*t10-X1*s2*t12*w2-X1*s3*t12*w3-Y1*s1*t12*w2+Y1*s2*t12*w1*2.0-Z1*s1*t12*w3+Z1*s3*t12*w1*2.0+Y1*s3*t5*t10*t11-Z1*s2*t5*t10*t11+X1*s2*t12*w1*w3-X1*s3*t12*w1*w2-Y1*s1*t12*w1*w3+Z1*s1*t12*w1*w2+X1*s2*t10*t11*w1*w3-X1*s3*t10*t11*w1*w2-Y1*s1*t10*t11*w1*w3+Z1*s1*t10*t11*w1*w2-X1*s1*t6*t10*t11*w1-X1*s1*t7*t10*t11*w1+X1*s2*t5*t10*t11*w2+X1*s3*t5*t10*t11*w3+Y1*s1*t5*t10*t11*w2-Y1*s2*t5*t10*t11*w1-Y1*s2*t7*t10*t11*w1+Z1*s1*t5*t10*t11*w3-Z1*s3*t5*t10*t11*w1-Z1*s3*t6*t10*t11*w1+Y1*s3*t10*t11*w1*w2*w3+Z1*s2*t10*t11*w1*w2*w3)-t14*t101*t167*(t130+t15*(Y1*(t46+t47+t48-t113-t138)+Z1*(t35+t36+t37-t94-t139)-X1*t121)*2.0+t18*(t135+t137-Y1*(-t131+t132+t133))*2.0)*(1.0/2.0)-t26*t65*t167*w1*2.0;
	jacs(1, 1) = t14*t65*(X1*s2*w1+Y1*s1*w1+Y1*s2*w2*2.0+Y1*s3*w3+Z1*s2*w3+s1*t1*w2*2.0+s2*t2*w2*2.0+s3*t3*w2*2.0-X1*s3*t6*t12-X1*s3*t9*t10+Z1*s1*t6*t12+Z1*s1*t9*t10+X1*s1*t12*w2*2.0-X1*s2*t12*w1-Y1*s1*t12*w1-Y1*s3*t12*w3-Z1*s2*t12*w3+Z1*s3*t12*w2*2.0-X1*s3*t6*t10*t11+Z1*s1*t6*t10*t11+X1*s2*t12*w2*w3-Y1*s1*t12*w2*w3+Y1*s3*t12*w1*w2-Z1*s2*t12*w1*w2+X1*s2*t10*t11*w2*w3-Y1*s1*t10*t11*w2*w3+Y1*s3*t10*t11*w1*w2-Z1*s2*t10*t11*w1*w2-X1*s1*t6*t10*t11*w2+X1*s2*t6*t10*t11*w1-X1*s1*t7*t10*t11*w2+Y1*s1*t6*t10*t11*w1-Y1*s2*t5*t10*t11*w2-Y1*s2*t7*t10*t11*w2+Y1*s3*t6*t10*t11*w3-Z1*s3*t5*t10*t11*w2+Z1*s2*t6*t10*t11*w3-Z1*s3*t6*t10*t11*w2+X1*s3*t10*t11*w1*w2*w3+Z1*s1*t10*t11*w1*w2*w3)-t26*t65*t167*w2*2.0-t14*t101*t167*(t18*(X1*(t97+t98+t99-t112-t192)+Z1*(-t35+t94+t95+t96-t139)-Y1*t170)*2.0+t15*(t180+t182-X1*(-t176+t177+t178))*2.0+t23*(t175+Y1*(t35-t94+t95+t96-t139)-Z1*t173)*2.0)*(1.0/2.0);
	jacs(1, 2) = t14*t65*(X1*s3*w1+Y1*s3*w2+Z1*s1*w1+Z1*s2*w2+Z1*s3*w3*2.0+s1*t1*w3*2.0+s2*t2*w3*2.0+s3*t3*w3*2.0+X1*s2*t7*t12+X1*s2*t9*t10-Y1*s1*t7*t12-Y1*s1*t9*t10+X1*s1*t12*w3*2.0-X1*s3*t12*w1+Y1*s2*t12*w3*2.0-Y1*s3*t12*w2-Z1*s1*t12*w1-Z1*s2*t12*w2+X1*s2*t7*t10*t11-Y1*s1*t7*t10*t11-X1*s3*t12*w2*w3+Y1*s3*t12*w1*w3+Z1*s1*t12*w2*w3-Z1*s2*t12*w1*w3-X1*s3*t10*t11*w2*w3+Y1*s3*t10*t11*w1*w3+Z1*s1*t10*t11*w2*w3-Z1*s2*t10*t11*w1*w3-X1*s1*t6*t10*t11*w3-X1*s1*t7*t10*t11*w3+X1*s3*t7*t10*t11*w1-Y1*s2*t5*t10*t11*w3-Y1*s2*t7*t10*t11*w3+Y1*s3*t7*t10*t11*w2+Z1*s1*t7*t10*t11*w1+Z1*s2*t7*t10*t11*w2-Z1*s3*t5*t10*t11*w3-Z1*s3*t6*t10*t11*w3+X1*s2*t10*t11*w1*w2*w3+Y1*s1*t10*t11*w1*w2*w3)-t26*t65*t167*w3*2.0-t14*t101*t167*(t18*(Z1*(t46-t113+t114+t115-t138)-Y1*t198+X1*(t49+t51+t52+t118-t199))*2.0+t23*(X1*(-t97+t112+t116+t117-t192)+Y1*(-t46+t113+t114+t115-t138)-Z1*t195)*2.0+t15*(t204+Z1*(t97-t112+t116+t117-t192)-X1*(-t200+t201+t202))*2.0)*(1.0/2.0);
	jacs(1, 3) = s1*t65-t14*t101*t167*t208*(1.0/2.0);
	jacs(1, 4) = s2*t65-t14*t101*t167*t212*(1.0/2.0);
	jacs(1, 5) = s3*t65-t14*t101*t167*t216*(1.0/2.0);
}

    
} // namespace theia
