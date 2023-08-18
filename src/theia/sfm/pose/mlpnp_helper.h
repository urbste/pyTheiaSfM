// Copyright (C) 2023, Steffen Urban
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

#ifndef THEIA_SFM_POSE_MLPNP_HELPER_H_
#define THEIA_SFM_POSE_MLPNP_HELPER_H_

#include <Eigen/Core>
#include <vector>

namespace theia
{
void mlpnpJacs(const point_t& pt,
		const Eigen::Vector3d& nullspace_r,
		const Eigen::Vector3d& nullspace_s,
		const rodrigues_t& c,
		const translation_t& t,
		Eigen::MatrixXd& jacs);


void mlpnp_residuals_and_jacs(
	const Eigen::VectorXd& x, 
	const points_t& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	Eigen::VectorXd& r, 
	Eigen::MatrixXd& fjac,
	bool getJacs);


void mlpnp_gn(
	Eigen::VectorXd& x,
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	const Eigen::SparseMatrix<double> Kll,
	bool use_cov);

void mlpnp_gn(
	Eigen::VectorXd& x,
	const std::vector<Eigen::Vector3d>& pts,
	const std::vector<Eigen::MatrixXd>& nullspaces,
	const Eigen::SparseMatrix<double> Kll,
	Eigen::MatrixXd& Qldld,
	Eigen::MatrixXd& Qxx,
	bool use_cov);

} // namespace theia
#endif