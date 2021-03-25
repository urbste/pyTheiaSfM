// Copyright (C) 2016 The Regents of the University of California (Regents).
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
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// **  NOTE: Parts of this file were borrowed or inspired from the Ceres   **
// ** Solver library. The Ceres license is included here for completeness. **
//
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

// edited by Steffen Urban 2019, January
// removed SuiteSparse dependency
// use Eigens LDLT

#include "theia/math/matrix/sparse_cholesky_llt.h"

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/SparseCore>

// UF_long is deprecated but SuiteSparse_long is only available in
// newer versions of SuiteSparse. So for older versions of
// SuiteSparse, we define SuiteSparse_long to be the same as UF_long,
// which is what recent versions of SuiteSparse do anyways.
#ifndef SuiteSparse_long
#define SuiteSparse_long UF_long
#endif

namespace theia {

// A class for performing the choleksy decomposition of a sparse matrix using
// CHOLMOD from SuiteSparse. This allows us to utilize the supernodal algorithms
// which are not included with Eigen. The interface is meant to mimic the Eigen
// linear solver interface except that it is not templated and requires sparse
// matrices.
SparseCholeskyLLt::SparseCholeskyLLt(const Eigen::SparseMatrix<double>& mat)
    : is_factorization_ok_(false),
      is_analysis_ok_(false),
      info_(Eigen::Success) {
  Compute(mat);
}

SparseCholeskyLLt::SparseCholeskyLLt()
    : is_factorization_ok_(false),
      is_analysis_ok_(false),
      info_(Eigen::Success) {}

SparseCholeskyLLt::~SparseCholeskyLLt() {}

void SparseCholeskyLLt::AnalyzePattern(const Eigen::SparseMatrix<double>& mat) {
  solver_.analyzePattern(mat);
  info_ = solver_.info();
  if (info_ == Eigen::Success)
    is_analysis_ok_ = true;
  else
    is_analysis_ok_ = false;
  is_factorization_ok_ = false;
  info_ = Eigen::Success;
}

void SparseCholeskyLLt::Factorize(const Eigen::SparseMatrix<double>& mat) {
  solver_.factorize(mat);

  info_ = solver_.info();
  if (info_ == Eigen::Success)
    is_factorization_ok_ = true;
  else
    is_factorization_ok_ = false;
}

void SparseCholeskyLLt::Compute(const Eigen::SparseMatrix<double>& mat) {
  solver_.compute(mat);
  info_ = solver_.info();
  if (info_ == Eigen::Success) {
    is_factorization_ok_ = true;
    is_analysis_ok_ = true;
  }
}

Eigen::ComputationInfo SparseCholeskyLLt::Info() { return info_; }

// Using the cholesky decomposition, solve for x that minimizes
//    lhs * x = rhs
// where lhs is the factorized matrix.
Eigen::VectorXd SparseCholeskyLLt::Solve(const Eigen::VectorXd& rhs) {
  CHECK(is_analysis_ok_) << "Cannot call Solve() because symbolic analysis "
                            "of the matrix (i.e. AnalyzePattern()) failed!";
  CHECK(is_factorization_ok_)
      << "Cannot call Solve() because numeric factorization "
         "of the matrix (i.e. Factorize()) failed!";

  Eigen::VectorXd solution;
  solution = solver_.solve(rhs);
  return solution;
}

}  // namespace theia

