// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/reduced_camera_solver.h"

#include <Eigen/Cholesky>
#include <glog/logging.h>

#ifndef THEIA_WITH_CUDA_SOLVERS
#define THEIA_WITH_CUDA_SOLVERS 0
#endif

namespace theia {

#if THEIA_WITH_CUDA_SOLVERS
// Defined in cuda_dense_cholesky_solver.cc.
std::unique_ptr<ReducedCameraSolver> CreateCudaDenseCholeskySolver(
    std::string* error);
bool CudaDeviceAvailable();
#endif

void ReducedCameraSystem::FormDenseSchurComplement(Eigen::MatrixXd* S) const {
  constexpr int b = kBlockSize;
  S->setZero(Dimension(), Dimension());
  for (int i = 0; i < num_cameras; ++i) {
    S->block<b, b>(b * i, b * i) = U_diagonal[i];
  }
  for (const CameraPairBlock& pair : U_off_diagonal) {
    S->block<b, b>(b * pair.camera_a, b * pair.camera_b) += pair.block;
  }
  for (size_t j = 0; j + 1 < point_offsets.size(); ++j) {
    const int begin = point_offsets[j];
    const int end = point_offsets[j + 1];
    for (int k = begin; k < end; ++k) {
      const Matrix3x12d VinvWk = V_inverse[j] * W_blocks[k];
      for (int l = begin; l < end; ++l) {
        S->block<b, b>(b * W_camera[l], b * W_camera[k]).noalias() -=
            W_blocks[l].transpose() * VinvWk;
      }
    }
  }
}

namespace {

class DenseCholeskySolver : public ReducedCameraSolver {
 public:
  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    if (cached_revision_ != system.revision || S_.rows() != rhs.size()) {
      system.FormDenseSchurComplement(&S_);
      cached_revision_ = system.revision;
    }
    Eigen::MatrixXd damped = S_;
    damped.diagonal().array() += lambda;
    Eigen::LLT<Eigen::MatrixXd> llt(damped);
    if (llt.info() != Eigen::Success) {
      *error = "Dense Cholesky failed (system not positive definite).";
      return false;
    }
    *x = llt.solve(rhs);
    return x->allFinite();
  }

 private:
  Eigen::MatrixXd S_;
  int cached_revision_ = -1;
};

}  // namespace

std::unique_ptr<ReducedCameraSolver> CreateReducedCameraSolver(
    const VarProOptions& options, std::string* error) {
  switch (options.linear_solver_type) {
    case ReducedCameraSolverType::DENSE_CHOLESKY:
      return std::make_unique<DenseCholeskySolver>();
    case ReducedCameraSolverType::CUDA_DENSE_CHOLESKY:
#if THEIA_WITH_CUDA_SOLVERS
      return CreateCudaDenseCholeskySolver(error);
#else
      *error =
          "CUDA_DENSE_CHOLESKY needs a build with -DTHEIA_WITH_CUDA_SOLVERS=ON.";
      return nullptr;
#endif
    case ReducedCameraSolverType::SPARSE_CHOLESKY:
      *error = "SPARSE_CHOLESKY is not implemented yet.";
      return nullptr;
    case ReducedCameraSolverType::POWER_SERIES:
      *error = "POWER_SERIES is not implemented yet.";
      return nullptr;
  }
  *error = "Unknown reduced camera solver type.";
  return nullptr;
}

bool CudaReducedCameraSolverAvailable() {
#if THEIA_WITH_CUDA_SOLVERS
  return CudaDeviceAvailable();
#else
  return false;
#endif
}

}  // namespace theia
