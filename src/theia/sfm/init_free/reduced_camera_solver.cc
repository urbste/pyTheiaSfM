// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/reduced_camera_solver.h"

#include <Eigen/Cholesky>
#include <glog/logging.h>

#include <cmath>

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

// Truncated power series of the Schur complement inverse (PoBA, Weber et al.
// CVPR 2023; PoVar, ECCV 2024). With U = D + O (block diagonal D, camera-pair
// blocks O) and M = W^T V^-1 W - O, the damped system is
//
//   S + lambda I = D_lambda - M = D_lambda (I - D_lambda^-1 M),
//   (S + lambda I)^-1 b = sum_k (D_lambda^-1 M)^k D_lambda^-1 b.
//
// For VarPro the series converges for every lambda > 0: W^T V^-1 W <= D_obs
// (the observation part of D, which is block diagonal because every residual
// depends on one camera), and each pairwise rotation term contributes
// [Ja -Jb]^T [Ja -Jb] >= 0, so -D_rot <= O <= D_rot. Hence
// -D_lambda < M < D_lambda and the spectral radius of D_lambda^-1 M is < 1.
// Only matrix-vector products with the blocks are needed, so memory and work
// per term are linear in the number of observations.
class PowerSeriesSolver : public ReducedCameraSolver {
 public:
  PowerSeriesSolver(const int max_iterations, const double tolerance)
      : max_iterations_(max_iterations), tolerance_(tolerance) {}

  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    constexpr int b = ReducedCameraSystem::kBlockSize;
    const int n = system.num_cameras;
    D_inverse_.resize(n);
    for (int i = 0; i < n; ++i) {
      Matrix12d D = system.U_diagonal[i];
      D.diagonal().array() += lambda;
      Eigen::LLT<Matrix12d> llt(D);
      if (llt.info() != Eigen::Success) {
        *error = "Power series: camera block not positive definite.";
        return false;
      }
      D_inverse_[i] = llt.solve(Matrix12d::Identity());
    }

    Eigen::VectorXd term(b * n);
    ApplyDInverse(rhs, &term);
    *x = term;
    Eigen::VectorXd product(b * n);
    for (int k = 1; k < max_iterations_; ++k) {
      ApplyM(system, term, &product);
      ApplyDInverse(product, &term);
      *x += term;
      const double term_norm = term.norm();
      if (!std::isfinite(term_norm)) {
        *error = "Power series produced non-finite values.";
        return false;
      }
      if (term_norm <= tolerance_ * x->norm()) {
        break;
      }
    }
    return x->allFinite();
  }

 private:
  void ApplyDInverse(const Eigen::VectorXd& in, Eigen::VectorXd* out) const {
    constexpr int b = ReducedCameraSystem::kBlockSize;
    for (size_t i = 0; i < D_inverse_.size(); ++i) {
      out->segment<b>(b * i).noalias() = D_inverse_[i] * in.segment<b>(b * i);
    }
  }

  // out = (W^T V^-1 W - O) in
  static void ApplyM(const ReducedCameraSystem& system,
                     const Eigen::VectorXd& in,
                     Eigen::VectorXd* out) {
    constexpr int b = ReducedCameraSystem::kBlockSize;
    out->setZero();
    for (size_t j = 0; j + 1 < system.point_offsets.size(); ++j) {
      const int begin = system.point_offsets[j];
      const int end = system.point_offsets[j + 1];
      Eigen::Vector3d s = Eigen::Vector3d::Zero();
      for (int k = begin; k < end; ++k) {
        s.noalias() += system.W_blocks[k] * in.segment<b>(b * system.W_camera[k]);
      }
      const Eigen::Vector3d t = system.V_inverse[j] * s;
      for (int k = begin; k < end; ++k) {
        out->segment<b>(b * system.W_camera[k]).noalias() +=
            system.W_blocks[k].transpose() * t;
      }
    }
    for (const auto& pair : system.U_off_diagonal) {
      out->segment<b>(b * pair.camera_a).noalias() -=
          pair.block * in.segment<b>(b * pair.camera_b);
    }
  }

  const int max_iterations_;
  const double tolerance_;
  std::vector<Matrix12d, Eigen::aligned_allocator<Matrix12d>> D_inverse_;
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
      if (options.power_series_max_iterations < 1) {
        *error = "power_series_max_iterations must be at least 1.";
        return nullptr;
      }
      return std::make_unique<PowerSeriesSolver>(
          options.power_series_max_iterations,
          options.power_series_tolerance);
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
