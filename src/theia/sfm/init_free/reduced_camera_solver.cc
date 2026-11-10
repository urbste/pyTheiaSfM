// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/reduced_camera_solver.h"

#include <Eigen/Cholesky>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>

#include "theia/sfm/init_free/parallel_for.h"

#ifndef THEIA_WITH_CUDA_SOLVERS
#define THEIA_WITH_CUDA_SOLVERS 0
#endif

namespace theia {

#if THEIA_WITH_CUDA_SOLVERS
// Defined in cuda_dense_cholesky_solver.cc and cuda_power_series_solver.cc.
std::unique_ptr<ReducedCameraSolver> CreateCudaDenseCholeskySolver(
    std::string* error);
std::unique_ptr<ReducedCameraSolver> CreateCudaPowerSeriesSolver(
    int max_iterations, double tolerance, std::string* error);
bool CudaDeviceAvailable();
#endif

namespace {

constexpr int kB = ReducedCameraSystem::kBlockSize;

// Minimum work per thread, to keep small problems on one thread.
constexpr int kMinCamerasPerChunk = 4;
constexpr int kMinPointsPerChunk = 256;

// out = (W^T V^-1 W - O) in, in two parallel passes: per point
// t_j = V_j^-1 sum_k W_k in_{c(k)}, then per camera
// out_i = sum_{k in obs(i)} W_k^T t_{p(k)} - sum_b O_ib in_b.
void ApplyM(const ReducedCameraSystem& system,
            const Eigen::VectorXd& in,
            Eigen::Matrix3Xd* point_buffer,
            Eigen::VectorXd* out) {
  const int num_points = static_cast<int>(system.V_inverse.size());
  point_buffer->resize(3, num_points);
  ParallelFor(system.num_threads,
              num_points,
              kMinPointsPerChunk,
              [&](const int begin, const int end, int) {
                for (int j = begin; j < end; ++j) {
                  Eigen::Vector3d s = Eigen::Vector3d::Zero();
                  for (int k = system.point_offsets[j];
                       k < system.point_offsets[j + 1];
                       ++k) {
                    s.noalias() += system.W_blocks[k] *
                                   in.segment<kB>(kB * system.W_camera[k]);
                  }
                  point_buffer->col(j).noalias() = system.V_inverse[j] * s;
                }
              });
  out->resize(in.size());
  ParallelFor(
      system.num_threads,
      system.num_cameras,
      kMinCamerasPerChunk,
      [&](const int begin, const int end, int) {
        for (int i = begin; i < end; ++i) {
          Eigen::Matrix<double, kB, 1> acc =
              Eigen::Matrix<double, kB, 1>::Zero();
          for (int idx = system.camera_offsets[i];
               idx < system.camera_offsets[i + 1];
               ++idx) {
            const int k = system.camera_W_index[idx];
            acc.noalias() += system.W_blocks[k].transpose() *
                             point_buffer->col(system.W_point[k]);
          }
          for (int p = system.off_diagonal_offsets[i];
               p < system.off_diagonal_offsets[i + 1];
               ++p) {
            const auto& pair = system.U_off_diagonal[p];
            acc.noalias() -= pair.block * in.segment<kB>(kB * pair.camera_b);
          }
          out->segment<kB>(kB * i) = acc;
        }
      });
}

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
    const int n = system.num_cameras;
    D_inverse_.resize(n);
    std::vector<char> ok(n, 1);
    ParallelFor(system.num_threads,
                n,
                kMinCamerasPerChunk,
                [&](const int begin, const int end, int) {
                  for (int i = begin; i < end; ++i) {
                    Matrix12d D = system.U_diagonal[i];
                    D.diagonal().array() += lambda;
                    Eigen::LLT<Matrix12d> llt(D);
                    ok[i] = llt.info() == Eigen::Success;
                    D_inverse_[i] = llt.solve(Matrix12d::Identity());
                  }
                });
    if (std::find(ok.begin(), ok.end(), 0) != ok.end()) {
      *error = "Power series: camera block not positive definite.";
      return false;
    }

    Eigen::VectorXd term(kB * n);
    ApplyDInverse(rhs, &term);
    *x = term;
    Eigen::VectorXd product(kB * n);
    for (int k = 1; k < max_iterations_; ++k) {
      ApplyM(system, term, &point_buffer_, &product);
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
    for (size_t i = 0; i < D_inverse_.size(); ++i) {
      out->segment<kB>(kB * i).noalias() =
          D_inverse_[i] * in.segment<kB>(kB * i);
    }
  }

  const int max_iterations_;
  const double tolerance_;
  std::vector<Matrix12d, Eigen::aligned_allocator<Matrix12d>> D_inverse_;
  Eigen::Matrix3Xd point_buffer_;
};

}  // namespace

void ReducedCameraSystem::BuildCameraIndex() {
  camera_offsets.assign(num_cameras + 1, 0);
  for (const int c : W_camera) {
    ++camera_offsets[c + 1];
  }
  for (int i = 0; i < num_cameras; ++i) {
    camera_offsets[i + 1] += camera_offsets[i];
  }
  camera_W_index.resize(W_camera.size());
  std::vector<int> next(camera_offsets.begin(), camera_offsets.end() - 1);
  for (size_t k = 0; k < W_camera.size(); ++k) {
    camera_W_index[next[W_camera[k]]++] = static_cast<int>(k);
  }
}

void ReducedCameraSystem::FormDenseSchurComplement(Eigen::MatrixXd* S) const {
  S->setZero(Dimension(), Dimension());
  // Every task writes only to the column block of its camera c, using the
  // symmetric storage of U_off_diagonal and of W^T V^-1 W.
  ParallelFor(
      num_threads, num_cameras, 1, [&](const int begin, const int end, int) {
        for (int c = begin; c < end; ++c) {
          S->block<kB, kB>(kB * c, kB * c) = U_diagonal[c];
          for (int p = off_diagonal_offsets[c];
               p < off_diagonal_offsets[c + 1];
               ++p) {
            const CameraPairBlock& pair = U_off_diagonal[p];
            // U_{b c} = U_{c b}^T.
            S->block<kB, kB>(kB * pair.camera_b, kB * c) +=
                pair.block.transpose();
          }
          for (int idx = camera_offsets[c]; idx < camera_offsets[c + 1];
               ++idx) {
            const int k = camera_W_index[idx];
            const int j = W_point[k];
            const Matrix3x12d VinvWk = V_inverse[j] * W_blocks[k];
            for (int l = point_offsets[j]; l < point_offsets[j + 1]; ++l) {
              S->block<kB, kB>(kB * W_camera[l], kB * c).noalias() -=
                  W_blocks[l].transpose() * VinvWk;
            }
          }
        }
      });
}

void ReducedCameraSystem::MultiplyDamped(const double lambda,
                                         const Eigen::VectorXd& x,
                                         Eigen::VectorXd* out) const {
  Eigen::Matrix3Xd buffer;
  Eigen::VectorXd Mx;
  ApplyM(*this, x, &buffer, &Mx);
  out->resize(x.size());
  for (int i = 0; i < num_cameras; ++i) {
    out->segment<kB>(kB * i) = U_diagonal[i] * x.segment<kB>(kB * i) +
                               lambda * x.segment<kB>(kB * i) -
                               Mx.segment<kB>(kB * i);
  }
}

std::unique_ptr<ReducedCameraSolver> CreateReducedCameraSolver(
    const VarProOptions& options, std::string* error) {
  const bool power_series =
      options.linear_solver_type == ReducedCameraSolverType::POWER_SERIES ||
      options.linear_solver_type == ReducedCameraSolverType::CUDA_POWER_SERIES;
  if (power_series && options.power_series_max_iterations < 1) {
    *error = "power_series_max_iterations must be at least 1.";
    return nullptr;
  }
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
      return std::make_unique<PowerSeriesSolver>(
          options.power_series_max_iterations,
          options.power_series_tolerance);
    case ReducedCameraSolverType::CUDA_POWER_SERIES:
#if THEIA_WITH_CUDA_SOLVERS
      return CreateCudaPowerSeriesSolver(options.power_series_max_iterations,
                                         options.power_series_tolerance,
                                         error);
#else
      *error =
          "CUDA_POWER_SERIES needs a build with -DTHEIA_WITH_CUDA_SOLVERS=ON.";
      return nullptr;
#endif
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
