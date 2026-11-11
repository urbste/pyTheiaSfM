// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#include "theia/sfm/init_free/reduced_camera_solver.h"

#include <Eigen/Cholesky>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
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

// Sparse Cholesky of the Schur complement. Camera blocks a and b are coupled
// only if the cameras share a point or a pairwise rotation term, so S is
// block sparse for sequences and large scenes. The lower triangle is stored
// in compressed column form, one 12-column block per camera (diagonal block
// first, then the neighbor blocks in ascending camera order), and factorized
// with Eigen's simplicial LLT and AMD ordering. Every task writes only its
// own column blocks, so the assembly is deterministic for any thread count.
// The symbolic analysis is reused while the sparsity pattern stays the same.
class SparseCholeskySolver : public ReducedCameraSolver {
 public:
  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    if (cached_revision_ != system.revision || S_.rows() != rhs.size()) {
      const bool pattern_changed = BuildPattern(system);
      FillValues(system);
      if (pattern_changed || !analyzed_) {
        damped_ = S_;
        llt_.analyzePattern(damped_);
        analyzed_ = true;
      }
      cached_revision_ = system.revision;
    }
    damped_ = S_;
    double* values = damped_.valuePtr();
    const int* outer = damped_.outerIndexPtr();
    for (int col = 0; col < damped_.cols(); ++col) {
      values[outer[col]] += lambda;  // first entry of a column: the diagonal
    }
    llt_.factorize(damped_);
    if (llt_.info() != Eigen::Success) {
      *error = "Sparse Cholesky failed (system not positive definite).";
      return false;
    }
    *x = llt_.solve(rhs);
    return x->allFinite();
  }

 private:
  // Neighbor lists (cameras b > c coupled to c) and the CSC index arrays.
  // Returns true if the pattern differs from the previous one.
  bool BuildPattern(const ReducedCameraSystem& system) {
    const int n = system.num_cameras;
    std::vector<std::vector<int>> neighbors(n);
    ParallelFor(
        system.num_threads, n, 1, [&](const int begin, const int end, int) {
          for (int c = begin; c < end; ++c) {
            std::vector<int>& list = neighbors[c];
            for (int idx = system.camera_offsets[c];
                 idx < system.camera_offsets[c + 1];
                 ++idx) {
              const int j = system.W_point[system.camera_W_index[idx]];
              for (int l = system.point_offsets[j];
                   l < system.point_offsets[j + 1];
                   ++l) {
                if (system.W_camera[l] > c) list.push_back(system.W_camera[l]);
              }
            }
            for (int p = system.off_diagonal_offsets[c];
                 p < system.off_diagonal_offsets[c + 1];
                 ++p) {
              if (system.U_off_diagonal[p].camera_b > c) {
                list.push_back(system.U_off_diagonal[p].camera_b);
              }
            }
            std::sort(list.begin(), list.end());
            list.erase(std::unique(list.begin(), list.end()), list.end());
          }
        });
    if (neighbors == neighbors_ && S_.rows() == kB * n) {
      return false;
    }
    neighbors_ = std::move(neighbors);
    block_offsets_.assign(n + 1, 0);  // first nonzero of each column block
    for (int c = 0; c < n; ++c) {
      // Diagonal block: 12 + 11 + ... + 1 = 78 entries; neighbors 144 each.
      block_offsets_[c + 1] = block_offsets_[c] + 78 +
                              kB * kB * static_cast<int>(neighbors_[c].size());
    }
    S_ = Eigen::SparseMatrix<double>(kB * n, kB * n);
    S_.resizeNonZeros(block_offsets_[n]);
    int* outer = S_.outerIndexPtr();
    int* inner = S_.innerIndexPtr();
    for (int c = 0; c < n; ++c) {
      int pos = block_offsets_[c];
      for (int i = 0; i < kB; ++i) {
        outer[kB * c + i] = pos;
        for (int r = i; r < kB; ++r) inner[pos++] = kB * c + r;
        for (const int b : neighbors_[c]) {
          for (int r = 0; r < kB; ++r) inner[pos++] = kB * b + r;
        }
      }
    }
    outer[kB * n] = block_offsets_[n];
    return true;
  }

  void FillValues(const ReducedCameraSystem& system) {
    const int n = system.num_cameras;
    double* values = S_.valuePtr();
    ParallelFor(
        system.num_threads, n, 1, [&](const int begin, const int end, int) {
          // slot[b]: position of camera b in the block column (0 = diagonal).
          std::vector<int> slot(n, -1);
          std::vector<Matrix12d, Eigen::aligned_allocator<Matrix12d>> blocks;
          for (int c = begin; c < end; ++c) {
            const std::vector<int>& list = neighbors_[c];
            blocks.assign(list.size() + 1, Matrix12d::Zero());
            slot[c] = 0;
            for (size_t q = 0; q < list.size(); ++q) slot[list[q]] = q + 1;
            // blocks[q] = S_{b c} (rows of camera b, columns of camera c).
            blocks[0] = system.U_diagonal[c];
            for (int p = system.off_diagonal_offsets[c];
                 p < system.off_diagonal_offsets[c + 1];
                 ++p) {
              const auto& pair = system.U_off_diagonal[p];
              if (pair.camera_b > c) {
                blocks[slot[pair.camera_b]] += pair.block.transpose();
              }
            }
            for (int idx = system.camera_offsets[c];
                 idx < system.camera_offsets[c + 1];
                 ++idx) {
              const int k = system.camera_W_index[idx];
              const int j = system.W_point[k];
              const Matrix3x12d VinvWk = system.V_inverse[j] * system.W_blocks[k];
              for (int l = system.point_offsets[j];
                   l < system.point_offsets[j + 1];
                   ++l) {
                const int b = system.W_camera[l];
                if (b < c) continue;
                blocks[slot[b]].noalias() -=
                    system.W_blocks[l].transpose() * VinvWk;
              }
            }
            int pos = block_offsets_[c];
            for (int i = 0; i < kB; ++i) {
              for (int r = i; r < kB; ++r) values[pos++] = blocks[0](r, i);
              for (size_t q = 0; q < list.size(); ++q) {
                for (int r = 0; r < kB; ++r) {
                  values[pos++] = blocks[q + 1](r, i);
                }
              }
            }
            slot[c] = -1;
            for (const int b : list) slot[b] = -1;
          }
        });
  }

  Eigen::SparseMatrix<double> S_;
  Eigen::SparseMatrix<double> damped_;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>,
                       Eigen::Lower,
                       Eigen::AMDOrdering<int>>
      llt_;
  std::vector<std::vector<int>> neighbors_;
  std::vector<int> block_offsets_;
  int cached_revision_ = -1;
  bool analyzed_ = false;
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
      return std::make_unique<SparseCholeskySolver>();
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
