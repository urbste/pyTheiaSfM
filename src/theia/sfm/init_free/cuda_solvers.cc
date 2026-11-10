// Copyright (C) 2026, Steffen Urban
// All rights reserved.
//
// CUDA backends for the reduced camera system of VarPro. Compiled only with
// -DTHEIA_WITH_CUDA_SOLVERS=ON. They use the CUDA host APIs (cudart, cuBLAS,
// cuSPARSE, cuSOLVER) only, so no CUDA compiler is needed.
//
// Both solvers upload the point blocks once per VarPro iteration as the sparse
// matrix Y (3 rows per point, 12 columns per camera) with blocks
// Y_jk = F_j W_jk, where F_j^T F_j = V_j^-1, so that
//
//   W^T V^-1 W = Y^T Y.
//
// Y^T is formed on the device (csr2csc). The per-observation Jacobians and the
// 3x3 point blocks are still computed on the CPU (multithreaded).
//
//   CUDA_POWER_SERIES:   every term of the power series is two SpMVs with Y
//                        and Y^T, one with the camera-pair blocks O, and a
//                        batched 12x12 product with D_lambda^-1.
//   CUDA_DENSE_CHOLESKY: S = U - Y^T Y is formed on the device (U by
//                        sparse-to-dense, Y^T Y column block by column block
//                        with SpMM), then factorized with cuSOLVER potrf.

#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>
#include <cusparse.h>
#include <glog/logging.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "theia/sfm/init_free/parallel_for.h"
#include "theia/sfm/init_free/reduced_camera_solver.h"

namespace theia {

namespace {

constexpr int kB = ReducedCameraSystem::kBlockSize;

#define THEIA_CUDA_OK(expr, message) \
  do {                               \
    if ((expr) != cudaSuccess) {     \
      *error = message;              \
      return false;                  \
    }                                \
  } while (0)
#define THEIA_CUSPARSE_OK(expr, message)          \
  do {                                            \
    if ((expr) != CUSPARSE_STATUS_SUCCESS) {      \
      *error = std::string(message) + " (cuSPARSE)"; \
      return false;                               \
    }                                             \
  } while (0)
#define THEIA_CUBLAS_OK(expr, message)          \
  do {                                          \
    if ((expr) != CUBLAS_STATUS_SUCCESS) {      \
      *error = std::string(message) + " (cuBLAS)"; \
      return false;                             \
    }                                           \
  } while (0)

// A device array that only grows.
template <typename T>
class DeviceArray {
 public:
  DeviceArray() = default;
  DeviceArray(const DeviceArray&) = delete;
  DeviceArray& operator=(const DeviceArray&) = delete;
  ~DeviceArray() { Free(); }

  bool Reserve(const size_t n) {
    if (n <= capacity_) {
      return true;
    }
    Free();
    if (cudaMalloc(&ptr_, sizeof(T) * std::max<size_t>(n, 1)) != cudaSuccess) {
      ptr_ = nullptr;
      return false;
    }
    capacity_ = n;
    return true;
  }

  bool Upload(const T* host, const size_t n) {
    return Reserve(n) &&
           cudaMemcpy(ptr_, host, sizeof(T) * n, cudaMemcpyHostToDevice) ==
               cudaSuccess;
  }

  void Free() {
    if (ptr_ != nullptr) {
      cudaFree(ptr_);
    }
    ptr_ = nullptr;
    capacity_ = 0;
  }

  T* get() const { return ptr_; }

 private:
  T* ptr_ = nullptr;
  size_t capacity_ = 0;
};

void DestroySpMat(cusparseSpMatDescr_t* mat) {
  if (*mat != nullptr) {
    cusparseDestroySpMat(*mat);
    *mat = nullptr;
  }
}

void DestroyDnVec(cusparseDnVecDescr_t* vec) {
  if (*vec != nullptr) {
    cusparseDestroyDnVec(*vec);
    *vec = nullptr;
  }
}

// F with F^T F = A for a symmetric positive (semi)definite 3x3 A.
Eigen::Matrix3d SquareRootFactor(const Eigen::Matrix3d& A) {
  Eigen::LLT<Eigen::Matrix3d> llt(A);
  if (llt.info() == Eigen::Success) {
    return llt.matrixL().transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(A);
  return eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
         eig.eigenvectors().transpose();
}

// The device copy of a ReducedCameraSystem: Y, Y^T and the camera-pair blocks
// as CSR matrices (32-bit indices).
class DeviceReducedSystem {
 public:
  DeviceReducedSystem() = default;
  DeviceReducedSystem(const DeviceReducedSystem&) = delete;
  DeviceReducedSystem& operator=(const DeviceReducedSystem&) = delete;

  ~DeviceReducedSystem() {
    DestroySpMat(&Y_);
    DestroySpMat(&Yt_);
    DestroySpMat(&pairs_);
    if (sparse_ != nullptr) cusparseDestroy(sparse_);
  }

  bool Init(std::string* error) {
    THEIA_CUSPARSE_OK(cusparseCreate(&sparse_), "cusparseCreate failed");
    return true;
  }

  cusparseHandle_t sparse() const { return sparse_; }
  cusparseSpMatDescr_t Y() const { return Y_; }
  cusparseSpMatDescr_t Yt() const { return Yt_; }
  // Camera-pair blocks O (with the diagonal blocks U_ii if requested in
  // Upload); nullptr if empty.
  cusparseSpMatDescr_t pairs() const { return pairs_; }
  int num_point_rows() const { return 3 * num_points_; }
  int dimension() const { return kB * num_cameras_; }
  int num_cameras() const { return num_cameras_; }

  const std::vector<int>& yt_row_offsets() const { return h_yt_rowptr_; }
  const int* yt_columns() const { return d_yt_col_.get(); }
  const double* yt_values() const { return d_yt_val_.get(); }

  bool Upload(const ReducedCameraSystem& system,
              const bool with_diagonal,
              std::string* error) {
    if (!SameStructure(system) && !BuildStructure(system, error)) {
      return false;
    }
    // Values of Y.
    const int num_points = num_points_;
    ParallelFor(
        system.num_threads,
        num_points,
        256,
        [&](const int begin, const int end, int) {
          for (int j = begin; j < end; ++j) {
            const Eigen::Matrix3d F = SquareRootFactor(system.V_inverse[j]);
            const int first = system.point_offsets[j];
            const int count = system.point_offsets[j + 1] - first;
            for (int k = first; k < first + count; ++k) {
              const Matrix3x12d Yk = F * system.W_blocks[k];
              for (int r = 0; r < 3; ++r) {
                double* row = h_y_val_.data() + h_y_rowptr_[3 * j + r] +
                              kB * (k - first);
                for (int q = 0; q < kB; ++q) {
                  row[q] = Yk(r, q);
                }
              }
            }
          }
        });
    THEIA_CUDA_OK(cudaMemcpy(d_y_val_.get(),
                             h_y_val_.data(),
                             sizeof(double) * nnz_y_,
                             cudaMemcpyHostToDevice),
                  "Uploading Y failed.");
    THEIA_CUSPARSE_OK(cusparseCsr2cscEx2(sparse_,
                                         3 * num_points_,
                                         dimension(),
                                         static_cast<int>(nnz_y_),
                                         d_y_val_.get(),
                                         d_y_rowptr_.get(),
                                         d_y_col_.get(),
                                         d_yt_val_.get(),
                                         d_yt_rowptr_.get(),
                                         d_yt_col_.get(),
                                         CUDA_R_64F,
                                         CUSPARSE_ACTION_NUMERIC,
                                         CUSPARSE_INDEX_BASE_ZERO,
                                         CUSPARSE_CSR2CSC_ALG1,
                                         d_csr2csc_buffer_.get()),
                      "Transposing Y failed");
    return UploadPairs(system, with_diagonal, error);
  }

 private:
  bool SameStructure(const ReducedCameraSystem& system) const {
    return Y_ != nullptr && system.num_cameras == num_cameras_ &&
           static_cast<int>(system.V_inverse.size()) == num_points_ &&
           system.W_camera == w_camera_;
  }

  bool BuildStructure(const ReducedCameraSystem& system, std::string* error) {
    num_cameras_ = system.num_cameras;
    num_points_ = static_cast<int>(system.V_inverse.size());
    w_camera_ = system.W_camera;
    const size_t nnz = static_cast<size_t>(kB) * 3 * system.W_blocks.size();
    if (nnz > static_cast<size_t>(std::numeric_limits<int>::max())) {
      *error = "Problem too large for the 32-bit CUDA sparse backend.";
      return false;
    }
    nnz_y_ = nnz;
    h_y_rowptr_.resize(3 * num_points_ + 1);
    std::vector<int> col(nnz);
    for (int j = 0; j < num_points_; ++j) {
      const int first = system.point_offsets[j];
      const int count = system.point_offsets[j + 1] - first;
      for (int r = 0; r < 3; ++r) {
        const int start = 3 * kB * first + r * kB * count;
        h_y_rowptr_[3 * j + r] = start;
        for (int i = 0; i < count; ++i) {
          for (int q = 0; q < kB; ++q) {
            col[start + kB * i + q] = kB * system.W_camera[first + i] + q;
          }
        }
      }
    }
    h_y_rowptr_[3 * num_points_] = static_cast<int>(nnz);
    h_y_val_.assign(nnz, 0.0);

    // Y^T row offsets: row 12 c + q has 3 entries per observation of camera c.
    h_yt_rowptr_.assign(dimension() + 1, 0);
    for (int c = 0; c < num_cameras_; ++c) {
      const int count = system.camera_offsets[c + 1] - system.camera_offsets[c];
      for (int q = 0; q < kB; ++q) {
        h_yt_rowptr_[kB * c + q + 1] = 3 * count;
      }
    }
    for (int i = 0; i < dimension(); ++i) {
      h_yt_rowptr_[i + 1] += h_yt_rowptr_[i];
    }

    if (!d_y_rowptr_.Upload(h_y_rowptr_.data(), h_y_rowptr_.size()) ||
        !d_y_col_.Upload(col.data(), nnz) || !d_y_val_.Reserve(nnz) ||
        !d_yt_rowptr_.Reserve(dimension() + 1) || !d_yt_col_.Reserve(nnz) ||
        !d_yt_val_.Reserve(nnz)) {
      *error = "cudaMalloc failed for the sparse point blocks (" +
               std::to_string(nnz) + " nonzeros).";
      return false;
    }
    DestroySpMat(&Y_);
    DestroySpMat(&Yt_);
    THEIA_CUSPARSE_OK(cusparseCreateCsr(&Y_,
                                        3 * num_points_,
                                        dimension(),
                                        static_cast<int64_t>(nnz),
                                        d_y_rowptr_.get(),
                                        d_y_col_.get(),
                                        d_y_val_.get(),
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_BASE_ZERO,
                                        CUDA_R_64F),
                      "Creating Y failed");
    THEIA_CUSPARSE_OK(cusparseCreateCsr(&Yt_,
                                        dimension(),
                                        3 * num_points_,
                                        static_cast<int64_t>(nnz),
                                        d_yt_rowptr_.get(),
                                        d_yt_col_.get(),
                                        d_yt_val_.get(),
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_BASE_ZERO,
                                        CUDA_R_64F),
                      "Creating Y^T failed");
    size_t buffer_size = 0;
    THEIA_CUSPARSE_OK(cusparseCsr2cscEx2_bufferSize(sparse_,
                                                    3 * num_points_,
                                                    dimension(),
                                                    static_cast<int>(nnz),
                                                    d_y_val_.get(),
                                                    d_y_rowptr_.get(),
                                                    d_y_col_.get(),
                                                    d_yt_val_.get(),
                                                    d_yt_rowptr_.get(),
                                                    d_yt_col_.get(),
                                                    CUDA_R_64F,
                                                    CUSPARSE_ACTION_NUMERIC,
                                                    CUSPARSE_INDEX_BASE_ZERO,
                                                    CUSPARSE_CSR2CSC_ALG1,
                                                    &buffer_size),
                      "csr2csc buffer size failed");
    if (!d_csr2csc_buffer_.Reserve(buffer_size)) {
      *error = "cudaMalloc failed for the csr2csc buffer.";
      return false;
    }
    return true;
  }

  // CSR of the camera-pair blocks (12 entries per block row), optionally with
  // the diagonal blocks; blocks of a row are sorted by column camera.
  bool UploadPairs(const ReducedCameraSystem& system,
                   const bool with_diagonal,
                   std::string* error) {
    const int n = num_cameras_;
    std::vector<std::vector<std::pair<int, const Matrix12d*>>> rows(n);
    for (int i = 0; i < n; ++i) {
      if (with_diagonal) {
        rows[i].emplace_back(i, &system.U_diagonal[i]);
      }
      for (int p = system.off_diagonal_offsets[i];
           p < system.off_diagonal_offsets[i + 1];
           ++p) {
        rows[i].emplace_back(system.U_off_diagonal[p].camera_b,
                             &system.U_off_diagonal[p].block);
      }
      std::sort(rows[i].begin(), rows[i].end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
      });
    }
    std::vector<int> rowptr(dimension() + 1, 0);
    std::vector<int> col;
    std::vector<double> val;
    for (int i = 0; i < n; ++i) {
      for (int q = 0; q < kB; ++q) {
        for (const auto& [b, block] : rows[i]) {
          for (int s = 0; s < kB; ++s) {
            col.push_back(kB * b + s);
            val.push_back((*block)(q, s));
          }
        }
        rowptr[kB * i + q + 1] = static_cast<int>(col.size());
      }
    }
    DestroySpMat(&pairs_);
    if (col.empty()) {
      return true;
    }
    if (!d_pair_rowptr_.Upload(rowptr.data(), rowptr.size()) ||
        !d_pair_col_.Upload(col.data(), col.size()) ||
        !d_pair_val_.Upload(val.data(), val.size())) {
      *error = "Uploading the camera-pair blocks failed.";
      return false;
    }
    THEIA_CUSPARSE_OK(cusparseCreateCsr(&pairs_,
                                        dimension(),
                                        dimension(),
                                        static_cast<int64_t>(col.size()),
                                        d_pair_rowptr_.get(),
                                        d_pair_col_.get(),
                                        d_pair_val_.get(),
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_32I,
                                        CUSPARSE_INDEX_BASE_ZERO,
                                        CUDA_R_64F),
                      "Creating the camera-pair matrix failed");
    return true;
  }

  cusparseHandle_t sparse_ = nullptr;
  int num_cameras_ = 0;
  int num_points_ = 0;
  size_t nnz_y_ = 0;
  std::vector<int> w_camera_;

  std::vector<int> h_y_rowptr_;
  std::vector<double> h_y_val_;
  std::vector<int> h_yt_rowptr_;
  DeviceArray<int> d_y_rowptr_, d_y_col_, d_yt_rowptr_, d_yt_col_;
  DeviceArray<double> d_y_val_, d_yt_val_;
  DeviceArray<char> d_csr2csc_buffer_;
  DeviceArray<int> d_pair_rowptr_, d_pair_col_;
  DeviceArray<double> d_pair_val_;

  cusparseSpMatDescr_t Y_ = nullptr;
  cusparseSpMatDescr_t Yt_ = nullptr;
  cusparseSpMatDescr_t pairs_ = nullptr;
};

// y = alpha A x + beta y, with a growing workspace.
class SpMV {
 public:
  ~SpMV() {
    DestroyDnVec(&x_);
    DestroyDnVec(&y_);
  }

  bool Run(cusparseHandle_t handle,
           cusparseSpMatDescr_t A,
           const double alpha,
           const double* x,
           const int x_size,
           const double beta,
           double* y,
           const int y_size,
           std::string* error) {
    DestroyDnVec(&x_);
    DestroyDnVec(&y_);
    THEIA_CUSPARSE_OK(
        cusparseCreateDnVec(&x_, x_size, const_cast<double*>(x), CUDA_R_64F),
        "Creating a dense vector failed");
    THEIA_CUSPARSE_OK(cusparseCreateDnVec(&y_, y_size, y, CUDA_R_64F),
                      "Creating a dense vector failed");
    size_t size = 0;
    THEIA_CUSPARSE_OK(cusparseSpMV_bufferSize(handle,
                                              CUSPARSE_OPERATION_NON_TRANSPOSE,
                                              &alpha,
                                              A,
                                              x_,
                                              &beta,
                                              y_,
                                              CUDA_R_64F,
                                              CUSPARSE_SPMV_CSR_ALG2,
                                              &size),
                      "SpMV buffer size failed");
    if (!buffer_.Reserve(size)) {
      *error = "cudaMalloc failed for the SpMV buffer.";
      return false;
    }
    THEIA_CUSPARSE_OK(cusparseSpMV(handle,
                                   CUSPARSE_OPERATION_NON_TRANSPOSE,
                                   &alpha,
                                   A,
                                   x_,
                                   &beta,
                                   y_,
                                   CUDA_R_64F,
                                   CUSPARSE_SPMV_CSR_ALG2,
                                   buffer_.get()),
                      "SpMV failed");
    return true;
  }

 private:
  cusparseDnVecDescr_t x_ = nullptr;
  cusparseDnVecDescr_t y_ = nullptr;
  DeviceArray<char> buffer_;
};

class CudaPowerSeriesSolver : public ReducedCameraSolver {
 public:
  CudaPowerSeriesSolver(const int max_iterations, const double tolerance)
      : max_iterations_(max_iterations), tolerance_(tolerance) {}

  ~CudaPowerSeriesSolver() override {
    if (blas_ != nullptr) cublasDestroy(blas_);
  }

  bool Init(std::string* error) {
    THEIA_CUBLAS_OK(cublasCreate(&blas_), "cublasCreate failed");
    return device_.Init(error);
  }

  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    if (cached_revision_ != system.revision) {
      if (!device_.Upload(system, /*with_diagonal=*/false, error)) {
        return false;
      }
      cached_revision_ = system.revision;
    }
    const int n = system.num_cameras;
    const int dim = kB * n;
    const int point_rows = device_.num_point_rows();

    // D_lambda^-1 on the host (n small 12x12 inversions).
    h_dinv_.resize(static_cast<size_t>(kB) * kB * n);
    std::vector<char> ok(n, 1);
    ParallelFor(system.num_threads, n, 4, [&](const int begin, const int end, int) {
      for (int i = begin; i < end; ++i) {
        Matrix12d D = system.U_diagonal[i];
        D.diagonal().array() += lambda;
        Eigen::LLT<Matrix12d> llt(D);
        ok[i] = llt.info() == Eigen::Success;
        Eigen::Map<Matrix12d>(h_dinv_.data() + kB * kB * i) =
            llt.solve(Matrix12d::Identity());
      }
    });
    if (std::find(ok.begin(), ok.end(), 0) != ok.end()) {
      *error = "Power series: camera block not positive definite.";
      return false;
    }
    if (!d_dinv_.Upload(h_dinv_.data(), h_dinv_.size()) ||
        !d_b_.Upload(rhs.data(), dim) || !d_x_.Reserve(dim) ||
        !d_term_.Reserve(dim) || !d_prod_.Reserve(dim) ||
        !d_points_.Reserve(point_rows)) {
      *error = "CUDA power series: allocation or upload failed.";
      return false;
    }

    if (!ApplyDInverse(n, d_b_.get(), d_term_.get(), error)) return false;
    THEIA_CUDA_OK(cudaMemcpy(d_x_.get(),
                             d_term_.get(),
                             sizeof(double) * dim,
                             cudaMemcpyDeviceToDevice),
                  "Device copy failed.");
    const double one = 1.0;
    for (int k = 1; k < max_iterations_; ++k) {
      // prod = (Y^T Y - O) term.
      if (!spmv_y_.Run(device_.sparse(), device_.Y(), 1.0, d_term_.get(), dim,
                       0.0, d_points_.get(), point_rows, error) ||
          !spmv_yt_.Run(device_.sparse(), device_.Yt(), 1.0, d_points_.get(),
                        point_rows, 0.0, d_prod_.get(), dim, error)) {
        return false;
      }
      if (device_.pairs() != nullptr &&
          !spmv_pairs_.Run(device_.sparse(), device_.pairs(), -1.0,
                           d_term_.get(), dim, 1.0, d_prod_.get(), dim,
                           error)) {
        return false;
      }
      if (!ApplyDInverse(n, d_prod_.get(), d_term_.get(), error)) return false;
      THEIA_CUBLAS_OK(cublasDaxpy(blas_, dim, &one, d_term_.get(), 1,
                                  d_x_.get(), 1),
                      "cublasDaxpy failed");
      double term_norm = 0.0, x_norm = 0.0;
      THEIA_CUBLAS_OK(cublasDnrm2(blas_, dim, d_term_.get(), 1, &term_norm),
                      "cublasDnrm2 failed");
      THEIA_CUBLAS_OK(cublasDnrm2(blas_, dim, d_x_.get(), 1, &x_norm),
                      "cublasDnrm2 failed");
      if (!std::isfinite(term_norm)) {
        *error = "Power series produced non-finite values.";
        return false;
      }
      if (term_norm <= tolerance_ * x_norm) {
        break;
      }
    }
    x->resize(dim);
    THEIA_CUDA_OK(cudaMemcpy(x->data(),
                             d_x_.get(),
                             sizeof(double) * dim,
                             cudaMemcpyDeviceToHost),
                  "Downloading the solution failed.");
    return x->allFinite();
  }

 private:
  bool ApplyDInverse(const int n,
                     const double* in,
                     double* out,
                     std::string* error) {
    const double one = 1.0, zero = 0.0;
    THEIA_CUBLAS_OK(cublasDgemvStridedBatched(blas_,
                                              CUBLAS_OP_N,
                                              kB,
                                              kB,
                                              &one,
                                              d_dinv_.get(),
                                              kB,
                                              kB * kB,
                                              in,
                                              1,
                                              kB,
                                              &zero,
                                              out,
                                              1,
                                              kB,
                                              n),
                    "Batched D^-1 product failed");
    return true;
  }

  const int max_iterations_;
  const double tolerance_;
  cublasHandle_t blas_ = nullptr;
  DeviceReducedSystem device_;
  SpMV spmv_y_, spmv_yt_, spmv_pairs_;
  std::vector<double> h_dinv_;
  DeviceArray<double> d_dinv_, d_b_, d_x_, d_term_, d_prod_, d_points_;
  int cached_revision_ = -1;
};

class CudaDenseCholeskySolver : public ReducedCameraSolver {
 public:
  ~CudaDenseCholeskySolver() override {
    if (solver_ != nullptr) cusolverDnDestroy(solver_);
    if (blas_ != nullptr) cublasDestroy(blas_);
  }

  bool Init(std::string* error) {
    if (cusolverDnCreate(&solver_) != CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnCreate failed.";
      return false;
    }
    THEIA_CUBLAS_OK(cublasCreate(&blas_), "cublasCreate failed");
    return device_.Init(error);
  }

  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    const int n = system.Dimension();
    const size_t matrix_bytes = sizeof(double) * static_cast<size_t>(n) * n;
    if (!d_S_.Reserve(static_cast<size_t>(n) * n) ||
        !d_A_.Reserve(static_cast<size_t>(n) * n) || !d_ones_.Reserve(n) ||
        !d_rhs_.Reserve(n) || !d_info_.Reserve(1)) {
      *error = "cudaMalloc failed for a reduced system of dimension " +
               std::to_string(n) + ".";
      return false;
    }
    if (cached_revision_ != system.revision) {
      if (!FormSchurComplement(system, error)) {
        return false;
      }
      cached_revision_ = system.revision;
    }
    THEIA_CUDA_OK(cudaMemcpy(d_A_.get(), d_S_.get(), matrix_bytes,
                             cudaMemcpyDeviceToDevice),
                  "Device copy failed.");
    if (ones_size_ != n) {
      const std::vector<double> ones(n, 1.0);
      if (!d_ones_.Upload(ones.data(), n)) {
        *error = "Upload failed.";
        return false;
      }
      ones_size_ = n;
    }
    // A += lambda * I: the diagonal of a column-major n x n matrix is a vector
    // with stride n + 1.
    THEIA_CUBLAS_OK(cublasDaxpy(blas_, n, &lambda, d_ones_.get(), 1,
                                d_A_.get(), n + 1),
                    "cublasDaxpy failed");
    int lwork = 0;
    if (cusolverDnDpotrf_bufferSize(solver_, CUBLAS_FILL_MODE_LOWER, n,
                                    d_A_.get(), n, &lwork) !=
            CUSOLVER_STATUS_SUCCESS ||
        !d_work_.Reserve(lwork)) {
      *error = "Allocating the cuSOLVER workspace failed.";
      return false;
    }
    if (cusolverDnDpotrf(solver_, CUBLAS_FILL_MODE_LOWER, n, d_A_.get(), n,
                         d_work_.get(), lwork, d_info_.get()) !=
        CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnDpotrf failed.";
      return false;
    }
    int info = 0;
    cudaMemcpy(&info, d_info_.get(), sizeof(int), cudaMemcpyDeviceToHost);
    if (info != 0) {
      *error = "CUDA Cholesky failed (system not positive definite).";
      return false;
    }
    if (!d_rhs_.Upload(rhs.data(), n)) {
      *error = "Uploading the right-hand side failed.";
      return false;
    }
    if (cusolverDnDpotrs(solver_, CUBLAS_FILL_MODE_LOWER, n, 1, d_A_.get(), n,
                         d_rhs_.get(), n, d_info_.get()) !=
        CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnDpotrs failed.";
      return false;
    }
    x->resize(n);
    THEIA_CUDA_OK(cudaMemcpy(x->data(), d_rhs_.get(), sizeof(double) * n,
                             cudaMemcpyDeviceToHost),
                  "Downloading the solution failed.");
    return x->allFinite();
  }

 private:
  // S = U - Y^T Y on the device. U (diagonal and pair blocks) is expanded with
  // sparse-to-dense. Y^T Y is accumulated for a band of camera columns at a
  // time: Z = Y[:, band] is obtained densely from the rows of Y^T in the band
  // (a dense row-major 12w x 3P matrix is Z in column-major order), then
  // S[:, band] -= Y^T Z with SpMM.
  bool FormSchurComplement(const ReducedCameraSystem& system,
                           std::string* error) {
    if (!device_.Upload(system, /*with_diagonal=*/true, error)) {
      return false;
    }
    const int dim = device_.dimension();
    const int point_rows = device_.num_point_rows();
    cusparseHandle_t handle = device_.sparse();

    cusparseDnMatDescr_t S = nullptr;
    THEIA_CUSPARSE_OK(cusparseCreateDnMat(&S, dim, dim, dim, d_S_.get(),
                                          CUDA_R_64F, CUSPARSE_ORDER_COL),
                      "Creating S failed");
    size_t size = 0;
    bool ok = cusparseSparseToDense_bufferSize(
                  handle, device_.pairs(), S,
                  CUSPARSE_SPARSETODENSE_ALG_DEFAULT, &size) ==
                  CUSPARSE_STATUS_SUCCESS &&
              buffer_.Reserve(size) &&
              cusparseSparseToDense(handle, device_.pairs(), S,
                                    CUSPARSE_SPARSETODENSE_ALG_DEFAULT,
                                    buffer_.get()) == CUSPARSE_STATUS_SUCCESS;
    cusparseDestroyDnMat(S);
    if (!ok) {
      *error = "Expanding U on the device failed.";
      return false;
    }

    // Band width in cameras: Z (3P x 12w doubles) stays below ~512 MB.
    const size_t max_z = (size_t{512} << 20) / sizeof(double);
    const int band = std::max(
        1,
        std::min(device_.num_cameras(),
                 static_cast<int>(max_z / (static_cast<size_t>(point_rows) *
                                           kB + 1))));
    if (!d_Z_.Reserve(static_cast<size_t>(point_rows) * kB * band)) {
      *error = "cudaMalloc failed for the Schur band buffer.";
      return false;
    }
    const std::vector<int>& yt_rowptr = device_.yt_row_offsets();
    std::vector<int> band_rowptr;
    for (int c0 = 0; c0 < device_.num_cameras(); c0 += band) {
      const int c1 = std::min(device_.num_cameras(), c0 + band);
      const int rows = kB * (c1 - c0);
      const int base = yt_rowptr[kB * c0];
      const int nnz = yt_rowptr[kB * c1] - base;
      band_rowptr.resize(rows + 1);
      for (int i = 0; i <= rows; ++i) {
        band_rowptr[i] = yt_rowptr[kB * c0 + i] - base;
      }
      if (!d_band_rowptr_.Upload(band_rowptr.data(), band_rowptr.size())) {
        *error = "Upload failed.";
        return false;
      }
      THEIA_CUDA_OK(cudaMemset(d_Z_.get(), 0,
                               sizeof(double) * point_rows * rows),
                    "cudaMemset failed.");
      if (nnz == 0) {
        continue;
      }
      cusparseSpMatDescr_t Yt_band = nullptr;
      cusparseDnMatDescr_t Z = nullptr, Z_col = nullptr, S_band = nullptr;
      bool band_ok =
          cusparseCreateCsr(&Yt_band, rows, point_rows, nnz,
                            d_band_rowptr_.get(),
                            const_cast<int*>(device_.yt_columns()) + base,
                            const_cast<double*>(device_.yt_values()) + base,
                            CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                            CUSPARSE_INDEX_BASE_ZERO,
                            CUDA_R_64F) == CUSPARSE_STATUS_SUCCESS &&
          cusparseCreateDnMat(&Z, rows, point_rows, point_rows, d_Z_.get(),
                              CUDA_R_64F,
                              CUSPARSE_ORDER_ROW) == CUSPARSE_STATUS_SUCCESS &&
          cusparseCreateDnMat(&Z_col, point_rows, rows, point_rows,
                              d_Z_.get(), CUDA_R_64F,
                              CUSPARSE_ORDER_COL) == CUSPARSE_STATUS_SUCCESS &&
          cusparseCreateDnMat(&S_band, dim, rows, dim,
                              d_S_.get() + static_cast<size_t>(dim) * kB * c0,
                              CUDA_R_64F,
                              CUSPARSE_ORDER_COL) == CUSPARSE_STATUS_SUCCESS;
      if (band_ok) {
        size = 0;
        band_ok = cusparseSparseToDense_bufferSize(
                      handle, Yt_band, Z, CUSPARSE_SPARSETODENSE_ALG_DEFAULT,
                      &size) == CUSPARSE_STATUS_SUCCESS &&
                  buffer_.Reserve(size) &&
                  cusparseSparseToDense(handle, Yt_band, Z,
                                        CUSPARSE_SPARSETODENSE_ALG_DEFAULT,
                                        buffer_.get()) ==
                      CUSPARSE_STATUS_SUCCESS;
      }
      const double minus_one = -1.0, one = 1.0;
      if (band_ok) {
        size = 0;
        band_ok =
            cusparseSpMM_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                                    CUSPARSE_OPERATION_NON_TRANSPOSE,
                                    &minus_one, device_.Yt(), Z_col, &one,
                                    S_band, CUDA_R_64F, CUSPARSE_SPMM_CSR_ALG1,
                                    &size) == CUSPARSE_STATUS_SUCCESS &&
            buffer_.Reserve(size) &&
            cusparseSpMM(handle, CUSPARSE_OPERATION_NON_TRANSPOSE,
                         CUSPARSE_OPERATION_NON_TRANSPOSE, &minus_one,
                         device_.Yt(), Z_col, &one, S_band, CUDA_R_64F,
                         CUSPARSE_SPMM_CSR_ALG1,
                         buffer_.get()) == CUSPARSE_STATUS_SUCCESS;
      }
      if (Yt_band != nullptr) cusparseDestroySpMat(Yt_band);
      if (Z != nullptr) cusparseDestroyDnMat(Z);
      if (Z_col != nullptr) cusparseDestroyDnMat(Z_col);
      if (S_band != nullptr) cusparseDestroyDnMat(S_band);
      if (!band_ok) {
        *error = "Forming the Schur complement on the device failed.";
        return false;
      }
    }
    return true;
  }

  cusolverDnHandle_t solver_ = nullptr;
  cublasHandle_t blas_ = nullptr;
  DeviceReducedSystem device_;
  DeviceArray<double> d_S_;  // undamped Schur complement
  DeviceArray<double> d_A_;  // damped copy, overwritten by the factorization
  DeviceArray<double> d_ones_, d_rhs_, d_work_, d_Z_;
  DeviceArray<int> d_info_, d_band_rowptr_;
  DeviceArray<char> buffer_;
  int ones_size_ = 0;
  int cached_revision_ = -1;
};

}  // namespace

bool CudaDeviceAvailable() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

std::unique_ptr<ReducedCameraSolver> CreateCudaDenseCholeskySolver(
    std::string* error) {
  if (!CudaDeviceAvailable()) {
    *error = "No CUDA device available.";
    return nullptr;
  }
  auto solver = std::make_unique<CudaDenseCholeskySolver>();
  if (!solver->Init(error)) {
    return nullptr;
  }
  return solver;
}

std::unique_ptr<ReducedCameraSolver> CreateCudaPowerSeriesSolver(
    const int max_iterations, const double tolerance, std::string* error) {
  if (!CudaDeviceAvailable()) {
    *error = "No CUDA device available.";
    return nullptr;
  }
  auto solver =
      std::make_unique<CudaPowerSeriesSolver>(max_iterations, tolerance);
  if (!solver->Init(error)) {
    return nullptr;
  }
  return solver;
}

}  // namespace theia
