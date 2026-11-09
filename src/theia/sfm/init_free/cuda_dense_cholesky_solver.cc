// Copyright (C) 2026, Steffen Urban
// All rights reserved.
//
// cuSOLVER dense Cholesky backend for the reduced camera system. Compiled only
// with -DTHEIA_WITH_CUDA_SOLVERS=ON. Uses the CUDA host APIs (cudart, cuBLAS,
// cuSOLVER) only, so no CUDA compiler is needed.
//
// The Schur complement is still formed on the CPU and uploaded once per VarPro
// iteration. Repeated solves with a different lambda (rejected steps) reuse the
// uploaded matrix. The O(n^3) factorization runs on the GPU.

#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <cusolverDn.h>
#include <glog/logging.h>

#include <memory>
#include <string>
#include <vector>

#include "theia/sfm/init_free/reduced_camera_solver.h"

namespace theia {

namespace {

template <typename T>
void FreeDevice(T** ptr) {
  if (*ptr != nullptr) {
    cudaFree(*ptr);
    *ptr = nullptr;
  }
}

class CudaDenseCholeskySolver : public ReducedCameraSolver {
 public:
  CudaDenseCholeskySolver() = default;

  ~CudaDenseCholeskySolver() override {
    Release();
    if (solver_ != nullptr) cusolverDnDestroy(solver_);
    if (blas_ != nullptr) cublasDestroy(blas_);
  }

  bool Init(std::string* error) {
    if (cusolverDnCreate(&solver_) != CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnCreate failed.";
      return false;
    }
    if (cublasCreate(&blas_) != CUBLAS_STATUS_SUCCESS) {
      *error = "cublasCreate failed.";
      return false;
    }
    return true;
  }

  bool Solve(const ReducedCameraSystem& system,
             const double lambda,
             const Eigen::VectorXd& rhs,
             Eigen::VectorXd* x,
             std::string* error) override {
    const int n = system.Dimension();
    if (n != n_ && !Allocate(n, error)) {
      return false;
    }
    if (cached_revision_ != system.revision) {
      system.FormDenseSchurComplement(&S_host_);
      if (cudaMemcpy(d_S_,
                     S_host_.data(),
                     sizeof(double) * n * n,
                     cudaMemcpyHostToDevice) != cudaSuccess) {
        *error = "Uploading the Schur complement failed.";
        return false;
      }
      cached_revision_ = system.revision;
    }
    if (cudaMemcpy(d_A_, d_S_, sizeof(double) * n * n,
                   cudaMemcpyDeviceToDevice) != cudaSuccess) {
      *error = "Device copy failed.";
      return false;
    }
    // A += lambda * I: the diagonal of a column-major n x n matrix is a vector
    // with stride n + 1.
    if (cublasDaxpy(blas_, n, &lambda, d_ones_, 1, d_A_, n + 1) !=
        CUBLAS_STATUS_SUCCESS) {
      *error = "cublasDaxpy failed.";
      return false;
    }
    if (cusolverDnDpotrf(solver_, CUBLAS_FILL_MODE_LOWER, n, d_A_, n,
                         d_work_, lwork_, d_info_) !=
        CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnDpotrf failed.";
      return false;
    }
    int info = 0;
    cudaMemcpy(&info, d_info_, sizeof(int), cudaMemcpyDeviceToHost);
    if (info != 0) {
      *error = "CUDA Cholesky failed (system not positive definite).";
      return false;
    }
    if (cudaMemcpy(d_rhs_, rhs.data(), sizeof(double) * n,
                   cudaMemcpyHostToDevice) != cudaSuccess) {
      *error = "Uploading the right-hand side failed.";
      return false;
    }
    if (cusolverDnDpotrs(solver_, CUBLAS_FILL_MODE_LOWER, n, 1, d_A_, n,
                         d_rhs_, n, d_info_) != CUSOLVER_STATUS_SUCCESS) {
      *error = "cusolverDnDpotrs failed.";
      return false;
    }
    x->resize(n);
    if (cudaMemcpy(x->data(), d_rhs_, sizeof(double) * n,
                   cudaMemcpyDeviceToHost) != cudaSuccess) {
      *error = "Downloading the solution failed.";
      return false;
    }
    return x->allFinite();
  }

 private:
  bool Allocate(const int n, std::string* error) {
    Release();
    const size_t matrix_bytes = sizeof(double) * static_cast<size_t>(n) * n;
    if (cudaMalloc(&d_S_, matrix_bytes) != cudaSuccess ||
        cudaMalloc(&d_A_, matrix_bytes) != cudaSuccess ||
        cudaMalloc(&d_ones_, sizeof(double) * n) != cudaSuccess ||
        cudaMalloc(&d_rhs_, sizeof(double) * n) != cudaSuccess ||
        cudaMalloc(&d_info_, sizeof(int)) != cudaSuccess) {
      *error = "cudaMalloc failed for a reduced system of dimension " +
               std::to_string(n) + ".";
      Release();
      return false;
    }
    const std::vector<double> ones(n, 1.0);
    cudaMemcpy(d_ones_, ones.data(), sizeof(double) * n,
               cudaMemcpyHostToDevice);
    if (cusolverDnDpotrf_bufferSize(solver_, CUBLAS_FILL_MODE_LOWER, n, d_A_,
                                    n, &lwork_) != CUSOLVER_STATUS_SUCCESS ||
        cudaMalloc(&d_work_, sizeof(double) * lwork_) != cudaSuccess) {
      *error = "Allocating the cuSOLVER workspace failed.";
      Release();
      return false;
    }
    n_ = n;
    cached_revision_ = -1;
    return true;
  }

  void Release() {
    FreeDevice(&d_S_);
    FreeDevice(&d_A_);
    FreeDevice(&d_ones_);
    FreeDevice(&d_rhs_);
    FreeDevice(&d_work_);
    FreeDevice(&d_info_);
    n_ = 0;
    lwork_ = 0;
  }

  cusolverDnHandle_t solver_ = nullptr;
  cublasHandle_t blas_ = nullptr;
  double* d_S_ = nullptr;     // undamped Schur complement
  double* d_A_ = nullptr;     // damped copy, overwritten by the factorization
  double* d_ones_ = nullptr;
  double* d_rhs_ = nullptr;
  double* d_work_ = nullptr;
  int* d_info_ = nullptr;
  int n_ = 0;
  int lwork_ = 0;
  int cached_revision_ = -1;
  Eigen::MatrixXd S_host_;
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

}  // namespace theia
