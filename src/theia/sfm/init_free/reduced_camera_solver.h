// Copyright (C) 2026, Steffen Urban
// All rights reserved.

#ifndef THEIA_SFM_INIT_FREE_REDUCED_CAMERA_SOLVER_H_
#define THEIA_SFM_INIT_FREE_REDUCED_CAMERA_SOLVER_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <memory>
#include <string>
#include <vector>

#include "theia/sfm/init_free/init_free_options.h"

namespace theia {

using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix3x12d = Eigen::Matrix<double, 3, 12>;

// The Gauss-Newton system of one VarPro step in block form. With the points
// eliminated, the camera update solves
//
//   (S + lambda I) dx = rhs,   S = U - sum_j W_j^T V_j^{-1} W_j,
//
// where U holds the camera-camera blocks (observations and camera-only terms),
// W_j the point-camera blocks of point j and V_j its 3x3 point block. Solvers
// may form S densely (DENSE_CHOLESKY, CUDA_DENSE_CHOLESKY) or use the blocks
// matrix-free (POWER_SERIES).
struct ReducedCameraSystem {
  static constexpr int kBlockSize = 12;

  struct CameraPairBlock {
    int camera_a;  // row block
    int camera_b;  // column block, camera_a != camera_b
    Matrix12d block;
  };

  int num_cameras = 0;

  // Incremented every time the system is rebuilt, so solvers can cache
  // factorization inputs across repeated solves with different lambda.
  int revision = 0;

  std::vector<Matrix12d, Eigen::aligned_allocator<Matrix12d>> U_diagonal;
  // Both orientations (a, b) and (b, a) are stored, so that the blocks
  // describe the full symmetric matrix.
  std::vector<CameraPairBlock, Eigen::aligned_allocator<CameraPairBlock>>
      U_off_diagonal;

  // Point blocks: W blocks of point j are [point_offsets[j],
  // point_offsets[j + 1]).
  std::vector<Eigen::Matrix3d> V_inverse;
  std::vector<int> point_offsets;
  std::vector<int> W_camera;
  std::vector<Matrix3x12d, Eigen::aligned_allocator<Matrix3x12d>> W_blocks;

  int Dimension() const { return kBlockSize * num_cameras; }

  // Forms the dense, symmetric Schur complement S (without damping).
  void FormDenseSchurComplement(Eigen::MatrixXd* S) const;
};

class ReducedCameraSolver {
 public:
  virtual ~ReducedCameraSolver() = default;

  // Solves (S + lambda I) x = rhs. Returns false if the system could not be
  // factorized (e.g. not positive definite); the caller then increases lambda.
  virtual bool Solve(const ReducedCameraSystem& system,
                     double lambda,
                     const Eigen::VectorXd& rhs,
                     Eigen::VectorXd* x,
                     std::string* error) = 0;
};

// Returns nullptr and sets `error` if the type is not implemented or not
// compiled in.
std::unique_ptr<ReducedCameraSolver> CreateReducedCameraSolver(
    const VarProOptions& options, std::string* error);

// True if this build has the cuSOLVER backend and a CUDA device is visible.
bool CudaReducedCameraSolverAvailable();

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_REDUCED_CAMERA_SOLVER_H_
