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
// matrix-free (POWER_SERIES, CUDA_POWER_SERIES).
struct ReducedCameraSystem {
  static constexpr int kBlockSize = 12;

  struct CameraPairBlock {
    int camera_a;  // row block
    int camera_b;  // column block, camera_a != camera_b
    Matrix12d block;
  };

  int num_cameras = 0;

  // Threads the CPU solvers may use.
  int num_threads = 1;

  // Incremented every time the system is rebuilt, so solvers can cache
  // factorization inputs across repeated solves with different lambda.
  int revision = 0;

  std::vector<Matrix12d, Eigen::aligned_allocator<Matrix12d>> U_diagonal;
  // Both orientations (a, b) and (b, a) are stored, so that the blocks
  // describe the full symmetric matrix. Sorted by camera_a; the blocks of row
  // camera i are [off_diagonal_offsets[i], off_diagonal_offsets[i + 1]).
  std::vector<CameraPairBlock, Eigen::aligned_allocator<CameraPairBlock>>
      U_off_diagonal;
  std::vector<int> off_diagonal_offsets;

  // Point blocks: W blocks of point j are [point_offsets[j],
  // point_offsets[j + 1]); W block k belongs to camera W_camera[k] and point
  // W_point[k].
  std::vector<Eigen::Matrix3d> V_inverse;
  std::vector<int> point_offsets;
  std::vector<int> W_camera;
  std::vector<int> W_point;
  std::vector<Matrix3x12d, Eigen::aligned_allocator<Matrix3x12d>> W_blocks;

  // Camera-major index of the W blocks: the blocks of camera i are
  // camera_W_index[camera_offsets[i] .. camera_offsets[i + 1]).
  std::vector<int> camera_offsets;
  std::vector<int> camera_W_index;

  int Dimension() const { return kBlockSize * num_cameras; }

  // Fills camera_offsets / camera_W_index from W_camera.
  void BuildCameraIndex();

  // Forms the dense, symmetric Schur complement S (without damping), in
  // parallel over column blocks.
  void FormDenseSchurComplement(Eigen::MatrixXd* S) const;

  // out = (S + lambda I) x, matrix-free. For tests and diagnostics.
  void MultiplyDamped(double lambda,
                      const Eigen::VectorXd& x,
                      Eigen::VectorXd* out) const;
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

// True if this build has the CUDA backends and a CUDA device is visible.
bool CudaReducedCameraSolverAvailable();

}  // namespace theia

#endif  // THEIA_SFM_INIT_FREE_REDUCED_CAMERA_SOLVER_H_
