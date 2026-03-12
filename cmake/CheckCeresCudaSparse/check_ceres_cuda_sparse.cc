// Check whether Ceres was built with CUDA sparse (cuDSS) support.
// Compiles only if SparseLinearAlgebraLibraryType::CUDA_SPARSE exists.
#include <ceres/types.h>
int main() {
  ceres::SparseLinearAlgebraLibraryType t =
      ceres::SparseLinearAlgebraLibraryType::CUDA_SPARSE;
  (void)t;
  return 0;
}
