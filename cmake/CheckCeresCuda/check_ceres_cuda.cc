// Check whether Ceres was built with CUDA (CERES_NO_CUDA not defined).
#include <ceres/internal/config.h>
#ifdef CERES_NO_CUDA
#error "Ceres was built without CUDA"
#endif
int main() { return 0; }
