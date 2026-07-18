# CheckCeresCudaSparse (reference only)

This try_compile probe is **not used** by the main CMake configuration.

**Active cuDSS detection** is in the root [`CMakeLists.txt`](../../CMakeLists.txt): after `find_package(Ceres)`, CMake checks `CERES_COMPILED_COMPONENTS` for the `cuDSS` component. That is the reliable signal that Ceres was built with NVIDIA cuDSS.

The `CUDA_SPARSE` enum exists in Ceres headers even when cuDSS is not linked, so a compile-only check here can report false positives. Override at configure time with `-DTHEIA_CERES_USE_CUDA_SPARSE=ON|OFF` if needed.

This directory is kept as a reference probe and for manual experimentation.
