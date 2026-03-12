cmake_minimum_required(VERSION 3.6)
project(CheckCeresCudaSparse CXX)
include_directories(${CERES_INCLUDE_DIRS})
add_executable(check_ceres_cuda_sparse check_ceres_cuda_sparse.cc)
