cmake_minimum_required(VERSION 3.6)
project(CheckCeresCuda CXX)
include_directories(${CERES_INCLUDE_DIRS})
add_executable(check_ceres_cuda check_ceres_cuda.cc)
