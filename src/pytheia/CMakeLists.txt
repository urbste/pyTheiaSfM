
set(PACKAGE_NAME pytheia)

set(PY_ALL_SOURCE_FILES
    pytheia_pybind.cc
    matching/matching.cc
    math/math.cc
    mvs/mvs.cc
    solvers/solvers.cc
    sfm/sfm.cc
    io/io.cc)

set(PY_ALL_HEADER_FILES
    pytheia_pybind.h
    matching/matching.h
    math/math.h
    mvs/mvs.h
    solvers/solvers.h
    sfm/sfm.h
    io/io.h)


pybind11_add_module(${PACKAGE_NAME}
    ${PY_ALL_HEADER_FILES} ${PY_ALL_SOURCE_FILES})

target_link_libraries(${PACKAGE_NAME} PRIVATE ${CMAKE_PROJECT_NAME})
target_compile_definitions(${PACKAGE_NAME} PRIVATE
  THEIA_CERES_HAS_CUDA_DENSE=${THEIA_CERES_HAS_CUDA_DENSE_VALUE}
  THEIA_CERES_HAS_CUDA_SPARSE=${THEIA_CERES_HAS_CUDA_SPARSE_VALUE})
if (WIN32)
    target_link_options(${PACKAGE_NAME} PUBLIC "/force:multiple")
endif()