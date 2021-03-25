
# Add headers
file(GLOB_RECURSE THEIA_MATH_HDRS *.h)


# Add sources
set(THEIA_MATH_SRC
  closed_form_polynomial_solver.cc
  constrained_l1_solver.cc
  find_polynomial_roots_companion_matrix.cc
  find_polynomial_roots_jenkins_traub.cc
  matrix/sparse_cholesky_llt.cc
  matrix/sparse_matrix.cc
  polynomial.cc
  probability/sequential_probability_ratio.cc
  qp_solver.cc
  rotation.cc
  )

set(THEIA_MATH_LIBRARY_DEPENDENCIES
  ${CERES_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  #${OPENIMAGEIO_LIBRARIES}
  ${ROCKSDB_LIBRARIES}
  akaze
  flann_cpp
  statx
  stlplus3
  vlfeat
  visual_sfm
)

set(THEIA_MATH_LIBRARY_SOURCE
  ${THEIA_MATH_SRC}
  ${THEIA_MATH_HDRS})

add_library(theia_math ${THEIA_MATH_LIBRARY_SOURCE})

set_target_properties(theia_math PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_math ${THEIA_MATH_LIBRARY_DEPENDENCIES} theia_util theia_sfm)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

#pybind11_add_module(pytheia_math python/pybind.cc)
#target_link_libraries(pytheia_math theia ${THEIA_MATH_LIBRARY_DEPENDENCIES} pybind11_headers)
