
# Add headers
file(GLOB_RECURSE THEIA_SOLVERS_HDRS *.h)


# Add sources
set(THEIA_SOLVERS_SRC
  exhaustive_sampler.cc
  prosac_sampler.cc
  random_sampler.cc
  )

set(THEIA_SOLVERS_LIBRARY_DEPENDENCIES
  ${CERES_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${OPENIMAGEIO_LIBRARIES}
  ${ROCKSDB_LIBRARIES}
  akaze
  flann_cpp
  statx
  stlplus3
  vlfeat
  visual_sfm
)

set(THEIA_SOLVERS_LIBRARY_SOURCE
  ${THEIA_SOLVERS_SRC}
  ${THEIA_SOLVERS_HDRS})

add_library(theia_solvers ${THEIA_SOLVERS_LIBRARY_SOURCE})

set_target_properties(theia_solvers PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_solvers ${THEIA_SOLVERS_LIBRARY_DEPENDENCIES} theia_util)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

pybind11_add_module(pytheia_solvers python/pybind.cc)
target_link_libraries(pytheia_solvers theia ${THEIA_SOLVERS_LIBRARY_DEPENDENCIES} pybind11_headers)
