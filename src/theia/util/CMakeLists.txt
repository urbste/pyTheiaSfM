
# Add headers
file(GLOB_RECURSE THEIA_UTIL_HDRS *.h)


# Add sources
set(THEIA_UTIL_SRC
    filesystem.cc
    random.cc
    stringprintf.cc
    threadpool.cc
    timer.cc
  )

set(THEIA_UTIL_LIBRARY_DEPENDENCIES
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

set(THEIA_UTIL_LIBRARY_SOURCE
  ${THEIA_UTIL_SRC}
  ${THEIA_UTIL_HDRS})

add_library(theia_util ${THEIA_UTIL_LIBRARY_SOURCE})

set_target_properties(theia_util PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_util ${THEIA_UTIL_LIBRARY_DEPENDENCIES})

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

#pybind11_add_module(pytheia_util python/pybind.cc)
#target_link_libraries(pytheia_util theia ${THEIA_UTIL_LIBRARY_DEPENDENCIES} pybind11_headers)
