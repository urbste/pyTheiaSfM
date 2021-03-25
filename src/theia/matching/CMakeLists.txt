
# Add headers
file(GLOB_RECURSE THEIA_MATCHING_HDRS *.h)
list(APPEND THEIA_IO_HDRS "${CMAKE_SOURCE_DIR}/src/theia/alignment/alignment.h")

# Add sources
set(THEIA_MATCHING_SRC
  brute_force_feature_matcher.cc
  cascade_hasher.cc
  cascade_hashing_feature_matcher.cc
  create_feature_matcher.cc
  feature_matcher_utils.cc
  feature_matcher.cc
  fisher_vector_extractor.cc
  guided_epipolar_matcher.cc
  in_memory_features_and_matches_database.cc
  rocksdb_features_and_matches_database.cc
  )

set(THEIA_MATCHING_LIBRARY_DEPENDENCIES
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

set(THEIA_MATCHING_LIBRARY_SOURCE
  ${THEIA_MATCHING_SRC}
  ${THEIA_MATCHING_HDRS})

add_library(theia_matching ${THEIA_MATCHING_LIBRARY_SOURCE})

set_target_properties(theia_matching PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_matching ${THEIA_MATCHING_LIBRARY_DEPENDENCIES} theia_util theia_image theia_sfm theia_io theia_math pybind11_headers)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

pybind11_add_module(pytheia_matching python/pybind.cc)
target_link_libraries(pytheia_matching theia ${THEIA_MATCHING_LIBRARY_DEPENDENCIES} pybind11_headers)
