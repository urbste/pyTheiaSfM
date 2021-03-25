
# Add headers
file(GLOB_RECURSE THEIA_IMAGE_HDRS *.h)


# Add sources
set(THEIA_IMAGE_SRC
  #descriptor/akaze_descriptor.cc
  #descriptor/create_descriptor_extractor.cc
  #descriptor/descriptor_extractor.cc
  #descriptor/sift_descriptor.cc
  #image_cache.cc
  #image.cc
  #keypoint_detector/sift_detector.cc
  )

set(THEIA_IMAGE_LIBRARY_DEPENDENCIES
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

set(THEIA_IMAGE_LIBRARY_SOURCE
  ${THEIA_IMAGE_SRC}
  ${THEIA_IMAGE_HDRS})

add_library(theia_image ${THEIA_IMAGE_LIBRARY_SOURCE})

set_target_properties(theia_image PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_image ${THEIA_IMAGE_LIBRARY_DEPENDENCIES} theia_util)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

pybind11_add_module(pytheia_image python/pybind.cc)
target_link_libraries(pytheia_image theia ${THEIA_IMAGE_LIBRARY_DEPENDENCIES} pybind11_headers)
