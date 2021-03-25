
# Add headers
file(GLOB_RECURSE THEIA_IO_HDRS *.h)
list(APPEND THEIA_IO_HDRS "${CMAKE_SOURCE_DIR}/src/theia/alignment/alignment.h")

# Add sources
set(THEIA_IO_SRC
  io_wrapper.cc
  bundler_file_reader.cc
  import_nvm_file.cc
  populate_image_sizes.cc
  read_1dsfm.cc
  read_bundler_files.cc
  read_calibration.cc
  read_keypoints_and_descriptors.cc
  read_strecha_dataset.cc
  reconstruction_reader.cc
  reconstruction_writer.cc
  sift_binary_file.cc
  sift_text_file.cc
  write_bundler_files.cc
  write_calibration.cc
  write_colmap_files.cc
  write_keypoints_and_descriptors.cc
  write_nvm_file.cc
  write_ply_file.cc
  )

set(THEIA_IO_LIBRARY_DEPENDENCIES
  ${CERES_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  #${OPENIMAGEIO_LIBRARIES}
  ${ROCKSDB_LIBRARIES}
  # akaze
  flann_cpp
  statx
  stlplus3
  vlfeat
  visual_sfm
)

set(THEIA_IO_LIBRARY_SOURCE
  ${THEIA_IO_SRC}
  ${THEIA_IO_HDRS})

add_library(theia_io ${THEIA_IO_LIBRARY_SOURCE})

set_target_properties(theia_io PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_io ${THEIA_IO_LIBRARY_DEPENDENCIES} theia_util theia_image theia_sfm)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

pybind11_add_module(pytheia_io python/pybind.cc)
target_link_libraries(pytheia_io theia ${THEIA_IO_LIBRARY_DEPENDENCIES} pybind11_headers)
