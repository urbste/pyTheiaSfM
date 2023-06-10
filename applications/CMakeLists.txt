## Basic programs

add_executable(test_inv_track_ba test_inv_track_ba.cc)
target_link_libraries(test_inv_track_ba ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})


add_executable(test_align_recon test_align_recon.cc)
target_link_libraries(test_align_recon ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})



#add_executable(extract_features extract_features.cc)
#target_link_libraries(extract_features ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES})

#add_executable(compute_two_view_geometry compute_two_view_geometry.cc)
#target_link_libraries(compute_two_view_geometry ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

## Tools for building SfM models
add_executable(build_reconstruction build_reconstruction.cc)
target_link_libraries(build_reconstruction ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(build_1dsfm_reconstruction build_1dsfm_reconstruction.cc)
target_link_libraries(build_1dsfm_reconstruction ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(calibrate_camera_intrinsics calibrate_camera_intrinsics.cc)
#target_link_libraries(calibrate_camera_intrinsics ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(undistort_images undistort_images.cc)
#target_link_libraries(undistort_images ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

## Reconstruction visualization.
#find_package(OpenGL REQUIRED)
#if (NOT MSVC)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
#else (NOT MSVC)
#  find_package(GLEW REQUIRED)
#  if (GLEW_FOUND)
#    message ("-- Found GLEW: ${GLEW_INCLUDE_DIRS}")
#  else (GLEW FOUND)
#    message (FATAL_ERROR "-- Can't find GLEW. Please set GLEW_INCLUDE_DIR & GLEW_LIBRARY")
#  endif (GLEW_FOUND)
#  include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLEW_INCLUDE_DIRS} )
#  list(APPEND EXTRA_GL_LIBRARIES ${GLEW_LIBRARY} ${GLEW_LIBRARIES})
#endif (NOT MSVC)

#find_package(GLUT REQUIRED)
#if (GLUT_FOUND)
#  message ("-- Found GLUT: ${GLUT_INCLUDE_DIR}")
#else (GLUT FOUND)
#  message (FATAL_ERROR "-- Can't find GLUT. Please set GLUT_INCLUDE_DIR & GLUT_LIBRARY")
#endif (GLUT_FOUND)
#include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIR} )
#list(APPEND EXTRA_GL_LIBRARIES ${GLUT_LIBRARIES})

#add_executable(view_reconstruction view_reconstruction.cc)
#target_link_libraries(view_reconstruction ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES}
#  ${OPENGL_LIBRARIES}
#  ${EXTRA_GL_LIBRARIES})

## Useful tools for analyzing reconstructions.
#add_executable(compute_reconstruction_statistics compute_reconstruction_statistics.cc)
#target_link_libraries(compute_reconstruction_statistics ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(colorize_reconstruction colorize_reconstruction.cc)
#target_link_libraries(colorize_reconstruction ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(compare_reconstructions compare_reconstructions.cc)
target_link_libraries(compare_reconstructions ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(compute_matching_relative_pose_errors compute_matching_relative_pose_errors.cc)
#target_link_libraries(compute_matching_relative_pose_errors ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(evaluate_relative_translation_optimization evaluate_relative_translation_optimization.cc)
#target_link_libraries(evaluate_relative_translation_optimization ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(verify_1dsfm_input verify_1dsfm_input.cc)
target_link_libraries(verify_1dsfm_input ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

## File conversions and exporters.
#add_executable(convert_sift_key_file convert_sift_key_file.cc)
#target_link_libraries(convert_sift_key_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(convert_bundle_file convert_bundle_file.cc)
#target_link_libraries(convert_bundle_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(export_colmap_files export_colmap_files.cc)
#target_link_libraries(export_colmap_files ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(export_to_nvm_file export_to_nvm_file.cc)
#target_link_libraries(export_to_nvm_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(convert_nvm_file convert_nvm_file.cc)
#target_link_libraries(convert_nvm_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(convert_theia_reconstruction_to_bundler_file convert_theia_reconstruction_to_bundler_file.cc)
#target_link_libraries(convert_theia_reconstruction_to_bundler_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(write_reconstruction_ply_file write_reconstruction_ply_file.cc)
#target_link_libraries(write_reconstruction_ply_file ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(export_reconstruction_to_pmvs export_reconstruction_to_pmvs.cc)
#target_link_libraries(export_reconstruction_to_pmvs ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(create_calibration_file_from_exif create_calibration_file_from_exif.cc)
#target_link_libraries(create_calibration_file_from_exif ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(create_reconstruction_from_strecha_dataset create_reconstruction_from_strecha_dataset.cc)
#target_link_libraries(create_reconstruction_from_strecha_dataset ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(estimate_covariance_for_tracks estimate_covariance_for_tracks.cc)
#target_link_libraries(estimate_covariance_for_tracks ${CMAKE_PROJECT_NAME} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
