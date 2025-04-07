## Basic programs

# add_executable(test_inv_track_ba test_inv_track_ba.cc)
# target_link_libraries(test_inv_track_ba PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

# add_executable(test_align_recon test_align_recon.cc)
# target_link_libraries(test_align_recon PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(extract_features extract_features.cc)
# target_link_libraries(extract_features PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags) # Assuming glog not needed

#add_executable(compute_two_view_geometry compute_two_view_geometry.cc)
# target_link_libraries(compute_two_view_geometry PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

## Tools for building SfM models
# add_executable(build_reconstruction build_reconstruction.cc)
# target_link_libraries(build_reconstruction PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

# add_executable(build_1dsfm_reconstruction build_1dsfm_reconstruction.cc)
# target_link_libraries(build_1dsfm_reconstruction PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(calibrate_camera_intrinsics calibrate_camera_intrinsics.cc)
# target_link_libraries(calibrate_camera_intrinsics PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(undistort_images undistort_images.cc)
# target_link_libraries(undistort_images PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

## Reconstruction visualization.
# Remove old find_package/include/link logic for OpenGL/GLEW/GLUT
# If view_reconstruction is uncommented, use:
# find_package(OpenGL REQUIRED)
# find_package(GLEW REQUIRED) # GLEW exports GLEW::glew
# find_package(GLUT REQUIRED) # GLUT exports GLUT::glut
# ...
#add_executable(view_reconstruction view_reconstruction.cc)
# target_link_libraries(view_reconstruction PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags OpenGL::GL GLEW::glew GLUT::glut)

## Useful tools for analyzing reconstructions.
#add_executable(compute_reconstruction_statistics compute_reconstruction_statistics.cc)
# target_link_libraries(compute_reconstruction_statistics PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(colorize_reconstruction colorize_reconstruction.cc)
# target_link_libraries(colorize_reconstruction PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

# add_executable(compare_reconstructions compare_reconstructions.cc)
# target_link_libraries(compare_reconstructions PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(compute_matching_relative_pose_errors compute_matching_relative_pose_errors.cc)
# target_link_libraries(compute_matching_relative_pose_errors PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(evaluate_relative_translation_optimization evaluate_relative_translation_optimization.cc)
# target_link_libraries(evaluate_relative_translation_optimization PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

# add_executable(verify_1dsfm_input verify_1dsfm_input.cc)
# target_link_libraries(verify_1dsfm_input PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

## File conversions and exporters.
#add_executable(convert_sift_key_file convert_sift_key_file.cc)
# target_link_libraries(convert_sift_key_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(convert_bundle_file convert_bundle_file.cc)
# target_link_libraries(convert_bundle_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(export_colmap_files export_colmap_files.cc)
# target_link_libraries(export_colmap_files PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(export_to_nvm_file export_to_nvm_file.cc)
# target_link_libraries(export_to_nvm_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(convert_nvm_file convert_nvm_file.cc)
# target_link_libraries(convert_nvm_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(convert_theia_reconstruction_to_bundler_file convert_theia_reconstruction_to_bundler_file.cc)
# target_link_libraries(convert_theia_reconstruction_to_bundler_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(write_reconstruction_ply_file write_reconstruction_ply_file.cc)
# target_link_libraries(write_reconstruction_ply_file PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(export_reconstruction_to_pmvs export_reconstruction_to_pmvs.cc)
# target_link_libraries(export_reconstruction_to_pmvs PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(create_calibration_file_from_exif create_calibration_file_from_exif.cc)
# target_link_libraries(create_calibration_file_from_exif PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(create_reconstruction_from_strecha_dataset create_reconstruction_from_strecha_dataset.cc)
# target_link_libraries(create_reconstruction_from_strecha_dataset PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)

#add_executable(estimate_covariance_for_tracks estimate_covariance_for_tracks.cc)
# target_link_libraries(estimate_covariance_for_tracks PRIVATE ${CMAKE_PROJECT_NAME} gflags::gflags glog::glog)
