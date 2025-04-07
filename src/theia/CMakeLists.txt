# Copyright (C) 2013 The Regents of the University of California (Regents).
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
#     * Neither the name of The Regents or University of California nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Please contact the author of this library if you have any questions.
# Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

function(create_resource_file file output)
    # Create empty output file
    file(WRITE ${output} "")
    # Get short filename
    get_filename_component(filename "${file}" NAME)
    # Replace filename spaces & extension separator for C compatibility
    string(MAKE_C_IDENTIFIER ${filename} filename)
    # Read hex data from file
    file(READ ${file} filedata HEX)
    # Convert hex data for C compatibility
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})
    # Append data to output file
    file(WRITE ${output} "const char ${filename}[] = {${filedata}};\nconst size_t ${filename}_size = sizeof(${filename});\n")
endfunction()

create_resource_file("${PROJECT_SOURCE_DIR}/data/camera_sensor_database.txt" "${CMAKE_CURRENT_BINARY_DIR}/camera_sensor_database.h")

# Add headers
file(GLOB_RECURSE THEIA_HDRS *.h)

# Add sources
set(THEIA_SRC
  io/bundler_file_reader.cc
  io/import_nvm_file.cc
  io/populate_image_sizes.cc
  io/read_1dsfm.cc
  io/read_bundler_files.cc
  io/read_calibration.cc
  io/read_keypoints_and_descriptors.cc
  io/read_strecha_dataset.cc
  io/reconstruction_reader.cc
  io/reconstruction_writer.cc
  io/sift_binary_file.cc
  io/sift_text_file.cc
  io/write_bundler_files.cc
  io/write_calibration.cc
  io/write_colmap_files.cc
  io/write_keypoints_and_descriptors.cc
  io/write_matches.cc
  io/write_nerfstudio.cc
  io/write_sdfstudio.cc
  io/write_nvm_file.cc
  io/write_ply_file.cc
  matching/brute_force_feature_matcher.cc
  matching/cascade_hasher.cc
  matching/cascade_hashing_feature_matcher.cc
  matching/create_feature_matcher.cc
  matching/feature_matcher_utils.cc
  matching/feature_matcher.cc
  matching/fisher_vector_extractor.cc
  matching/graph_match.cc
  matching/guided_epipolar_matcher.cc
  matching/in_memory_features_and_matches_database.cc
  matching/rocksdb_features_and_matches_database.cc
  math/closed_form_polynomial_solver.cc
  math/constrained_l1_solver.cc
  math/find_polynomial_roots_companion_matrix.cc
  math/find_polynomial_roots_jenkins_traub.cc
  math/matrix/sparse_cholesky_llt.cc
  math/matrix/sparse_matrix.cc
  math/polynomial.cc
  math/probability/sequential_probability_ratio.cc
  math/qp_solver.cc
  math/rotation.cc
  math/rank_restricted_sdp_solver.cc
  math/rbr_sdp_solver.cc
  math/riemannian_staircase.cc
  math/matrix/matrix_square_root.cc
  mvs/view_selection_mvsnet.cc
  sfm/bundle_adjustment/bundle_adjust_two_views.cc
  sfm/bundle_adjustment/bundle_adjuster.cc
  sfm/bundle_adjustment/bundle_adjustment.cc
  sfm/bundle_adjustment/create_loss_function.cc
  sfm/bundle_adjustment/loss_functions.cc
  sfm/bundle_adjustment/optimize_relative_position_with_known_rotation.cc
  sfm/camera/camera_intrinsics_model.cc
  sfm/camera/camera.cc
  sfm/camera/division_undistortion_camera_model.cc
  sfm/camera/extended_unified_camera_model.cc
  sfm/camera/double_sphere_camera_model.cc
  sfm/camera/fisheye_camera_model.cc
  sfm/camera/fov_camera_model.cc
  sfm/camera/orthographic_camera_model.cc
  sfm/camera/pinhole_camera_model.cc
  sfm/camera/pinhole_radial_tangential_camera_model.cc
  sfm/camera/projection_matrix_utils.cc
  sfm/colorize_reconstruction.cc
  sfm/estimate_track.cc
  sfm/estimate_twoview_info.cc
  sfm/estimators/estimate_absolute_pose_with_known_orientation.cc
  sfm/estimators/estimate_calibrated_absolute_pose.cc
  sfm/estimators/estimate_dominant_plane_from_points.cc
  sfm/estimators/estimate_essential_matrix.cc
  sfm/estimators/estimate_fundamental_matrix.cc
  sfm/estimators/estimate_homography.cc
  sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.cc
  sfm/estimators/estimate_radial_distortion_homography.cc
  sfm/estimators/estimate_relative_pose_with_known_orientation.cc
  sfm/estimators/estimate_relative_pose.cc
  sfm/estimators/estimate_rigid_transformation_2d_3d.cc
  sfm/estimators/estimate_similarity_transformation_2d_3d.cc
  sfm/estimators/estimate_triangulation.cc
  sfm/estimators/estimate_uncalibrated_absolute_pose.cc
  sfm/estimators/estimate_uncalibrated_relative_pose.cc
  #sfm/exif_reader.cc
  sfm/extract_maximally_parallel_rigid_subgraph.cc
  sfm/feature_extractor_and_matcher.cc
  sfm/feature_extractor.cc
  sfm/filter_view_graph_cycles_by_rotation.cc
  sfm/filter_view_pairs_from_orientation.cc
  sfm/filter_view_pairs_from_relative_translation.cc
  sfm/find_common_tracks_in_views.cc
  sfm/find_common_views_by_name.cc
  sfm/global_pose_estimation/compute_triplet_baseline_ratios.cc
  sfm/global_pose_estimation/bata_position_estimator.cc
  sfm/global_pose_estimation/hybrid_rotation_estimator.cc
  sfm/global_pose_estimation/irls_rotation_local_refiner.cc
  sfm/global_pose_estimation/l1_rotation_global_estimator.cc
  sfm/global_pose_estimation/lagrange_dual_rotation_estimator.cc
  sfm/global_pose_estimation/least_unsquared_deviation_position_estimator.cc
  sfm/global_pose_estimation/LiGT_position_estimator.cc
  sfm/global_pose_estimation/linear_position_estimator.cc
  sfm/global_pose_estimation/linear_rotation_estimator.cc
  sfm/global_pose_estimation/nonlinear_position_estimator.cc
  sfm/global_pose_estimation/nonlinear_rotation_estimator.cc
  sfm/global_pose_estimation/pairwise_rotation_error.cc
  sfm/global_pose_estimation/pairwise_translation_and_scale_error.cc
  sfm/global_pose_estimation/pairwise_translation_error.cc
  sfm/global_pose_estimation/robust_rotation_estimator.cc
  sfm/global_reconstruction_estimator.cc
  sfm/gps_converter.cc
  sfm/hybrid_reconstruction_estimator.cc
  sfm/incremental_reconstruction_estimator.cc
  sfm/localize_view_to_reconstruction.cc
  sfm/pose/build_upnp_action_matrix.cc
  sfm/pose/build_upnp_action_matrix_using_symmetry.cc
  sfm/pose/dls_impl.cc
  sfm/pose/dls_pnp.cc
  sfm/pose/eight_point_fundamental_matrix.cc
  sfm/pose/essential_matrix_utils.cc
  sfm/pose/five_point_focal_length_radial_distortion.cc
  sfm/pose/five_point_relative_pose.cc
  sfm/pose/four_point_focal_length_helper.cc
  sfm/pose/four_point_focal_length.cc
  sfm/pose/four_point_focal_length_radial_distortion.cc
  sfm/pose/four_point_focal_length_radial_distortion_helper.cc
  sfm/pose/four_point_homography.cc
  sfm/pose/four_point_relative_pose_partial_rotation.cc
  sfm/pose/fundamental_matrix_util.cc
  sfm/pose/mlpnp.cc
  sfm/pose/mlpnp_helper.cc
  sfm/pose/orthographic_four_point.cc
  sfm/pose/perspective_three_point.cc
  sfm/pose/position_from_two_rays.cc
  sfm/pose/relative_pose_from_two_points_with_known_rotation.cc
  sfm/pose/seven_point_fundamental_matrix.cc
  sfm/pose/sim_transform_partial_rotation.cc
  sfm/pose/six_point_radial_distortion_homography.cc
  sfm/pose/sqpnp.cc
  sfm/pose/sqpnp_helper.cc
  sfm/pose/test_util.cc
  sfm/pose/three_point_relative_pose_partial_rotation.cc
  sfm/pose/two_point_pose_partial_rotation.cc
  sfm/pose/upnp.cc
  sfm/pose/util.cc
  sfm/reconstruction_builder.cc
  sfm/reconstruction_estimator_utils.cc
  sfm/reconstruction_estimator.cc
  sfm/reconstruction.cc
  sfm/select_good_tracks_for_bundle_adjustment.cc
  sfm/set_camera_intrinsics_from_priors.cc
  sfm/set_outlier_tracks_to_unestimated.cc
  sfm/track_builder.cc
  sfm/track.cc
  sfm/transformation/align_point_clouds.cc
  sfm/transformation/align_reconstructions.cc
  sfm/transformation/align_reconstructions_pose_graph_optim.cc
  sfm/transformation/align_rotations.cc
  sfm/transformation/gdls_similarity_transform.cc
  sfm/transformation/transform_reconstruction.cc
  sfm/triangulation/triangulation.cc
  sfm/two_view_match_geometric_verification.cc
  sfm/twoview_info.cc
  # sfm/undistort_image.cc
  sfm/view_graph/orientations_from_maximum_spanning_tree.cc
  sfm/view_graph/remove_disconnected_view_pairs.cc
  sfm/view_graph/view_graph.cc
  sfm/view_graph/view_graph_from_reconstruction.cc
  sfm/view.cc
  sfm/visibility_pyramid.cc
  solvers/exhaustive_sampler.cc
  solvers/prosac_sampler.cc
  solvers/random_sampler.cc
  util/filesystem.cc
  util/random.cc
  util/stringprintf.cc
  util/threadpool.cc
  util/timer.cc
  )

# for pytheia
set(PYTHEIA_SRC
    io/io_wrapper.cc
    math/math_wrapper.cc
    sfm/sfm_wrapper.cc
    sfm/bundle_adjustment/bundle_adjustment_wrapper.cc
    sfm/global_pose_estimation/global_pose_estimation_wrapper.cc
    sfm/camera/camera_wrapper.cc
    sfm/estimators/estimators_wrapper.cc
    sfm/pose/pose_wrapper.cc
    sfm/transformation/transformation_wrapper.cc
    sfm/triangulation/triangulation_wrapper.cc
    )


set(THEIA_LIBRARY_DEPENDENCIES
  ${CERES_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  #${SUITESPARSE_LIBRARIES}
  flann_cpp_s
  statx
  stlplus3
  vlfeat
)


if (PYTHON_BUILD)
    set(THEIA_LIBRARY_SOURCE
      ${PYTHEIA_SRC}
      ${THEIA_SRC}
      ${THEIA_HDRS})
else (PYTHON_BUILD)
    set(THEIA_LIBRARY_SOURCE
      ${THEIA_SRC}
      ${THEIA_HDRS})
endif (PYTHON_BUILD)

# Target definition and linking
add_library(${CMAKE_PROJECT_NAME} ${THEIA_LIBRARY_SOURCE})

# Add include directories to the target
target_include_directories(${CMAKE_PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # For headers within the src dir
    $<INSTALL_INTERFACE:include/theia> # Public install interface
  PRIVATE
    # Internal includes needed for compiling the library itself
    ${CMAKE_SOURCE_DIR}/libraries/stlplus3
    ${CMAKE_SOURCE_DIR}/libraries/vlfeat
    ${CMAKE_SOURCE_DIR}/libraries/statx
    ${CMAKE_SOURCE_DIR}/libraries/optimo
    ${CMAKE_SOURCE_DIR}/libraries/spectra
    ${CMAKE_SOURCE_DIR}/libraries/Sophus
    ${CMAKE_SOURCE_DIR}/libraries/cereal/include
    ${CMAKE_SOURCE_DIR}/libraries/akaze
    ${CMAKE_SOURCE_DIR}/libraries/flann/src/cpp
    ${CMAKE_SOURCE_DIR}/libraries/gtest/include # If gtest headers used internally
    ${CMAKE_SOURCE_DIR}/libraries/pybind11/include # If pybind11 headers used internally
    # Add include dirs from find_package results
    ${GFLAGS_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIR}
    ${ROCKSDB_INCLUDE_DIRS} # Add if RocksDB is found and needed directly
  SYSTEM INTERFACE # Propagate system include property for Eigen
    ${EIGEN_INCLUDE_DIRS}
)

# Link libraries using variables instead of imported targets
target_link_libraries(${CMAKE_PROJECT_NAME}
  PUBLIC
    # Libraries needed by users of theia
    Eigen3::Eigen # Eigen is likely part of the public API
    ${GFLAGS_LIBRARIES} # Use variable instead of gflags::gflags
    ${GLOG_LIBRARIES}   # Use variable instead of glog::glog
  PRIVATE
    # Libraries only needed to build theia itself
    # Link bundled libraries (assuming they are built as targets)
    flann_cpp_s # Assuming this is the static flann target name from libraries/flann/src/cpp/CMakeLists.txt
    statx       # Assuming this is the target name from libraries/statx/CMakeLists.txt
    stlplus3    # Assuming this is the target name from libraries/stlplus3/CMakeLists.txt
    vlfeat      # Assuming this is the target name from libraries/vlfeat/CMakeLists.txt
    # Link other private dependencies
    ${CMAKE_THREAD_LIBS_INIT} # Use variable instead of Threads::Threads
    ${ROCKSDB_LIBRARIES} # Link RocksDB if WITH_ROCKSDB is ON
)

# Set target properties (versioning)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
)

# Install target
install(TARGETS ${CMAKE_PROJECT_NAME}
  EXPORT  TheiaExport
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX})

if (BUILD_TESTING)
  macro (GTEST FILENAME)
    string(REPLACE "/" ";" TEST_NAME_LIST ${FILENAME})
    list(REVERSE TEST_NAME_LIST)
    list(GET TEST_NAME_LIST 0 TEST_NAME)
    add_executable(${TEST_NAME}_test test/test_main.cc ${FILENAME}_test.cc)
    target_link_libraries(${TEST_NAME}_test
      PRIVATE
      ${CMAKE_PROJECT_NAME}
      GTest::gtest_main
      )
    add_test(NAME ${TEST_NAME}_test
      COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_NAME}_test)
  endmacro (GTEST)
#  gtest(io/read_calibration)
#  gtest(io/write_calibration)
#  gtest(matching/brute_force_feature_matcher)
#  gtest(matching/cascade_hashing_feature_matcher)
#  gtest(matching/distance)
#  gtest(matching/feature_correspondence)
#  gtest(matching/feature_matcher_utils)
#  gtest(matching/guided_epipolar_matcher)
#  gtest(math/closed_form_polynomial_solver)
#  gtest(math/find_polynomial_roots_companion_matrix)
#  gtest(math/find_polynomial_roots_jenkins_traub)
#  gtest(math/graph/connected_components)
#  gtest(math/graph/minimum_spanning_tree)
#  gtest(math/graph/normalized_graph_cut)
#  gtest(math/graph/triplet_extractor)
#  gtest(math/l1_solver)
#  gtest(math/matrix/gauss_jordan)
#  gtest(math/matrix/rq_decomposition)
#  gtest(math/polynomial)
#  gtest(math/probability/sprt)
#  gtest(math/qp_solver)
#  gtest(math/reservoir_sampler)
#  gtest(math/rotation)
#  gtest(sfm/bundle_adjustment/bundle_adjustment)
#  gtest(sfm/bundle_adjustment/optimize_relative_position_with_known_rotation)
#  gtest(sfm/camera/camera)
#  gtest(sfm/camera/division_undistortion_camera_model)
 # gtest(sfm/camera/double_sphere_camera_model)
#  gtest(sfm/camera/extended_unified_camera_model)
#  gtest(sfm/camera/fisheye_camera_model)
#  gtest(sfm/camera/fov_camera_model)
#  gtest(sfm/camera/orthographic_camera_model)
#  gtest(sfm/camera/pinhole_camera_model)
#  gtest(sfm/camera/pinhole_radial_tangential_camera_model)
#  gtest(sfm/camera/projection_matrix_utils)
#  gtest(sfm/estimators/estimate_absolute_pose_with_known_orientation)
#  gtest(sfm/estimators/estimate_calibrated_absolute_pose)
#  gtest(sfm/estimators/estimate_dominant_plane_from_points)
#  gtest(sfm/estimators/estimate_essential_matrix)
#  gtest(sfm/estimators/estimate_fundamental_matrix)
#  gtest(sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose)
#  gtest(sfm/estimators/estimate_radial_distortion_homography)
#  gtest(sfm/estimators/estimate_homography)
#  gtest(sfm/estimators/estimate_relative_pose)
#  gtest(sfm/estimators/estimate_relative_pose_with_known_orientation)
#  gtest(sfm/estimators/estimate_rigid_transformation_2d_3d)
#  gtest(sfm/estimators/estimate_similarity_transformation_2d_3d)
#  gtest(sfm/estimators/estimate_triangulation)
#  gtest(sfm/estimators/estimate_uncalibrated_absolute_pose)
#  gtest(sfm/estimators/estimate_uncalibrated_relative_pose)
#  gtest(sfm/exif_reader)
#  gtest(sfm/extract_maximally_parallel_rigid_subgraph)
#  gtest(sfm/filter_view_graph_cycles_by_rotation)
#  gtest(sfm/filter_view_pairs_from_orientation)
#  gtest(sfm/filter_view_pairs_from_relative_translation)
#  gtest(sfm/find_common_tracks_in_views)
#  gtest(sfm/find_common_views_by_name)
#  gtest(sfm/global_pose_estimation/compute_triplet_baseline_ratios)
#  gtest(sfm/global_pose_estimation/hybrid_rotation_estimator)
#  gtest(sfm/global_pose_estimation/lagrange_dual_rotation_estimator)
#  gtest(sfm/global_pose_estimation/least_unsquared_deviation_position_estimator)
#  gtest(sfm/global_pose_estimation/LiGT_position_estimator)
#  gtest(sfm/global_pose_estimation/linear_position_estimator)
#  gtest(sfm/global_pose_estimation/linear_rotation_estimator)
#  gtest(sfm/global_pose_estimation/nonlinear_position_estimator)
#  gtest(sfm/global_pose_estimation/pairwise_rotation_error)
#  gtest(sfm/global_pose_estimation/pairwise_translation_and_scale_error)
#  gtest(sfm/global_pose_estimation/pairwise_translation_error)
#  gtest(sfm/global_pose_estimation/robust_rotation_estimator)
# gtest(sfm/global_pose_estimation/bata_position_estimator)
#  gtest(sfm/gps_converter)
#  gtest(sfm/hybrid_reconstruction_estimator)
#  gtest(sfm/incremental_reconstruction_estimator)
#  gtest(sfm/pose/build_upnp_action_matrix)
#  gtest(sfm/pose/build_upnp_action_matrix_using_symmetry)
#  gtest(sfm/pose/dls_pnp)
#  gtest(sfm/pose/eight_point_fundamental_matrix)
#  gtest(sfm/pose/essential_matrix_utils)
#  gtest(sfm/pose/five_point_focal_length_radial_distortion)
#  gtest(sfm/pose/five_point_relative_pose)
#  gtest(sfm/pose/four_point_focal_length)
#  gtest(sfm/pose/four_point_focal_length_radial_distortion)
#  gtest(sfm/pose/four_point_homography)
#  gtest(sfm/pose/four_point_relative_pose_partial_rotation)
#  gtest(sfm/pose/fundamental_matrix_util)
#  gtest(sfm/pose/mlpnp)
#  gtest(sfm/pose/orthographic_four_point)
#  gtest(sfm/pose/perspective_three_point)
#  gtest(sfm/pose/position_from_two_rays)
#  gtest(sfm/pose/relative_pose_from_two_points_with_known_rotation)
#  gtest(sfm/pose/seven_point_fundamental_matrix)
#  gtest(sfm/pose/sim_transform_partial_rotation)
#  gtest(sfm/pose/six_point_radial_distortion_homography)
#  gtest(sfm/pose/sqpnp)
#  gtest(sfm/pose/three_point_relative_pose_partial_rotation)
 # gtest(sfm/pose/two_point_pose_partial_rotation)
#  gtest(sfm/pose/upnp)
#  gtest(sfm/reconstruction)
#  gtest(sfm/track)
#  gtest(sfm/track_builder)
#  gtest(sfm/transformation/align_point_clouds)
#  gtest(sfm/transformation/align_reconstructions)
#  gtest(sfm/transformation/align_reconstructions_pose_graph_optim)
#  gtest(sfm/transformation/align_rotations)
#  gtest(sfm/transformation/gdls_similarity_transform)
#  gtest(sfm/triangulation/triangulation)
#  gtest(sfm/twoview_info)
#  gtest(sfm/view)
#  gtest(sfm/view_graph/orientations_from_maximum_spanning_tree)
#  gtest(sfm/view_graph/remove_disconnected_view_pairs)
 # gtest(sfm/view_graph/view_graph_from_reconstruction)
#  gtest(sfm/view_graph/view_graph)
#  gtest(solvers/exhaustive_ransac)
#  gtest(solvers/exhaustive_sampler)
#  gtest(solvers/evsac)
#  gtest(solvers/lmed)
#  gtest(solvers/prosac)
#  gtest(solvers/random_sampler)
#  gtest(solvers/ransac)
#  gtest(util/mutable_priority_queue)
#  gtest(util/lru_cache)
endif (BUILD_TESTING)
