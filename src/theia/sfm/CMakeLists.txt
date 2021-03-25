
# Add headers
file(GLOB_RECURSE THEIA_SFM_HDRS *.h)
list(APPEND THEIA_IO_HDRS "${CMAKE_SOURCE_DIR}/src/theia/alignment/alignment.h")
# Add sources
set(THEIA_SFM_SRC
  sfm_wrapper.cc
  bundle_adjustment/bundle_adjustment_wrapper.cc
  bundle_adjustment/bundle_adjust_two_views.cc
  bundle_adjustment/bundle_adjuster.cc
  bundle_adjustment/bundle_adjustment.cc
  bundle_adjustment/create_loss_function.cc
  bundle_adjustment/optimize_relative_position_with_known_rotation.cc
  camera/camera_wrapper.cc
  camera/camera_intrinsics_model.cc
  camera/camera.cc
  camera/division_undistortion_camera_model.cc
  camera/fisheye_camera_model.cc
  camera/fov_camera_model.cc
  camera/pinhole_camera_model.cc
  camera/pinhole_radial_tangential_camera_model.cc
  camera/projection_matrix_utils.cc
  colorize_reconstruction.cc
  estimate_track.cc
  estimate_twoview_info.cc
  estimators/estimators_wrapper.cc
  estimators/estimate_absolute_pose_with_known_orientation.cc
  estimators/estimate_calibrated_absolute_pose.cc
  estimators/estimate_dominant_plane_from_points.cc
  estimators/estimate_essential_matrix.cc
  estimators/estimate_fundamental_matrix.cc
  estimators/estimate_homography.cc
  estimators/estimate_radial_distortion_homography.cc
  estimators/estimate_relative_pose_with_known_orientation.cc
  estimators/estimate_relative_pose.cc
  estimators/estimate_rigid_transformation_2d_3d.cc
  estimators/estimate_similarity_transformation_2d_3d.cc
  estimators/estimate_triangulation.cc
  estimators/estimate_uncalibrated_absolute_pose.cc
  estimators/estimate_uncalibrated_relative_pose.cc
  #exif_reader.cc
  extract_maximally_parallel_rigid_subgraph.cc
  feature_extractor_and_matcher.cc
  feature_extractor.cc
  filter_view_graph_cycles_by_rotation.cc
  filter_view_pairs_from_orientation.cc
  filter_view_pairs_from_relative_translation.cc
  find_common_tracks_in_views.cc
  find_common_views_by_name.cc
  global_pose_estimation/global_pose_estimation_wrapper.cc
  global_pose_estimation/compute_triplet_baseline_ratios.cc
  global_pose_estimation/least_unsquared_deviation_position_estimator.cc
  global_pose_estimation/linear_position_estimator.cc
  global_pose_estimation/linear_rotation_estimator.cc
  global_pose_estimation/nonlinear_position_estimator.cc
  global_pose_estimation/nonlinear_rotation_estimator.cc
  global_pose_estimation/pairwise_rotation_error.cc
  global_pose_estimation/pairwise_translation_and_scale_error.cc
  global_pose_estimation/pairwise_translation_error.cc
  global_pose_estimation/robust_rotation_estimator.cc
  global_reconstruction_estimator.cc
  gps_converter.cc
  hybrid_reconstruction_estimator.cc
  incremental_reconstruction_estimator.cc
  localize_view_to_reconstruction.cc
  pose/pose_wrapper.cc
  pose/build_upnp_action_matrix.cc
  pose/build_upnp_action_matrix_using_symmetry.cc
  pose/dls_impl.cc
  pose/dls_pnp.cc
  pose/eight_point_fundamental_matrix.cc
  pose/essential_matrix_utils.cc
  pose/five_point_focal_length_radial_distortion.cc
  pose/five_point_relative_pose.cc
  pose/four_point_focal_length_helper.cc
  pose/four_point_focal_length.cc
  pose/four_point_focal_length_radial_distortion.cc
  pose/four_point_focal_length_radial_distortion_helper.cc
  pose/four_point_homography.cc
  pose/four_point_relative_pose_partial_rotation.cc
  pose/fundamental_matrix_util.cc
  pose/perspective_three_point.cc
  pose/position_from_two_rays.cc
  pose/relative_pose_from_two_points_with_known_rotation.cc
  pose/seven_point_fundamental_matrix.cc
  pose/sim_transform_partial_rotation.cc
  pose/six_point_radial_distortion_homography.cc
  pose/test_util.cc
  pose/three_point_relative_pose_partial_rotation.cc
  pose/two_point_pose_partial_rotation.cc
  pose/upnp.cc
  pose/util.cc
  reconstruction_builder.cc
  reconstruction_estimator_utils.cc
  reconstruction_estimator.cc
  reconstruction.cc
  select_good_tracks_for_bundle_adjustment.cc
  set_camera_intrinsics_from_priors.cc
  set_outlier_tracks_to_unestimated.cc
  track_builder.cc
  track.cc
  transformation/transformation_wrapper.cc
  transformation/align_point_clouds.cc
  transformation/align_reconstructions.cc
  transformation/align_rotations.cc
  transformation/gdls_similarity_transform.cc
  transformation/transform_reconstruction.cc
  triangulation/triangulation_wrapper.cc
  triangulation/triangulation.cc
  two_view_match_geometric_verification.cc
  twoview_info.cc
  #undistort_image.cc
  view_graph/orientations_from_maximum_spanning_tree.cc
  view_graph/remove_disconnected_view_pairs.cc
  view_graph/view_graph.cc
  view.cc
  visibility_pyramid.cc
  )

set(THEIA_SFM_LIBRARY_DEPENDENCIES
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

set(THEIA_SFM_LIBRARY_SOURCE
  ${THEIA_SFM_SRC}
  ${THEIA_SFM_HDRS})

add_library(theia_sfm ${THEIA_SFM_LIBRARY_SOURCE})

set_target_properties(theia_sfm PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )

target_link_libraries(theia_sfm ${THEIA_SFM_LIBRARY_DEPENDENCIES} theia_util theia_math theia_image theia_solvers theia_io theia_matching)

#install(TARGETS theia
#  EXPORT  TheiaExport
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib${LIB_SUFFIX}
#  ARCHIVE DESTINATION lib${LIB_SUFFIX})



#Add python binding

pybind11_add_module(pytheia_sfm python/pybind.cc)
target_link_libraries(pytheia_sfm theia ${THEIA_SFM_LIBRARY_DEPENDENCIES} pybind11_headers)


