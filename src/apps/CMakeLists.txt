# Native command line applications for TheiaSfM / pyTheia.
#
# Enabled with -DBUILD_APPS=ON (off by default, not part of the Python wheel).
#   theia_global_sfm, theia_incremental_sfm, theia_hybrid_sfm
#       SfM from reconstruction.recon + view_graph.bin (same inputs as `pytheia sfm`).
#   theia_info      reconstruction summary (views, tracks, mean reprojection error)
#   theia_convert   reconstruction -> COLMAP text/bin, NVM, Bundler, PLY, nerfstudio,
#                   transforms.json, JSON
#
# The applications use gflags and glog (already required by libTHEIA) and the
# rapidjson copy that ships with cereal.

set(THEIA_APPS_COMMON_SRC
  options_json.cc
  sfm_app_common.cc
  )

add_library(theia_apps_common STATIC ${THEIA_APPS_COMMON_SRC})
target_include_directories(theia_apps_common PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/libraries
  ${CMAKE_SOURCE_DIR}/libraries/spectra
  ${CMAKE_SOURCE_DIR}/libraries/Sophus
  ${CMAKE_SOURCE_DIR}/libraries/cereal/include
  ${CMAKE_SOURCE_DIR}/libraries/stlplus3
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  )
target_compile_definitions(theia_apps_common PRIVATE
  THEIA_CERES_HAS_CUDA_DENSE=${THEIA_CERES_HAS_CUDA_DENSE_VALUE}
  THEIA_CERES_HAS_CUDA_SPARSE=${THEIA_CERES_HAS_CUDA_SPARSE_VALUE})
target_link_libraries(theia_apps_common PUBLIC
  ${CMAKE_PROJECT_NAME}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES})

set(THEIA_APP_TARGETS
  theia_global_sfm
  theia_incremental_sfm
  theia_hybrid_sfm
  theia_info
  theia_convert
  )

foreach(app_target IN LISTS THEIA_APP_TARGETS)
  add_executable(${app_target} ${app_target}.cc)
  target_include_directories(${app_target} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/libraries/cereal/include
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS})
endforeach()

foreach(sfm_target theia_global_sfm theia_incremental_sfm theia_hybrid_sfm)
  target_link_libraries(${sfm_target} PRIVATE theia_apps_common)
endforeach()
target_link_libraries(theia_info PRIVATE theia_apps_common)
target_link_libraries(theia_convert PRIVATE theia_apps_common)

install(TARGETS ${THEIA_APP_TARGETS}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
