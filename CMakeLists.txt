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
# Author: Chris Sweeney (sweeney.chris.m@gmail.com)

# edited by Steffen Urban (urbste@googlemail.com)

cmake_minimum_required(VERSION 3.15)
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif (COMMAND cmake_policy)

# Disable warnings about rpath on MacOS X for now.
# TODO(cmsweeney): Figure out the proper way to handle this!
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW) # CMake 3.0
endif()

project(THEIA LANGUAGES CXX C)
include_directories(${PROJECT_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# Additional cmake find modules - Keep custom path for now, but prioritize modern find_package
# ==============================================================================
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(BUILD_TESTING "Enable testing" OFF)
option(BUILD_DOCUMENTATION "Build html User's Guide" OFF)
option(PYTHON_BUILD "If we are building python bindings" OFF)
option(WITH_ROCKSDB "If rcocksdb should be included as a feature database" OFF)

if (PYTHON_BUILD)
    add_definitions(-DPYTHON_BUILD)
endif()

if (WITH_ROCKSDB)
    add_definitions(-DWITH_ROCKSDB)
endif()

if(BUILD_TESTING)
  enable_testing()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_DEBUG_POSTFIX "-debug")

set(THEIA_VERSION_MAJOR 0)
set(THEIA_VERSION_MINOR 7)
set(THEIA_VERSION_PATCH 0)
set(THEIA_VERSION
    ${THEIA_VERSION_MAJOR}.${THEIA_VERSION_MINOR}.${THEIA_VERSION_PATCH})
set(THEIA_ABI_VERSION 0.7.0)

add_definitions(-DTHEIA_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find required packages
set(MIN_EIGEN_VERSION 3.4)
find_package(Eigen3 ${MIN_EIGEN_VERSION} REQUIRED)
message(STATUS "Found Eigen version ${EIGEN3_VERSION}: ${EIGEN3_INCLUDE_DIRS}")

# Find the threading package (required for Threads::Threads target)
find_package(Threads REQUIRED)
message(STATUS "Found Threads: ${CMAKE_THREAD_LIBS_INIT}")

find_package(Gflags REQUIRED)
if (GFLAGS_FOUND)
  message(STATUS "Found Gflags version ${GFLAGS_VERSION}: ${GFLAGS_INCLUDE_DIR} (Namespace: ${GFLAGS_NAMESPACE})")
else()
   message(FATAL_ERROR "Could not find Gflags")
endif()

find_package(Glog REQUIRED)
if (GLOG_FOUND)
  message(STATUS "Found Glog version ${GLOG_VERSION}: ${GLOG_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "Could not find Glog")
endif()

if(WITH_ROCKSDB)
  find_package(RocksDB REQUIRED)
  if(ROCKSDB_FOUND)
    message(STATUS "Found RocksDB: ${ROCKSDB_INCLUDE_DIRS}")
  else()
    message(FATAL_ERROR "Could not find RocksDB")
  endif()
endif()

include_directories(
  include
  src
  ${CERES_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

if (NOT PYTHON_BUILD AND WITH_ROCKSB)
    include_directories(${ROCKSDB_INCLUDE_DIR})
endif (NOT PYTHON_BUILD AND WITH_ROCKSB)

# build Third party libraries included in distro.
add_subdirectory(libraries)
include_directories(
  libraries
  libraries/spectra
  libraries/Sophus
  ${akaze_SOURCE_DIR}
  ${akaze_INCLUDE_DIR}
  ${cereal_SOURCE_DIR}
  ${cereal_SOURCE_DIR}/include
  ${flann_SOURCE_DIR}
  ${flann_SOURCE_DIR}/src/cpp
  ${gtest_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}
  ${optimo_SOURCE_DIR}
  ${statx_SOURCE_DIR}
  ${stlplus3_SOURCE_DIR}
  ${vlfeat_SOURCE_DIR}
  )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

add_subdirectory(src/theia)

if (NOT PYTHON_BUILD)
  add_subdirectory(applications)
endif (NOT PYTHON_BUILD)

if (BUILD_DOCUMENTATION)
  message("-- Documentation building is enabled")
  find_package(Sphinx)
  add_subdirectory(docs)
endif (BUILD_DOCUMENTATION)

# ---------------- Install commands ----------------- #
# Install the .h files
file(GLOB THEIA_HDRS ${CMAKE_SOURCE_DIR}/include/theia/*.h)
install(FILES ${THEIA_HDRS} DESTINATION include/theia)

file(GLOB_RECURSE THEIA_INTERNAL_HDRS ${CMAKE_SOURCE_DIR}/src/theia/*.h)
install(DIRECTORY src/theia/ DESTINATION include/theia FILES_MATCHING PATTERN "*.h")

install(DIRECTORY libraries/ DESTINATION include/theia/libraries FILES_MATCHING PATTERN "*.h*")

# Add an uninstall target to remove all installed files.
configure_file("${CMAKE_SOURCE_DIR}/cmake/uninstall.cmake.in"
               "${CMAKE_BINARY_DIR}/cmake/uninstall.cmake"
               IMMEDIATE @ONLY)

add_custom_target(uninstall
                  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake/uninstall.cmake)

# Set up install directories. INCLUDE_INSTALL_DIR, LIB_INSTALL_DIR and
# CMAKECONFIG_INSTALL_DIR must not be absolute paths.
if (WIN32)
  set(INCLUDE_INSTALL_DIR Include)
  set(LIB_INSTALL_DIR Lib)
  set(CMAKECONFIG_INSTALL_DIR CMake)
  set(RELATIVE_CMAKECONFIG_INSTALL_DIR CMake)
else ()
  set(INCLUDE_INSTALL_DIR include)
  set(LIB_INSTALL_DIR lib)
  set(CMAKECONFIG_INSTALL_DIR share/Theia)
  set(RELATIVE_CMAKECONFIG_INSTALL_DIR share/Theia)
endif ()

install(EXPORT TheiaExport
        DESTINATION ${RELATIVE_CMAKECONFIG_INSTALL_DIR} FILE TheiaTargets.cmake)

file(RELATIVE_PATH INSTALL_ROOT_REL_CONFIG_INSTALL_DIR
     ${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR} ${CMAKE_INSTALL_PREFIX})

configure_file("${CMAKE_SOURCE_DIR}/cmake/TheiaConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfig.cmake" @ONLY)

configure_file("${CMAKE_SOURCE_DIR}/cmake/TheiaConfigVersion.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfigVersion.cmake" @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfigVersion.cmake"
              "${CMAKE_SOURCE_DIR}/cmake/FindRocksDB.cmake"
              DESTINATION ${CMAKECONFIG_INSTALL_DIR})

if (PYTHON_BUILD)
  add_subdirectory(src/pytheia)
endif (PYTHON_BUILD)
