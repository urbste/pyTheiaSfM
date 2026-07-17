if (${BUILD_TESTING})
  add_subdirectory(gtest)
endif (${BUILD_TESTING})

# Cereal for portable IO.
add_subdirectory(cereal)

# STLPlus for filepath tools.
add_subdirectory(stlplus3)

# Add pybind11 files.
# only if python is enabled
if (PYTHON_BUILD)
message(STATUS "Building pybind11")
add_subdirectory(pybind11)
message(STATUS "Building nanobind (spike)")
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
if(Python_INCLUDE_DIR AND NOT Python_INCLUDE_DIRS)
  set(Python_INCLUDE_DIRS "${Python_INCLUDE_DIR}")
endif()
add_subdirectory(nanobind)
else()
message(STATUS "Skipping pybind11")
endif (PYTHON_BUILD)