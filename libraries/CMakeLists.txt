if (${BUILD_TESTING})
  add_subdirectory(gtest)
endif (${BUILD_TESTING})

# AKAZE feature extractor.
#add_subdirectory(akaze)

# Cereal for portable IO.
add_subdirectory(cereal)

# Flann for fast approximate nearest neighbor searches.
add_subdirectory(flann)

# Add Optimo.
add_subdirectory(optimo)

# Add Statx.
add_subdirectory(statx)

# STLPlus for filepath tools.
add_subdirectory(stlplus3)

# Add VLFeat.
add_subdirectory(vlfeat)

# Add VisualSfM files.
#add_subdirectory(visual_sfm)

# Add pybind11 files.
# only if python is enabled
if (PYTHON_BUILD)
message(STATUS "Building pybind11")
add_subdirectory(pybind11)
else()
message(STATUS "Skipping pybind11")
endif (PYTHON_BUILD)