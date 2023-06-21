
set(PACKAGE_NAME pytheia)

set(PY_ALL_SOURCE_FILES
    pytheia_pybind.cc
    matching/matching.cc
    math/math.cc
    mvs/mvs.cc
    solvers/solvers.cc
    sfm/sfm.cc
    io/io.cc)

set(PY_ALL_HEADER_FILES
    pytheia_pybind.h
    matching/matching.h
    math/math.h
    mvs/mvs.h
    solvers/solvers.h
    sfm/sfm.h
    io/io.h)


pybind11_add_module(${PACKAGE_NAME}
    ${PY_ALL_HEADER_FILES} ${PY_ALL_SOURCE_FILES})

target_link_libraries(${PACKAGE_NAME} PRIVATE ${CMAKE_PROJECT_NAME})
if (WIN32)
    target_link_options(${PACKAGE_NAME} PUBLIC "/force:multiple")
endif()

# Use `make pip-package` to create the pip package in the build directory
# add_custom_target(pip-package
#     COMMAND ${PYTHON_EXECUTABLE} setup.py bdist_wheel --dist-dir pip_package
#     COMMAND echo "pip wheel created at ${PYTHON_PACKAGE_DST_DIR}/pip_package"
#     WORKING_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}
#     DEPENDS python-package
# )