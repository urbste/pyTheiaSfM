.. _chapter-building:

======================
Building pyTheia Library
======================

Theia source code and documentation are hosted on `Github
<https://github.com/urbste/pyTheiaSfM>`_ where you can always grab the latest version

.. _section-dependencies:

Dependencies
------------

Theia relies on a number of open source libraries. Luckily, most of the will be included in Ceres

1. C++11 is needed for certain functionality and added models to the stdlib. C++0x will probably work in most cases, but is not guaranteed. As such, you need a compiler that supports C++11 appropriately.

2. `CMake <http://www.cmake.org>`_ is a cross platform build system. Theia needs a relatively recent version of CMake (version 2.8.0 or better).

3. `eigen3 <http://eigen.tuxfamily.org/index.php?title=Main_Page>`_ is used extensively for doing nearly all the matrix and linear algebra operations.

5. `Ceres Solver <https://code.google.com/p/ceres-solver/>`_ is a library for solving non-linear least squares problems. In particular, Theia uses it for Bundle Adjustment.

**NOTE**: Theia also depends on the following libraries, but they are included in the installation of Ceres so it is likely that you do not need to reinstall them.

6. `google-glog <http://code.google.com/p/google-glog>`_ is used for error checking and logging. Ceres needs glog version 0.3.1 or later. Version 0.3 (which ships with Fedora 16) has a namespace bug which prevents Ceres from building.

7. `gflags <http://code.google.com/p/gflags>`_ is a library for processing command line flags. It is used by some of the examples and tests.

Make sure all of these libraries are installed properly before proceeding. Improperly installing any of these libraries can cause Theia to not build.

.. _section-building:

Building on Linux or WSL2 (on Windows)
--------
This section describes how to build on Ubuntu locally or on WSL2 both with sudo rights. The basic dependency is:
First we need the basic libraries: Eigen3 and the ceres-solver

.. code-block:: bash
  sudo apt install cmake build-essential libgflags-dev libgoogle-glog-dev libatlas-base-dev
  # cd to your favourite library folder
  mkdir LIBS && cd LIBS
 
  git clone https://gitlab.com/libeigen/eigen
  cd eigen && git checkout 3.3.9
  mkdir -p build && cd build && cmake .. && sudo make install

  # ceres solver
  cd LIBS
  git clone https://ceres-solver.googlesource.com/ceres-solver
  cd ceres-solver && git checkout 2.0.0 && mkdir build && cd build 
  cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
  make -j && make install

Then, navigate to the source directory of the pyTheiaSfM library to build the Python wheel. 
In your favorite Python environment execute the following commands:

.. code-block:: bash
  python3 setup.py bdist_wheel
  cd dist && pip install *.whl
