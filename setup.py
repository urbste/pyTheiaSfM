# -*- coding: utf-8 -*-
import os
import sys
import subprocess

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}

from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

# https://stackoverflow.com/a/45150383/1255535
class bdist_wheel(_bdist_wheel):
    def finalize_options(self):
<<<<<<< HEAD
        _bdist_wheel.finalize_options(self)
        self.root_is_pure = False

# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = "Debug" if self.debug else "Release"

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
        # from Python.
        cmake_args = [
            "-DCMAKE_BUILD_TYPE=Release",
            "-DBUILD_TESTING=OFF",
            "-DPYTHON_BUILD=ON",
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
            "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DEXAMPLE_VERSION_INFO={}".format(self.distribution.get_version()),
            "-DCMAKE_BUILD_TYPE={}".format(cfg),  # not used on MSVC, but no harm
        ]
        build_args = []

        if self.compiler.compiler_type != "msvc":
            # Using Ninja-build since it a) is available as a wheel and b)
            # multithreads automatically. MSVC would require all variables be
            # exported for Ninja to pick it up, which is a little tricky to do.
            # Users can override the generator with CMAKE_GENERATOR in CMake
            # 3.15+.
            if not cmake_generator:
                cmake_args += ["-GNinja"]

        else:

            # Single config generators are handled "normally"
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})

            # CMake allows an arch-in-generator style for backward compatibility
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

            # Specify the arch if using MSVC generator, but only if it doesn't
            # contain a backward-compatibility arch spec already in the
            # generator name.
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]

            # Multi-config generators have a different way to specify configs
            if not single_config:
                cmake_args += [
                    "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}".format(cfg.upper(), extdir)
                ]
                build_args += ["--config", cfg]

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            if hasattr(self, "parallel") and self.parallel:
                # CMake 3.12+ only.
                build_args += ["-j{}".format(self.parallel)]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
=======
        bdist_wheel.finalize_options(self)
        self.root_is_pure = True
"""
def _find_packages():
    packages = setuptools.find_packages()
    packages.append('mypythonpackage.doc')
    packages.append('matlabsources')
    print('packages found: {}'.format(packages))
    return packages
""" 

def configure_c_extension():
    """Configure cmake project to C extension."""
    print("Configuring for python {}.{}...".format(sys.version_info.major,
                                                   sys.version_info.minor))
    os.makedirs('cmake_build', exist_ok=True)
    cmake_command = [
        'cmake',
        '../',
        '-DBUILD_TESTING=OFF',
        '-DPYTHON_BUILD=ON',
        '-DPYTHON_EXECUTABLE=' + sys.executable,
	    '-DPYTHON_LIBRARY=' + python_lib_location,
	    '-DPYTHON_INCLUDE_DIR=' +  python_include_dir
    ]
    subprocess.check_call(cmake_command, cwd='cmake_build')


def build_c_extension():
    """Compile C extension."""
    print("Compiling extension...")
    subprocess.check_call(['make', '-j20'], cwd='cmake_build')


def create_package():
    subprocess.run(['mkdir', '-p', 'pytheia'], cwd='src')
    files = glob('cmake_build/lib/*.so')
    subprocess.run(['cp'] + files + ['src/pytheia'])
    subprocess.run(['touch', 'src/pytheia/__init__.py'])


configure_c_extension()
build_c_extension()
create_package()

setuptools.setup(
>>>>>>> ff08927b6c71cb8c10fe7f3bc73f93461beb9064
    name='pytheia',
    version='0.1.0',
    description='A Python Structure from Motion Library',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/shengyu17/TheiaSfM/tree/feature/python_bindings',
    project_urls={
        "Documentation": "http://theia-sfm.org/",
    },
    author='Shengyu Yin',
    author_email = "shengyu952014@outlook.com",
    license='BSD',
    packages=find_packages(where='src'),
    include_package_data=True,
    package_dir={
        'pytheia': 'src/pytheia',
    },
    # package_data={
    #    'pytheia': ["pytheia*.so"]
    # },
    ext_modules=[CMakeExtension("pytheia")],
    cmdclass={'bdist_wheel': CMakeBuild},
    zip_safe=False,
    #cmdclass={'bdist_wheel': bdist_wheel},
)
