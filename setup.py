#!/usr/bin/env python3

import os
import setuptools
import subprocess
import sys
from glob import glob
from wheel.bdist_wheel import bdist_wheel

import distutils.sysconfig as sysconfig
import os
from distutils.sysconfig import get_python_inc
python_lib_location = os.path.join(sysconfig.get_config_var('LIBDIR'), sysconfig.get_config_var('LDLIBRARY'))
python_include_dir = get_python_inc()

class platform_bdist_wheel(bdist_wheel):
    """Patched bdist_well to make sure wheels include platform tag."""
    def finalize_options(self):
        bdist_wheel.finalize_options(self)
        self.root_is_pure = False
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

    build_march_native = int(os.environ.get("BUILD_MARCH_NATIVE", 0))

    cmake_command = [
        'cmake',
        '../',
        '-DBUILD_TESTING=OFF',
        '-DPYTHON_BUILD=ON',
        '-DCMAKE_BUILD_TYPE=Release',
        '-DPYTHON_EXECUTABLE=' + sys.executable,
	    '-DPYTHON_LIBRARY=' + python_lib_location,
	    '-DPYTHON_INCLUDE_DIR=' +  python_include_dir,
        '-DBUILD_WITH_MARCH_NATIVE={}'.format("ON" if build_march_native else "OFF")
    ]
    subprocess.check_call(cmake_command, cwd='cmake_build')


def build_c_extension():
    """Compile C extension."""
    print("Compiling extension...")
    subprocess.check_call(['make', '-j5'], cwd='cmake_build')


def create_package():
    files = glob('cmake_build/lib/pytheia*')
    subprocess.run(['cp'] + files + ['src/pytheia'])

configure_c_extension()
build_c_extension()
create_package()

setuptools.setup(
    name='pytheia',
    version='0.1.20',
    description='A performant Structure from Motion library for Python',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/urbste/pyTheiaSfM.git',
    # project_urls={
    #     "Documentation": "http://theia-sfm.org/",
    # },
    author='Steffen Urban, Shengyu Yin',
    author_email = "urbste@googlemail.com, shengyu952014@outlook.com",
    license='BSD',
    packages=setuptools.find_packages(where='src'),
    include_package_data=True,
    package_dir={
        'pytheia': 'src/pytheia',
    },
    package_data={
        'pytheia': [
            'pytheia.*',
            'libflann_cpp.*',
            #'stubs/pytheia/pytheia/__init__.pyi',
            #'stubs/pytheia/pytheia/io/__init__.pyi',
            #'stubs/pytheia/pytheia/matching/__init__.pyi',
            #'stubs/pytheia/pytheia/math/__init__.pyi',
            #'stubs/pytheia/pytheia/sfm/__init__.pyi',
            #'stubs/pytheia/pytheia/solvers/__init__.pyi',
        ]
    },
    cmdclass={'bdist_wheel': platform_bdist_wheel},
)
