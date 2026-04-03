#!/usr/bin/env python3

import os
import setuptools
import subprocess
import sys
from glob import glob
from pathlib import Path
from wheel.bdist_wheel import bdist_wheel

import distutils.sysconfig as sysconfig
import os
from distutils.sysconfig import get_python_inc
python_lib_location = os.path.join(sysconfig.get_config_var('LIBDIR'), sysconfig.get_config_var('LDLIBRARY'))
python_include_dir = get_python_inc()

_REPO_ROOT = Path(__file__).resolve().parent


def _package_version():
    """Must match CMake project() and pyproject dynamic version (root VERSION)."""
    return (_REPO_ROOT / "VERSION").read_text(encoding="utf-8").strip()


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
        '-DBUILD_WITH_MARCH_NATIVE={}'.format("ON" if build_march_native else "OFF"),
    ]
    subprocess.check_call(cmake_command, cwd='cmake_build')


def build_c_extension():
    """Compile C extension."""
    print("Compiling extension...")
    subprocess.check_call(['make', '-j10'], cwd='cmake_build')


def create_package():
    files = glob('cmake_build/lib/pytheia*')
    subprocess.run(['cp'] + files + ['src/pytheia'])

def generate_stubs():
    """Generate .pyi stubs using pybind11-stubgen if available.

    Controlled via env var GENERATE_STUBS (default on). Skips gracefully if
    the tool is not available or generation fails.
    """
    generate = os.environ.get('GENERATE_STUBS', '1') != '0'
    if not generate:
        print('Skipping stub generation (GENERATE_STUBS=0).')
        return
    try:
        env = os.environ.copy()
        env['PYTHONPATH'] = os.path.abspath('src') + os.pathsep + env.get('PYTHONPATH', '')
        cmd = [
            sys.executable, '-m', 'pybind11_stubgen', 'pytheia.pytheia', '-o', 'src',
        ]
        print('Generating .pyi stubs:', ' '.join(cmd))
        subprocess.check_call(cmd, env=env)
        import shutil
        # Alternate output layout (older stubgen / different module paths)
        alt = os.path.join('src', 'pytheia-stubs')
        target_pkg = os.path.join('src', 'pytheia', 'pytheia')
        if os.path.isdir(alt):
            for root, _, files in os.walk(alt):
                rel = os.path.relpath(root, alt)
                dest_root = os.path.join(target_pkg, rel) if rel != '.' else target_pkg
                os.makedirs(dest_root, exist_ok=True)
                for fname in files:
                    if fname.endswith('.pyi'):
                        shutil.copy2(
                            os.path.join(root, fname),
                            os.path.join(dest_root, fname),
                        )
        init_pyi = os.path.join(target_pkg, '__init__.pyi')
        flat_pyi = os.path.join('src', 'pytheia', 'pytheia.pyi')
        if os.path.isfile(init_pyi):
            # Single-file stub next to the extension module for Pylance/mypy (pytheia.pytheia)
            shutil.copy2(init_pyi, flat_pyi)
        print('Stub files updated under src/pytheia (see pytheia.pyi and pytheia/*.pyi)')
    except Exception as e:
        print('Stub generation skipped (tool missing or failed):', e)

configure_c_extension()
build_c_extension()
create_package()
generate_stubs()

setuptools.setup(
    name='pytheia',
    version=_package_version(),
    description='A performant Structure from Motion library for Python',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/urbste/pyTheiaSfM.git',
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
        '*.pyi',
        '*/*.pyi',
        '*/*/*.pyi',
        'py.typed',
        ]
     },
    cmdclass={'bdist_wheel': platform_bdist_wheel},
)
