#!/bin/bash

uname -a
echo "Current CentOS Version:"
cat /etc/centos-release

set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! "$AWPY" -m auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
         #auditwheel repair "$wheel" --plat "$PLAT" -w /wheelhouse/
         "$AWPY" -m auditwheel repair "$wheel" -w /home/wheelhouse/
    fi
}

cd /home
mkdir -p wheelhouse

export PIP_ROOT_USER_ACTION=ignore

# Choose a Python to run auditwheel (first available)
AWPY="$(ls -d /opt/python/*/bin | head -n1)/python"
# Ensure auditwheel is available for the chosen interpreter
( cd / && "$AWPY" -m pip install --upgrade --no-cache-dir pip wheel setuptools auditwheel )

# Compile wheels
for PYBIN in /opt/python/*/bin; do
    ( cd / && "${PYBIN}/python" -m pip install --upgrade --no-cache-dir pip wheel setuptools )
    cd /home
    "${PYBIN}/python" setup.py bdist_wheel
    rm -rf /home/cmake_build/lib/*.so
    rm -rf /home/build/lib/pytheia/*.so
    rm -rf /home/src/pytheia/*.so
    rm -rf /home/build/lib/pytheia/*.so
    rm -rf /home/pytheia.egg-info
    #fi
done
cp /home/dist/*.whl /home/wheelhouse
rm -rf /home/dist

# Bundle external shared libraries into the wheels
for whl in /home/wheelhouse/*.whl; do
    repair_wheel "$whl"
done