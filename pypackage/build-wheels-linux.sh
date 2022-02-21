#!/bin/bash

uname -a
echo "Current CentOS Version:"
cat /etc/centos-release

set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
         #auditwheel repair "$wheel" --plat "$PLAT" -w /wheelhouse/
         auditwheel repair "$wheel" -w /home/wheelhouse/
    fi
}

NUM_CORES=4

cd /home
mkdir -p wheelhouse
# Compile wheels
for PYBIN in /opt/python/*/bin; do
    "${PYBIN}/pip" install nose
    "${PYBIN}/python" setup.py bdist_wheel
    rm -rf /home/cmake_build/lib/*.so
    rm -rf /home/build/lib/pytheia/*.so
    rm -rf /home/src/pytheia/*.so
    rm -rf /home/build/lib/pytheia/*.so
    rm -rf /home/pytheia.egg-info
done
cp /home/dist/*.whl /home/wheelhouse
rm -rf /home/dist

# Bundle external shared libraries into the wheels
for whl in /home/wheelhouse/*.whl; do
    repair_wheel "$whl"
done