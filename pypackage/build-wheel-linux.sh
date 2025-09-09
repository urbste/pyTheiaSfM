#!/bin/bash

function repair_wheel {
    wheel="$1"
    if ! auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
         auditwheel repair "$wheel" -w /home/wheelhouse/
    fi
}

uname -a
echo "Current CentOS Version:"
cat /etc/centos-release

set -e -u -x

cd /home
mkdir -p wheelhouse

# we cannot simply use `pip` or `python`, since points to old 2.7 version
PYBIN="/opt/python/$PYTHON_VERSION/bin"
PYVER_NUM=$($PYBIN/python -c "import sys;print(sys.version.split(\" \")[0])")
PYTHONVER="$(basename $(dirname $PYBIN))"

echo "Python bin path: $PYBIN"
echo "Python version number: $PYVER_NUM"
echo "Python version: $PYTHONVER"

export PATH=$PYBIN:$PATH

${PYBIN}/pip install auditwheel setuptools

"${PYBIN}/python" setup.py bdist_wheel

cp /home/dist/*.whl /home/wheelhouse
rm -rf /home/dist

# Bundle external shared libraries into the wheels
for whl in /home/wheelhouse/*.whl; do
    repair_wheel "$whl"
done
