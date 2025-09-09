#!/bin/bash

function repair_wheel {
    wheel="$1"
    if ! "$PYBIN/python" -m auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
         "$PYBIN/python" -m auditwheel repair "$wheel" -w /home/wheelhouse/
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
export PIP_ROOT_USER_ACTION=ignore

( cd / && "${PYBIN}/python" -m pip install --upgrade --no-cache-dir pip wheel setuptools auditwheel )
cd /home

"${PYBIN}/python" setup.py bdist_wheel

# Repair built wheels and move only manylinux wheels to wheelhouse
for whl in /home/dist/*.whl; do
    repair_wheel "$whl"
    rm -f "$whl"
done

# Ensure only manylinux wheels remain in wheelhouse
find /home/wheelhouse -maxdepth 1 -type f -name "*.whl" ! -name "*manylinux*" -print -delete
