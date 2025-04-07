#!/bin/bash
set -e

# Clean previous builds
# rm -rf dist build
# rm -rf cmake_build
# mkdir -p cmake_build

# Set up environment for portable build
export BUILD_MARCH_NATIVE=0

# Optional: You can use auditwheel to make the wheel more portable
# First build the wheel
python setup.py bdist_wheel --plat-name=manylinux_2_35_x86_64

# Find the wheel file
WHEEL_FILE=$(ls dist/*.whl)

# If auditwheel is available, repair the wheel to bundle libstdc++
if command -v auditwheel &> /dev/null; then
    echo "Using auditwheel to repair wheel..."
    cd dist
    # Backup the original wheel
    cp $(basename $WHEEL_FILE) $(basename $WHEEL_FILE).backup
    # Repair the wheel to include libstdc++ and other libs
    auditwheel repair $(basename $WHEEL_FILE) --plat manylinux_2_35_x86_64 -w .
    # Remove the original wheel
    rm $(basename $WHEEL_FILE).backup
    cd ..
    echo "Wheel repaired."
else
    echo "auditwheel not found, skipping repair step. Consider installing auditwheel with pip."
fi

# Install the wheel
echo "Installing wheel..."
cd dist && pip install --force-reinstall *.whl