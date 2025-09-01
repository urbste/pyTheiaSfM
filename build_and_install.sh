rm src/pytheia/*.so
rm cmake_build
rm dist/

BUILD_MARCH_NATIVE=0 python setup.py bdist_wheel --plat-name=manylinux_2_35_x86_64
cd dist && pip install --force-reinstall *.whl 