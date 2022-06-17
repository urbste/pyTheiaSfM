BUILD_MARCH_NATIVE=0 python setup.py bdist_wheel
cd dist && pip install --force-reinstall *.whl 