FROM quay.io/pypa/manylinux2014_x86_64

COPY . .

RUN chmod +x travis/build-wheels.sh && sh travis/build-wheels.sh

