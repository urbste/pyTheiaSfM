FROM quay.io/pypa/manylinux2014_x86_64

COPY . ./home

#RUN chmod +x travis/build-wheels.sh && sh travis/build-wheels.sh

