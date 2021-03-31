FROM quay.io/pypa/manylinux2014_x86_64

COPY . ./home

RUN chmod +x /home/travis/build-wheels.sh && sh /home/travis/build-wheels.sh

