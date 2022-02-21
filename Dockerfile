FROM urbste/pytheia_base:1.0.1

COPY . ./home

RUN chmod +x /home/travis/build-wheels.sh && sh /home/travis/build-wheels.sh

