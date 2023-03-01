FROM urbste/pytheia_base:1.1.0

COPY . ./home

RUN chmod +x /home/pypackage/build-wheels-linux.sh && sh /home/pypackage/build-wheels-linux.sh

