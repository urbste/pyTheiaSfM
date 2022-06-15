FROM urbste/pytheia_base:1.0.1

COPY . ./home

RUN chmod +x /home/pypackage/build-wheels-linux.sh && sh /home/pypackage/build-wheels-linux.sh

