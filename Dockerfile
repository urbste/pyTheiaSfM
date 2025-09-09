FROM urbste/pytheia_base:1.3.0

COPY . ./home

RUN chmod +x /home/pypackage/build-wheels-linux.sh && bash /home/pypackage/build-wheels-linux.sh

