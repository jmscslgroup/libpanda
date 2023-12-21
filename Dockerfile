FROM ubuntu:20.04


ENV TERM xterm-256color
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update 
RUN apt-get install -y curl nano iputils-ping net-tools git pkg-config cmake build-essential
RUN apt-get install -y libusb-1.0-0-dev libjsoncpp-dev libncurses5-dev



RUN cd home/ && git clone https://github.com/jmscslgroup/libpanda
RUN mkdir -p home/libpanda/build
RUN cd home/libpanda/build && cmake ..
RUN cd home/libpanda/build && make -j4 && make install


CMD ["/bin/bash"]