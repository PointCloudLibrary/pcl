FROM ubuntu
MAINTAINER <iamdm99@gmail.com>

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update  -y \
 && apt-get install -y tzdata git cmake vim make wget gnupg libz-dev

RUN apt-get update -y && apt-get install -y llvm-10 llvm-10-dev clang-10 libclang-10-dev \
python-clang-9

RUN apt-get update -y && apt-get install -y black

RUN apt-get update -y && apt-get install -y libboost-all-dev \
libeigen3-dev libflann-dev libvtk7-dev libvtk7-qt-dev libqhull-dev \
libopenni2-dev libpcl-dev

RUN apt-get update -y && apt-get install -y python3-clang-10

ENV C clang-10
ENV CXX clang++-10
ENV PATH /usr/lib/llvm-10/bin:${PATH}

# Symbolically link for convenience
RUN ln -s /usr/lib/llvm-10/ /llvm

# These volumes should be mounted on the host.
VOLUME /home/
WORKDIR /home
