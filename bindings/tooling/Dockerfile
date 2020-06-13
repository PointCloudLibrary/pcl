FROM ubuntu:18.04
MAINTAINER <iamdm99@gmail.com>

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update  -y \
 && apt-get install -y tzdata git cmake vim make wget gnupg libz-dev

RUN apt-get update -y && apt-get install -y python3-dev

RUN wget -O - 'http://apt.llvm.org/llvm-snapshot.gpg.key' | apt-key add - \
 && echo 'deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic-10 main' >> /etc/apt/sources.list \
 && echo 'deb-src http://apt.llvm.org/bionic/ llvm-toolchain-bionic-10 main' >> /etc/apt/sources.list

RUN apt-get update -y && apt-get install -y llvm-10-dev clang-10 libclang-10-dev

RUN apt-get update -y && apt-get install -y libpcl-dev

RUN apt-get update -y && apt-get install -y python3-clang-10

RUN apt-get update -y && apt-get install -y python3-pip

RUN pip3 install black

ENV C clang-10
ENV CXX clang++
ENV PATH /usr/lib/llvm-10/bin:${PATH}

# Symbolically link for convenience
RUN ln -s /usr/lib/llvm-10/ /llvm

# These volumes should be mounted on the host.
VOLUME /home/
WORKDIR /home
