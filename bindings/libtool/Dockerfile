FROM ubuntu
MAINTAINER <iamdm99@gmail.com>

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update  -y \
 && apt-get install -y tzdata git cmake vim make wget gnupg libz-dev

RUN apt-get update -y && apt-get install -y llvm-10 llvm-10-dev clang-10 libclang-10-dev

ENV C clang-10
ENV CXX clang++-10
ENV PATH /usr/lib/llvm-10/bin:${PATH}

# Symbolically link for convenience
RUN ln -s /usr/lib/llvm-10/ /llvm

# These volumes should be mounted on the host.
VOLUME /home/
WORKDIR /home
