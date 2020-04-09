FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ARG CLANG_FORMAT_VERSION=10

RUN apt-get update \
 && apt-get install -y \
      clang-format-${CLANG_FORMAT_VERSION} \
      bash \
      git \
 && rm -rf /var/lib/apt/lists/*
