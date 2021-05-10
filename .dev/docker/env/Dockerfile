# For valid combinations, check the following repo:
# https://gitlab.com/nvidia/container-images/cuda/tree/master/dist
# To disable cuda, use "--build-arg CUDA_VERSION=" during image build process
ARG CUDA_VERSION="9.2"
ARG UBUNTU_VERSION="20.04"
ARG BASE_CUDA_IMAGE=${CUDA_VERSION:+"nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}"}
ARG BASE_IMAGE=${BASE_CUDA_IMAGE:-"ubuntu:${UBUNTU_VERSION}"}

FROM ${BASE_IMAGE}

# Eigen patch (https://eigen.tuxfamily.org/bz/show_bug.cgi?id=1462) to fix issue metioned
# in https://github.com/PointCloudLibrary/pcl/issues/3729 is available in Eigen 3.3.7.
# Not needed from 20.04 since it is the default version from apt
ARG EIGEN_MINIMUM_VERSION=3.3.7

# See https://www.optonic.com/support/download/ensenso-sdk/archiv/ for available versions
ARG ENSENSOSDK_VERSION=3.2.489

# See https://github.com/IntelRealSense/librealsense/tags for available tags of releases
ARG REALSENSE_VERSION=2.50.0

# Check https://packages.ubuntu.com/search?suite=all&arch=any&searchon=names&keywords=libvtk%20qt-dev
# for available packes for choosen UBUNTU_VERSION
ARG VTK_VERSION=6

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get -V install -y \
      build-essential \
      clang \
      clang-tidy \
      cmake \
      libbenchmark-dev \
      libblas-dev \
      libboost-date-time-dev \
      libboost-filesystem-dev \
      libboost-iostreams-dev \
      libflann-dev \
      libglew-dev \
      libgtest-dev \
      libopenni-dev \
      libopenni2-dev \
      libproj-dev \
      libqhull-dev \
      libqt5opengl5-dev \
      libusb-1.0-0-dev \
      libvtk${VTK_VERSION}-dev \
      libvtk${VTK_VERSION}-qt-dev \
      qtbase5-dev \
      software-properties-common \
      wget \
      xvfb \
 && rm -rf /var/lib/apt/lists/*

# Use libeigen3-dev if it meets the minimal version.
# In most cases libeigen3-dev is already installed as a dependency of libvtk6-dev & libvtk7-dev, but not in every case (libvtk9 doesn't seem to have this dependency),
# so install it from apt if the version is sufficient.
RUN if dpkg --compare-versions $(apt-cache show --no-all-versions libeigen3-dev | grep -P -o 'Version:\s*\K.*') ge ${EIGEN_MINIMUM_VERSION}; then \
      apt-get -V install -y libeigen3-dev ; \
    else \
      wget -qO- https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_MINIMUM_VERSION}/eigen-${EIGEN_MINIMUM_VERSION}.tar.gz | tar xz \
      && cd eigen-${EIGEN_MINIMUM_VERSION} \
      && mkdir build \
      && cd build \
      && cmake .. \
      && make install \
      && cd ../.. \
      && rm -rf eigen-${EIGEN_MINIMUM_VERSION}/ ; \
    fi

RUN wget -qO- https://github.com/IntelRealSense/librealsense/archive/v${REALSENSE_VERSION}.tar.gz | tar xz \
 && cd librealsense-${REALSENSE_VERSION} \
 && mkdir build \
 && cd build \
 && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
 && make -j2 \
 && make install \
 && cd ../.. \
 && rm -rf librealsense-${REALSENSE_VERSION}

RUN wget -qO ensenso.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-${ENSENSOSDK_VERSION}-x64.deb \
 && dpkg -i ensenso.deb \
 && rm ensenso.deb
