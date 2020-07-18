# For valid combinations, check the following repo:
# https://gitlab.com/nvidia/container-images/cuda/tree/master/dist
# To enable cuda, use "--build-arg USE_CUDA=true" during image build process
ARG USE_CUDA
ARG CUDA_VERSION="9.2"
ARG UBUNTU_DISTRO="16.04"
ARG BASE_CUDA_IMAGE=${USE_CUDA:+"nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_DISTRO}"}
ARG BASE_IMAGE=${BASE_CUDA_IMAGE:-"ubuntu:${UBUNTU_DISTRO}"}

FROM ${BASE_IMAGE}

ARG VTK_VERSION=6
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
      cmake \
      g++ \
      clang \
      wget \
      libboost-date-time-dev \
      libboost-filesystem-dev \
      libboost-iostreams-dev \
      libeigen3-dev \
      libblas-dev \
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
 && rm -rf /var/lib/apt/lists/*

# Eigen patch (https://eigen.tuxfamily.org/bz/show_bug.cgi?id=1462) to fix issue metioned
# in https://github.com/PointCloudLibrary/pcl/issues/3729 is available in Eigen 3.3.7
# Not needed from 20.04 since it is the default version from apt
RUN if [ `pkg-config --modversion eigen3 | cut -d. -f3` -lt 7 ]; then \
     wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz | tar xz \
     && apt install -y libblas-dev \
     && cd eigen-3.3.7 \
     && mkdir build \
     && cd build \
     && cmake .. \
     && make install \
     && cd ../.. \
     && rm -rf eigen-3.3.7/ \
     && rm -f eigen-3.3.7.tar.gz ; \
    fi

# To avoid CUDA build errors on CUDA 9.2+ GCC 7 is required
RUN if [ `gcc -dumpversion | cut -d. -f1` -lt 7 ]; then add-apt-repository ppa:ubuntu-toolchain-r/test \
     && apt update \
     && apt install g++-7 -y \
     && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
     && update-alternatives --config gcc ; \
    fi

RUN wget -qO- https://github.com/IntelRealSense/librealsense/archive/v2.23.0.tar.gz | tar xz \
 && cd librealsense-2.23.0 \
 && mkdir build \
 && cd build \
 && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
 && make -j2 \
 && make install \
 && cd ../.. \
 && rm -rf librealsense-2.23.0

RUN wget -qO ensenso.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.2.160-x64.deb \
 && dpkg -i ensenso.deb \
 && rm ensenso.deb
