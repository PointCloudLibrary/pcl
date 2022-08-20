ARG UBUNTU_VERSION=20.04

FROM "ubuntu:${UBUNTU_VERSION}"

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

# Use the latest version of CMake by adding the Kitware repository if true,
# otherwise use the default version of CMake of the system.
ARG USE_LATEST_CMAKE=false

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get -V install -y \
      build-essential \
      clang \
      clang-tidy \
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
      lsb-release \
      qtbase5-dev \
      software-properties-common \
      wget \
      xvfb \
 && if [ "$USE_LATEST_CMAKE" = true ] ; then \
      cmake_ubuntu_version=$(lsb_release -cs) ; \
      if ! wget -q --method=HEAD "https://apt.kitware.com/ubuntu/dists/$cmake_ubuntu_version/Release"; then \
        cmake_ubuntu_version="focal" ; \
      fi ; \
      wget -qO - https://apt.kitware.com/kitware-archive.sh | bash -s -- --release $cmake_ubuntu_version ; \
      apt-get update ; \
    fi \
 && apt-get -V install -y cmake \
 && if [ "$(lsb_release -sr)" = "18.04" ]; then \
       apt-get -V install -y nvidia-cuda-toolkit g++-6 ; \
     else \
       apt-get -V install -y nvidia-cuda-toolkit-gcc ; \
     fi \
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
      && make -j$(nproc) install \
      && cd ../.. \
      && rm -rf eigen-${EIGEN_MINIMUM_VERSION}/ ; \
    fi

RUN wget -qO- https://github.com/IntelRealSense/librealsense/archive/v${REALSENSE_VERSION}.tar.gz | tar xz \
 && cd librealsense-${REALSENSE_VERSION} \
 && mkdir build \
 && cd build \
 && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
 && make -j$(nproc) install \
 && cd ../.. \
 && rm -rf librealsense-${REALSENSE_VERSION}

RUN wget -qO ensenso.deb https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-${ENSENSOSDK_VERSION}-x64.deb \
 && dpkg -i ensenso.deb \
 && rm ensenso.deb
