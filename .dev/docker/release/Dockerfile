FROM debian:testing

ARG VTK_VERSION=7
ENV DEBIAN_FRONTEND=noninteractive

ARG PCL_INDEX_SIGNED=true
ARG PCL_INDEX_SIZE=32

# Add sources so we can just install build-dependencies of PCL
RUN sed -i 's/^deb \(.*\)$/deb \1\ndeb-src \1/' /etc/apt/sources.list \
 && apt update \
 && apt install -y \
    bash \
    cmake \
    dpkg-dev \
    git \
    g++ \
    libboost-date-time-dev \
    libboost-filesystem-dev \
    libboost-iostreams-dev \
    libeigen3-dev \
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
    wget \
 && rm -rf /var/lib/apt/lists/*

# CMake flags are from dpkg helper
# PCL config is from debian repo:
# https://salsa.debian.org/science-team/pcl/-/blob/master/debian/rules
# MinSizeRel is used for the CI and should have no impact on releaseability
RUN cd \
 && git clone --depth=1 https://github.com/PointCloudLibrary/pcl \
 && mkdir pcl/build \
 && cd pcl/build \
 && export DEB_BUILD_MAINT_OPTIONS='hardening=+all' \
 && export DEB_CFLAGS_MAINT_APPEND="-Wall -pedantic" \
 && export DEB_LDFLAGS_MAINT_APPEND="-Wl,--as-needed" \
 && cmake .. \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DCMAKE_CXX_FLAGS:STRING="`dpkg-buildflags --get CXXFLAGS`"          \
    -DCMAKE_EXE_LINKER_FLAGS:STRING="`dpkg-buildflags --get LDFLAGS`"    \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING="`dpkg-buildflags --get LDFLAGS`" \
    -DCMAKE_SKIP_RPATH=ON -DPCL_ENABLE_SSE=OFF                          \
    -DBUILD_TESTS=OFF -DBUILD_apps=ON -DBUILD_common=ON                 \
    -DBUILD_examples=ON -DBUILD_features=ON -DBUILD_filters=ON           \
    -DBUILD_geometry=ON -DBUILD_global_tests=OFF -DBUILD_io=ON          \
    -DBUILD_kdtree=ON -DBUILD_keypoints=ON -DBUILD_octree=ON            \
    -DBUILD_registration=ON -DBUILD_sample_consensus=ON                 \
    -DBUILD_search=ON -DBUILD_segmentation=ON -DBUILD_surface=ON        \
    -DBUILD_tools=ON -DBUILD_tracking=ON -DBUILD_visualization=ON       \
    -DBUILD_apps_cloud_composer=OFF -DBUILD_apps_modeler=ON             \
    -DBUILD_apps_point_cloud_editor=ON -DBUILD_apps_in_hand_scanner=ON  \
    -DPCL_INDEX_SIGNED=${PCL_INDEX_SIGNED} \
    -DPCL_INDEX_SIZE=${PCL_INDEX_SIZE} \
 && make install -j2 \
 && cd \
 && rm -fr pcl
