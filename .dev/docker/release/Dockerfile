FROM debian:testing

ENV DEBIAN_FRONTEND=noninteractive

# Add sources so we can just install build-dependencies of PCL
RUN sed -i 's/^deb \(.*\)$/deb \1\ndeb-src \1/' /etc/apt/sources.list \
 && apt update \
 && apt install -y \
    bash \
    cmake \
    dpkg-dev \
    git \
 && apt build-dep pcl -y \
 && rm -rf /var/lib/apt/lists/*

# CMake flags are from dpkg helper
# PCL config is from debian repo:
# https://salsa.debian.org/science-team/pcl/-/blob/master/debian/rules
RUN cd \
 && git clone --depth=1 https://github.com/PointCloudLibrary/pcl \
 && mkdir pcl/build \
 && cd pcl/build \
 && export DEB_BUILD_MAINT_OPTIONS='hardening=+all' \
 && export DEB_CFLAGS_MAINT_APPEND="-Wall -pedantic" \
 && export DEB_LDFLAGS_MAINT_APPEND="-Wl,--as-needed" \
 && cmake .. \
    -DCMAKE_CXX_FLAGS="`dpkg-buildflags --get CXXFLAGS`" \
    -DCMAKE_EXE_LINKER_FLAGS:STRING="`dpkg-buildflags --get LDFLAGS`" \
    -DCMAKE_SHARED_LINKER_FLAGS:STRING="`dpkg-buildflags --get LDFLAGS`" \
    -DCMAKE_SKIP_RPATH=ON -DPCL_ENABLE_SSE=OFF         \
    -DBUILD_TESTS=OFF -DBUILD_apps=ON -DBUILD_common=ON     \
    -DBUILD_examples=ON -DBUILD_features=ON -DBUILD_filters=ON  \
    -DBUILD_geometry=ON -DBUILD_global_tests=OFF -DBUILD_io=ON  \
    -DBUILD_kdtree=ON -DBUILD_keypoints=ON -DBUILD_octree=ON    \
    -DBUILD_registration=ON -DBUILD_sample_consensus=ON     \
    -DBUILD_search=ON -DBUILD_segmentation=ON -DBUILD_surface=ON    \
    -DBUILD_tools=ON -DBUILD_tracking=ON -DBUILD_visualization=ON   \
    -DBUILD_apps_cloud_composer=OFF -DBUILD_apps_modeler=ON            \
    -DBUILD_apps_point_cloud_editor=ON -DBUILD_apps_in_hand_scanner=ON \
 && make install -j2 \
 && cd \
 && rm -fr pcl
