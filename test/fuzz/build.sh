#!/bin/bash -eu
#
# SPDX-License-Identifier: BSD-3-Clause
#
#  Point Cloud Library (PCL) - www.pointclouds.org
#  Copyright (c) 2020-, Open Perception
#
#  All rights reserved
#

# This script is run inside OSS-fuzz's docker image
# and builds PCL's fuzzers to run continuously 
# through OSS-fuzz.
# In the OSS-fuzz  docker image PCL is located at $SRC/pcl.

# The Dockerfile that builds PCL can be found here:
# (url pending)

# Build PCL
cd pcl
mkdir build && cd build
pwd
cmake -DWITH_OPENGL=FALSE \
      -DWITH_PCAP=FALSE \
      -DWITH_VTK=FALSE \
      -DPCL_SHARED_LIBS:BOOL=OFF \
      -DWITH_LIBUSB=FALSE \
      -DWITH_QT=FALSE \
      -DBUILD_features=OFF \
      -DBUILD_filters=OFF \
      -DBUILD_geometry=OFF \
      -DBUILD_kdtree=OFF \
      -DBUILD_keypoints=OFF \
      -DBUILD_ml=OFF \
      -DBUILD_outofcore=OFF \
      -DBUILD_people=OFF \
      -DBUILD_recognition=OFF \
      -DBUILD_registration=OFF \
      -DBUILD_sample_consensus=OFF \
      -DBUILD_search=OFF \
      -DBUILD_segmentation=OFF \
      -DBUILD_stereo=OFF \
      -DBUILD_surface=OFF \
      -DBUILD_tracking=OFF \
      -DBUILD_visualization=OFF \
      ..

make -j$(nproc)

# Build fuzzers
cd ../test/fuzz

$CXX $CXXFLAGS -DPCLAPI_EXPORTS \
        -I/src/pcl/build/include -I/src/pcl/common/include \
        -I/src/pcl/dssdk/include \
        -I/src/pcl/io/include -isystem /usr/include/eigen3 \
        -O2 -g -DNDEBUG -fPIC -std=c++14 \
        -o ply_reader_fuzzer.o -c ply_reader_fuzzer.cpp

$CXX $CXXFLAGS $LIB_FUZZING_ENGINE ply_reader_fuzzer.o \
        ../../build/lib/libpcl_io.a ../../build/lib/libpcl_io_ply.a \
        ../../build/lib/libpcl_common.a \
        /usr/local/lib/libboost_filesystem.a -o $OUT/ply_reader_fuzzer -lm

zip $OUT/ply_reader_fuzzer_seed_corpus.zip $SRC/pcl/test/cube.ply
