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
# and builds PCL's fuzzers to run continuously through OSS-fuzz.
# In the OSS-fuzz  docker image PCL is located at $SRC/pcl

# The Dockerfile that builds PCL can be found here:
# https://github.com/google/oss-fuzz/tree/master/projects/pcl/Dockerfile

# Build PCL
mkdir -p build && cd build

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
      $SRC/pcl

make -j$(nproc)

# Build fuzzers
cd $SRC/pcl/test/fuzz

# TODO: create a CMake/Makefile
$CXX $CXXFLAGS -DPCLAPI_EXPORTS \
        -O2 -g -DNDEBUG -fPIC -std=c++14 \
        -o ply_reader_fuzzer.o -c ply_reader_fuzzer.cpp
$CXX $CXXFLAGS -DPCLAPI_EXPORTS \
        -I/src/pcl/build/include -I/src/pcl/common/include \
        -I/src/pcl/dssdk/include \
        -I/src/pcl/io/include -isystem /usr/include/eigen3 \
        -O2 -g -DNDEBUG -fPIC -std=c++14 \
        -o io_wrappers.o -c io_wrappers.cpp

$CXX $CXXFLAGS $LIB_FUZZING_ENGINE ply_reader_fuzzer.o io_wrappers.o \
        $SRC/build/lib/libpcl_io.a $SRC/build/lib/libpcl_io_ply.a \
        $SRC/build/lib/libpcl_common.a \
        /usr/local/lib/libboost_filesystem.a -o $OUT/ply_reader_fuzzer -lm

zip -r $OUT/ply_reader_fuzzer_seed_corpus.zip ./corpus
