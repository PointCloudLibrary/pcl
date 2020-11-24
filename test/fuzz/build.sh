#!/bin/bash -eu
#
# SPDX-License-Identifier: BSD-3-Clause
#
#  Point Cloud Library (PCL) - www.pointclouds.org
#  Copyright (c) 2020-, Open Perception
#
#  All rights reserved
#


# Build PCL
cd pcl
mkdir build && cd build
cmake -DWITH_OPENGL=FALSE  -DWITH_PCAP=FALSE \
        -DWITH_VTK=FALSE -DPCL_SHARED_LIBS:BOOL=OFF ..
make

# Build fuzzers
cd ../test/fuzz

$CXX $CXXFLAGS -DPCLAPI_EXPORTS \
        -I/src/pcl/build/include -I/src/pcl/common/include \
        -I/src/pcl/dssdk/include \
        -I/src/pcl/io/include -isystem /usr/include/eigen3 \
        -O2 -g -DNDEBUG -fPIC -std=c++14 \
        -o read_fuzzer.o -c read_fuzzer.cc

$CXX $CXXFLAGS $LIB_FUZZING_ENGINE read_fuzzer.o \
        ../../build/lib/libpcl_io.a ../../build/lib/libpcl_io_ply.a \
        ../../build/lib/libpcl_common.a \
        /usr/local/lib/libboost_filesystem.a -o $OUT/read_fuzzer -lm
