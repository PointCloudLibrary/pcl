#!/bin/bash -eu
# Copyright 2020 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
################################################################################

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
