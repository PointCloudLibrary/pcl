#!/bin/sh

PCL_DIR=`pwd`
BUILD_DIR=$PCL_DIR/build
DOC_DIR=$BUILD_DIR/doc/doxygen/html

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build ()
{
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS=$CMAKE_C_FLAGS -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS -DPCL_ONLY_CORE_POINT_TYPES=ON -DBUILD_global_tests=OFF $PCL_DIR
  # Build
  make -j2
}

function test ()
{
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS=$CMAKE_C_FLAGS -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS -DPCL_ONLY_CORE_POINT_TYPES=ON -DBUILD_global_tests=ON -DPCL_NO_PRECOMPILE=ON $PCL_DIR
  # Build and run tests
  make pcl_filters -j3
  make test_filters
  make pcl_registration -j3
  make test_registration
  make test_registration_api
  make tests -j3
}

case $TASK in
  build ) build;;
  test ) test;;
esac
