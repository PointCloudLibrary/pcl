#!/bin/sh

PCL_DIR=`pwd`
BUILD_DIR=$PCL_DIR/build

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  if [ "$CC" == "clang" ]; then
    CMAKE_C_FLAGS="$CMAKE_C_FLAGS -Qunused-arguments"
    CMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS -Qunused-arguments"
  fi
fi

function before_install ()
{
  if [ "$TRAVIS_OS_NAME" == "linux" ]; then
    if [ "$CC" == "clang" ]; then
      sudo ln -s ../../bin/ccache /usr/lib/ccache/clang
      sudo ln -s ../../bin/ccache /usr/lib/ccache/clang++
    fi
  fi
}

function build_all ()
{
  # A complete build
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS="$CMAKE_C_FLAGS" -DCMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS" \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DPCL_QT_VERSION=5 \
        -DBUILD_simulation=ON \
        -DBUILD_global_tests=OFF \
        -DBUILD_examples=ON \
        -DBUILD_tools=ON \
        -DBUILD_apps=ON \
        -DBUILD_apps_3d_rec_framework=ON \
        -DBUILD_apps_cloud_composer=ON \
        -DBUILD_apps_in_hand_scanner=ON \
        -DBUILD_apps_modeler=ON \
        -DBUILD_apps_point_cloud_editor=ON \
        $PCL_DIR
  # Build
  make -j2
}

function test_all ()
{
  # A complete build
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS="$CMAKE_C_FLAGS" -DCMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS" \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DPCL_QT_VERSION=4 \
        -DBUILD_simulation=ON \
        -DBUILD_global_tests=ON \
        -DBUILD_examples=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_apps=OFF \
        $PCL_DIR
  # Build
  make -j2 tests
}

case $1 in
  before-install ) before_install;;
  build ) build_all;;
  test ) test_all;;
esac

