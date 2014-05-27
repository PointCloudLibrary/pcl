#!/bin/sh

PCL_DIR=`pwd`
BUILD_DIR=$PCL_DIR/build
DOC_DIR=$BUILD_DIR/doc/doxygen/html

TUTORIALS_DIR=$BUILD_DIR/doc/tutorials/html
ADVANCED_DIR=$BUILD_DIR/doc/advanced/html

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build ()
{
  case $CC in
    clang ) build_clang;;
    gcc ) build_gcc;;
  esac
}

function build_clang ()
{
  # A complete build
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS=$CMAKE_C_FLAGS -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DBUILD_global_tests=OFF \
        $PCL_DIR
  # Build
  make -j2
}

function build_gcc ()
{
  # A reduced build, only pcl_common
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DCMAKE_C_FLAGS=$CMAKE_C_FLAGS -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DBUILD_2d=OFF \
        -DBUILD_features=OFF \
        -DBUILD_filters=OFF \
        -DBUILD_geometry=OFF \
        -DBUILD_global_tests=OFF \
        -DBUILD_io=OFF \
        -DBUILD_kdtree=OFF \
        -DBUILD_keypoints=OFF \
        -DBUILD_ml=OFF \
        -DBUILD_octree=OFF \
        -DBUILD_outofcore=OFF \
        -DBUILD_people=OFF \
        -DBUILD_recognition=OFF \
        -DBUILD_registration=OFF \
        -DBUILD_sample_consensus=OFF \
        -DBUILD_search=OFF \
        -DBUILD_segmentation=OFF \
        -DBUILD_stereo=OFF \
        -DBUILD_surface=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_tracking=OFF \
        -DBUILD_visualization=OFF \
        $PCL_DIR
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

function doc ()
{
  # Do not generate documentation for pull requests
  if [[ $TRAVIS_PULL_REQUEST != 'false' ]]; then exit; fi
  # Install doxygen and sphinx
  sudo apt-get install doxygen doxygen-latex graphviz python-pip
  sudo pip install sphinx sphinxcontrib-doxylink
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake $PCL_DIR

  git config --global user.email "documentation@pointclouds.org"
  git config --global user.name "PointCloudLibrary (via TravisCI)"

  if [ -z "$id_rsa_{1..23}" ]; then echo 'No $id_rsa_{1..23} found !' ; exit 1; fi

  echo -n $id_rsa_{1..23} >> ~/.ssh/travis_rsa_64
  base64 --decode --ignore-garbage ~/.ssh/travis_rsa_64 > ~/.ssh/id_rsa

  chmod 600 ~/.ssh/id_rsa

  echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

  cd $DOC_DIR
  git clone git@github.com:PointCloudLibrary/documentation.git .

  # Generate documentation
  cd $BUILD_DIR
  make doc
  cd $DOC_DIR
  git commit -a -m "adding $TRAVIS_COMMIT"
  git push

  # Generate tutorials
  cd $BUILD_DIR
  make tutorials
  # upload to github...
}

case $TASK in
  build ) build;;
  test ) test;;
  doc ) doc;;
esac
