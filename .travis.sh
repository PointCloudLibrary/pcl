#!/bin/sh

PCL_DIR=`pwd`
BUILD_DIR=$PCL_DIR/build
DOC_DIR=$BUILD_DIR/doc/doxygen/html

TUTORIALS_DIR=$BUILD_DIR/doc/tutorials/html
ADVANCED_DIR=$BUILD_DIR/doc/advanced/html

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


function doc ()
{
  # Install sphinx
  pip3 install --user setuptools
  pip3 install --user Jinja2==2.8.1 sphinx sphinxcontrib-doxylink

  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DDOXYGEN_USE_SHORT_NAMES=OFF \
        -DSPHINX_HTML_FILE_SUFFIX=php \
        -DWITH_DOCS=ON \
        -DWITH_TUTORIALS=ON \
        $PCL_DIR

  git config --global user.email "documentation@pointclouds.org"
  git config --global user.name "PointCloudLibrary (via TravisCI)"

  cd $DOC_DIR
  git clone https://github.com/PointCloudLibrary/documentation.git .

  # Generate documentation and tutorials
  cd $BUILD_DIR
  make doc tutorials advanced

  # Do not push documentation in pull requests
  if [[ $TRAVIS_EVENT_TYPE == 'pull_request' ]] ; then exit; fi

  # update the remote url to git-ssh protocol for commit
  git remote set-url origin git@github.com:PointCloudLibrary/documentation.git

  if [ -z "$id_rsa_{1..23}" ]; then echo 'No $id_rsa_{1..23} found !' ; exit 1; fi

  echo -n $id_rsa_{1..23} >> ~/.ssh/travis_rsa_64
  base64 --decode --ignore-garbage ~/.ssh/travis_rsa_64 > ~/.ssh/id_rsa

  chmod 600 ~/.ssh/id_rsa

  echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

  # Upload to GitHub if generation succeeded
  if [[ $? == 0 ]]; then
    # Copy generated tutorials to the doc directory
    cp -r $TUTORIALS_DIR/* $DOC_DIR/tutorials
    cp -r $ADVANCED_DIR/* $DOC_DIR/advanced
    # Commit and push
    cd $DOC_DIR
    git add --all
    git commit --amend --reset-author -m "Documentation for commit $TRAVIS_COMMIT" -q
    git push --force
  else
    exit 2
  fi
}

case $1 in
  before-install ) before_install;;
  build ) build_all;;
  test ) test_all;;
  doc ) doc;;
esac

