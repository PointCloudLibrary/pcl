#!/bin/sh

PCL_DIR=`pwd`
BUILD_DIR=$PCL_DIR/build
DOC_DIR=$BUILD_DIR/doc/doxygen/html

TUTORIALS_DIR=$BUILD_DIR/doc/tutorials/html
ADVANCED_DIR=$BUILD_DIR/doc/advanced/html

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

DOWNLOAD_DIR=$HOME/download

export FLANN_ROOT=$HOME/flann
export VTK_DIR=$HOME/vtk
export QHULL_ROOT=$HOME/qhull
export DOXYGEN_DIR=$HOME/doxygen

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
  cmake -DCMAKE_C_FLAGS=$CMAKE_C_FLAGS \
        -DCMAKE_CXX_FLAGS=$CMAKE_CXX_FLAGS \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DBUILD_global_tests=ON \
        -DPCL_NO_PRECOMPILE=ON \
        $PCL_DIR
  # Build and run tests
  make tests
}

function doc ()
{
  # Do not generate documentation for pull requests
  if [[ $TRAVIS_PULL_REQUEST != 'false' ]]; then exit; fi
  # Add installed doxygen to path and install sphinx
  export PATH=$DOXYGEN_DIR/bin:$PATH
  pip install --user sphinx sphinxcontrib-doxylink
  # Configure
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake -DDOXYGEN_USE_SHORT_NAMES=OFF \
        -DSPHINX_HTML_FILE_SUFFIX=php \
        -DWITH_DOCS=ON \
        -DWITH_TUTORIALS=ON \
        $PCL_DIR

  git config --global user.email "documentation@pointclouds.org"
  git config --global user.name "PointCloudLibrary (via TravisCI)"

  if [ -z "$id_rsa_{1..23}" ]; then echo 'No $id_rsa_{1..23} found !' ; exit 1; fi

  echo -n $id_rsa_{1..23} >> ~/.ssh/travis_rsa_64
  base64 --decode --ignore-garbage ~/.ssh/travis_rsa_64 > ~/.ssh/id_rsa

  chmod 600 ~/.ssh/id_rsa

  echo -e "Host github.com\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

  cd $DOC_DIR
  git clone git@github.com:PointCloudLibrary/documentation.git .

  # Generate documentation and tutorials
  cd $BUILD_DIR
  make doc tutorials advanced

  # Upload to GitHub if generation succeeded
  if [[ $? == 0 ]]; then
    # Copy generated tutorials to the doc directory
    cp -r $TUTORIALS_DIR/* $DOC_DIR/tutorials
    cp -r $ADVANCED_DIR/* $DOC_DIR/advanced
    # Commit and push
    cd $DOC_DIR
    git add --all
    git commit --amend -m "Documentation for commit $TRAVIS_COMMIT" -q
    git push --force
  else
    exit 2
  fi
}

function install_flann()
{
  local pkg_ver=1.8.4
  local pkg_file="flann-${pkg_ver}-src"
  local pkg_url="http://people.cs.ubc.ca/~mariusm/uploads/FLANN/${pkg_file}.zip"
  local pkg_md5sum="a0ecd46be2ee11a68d2a7d9c6b4ce701"
  local FLANN_DIR=$HOME/flann
  local config=$FLANN_DIR/include/flann/config.h
  echo "Installing FLANN ${pkg_ver}"
  if [[ -d $FLANN_DIR ]]; then
    if [[ -e ${config} ]]; then
      local version=`grep -Po "(?<=FLANN_VERSION_ \").*(?=\")" ${config}`
      if [[ "$version" = "$pkg_ver" ]]; then
        local modified=`stat -c %y ${config} | cut -d ' ' -f1`
        echo " > Found cached installation of FLANN"
        echo " > Version ${pkg_ver}, built on ${modified}"
        return 0
      fi
    fi
  fi
  download ${pkg_url} ${pkg_md5sum}
  if [[ $? -ne 0 ]]; then
    return $?
  fi
  unzip -qq pkg
  cd ${pkg_file}
  mkdir build && cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$FLANN_DIR \
    -DBUILD_MATLAB_BINDINGS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_CUDA_LIB=OFF \
    -DBUILD_C_BINDINGS=OFF \
    -DUSE_OPENMP=OFF
  make -j2 && make install && touch ${config}
  return $?
}

function install_vtk()
{
  local pkg_ver=5.10.1
  local pkg_file="vtk-${pkg_ver}"
  local pkg_url="http://www.vtk.org/files/release/${pkg_ver:0:4}/${pkg_file}.tar.gz"
  local pkg_md5sum="264b0052e65bd6571a84727113508789"
  local VTK_DIR=$HOME/vtk
  local config=$VTK_DIR/include/vtk-${pkg_ver:0:4}/vtkConfigure.h
  echo "Installing VTK ${pkg_ver}"
  if [[ -d $VTK_DIR ]]; then
    if [[ -e ${config} ]]; then
      local version=`grep -Po "(?<=VTK_VERSION \").*(?=\")" ${config}`
      if [[ "$version" = "$pkg_ver" ]]; then
        local modified=`stat -c %y ${config} | cut -d ' ' -f1`
        echo " > Found cached installation of VTK"
        echo " > Version ${pkg_ver}, built on ${modified}"
        return 0
      fi
    fi
  fi
  download ${pkg_url} ${pkg_md5sum}
  if [[ $? -ne 0 ]]; then
    return $?
  fi
  tar xzf pkg
  cd "VTK${pkg_ver}"
  mkdir build && cd build
  cmake .. \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$VTK_DIR \
    -DBUILD_DOCUMENTATION=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DVTK_USE_BOOST=ON \
    -DVTK_USE_CHARTS=ON \
    -DVTK_USE_VIEWS=ON \
    -DVTK_USE_RENDERING=ON \
    -DVTK_USE_CHEMISTRY=OFF \
    -DVTK_USE_HYBRID=OFF \
    -DVTK_USE_PARALLEL=OFF \
    -DVTK_USE_PATENTED=OFF \
    -DVTK_USE_INFOVIS=ON \
    -DVTK_USE_GL2PS=OFF \
    -DVTK_USE_MYSQL=OFF \
    -DVTK_USE_FFMPEG_ENCODER=OFF \
    -DVTK_USE_TEXT_ANALYSIS=OFF \
    -DVTK_WRAP_JAVA=OFF \
    -DVTK_WRAP_PYTHON=OFF \
    -DVTK_WRAP_TCL=OFF \
    -DVTK_USE_QT=OFF \
    -DVTK_USE_GUISUPPORT=OFF \
    -DVTK_USE_SYSTEM_ZLIB=ON \
    -DCMAKE_CXX_FLAGS="-D__STDC_CONSTANT_MACROS"
  make -j2 && make install && touch ${config}
  return $?
}

function install_qhull()
{
  local pkg_ver=2012.1
  local pkg_file="qhull-${pkg_ver}"
  local pkg_url="http://www.qhull.org/download/${pkg_file}-src.tgz"
  local pkg_md5sum="d0f978c0d8dfb2e919caefa56ea2953c"
  local QHULL_DIR=$HOME/qhull
  local announce=$QHULL_DIR/share/doc/qhull/Announce.txt
  echo "Installing QHull ${pkg_ver}"
  if [[ -d $QHULL_DIR ]]; then
    if [[ -e ${announce} ]]; then
      local version=`grep -Po "(?<=Qhull )[0-9.]*(?= )" ${announce}`
      if [[ "$version" = "$pkg_ver" ]]; then
        local modified=`stat -c %y ${announce} | cut -d ' ' -f1`
        echo " > Found cached installation of QHull"
        echo " > Version ${pkg_ver}, built on ${modified}"
        return 0
      fi
    fi
  fi
  download ${pkg_url} ${pkg_md5sum}
  if [[ $? -ne 0 ]]; then
    return $?
  fi
  tar xzf pkg
  cd ${pkg_file}
  mkdir -p build && cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS=-fPIC \
    -DCMAKE_C_FLAGS=-fPIC \
    -DCMAKE_INSTALL_PREFIX=$QHULL_DIR
  make -j2 && make install && touch ${announce}
  return $?
}

function install_doxygen()
{
  local pkg_ver=1.8.9.1
  local pkg_file="doxygen-${pkg_ver}"
  local pkg_url="http://ftp.stack.nl/pub/users/dimitri/${pkg_file}.src.tar.gz"
  local pkg_md5sum="3d1a5c26bef358c10a3894f356a69fbc"
  local DOXYGEN_EXE=$DOXYGEN_DIR/bin/doxygen
  echo "Installing Doxygen ${pkg_ver}"
  if [[ -d $DOXYGEN_DIR ]]; then
    if [[ -e $DOXYGEN_EXE ]]; then
      local version=`$DOXYGEN_EXE --version`
      if [[ "$version" = "$pkg_ver" ]]; then
        local modified=`stat -c %y $DOXYGEN_EXE | cut -d ' ' -f1`
        echo " > Found cached installation of Doxygen"
        echo " > Version ${pkg_ver}, built on ${modified}"
        return 0
      fi
    fi
  fi
  download ${pkg_url} ${pkg_md5sum}
  if [[ $? -ne 0 ]]; then
    return $?
  fi
  tar xzf pkg
  cd ${pkg_file}
  ./configure --prefix $DOXYGEN_DIR
  make -j2 && make install
  return $?
}

function install_dependencies()
{
  install_flann
  install_vtk
  install_qhull
  install_doxygen
}

function download()
{
  mkdir -p $DOWNLOAD_DIR && cd $DOWNLOAD_DIR && rm -rf *
  wget --output-document=pkg $1
  if [[ $? -ne 0 ]]; then
    return $?
  fi
  if [[ $# -ge 2 ]]; then
    echo "$2  pkg" > "md5"
    md5sum -c "md5" --quiet --status
    if [[ $? -ne 0 ]]; then
      echo "MD5 mismatch"
      return 1
    fi
  fi
  return 0
}

case $1 in
  install ) install_dependencies;;
  build ) build;;
  test ) test;;
  doc ) doc;;
esac
