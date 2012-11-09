#!/bin/sh

set -e

ver=1.52.0
ver_und=$(echo $ver | tr '.' '_')

rm -rf src/boost 2> /dev/null
mkdir src/boost
cd src/boost

wget -vO- \
  "http://downloads.sourceforge.net/project/boost/boost/$ver/boost_$ver_und.tar.bz2?use_mirror=autoselect" \
  | tar -xj --strip-components=1
  
patch -p1 -i ../../patches/boost.patch

./bootstrap.sh --without-libraries=python,mpi,locale --without-icu
