#!/bin/sh

set -e

ver=1.51.0
ver_und=$(echo $ver | tr '.' '_')

rm -rf boost_$ver_und 2> /dev/null

wget -vO- \
  "http://downloads.sourceforge.net/project/boost/boost/$ver/boost_$ver_und.tar.bz2?use_mirror=autoselect" \
  | tar -xj
  
cd boost_$ver_und
patch -p1 -i ../boost.patch

./bootstrap.sh --without-libraries=python,mpi,locale --without-icu
