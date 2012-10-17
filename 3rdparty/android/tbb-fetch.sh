#!/bin/sh

set -e

ver=41_20120718oss
relpath=77/188/4.1

rm -rf tbb$ver 2> /dev/null

wget -vO- \
  "http://threadingbuildingblocks.org/uploads/$relpath/tbb${ver}_src.tgz" \
  | tar -xz
  
cd tbb$ver
patch -p1 -i ../tbb.patch
