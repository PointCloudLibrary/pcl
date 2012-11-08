#!/bin/sh

set -e

ver=41_20120718oss
relpath=77/188/4.1

rm -rf src/tbb 2> /dev/null

mkdir src/tbb
cd src/tbb

wget -vO- \
  "http://threadingbuildingblocks.org/uploads/$relpath/tbb${ver}_src.tgz" \
  | tar -xz --strip-components=1
  
patch -p1 -i ../../patches/tbb.patch
