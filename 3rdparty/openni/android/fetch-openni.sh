#!/bin/sh

set -e

. ./normalize-eol.sh

rm -rf OpenNI 2> /dev/null
git clone git://github.com/OpenNI/OpenNI.git OpenNI
cd OpenNI
git checkout Stable-1.5.2.23

normalize_eol

cat ../openni-patches/series | while read patchname; do
  patch -p1 -i ../openni-patches/$patchname
done
