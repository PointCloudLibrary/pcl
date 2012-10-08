#!/bin/sh

set -e

. ./normalize-eol.sh

rm -rf Sensor 2> /dev/null
git clone git://github.com/PrimeSense/Sensor.git Sensor
cd Sensor
git checkout Stable-5.1.0.41

normalize_eol

cat ../sensor-patches/series | while read patchname; do
  case "$patchname" in
    \#*)
      ;; # ignore comments
    *)
      git apply -v ../sensor-patches/$patchname
      ;;
  esac
done
