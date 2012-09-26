#!/bin/sh

set -e

. ./normalize-eol.sh

rm -rf Sensor 2> /dev/null
git clone git://github.com/PrimeSense/Sensor.git Sensor
cd Sensor
git checkout Stable-5.1.0.41

normalize_eol

patch -p1 -i ../sensor.patch
