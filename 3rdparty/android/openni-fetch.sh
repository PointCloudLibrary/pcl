#!/bin/sh

set -e

. ./functions.shinc

will_normalize_eol

rm -rf OpenNI Sensor 2> /dev/null

echo 'Downloading OpenNI...'

mkdir OpenNI
cd OpenNI
wget -vO- http://github.com/OpenNI/OpenNI/tarball/Stable-1.5.2.23 | tar -xz --strip-components=1

echo 'Normalizing EOL characters to LF...'

normalize_eol

echo 'Patching...'

apply_patch_series ../openni-patches/series

rm -rv Wrappers/OpenNI.java/res

echo 'Updating Android project...'

android update lib-project -p Wrappers/OpenNI.java

cd ..

echo 'Downloading Sensor...'

mkdir Sensor
cd Sensor
wget -vO- http://github.com/PrimeSense/Sensor/tarball/Unstable-5.1.0.41 | tar -xz --strip-components=1

echo 'Normalizing EOL characters to LF...'

normalize_eol

echo 'Patching...'

apply_patch_series ../sensor-patches/series

cd ..

echo 'Done.'
