#!/bin/sh

set -e

. ./functions.shinc

will_normalize_eol

rm -rf src/OpenNI src/Sensor 2> /dev/null

echo 'Downloading OpenNI...'

mkdir src/OpenNI
cd src/OpenNI
wget -vO- http://github.com/OpenNI/OpenNI/tarball/Stable-1.5.2.23 | tar -xz --strip-components=1

echo 'Normalizing EOL characters to LF...'

normalize_eol

echo 'Patching...'

apply_patch_series ../../patches/openni/series

rm -rv Wrappers/OpenNI.java/res

echo 'Updating Android project...'

android update lib-project -p Wrappers/OpenNI.java --target android-14

cd ../..

echo 'Downloading Sensor...'

mkdir src/Sensor
cd src/Sensor
wget -vO- http://github.com/PrimeSense/Sensor/tarball/Unstable-5.1.0.41 | tar -xz --strip-components=1

echo 'Normalizing EOL characters to LF...'

normalize_eol

echo 'Patching...'

apply_patch_series ../../patches/sensor/series

cd ../..

echo 'Done.'
