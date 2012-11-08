#!/bin/sh

set -e

rm -rf src/config src/libusb 2>&1

git clone git://git.savannah.gnu.org/config.git src/config
git clone git://git.libusb.org/libusb.git src/libusb

cd src/libusb
git checkout 1.0.9
patch -p1 -i ../../patches/libusb.patch

libtoolize --copy --force && aclocal && autoheader && autoconf && automake -a -c

cp ../config/config.guess ../config/config.sub .

(cd android/LibusbSupport && android update lib-project -p .)
