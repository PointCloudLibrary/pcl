#!/bin/sh

set -e

rm -rf config libusb 2>&1

git clone git://git.savannah.gnu.org/config.git config
git clone git://git.libusb.org/libusb.git libusb

cd libusb
git checkout 1.0.9
patch -p1 -i ../libusb-android.patch

libtoolize --copy --force && aclocal && autoheader && autoconf && automake -a -c

cp ../config/config.guess ../config/config.sub .
