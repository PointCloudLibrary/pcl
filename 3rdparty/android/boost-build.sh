#!/bin/sh

(cd src/boost &&
  ./b2 toolset=gcc-androidarmv7a abi=aapcs variant=release debug-symbols=on \
  threading=multi link=static -s NO_BZIP2=1 -j 4)
  
rm -r ndk-modules/boost/include/* ndk-modules/boost/lib/armeabi-v7a/* 2>/dev/null

cp -a src/boost/boost ndk-modules/boost/include
cp src/boost/stage/lib/*.a ndk-modules/boost/lib/armeabi-v7a
ar cr ndk-modules/boost/lib/armeabi-v7a/dummy.a
