#!/bin/sh

cd boost_1_51_0

./b2 toolset=gcc-androidarmv7a abi=aapcs variant=release debug-symbols=on threading=multi link=static -s NO_BZIP2=1 -j 4
cp -a boost ../ndk-modules/boost/include/boost
cp stage/lib/*.a ../ndk-modules/boost/lib/armeabi-v7a
ar cr ../ndk-modules/boost/lib/armeabi-v7a/dummy.a
