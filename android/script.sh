#!/bin/bash

mkdir -p build
cd build
android-cmake -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN -DANDROID_LEVEL=9 -DBUILD_search=OFF -DBUILD_io=OFF -DBUILD_kdtree=OFF -DBUILD_octree=OFF -DBUILD_range_image=OFF -DBUILD_sample_consensus=OFF -DBUILD_segmentation=OFF -DBUILD_surface=OFF -DBUILD_tools=OFF -DBUILD_tracking=OFF -DBUILD_tutorials=OFF -DBUILD_visualization=OFF ..


