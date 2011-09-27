#!/bin/bash

mkdir -p build
cd build
android-cmake -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN -DANDROID_LEVEL=9 -DBUILD_search=0 -DBUILD_io=0 -DBUILD_kdtree=0 -DBUILD_octree=0 -DBUILD_range_image=0 -DBUILD_sample_consensus=0 -DBUILD_segmentation=0 -DBUILD_surface=0 -DBUILD_tools=0 -DBUILD_tracking=0 -DBUILD_tutorials=0 -DBUILD_visualization=0 ..