cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(don_segmentation)

find_package(PCL 1.7 REQUIRED)

add_executable (don_segmentation don_segmentation.cpp)
target_link_libraries (don_segmentation ${PCL_LIBRARIES})
