cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(planar_segmentation)

find_package(PCL 1.2 REQUIRED)

add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
