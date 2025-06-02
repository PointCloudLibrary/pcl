cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(octree_change_detection)

find_package(PCL 1.2 REQUIRED)

add_executable (octree_change_detection octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})
