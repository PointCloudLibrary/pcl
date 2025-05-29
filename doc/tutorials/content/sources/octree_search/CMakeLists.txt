cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(octree_search)

find_package(PCL 1.2 REQUIRED)

add_executable (octree_search octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES})
