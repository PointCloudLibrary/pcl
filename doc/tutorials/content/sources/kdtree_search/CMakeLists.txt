cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(kdtree_search)

find_package(PCL 1.2 REQUIRED)

add_executable (kdtree_search kdtree_search.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})
