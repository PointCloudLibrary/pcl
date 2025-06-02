cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(narf_keypoint_extraction)

find_package(PCL 1.3 REQUIRED)

add_executable (narf_keypoint_extraction narf_keypoint_extraction.cpp)
target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})
