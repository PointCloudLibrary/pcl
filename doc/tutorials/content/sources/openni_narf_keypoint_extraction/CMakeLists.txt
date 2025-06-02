cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(openni_narf_keypoint_extraction)

find_package(PCL 1.3 REQUIRED)

add_executable (openni_narf_keypoint_extraction openni_narf_keypoint_extraction.cpp)
target_link_libraries (openni_narf_keypoint_extraction ${PCL_LIBRARIES})

