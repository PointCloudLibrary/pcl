cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(extract_indices)

find_package(PCL 1.2 REQUIRED)

add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})
