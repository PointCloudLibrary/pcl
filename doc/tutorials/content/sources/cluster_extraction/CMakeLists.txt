cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL 1.2 REQUIRED)

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
