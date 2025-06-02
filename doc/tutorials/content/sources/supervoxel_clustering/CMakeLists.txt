cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(supervoxel_clustering)

find_package(PCL 1.8 REQUIRED)

add_executable (supervoxel_clustering supervoxel_clustering.cpp)
target_link_libraries (supervoxel_clustering ${PCL_LIBRARIES})
