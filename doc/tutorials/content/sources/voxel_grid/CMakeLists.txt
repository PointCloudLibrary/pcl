cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(voxel_grid)

find_package(PCL 1.2 REQUIRED)

add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})
