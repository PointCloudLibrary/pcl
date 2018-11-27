cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(spherical_voxel_grid)

find_package(PCL 1.8.1.99 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (spherical_voxel_grid spherical_voxel_grid.cpp)
target_link_libraries (spherical_voxel_grid ${PCL_LIBRARIES})
