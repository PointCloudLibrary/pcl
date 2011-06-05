cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(octree_change_detection)

find_package(PCL 1.0 REQUIRED COMPONENTS common octree)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (octree_change_detection octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_COMMON_LIBRARIES} ${PCL_OCTREE_LIBRARIES})
