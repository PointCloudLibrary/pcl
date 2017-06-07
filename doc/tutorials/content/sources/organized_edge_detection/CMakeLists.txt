cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(organized_edge_detection)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (organized_edge_detection organized_edge_detection.cpp)
target_link_libraries (organized_edge_detection ${PCL_LIBRARIES})