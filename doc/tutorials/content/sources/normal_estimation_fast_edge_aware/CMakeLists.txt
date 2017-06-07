cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

project(normal_estimation_fast_edge_aware)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal_estimation_fast_edge_aware normal_estimation_fast_edge_aware.cpp)
target_link_libraries (normal_estimation_fast_edge_aware ${PCL_LIBRARIES})