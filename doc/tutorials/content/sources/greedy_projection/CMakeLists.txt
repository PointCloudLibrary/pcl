cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(greedy_projection)

find_package(PCL 1.2 REQUIRED)

add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})
