cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(resampling)

find_package(PCL 1.2 REQUIRED)

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})
