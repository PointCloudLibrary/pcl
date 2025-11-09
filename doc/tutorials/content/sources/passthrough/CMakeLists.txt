cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(passthrough)

find_package(PCL 1.2 REQUIRED)

add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})
