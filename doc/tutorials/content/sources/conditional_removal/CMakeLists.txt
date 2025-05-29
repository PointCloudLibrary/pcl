cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(conditional_removal)

find_package(PCL 1.2 REQUIRED)

add_executable (conditional_removal conditional_removal.cpp)
target_link_libraries (conditional_removal ${PCL_LIBRARIES})
