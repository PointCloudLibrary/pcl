cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(statistical_removal)

find_package(PCL 1.2 REQUIRED)

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
