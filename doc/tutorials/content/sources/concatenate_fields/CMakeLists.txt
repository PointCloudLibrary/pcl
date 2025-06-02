cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(concatenate_fields)

find_package(PCL 1.2 REQUIRED)

add_executable (concatenate_fields concatenate_fields.cpp)
target_link_libraries (concatenate_fields ${PCL_LIBRARIES})
