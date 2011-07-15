cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(concatenate_fields)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (concatenate_fields concatenate_fields.cpp)
target_link_libraries (concatenate_fields ${PCL_LIBRARIES})
