cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(regression)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (regression regression.cpp)
target_link_libraries (regression ${PCL_LIBRARIES})

