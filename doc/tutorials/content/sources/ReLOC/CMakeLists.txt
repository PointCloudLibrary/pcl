cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ReLOC)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ReLOC ReLOC.cpp)
target_link_libraries (ReLOC ${PCL_LIBRARIES})


