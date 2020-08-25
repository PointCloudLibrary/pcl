cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(executor)

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (executor executor.cpp)
target_link_libraries (executor ${PCL_LIBRARIES})
