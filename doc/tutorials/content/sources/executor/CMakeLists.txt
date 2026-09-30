cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(executor)

find_package(PCL 1.11.1.99 COMPONENTS common filters visualization REQUIRED)

add_executable (executor executor.cpp)
target_link_libraries (executor ${PCL_LIBRARIES})
