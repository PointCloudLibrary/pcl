cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(custom_executor)

find_package(PCL 1.11.1.99 COMPONENTS common filters REQUIRED)

add_executable (custom_executor custom_executor.cpp)
target_link_libraries (custom_executor ${PCL_LIBRARIES})
