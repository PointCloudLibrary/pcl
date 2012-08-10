cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test_baseline)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_baseline test_baseline.cpp)
target_link_libraries (test_baseline ${PCL_LIBRARIES})
