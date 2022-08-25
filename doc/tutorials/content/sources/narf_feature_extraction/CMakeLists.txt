cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(narf_feature_extraction)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})
