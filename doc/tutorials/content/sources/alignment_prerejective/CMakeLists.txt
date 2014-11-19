cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(alignment_prerejective)

find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (alignment_prerejective alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})
