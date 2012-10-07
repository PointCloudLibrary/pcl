cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(people_detect)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
FIND_PACKAGE(CUDA)

#Include the FindCUDA script
INCLUDE(FindCUDA)

cuda_add_executable (people_detect src/people_detect.cpp)
target_link_libraries (people_detect ${PCL_LIBRARIES})
