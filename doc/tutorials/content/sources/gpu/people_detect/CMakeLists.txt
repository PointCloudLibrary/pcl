cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(people_detect)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
find_package(CUDA)

#Include the FindCUDA script
include(FindCUDA)

cuda_add_executable (people_detect src/people_detect.cpp)
target_link_libraries (people_detect ${PCL_LIBRARIES})
