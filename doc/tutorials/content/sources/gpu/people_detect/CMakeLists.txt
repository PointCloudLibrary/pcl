cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(people_detect)

find_package(PCL 1.7 REQUIRED)

#Searching CUDA
find_package(CUDA)

#Include the FindCUDA script
include(FindCUDA)

cuda_add_executable (people_detect src/people_detect.cpp)
target_link_libraries (people_detect ${PCL_LIBRARIES})
