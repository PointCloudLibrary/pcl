cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gpu_octree)

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Searching CUDA
find_package(CUDA)

#Include the FindCUDA script
include(FindCUDA)

cuda_add_executable (gpu_octree gpu_octree.cpp)
target_link_libraries (gpu_octree ${PCL_LIBRARIES})

