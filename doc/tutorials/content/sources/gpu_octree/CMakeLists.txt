cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(gpu_octree)
find_package(PCL 1.11.1.99 REQUIRED)
# provides access cuda_add_executable
include(FindCUDA)
cuda_add_executable (gpu_octree gpu_octree.cpp)
target_link_libraries (gpu_octree ${PCL_LIBRARIES})
