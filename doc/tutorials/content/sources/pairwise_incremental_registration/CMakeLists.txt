cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(tuto-pairwise)

find_package(PCL 1.4 REQUIRED)

add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})
