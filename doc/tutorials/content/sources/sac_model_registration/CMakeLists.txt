cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sac_model_registration)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sac_model_registration sac_model_registration.cpp)
target_link_libraries (sac_model_registration ${PCL_LIBRARIES})
