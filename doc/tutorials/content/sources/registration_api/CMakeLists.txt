cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registration_api)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (example1 example1.cpp)
#target_link_libraries (example1 ${PCL_LIBRARIES})

add_executable (example2 example2.cpp)
target_link_libraries (example2 ${PCL_LIBRARIES})

#add_executable (example3 example3.cpp)
#target_link_libraries (example3 ${PCL_LIBRARIES})
