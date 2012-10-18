cmake_minimum_required(VERSION 2.8)
project(AndroidPeopleTools)

add_executable(treetxt2bin treetxt2bin.cpp)
add_executable(pcd2rgbd pcd2rgbd.cpp)

find_package(Boost REQUIRED)
include_directories(Boost_INCLUDE_DIRS)

find_package(PCL REQUIRED COMPONENTS io)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(pcd2rgbd ${PCL_IO_LIBRARIES})
