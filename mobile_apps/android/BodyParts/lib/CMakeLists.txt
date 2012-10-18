# Build with android-cmake!

cmake_minimum_required(VERSION 2.8)
project(AndroidPeopleLibrary)

add_library(people_demo SHARED com_itseez_peopledemo_RGBDImage.cpp RGBDImage.cpp)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
