# Build with android-cmake!

cmake_minimum_required(VERSION 2.8)
project(AndroidPeopleLibrary)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generic.vsh.cpp
  COMMAND bin2c -p 0 -n source_generic_vsh generic.vsh > ${CMAKE_CURRENT_BINARY_DIR}/generic.vsh.cpp
  MAIN_DEPENDENCY generic.vsh
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/tree_walk.fsh.cpp
  COMMAND bin2c -p 0 -n source_tree_walk_fsh tree_walk.fsh > ${CMAKE_CURRENT_BINARY_DIR}/tree_walk.fsh.cpp
  MAIN_DEPENDENCY tree_walk.fsh
)

add_library(people_demo SHARED
  com_itseez_peopledemo_RGBDImage.cpp
  com_itseez_peopledemo_BodyPartsRecognizer.cpp
  rgbd_image.cpp body_parts_recognizer.cpp stopwatch.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/generic.vsh.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/tree_walk.fsh.cpp
)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_path(TBB_INCLUDE_DIR tbb/tbb.h)
find_library(TBB_LIBRARY tbb)
include_directories(${TBB_INCLUDE_DIR})

target_link_libraries(people_demo log EGL GLESv2 ${TBB_LIBRARY})

if(CMAKE_COMPILER_IS_GNUCXX)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -funroll-loops")
endif()
