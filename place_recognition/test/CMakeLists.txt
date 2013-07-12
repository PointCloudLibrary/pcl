#########################################################
#  * Test demo: Place Recognition
#  * Copyright (c) 2013, Intelligent Robotics Lab, DLUT
#  * Author: Qinghua Li, Yan Zhuang
#  * Created on: July 07, 2013
#########################################################

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(place_recognition)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#############################
# PCL #######################
#############################
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

#############################
# Qt ########################
#############################
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

#############################
# OpenCV ####################
#############################
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})

##############################################################################
# Binaries
##############################################################################
add_executable(place_recognition test/test1.cpp src/bearing_angle.cpp src/scene_cognition.cpp)
add_executable(create_database test/test2.cpp src/bearing_angle.cpp src/scene_cognition.cpp)

#link libraries
target_link_libraries(place_recognition opencv_core opencv_highgui opencv_imgproc ${PCL_LIBRARIES})
target_link_libraries(create_database opencv_core opencv_highgui opencv_imgproc ${PCL_LIBRARIES})
