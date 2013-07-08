
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(place_recognition)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Required 3rd party
##############################################################################

#############################
# PCL #######################
#############################
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#############################
# Qt ########################
#############################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)

SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# This is necessary as all ui files etc will get dumped in the bottom of then binary directory.
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR})
 	
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})
SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})

#############################
# OpenCV ####################
#############################
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS} external/)
#link_directories(/usr/lib)

##############################################################################
# Some QT Magic (MOC Creation)
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp include/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources to Compile
##############################################################################
SET(ADDITIONAL_SOURCES src/*.cpp)

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS ${ADDITIONAL_SOURCES})

##############################################################################
# Binaries
##############################################################################
add_executable(place_recognition ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

#link libraries
target_link_libraries(place_recognition GLU opencv_core opencv_highgui opencv_imgproc ${PCL_LIBRARIES} ${QT_LIBRARIES})
