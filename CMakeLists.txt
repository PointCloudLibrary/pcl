cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include(${PCL_SOURCE_DIR}/cmake/pcl_options.cmake)
include(${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PCL_VERSION 2.0.0 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(PCL_RESOURCES_DIR ${PCL_SOURCE_DIR}/resources)
set(PCL_POINTCLOUDS_DIR ${PCL_RESOURCES_DIR}/pointclouds)

# Configure FindPCL.cmake
configure_file("${PCL_SOURCE_DIR}/FindPCL.cmake.in"
    "${PCL_BINARY_DIR}/FindPCL.cmake" @ONLY)
# Add an "uninstall" target
configure_file("${PCL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PCL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
    "${PCL_BINARY_DIR}/uninstall_target.cmake")

#This is definitely not where it should, just for test before moving
if(UNIX OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
add_definitions("-Wall -Wl,--no-undefined -fno-strict-aliasing")
else(UNIX OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
add_definitions ("-W0 -DEIGEN_USE_NEW_STDVECTOR -DEIGEN2_SUPPORT")
endif(UNIX OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")

# Find universal dependencies
# Find ROS
include(${PCL_SOURCE_DIR}/cmake/pcl_find_ros.cmake)
# Boost
include(${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-DBOOST_DISABLE_ASSERTS)
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
# SSE flags
include(${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
# Eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_USE_NEW_STDVECTOR
    -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN
find_package(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})
# CMinpack
find_package(CMinpack REQUIRED)
include_directories(${CMINPACK_INCLUDE_DIRS})
# OpenMP
include(${PCL_SOURCE_DIR}/cmake/pcl_openmp.cmake)
#OpenNI
find_package(OpenNI)
if(OPENNI_FOUND)
    include_directories(${OPENNI_INCLUDE_DIRS})
    message(STATUS "Did find OpenNI after all.")
    #message(STATUS "OpenNI include ${OPENNI_INCLUDE_DIR}")
    #message(STATUS "OpenNI libs ${OPENNI_LIBRARY}")
endif(OPENNI_FOUND)

# Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file(${pcl_config_h_in} ${pcl_config_h})
PCL_ADD_INCLUDES(common "" ${pcl_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

# Set up for tests
include(${PCL_SOURCE_DIR}/cmake/pcl_tests.cmake)

# Add the library subdirectories
include(${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake)
add_subdirectory(common)
add_subdirectory(io)
add_subdirectory(kdtree)
add_subdirectory(range_image)
add_subdirectory(features)
add_subdirectory(sample_consensus)
add_subdirectory(keypoints)
add_subdirectory(filters)
add_subdirectory(registration)
add_subdirectory(segmentation)
add_subdirectory(surface)
add_subdirectory(visualization)
add_subdirectory(octree)
add_subdirectory(test)

# Documentation
add_subdirectory(doc)

# Package creation
include(InstallRequiredSystemLibraries)
include(${PCL_SOURCE_DIR}/cmake/pcl_cpack.cmake)
PCL_MAKE_CPACK_INPUT()
set(CPACK_PROJECT_CONFIG_FILE "${PCL_CPACK_CFG_FILE}")
include(CPack)

# Make a pretty picture of the dependency graph
include(${PCL_SOURCE_DIR}/cmake/dep_graph.cmake)
MAKE_DEP_GRAPH()

# Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

