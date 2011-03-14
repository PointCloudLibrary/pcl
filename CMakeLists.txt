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

# Add an "uninstall" target
configure_file("${PCL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PCL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
    "${PCL_BINARY_DIR}/uninstall_target.cmake")

# Find universal dependencies
# TODO: Write proper Find___.cmake modules for these
# Boost
include(${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
# SSE flags
include(${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
# Eigen
GET_PKG_CONFIG_INFO(eigen3 REQUIRED)
APPLY_PKG_CONFIG_DIRS(eigen3)
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_USE_NEW_STDVECTOR
    -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN
GET_PKG_CONFIG_INFO(flann REQUIRED)
APPLY_PKG_CONFIG_DIRS(flann)
# CMinpack
GET_PKG_CONFIG_INFO(cminpack REQUIRED)
APPLY_PKG_CONFIG_DIRS(cminpack)
# OpenMP
include(${PCL_SOURCE_DIR}/cmake/pcl_openmp.cmake)
PCL_FIND_OPENMP_FLAGS()

# Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file(${pcl_config_h_in} ${pcl_config_h})
PCL_ADD_INCLUDES(common "" ${pcl_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

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
add_subdirectory(test)

# Package creation
include(InstallRequiredSystemLibraries)
configure_file("${PCL_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    "${PCL_BINARY_DIR}/cpack_options.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${PCL_BINARY_DIR}/cpack_options.cmake")
include(CPack)

# Make a pretty picture of the dependency graph
include(${PCL_SOURCE_DIR}/cmake/dep_graph.cmake)
MAKE_DEP_GRAPH()

# Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

#Generate doc
find_package(Doxygen)

if(DOXYGEN_FOUND)
  configure_file(${PCL_SOURCE_DIR}/doxyfile.in
    ${PCL_BINARY_DIR}/doc/doxyfile)
  
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${PCL_BINARY_DIR}/doc/doxyfile)
endif(DOXYGEN_FOUND)
