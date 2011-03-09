cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include(${PROJECT_SOURCE_DIR}/cmake/pcl_options.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PCL_VERSION 2.0.0 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

# Add an "uninstall" target
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

# Find universal dependencies
# Boost
include(${PROJECT_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
# SSE flags
include(${PROJECT_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
# Eigen
GET_PKG_CONFIG_INFO(eigen3 REQUIRED)
APPLY_PKG_CONFIG_DIRS(eigen3)
# FLANN
GET_PKG_CONFIG_INFO(flann REQUIRED)
APPLY_PKG_CONFIG_DIRS(flann)
# CMinpack
GET_PKG_CONFIG_INFO(cminpack REQUIRED)
APPLY_PKG_CONFIG_DIRS(cminpack)
# OpenMP
include(${PROJECT_SOURCE_DIR}/cmake/pcl_openmp.cmake)
PCL_FIND_OPENMP_FLAGS()

# Add the library subdirectories
include(${PROJECT_SOURCE_DIR}/cmake/pcl_targets.cmake)
add_subdirectory(common)
add_subdirectory(io)
add_subdirectory(kdtree)
add_subdirectory(sample_consensus)
add_subdirectory(features)
add_subdirectory(keypoints)
add_subdirectory(range_image)
add_subdirectory(filters)
add_subdirectory(registration)
add_subdirectory(segmentation)
add_subdirectory(surface)
add_subdirectory(visualization)

# Package creation
include(InstallRequiredSystemLibraries)
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    "${PROJECT_BINARY_DIR}/cpack_options.cmake" @ONLY)
set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_BINARY_DIR}/cpack_options.cmake")
include(CPack)

# Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

