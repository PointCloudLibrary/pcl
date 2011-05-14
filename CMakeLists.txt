### ---[ PCL global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)
# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
if(WIN32)
  set(CMAKE_DEBUG_POSTFIX "-gd")
endif()

# ---[ Unix/Darwin/Windows specific flags
if(UNIX OR APPLE)
  add_definitions("-Wall -Wl,--no-undefined -fno-strict-aliasing")
else(UNIX OR APPLE)
  if(MSVC)
    add_definitions ("-W0 -DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS")
  else(MSVC)
    add_definitions("-Wall -Wl,--no-undefined -fno-strict-aliasing")
    if(PCL_SHARED_LIBS)
      add_definitions("-Wl,--export-all-symbols -Wl,--enable-auto-import")
    else(PCL_SHARED_LIBS)
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif(PCL_SHARED_LIBS)
  endif(MSVC)
endif(UNIX OR APPLE)

include(${PCL_SOURCE_DIR}/cmake/pcl_options.cmake)
include(${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake)
set(PCL_VERSION 1.0.0 CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(PCL_RESOURCES_DIR ${PCL_SOURCE_DIR}/resources)
set(PCL_POINTCLOUDS_DIR ${PCL_RESOURCES_DIR}/pointclouds)
# ---[ Windows requires DLLs (shared libraries) to be installed in the same directory as executables
if(WIN32)
  set(PCL_OUTPUT_LIB_DIR ${PCL_BINARY_DIR}/${LIB_INSTALL_DIR})
  set(PCL_OUTPUT_BIN_DIR ${PCL_BINARY_DIR}/${BIN_INSTALL_DIR})
  make_directory(${PCL_OUTPUT_LIB_DIR})
  make_directory(${PCL_OUTPUT_BIN_DIR})
  foreach(CONF ${CMAKE_CONFIGURATION_TYPES})        # For each configuration (Debug, Release, MinSizeRel... and/or anything the user chooses)
    string(TOUPPER "${CONF}" CONF)                # Go uppercase (DEBUG, RELEASE...)
    set("CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONF}" "${PCL_OUTPUT_LIB_DIR}")
    set("CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONF}" "${PCL_OUTPUT_BIN_DIR}")
    set("CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${PCL_OUTPUT_BIN_DIR}")
  endforeach()
endif()

# Configure FindPCL.cmake
configure_file("${PCL_SOURCE_DIR}/FindPCL.cmake.in"
               "${PCL_BINARY_DIR}/FindPCL.cmake" @ONLY)
# Add an "uninstall" target
configure_file("${PCL_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${PCL_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${PCL_BINARY_DIR}/uninstall_target.cmake")

###
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
###
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${is_system_dir}" STREQUAL "-1")

### ---[ Find universal dependencies
# SSE flags
include(${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()
# Boost (required)
include(${PCL_SOURCE_DIR}/cmake/pcl_find_boost.cmake)
# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
# FLANN (required)
find_package(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})
# CMinpack
find_package(CMinpack REQUIRED)
include_directories(${CMINPACK_INCLUDE_DIRS})
# OpenMP
include(${PCL_SOURCE_DIR}/cmake/pcl_openmp.cmake)
# OpenNI
find_package(OpenNI)
# Find ROS
include(${PCL_SOURCE_DIR}/cmake/pcl_find_ros.cmake)

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file(${pcl_config_h_in} ${pcl_config_h})
PCL_ADD_INCLUDES(common "" ${pcl_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

### ---[ Set up for tests
include(${PCL_SOURCE_DIR}/cmake/pcl_tests.cmake)

### ---[ Add the library subdirectories
include(${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake)
add_subdirectory(common)
add_subdirectory(octree)
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

### ---[ Documentation
add_subdirectory(doc)

### ---[ Package creation
include(${PCL_SOURCE_DIR}/cmake/pcl_cpack.cmake)
set(CPACK_PACKAGE_VERSION "${PCL_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${PCL_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${PCL_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${PCL_REVISION_VERSION}")
if(MSVC)
  if(CMAKE_CL_64)
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(win_system_name win64)
  else(CMAKE_CL_64)
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES32")
    set(win_system_name win32)
  endif(CMAKE_CL_64)
  set(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}-${PCL_VERSION}-${win_system_name}")
  # force CPACK_PACKAGE_INSTALL_REGISTRY_KEY because of a known limitation in cmake/cpack to be fixed in next releases
  # http://public.kitware.com/Bug/view.php?id=9094
  # This is to allow a 32bit and a 64bit of PCL to get installed on one system
  set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "${CPACK_PACKAGE_NAME} ${CPACK_PACKAGE_VERSION} ${win_system_name}" )
endif(MSVC)

if(CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")
  PCL_MAKE_CPACK_INPUT()
  set(CPACK_PROJECT_CONFIG_FILE "${PCL_CPACK_CFG_FILE}")
  include(CPack)
endif(CPACK_GENERATOR)
### ---[ Make a pretty picture of the dependency graph
include(${PCL_SOURCE_DIR}/cmake/dep_graph.cmake)
MAKE_DEP_GRAPH()

### ---[ Finish up
PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()

