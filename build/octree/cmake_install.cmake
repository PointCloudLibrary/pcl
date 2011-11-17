# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/libpcl_octree.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/libpcl_octree.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/libpcl_octree.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:::::::::::::::"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/pcl_octree-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/octree" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_base.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_impl.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_nodes.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_density.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_occupancy.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_singlepoint.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_pointvector.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_changedetector.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud_voxelcentroid.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_pointcloud.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree2buf_base.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/octree_lowmemory_base.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/octree/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/impl/octree_base.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/impl/octree_pointcloud.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/impl/octree2buf_base.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/octree/include/pcl/octree/impl/octree_lowmemory_base.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "octree")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

