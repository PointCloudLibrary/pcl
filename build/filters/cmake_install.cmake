# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/libpcl_filters.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/libpcl_filters.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/libpcl_filters.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/pcl_filters-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/filters" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/conditional_removal.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/extract_indices.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/filter.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/passthrough.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/project_inliers.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/radius_outlier_removal.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/statistical_outlier_removal.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/voxel_grid.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/filters/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/conditional_removal.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/extract_indices.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/filter.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/passthrough.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/project_inliers.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/radius_outlier_removal.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/statistical_outlier_removal.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/filters/include/pcl/filters/impl/voxel_grid.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "filters")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/test/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/tools/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

