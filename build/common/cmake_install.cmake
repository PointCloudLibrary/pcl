# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/libpcl_common.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/libpcl_common.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/libpcl_common.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so"
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
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/pcl_common-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/exceptions.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/pcl_base.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/pcl_macros.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/point_cloud.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/point_representation.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/point_types.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/win32_macros.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/ModelCoefficients.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/PolygonMesh.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/Vertices.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/PointIndices.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/TextureMesh.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/../sensor_msgs" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/sensor_msgs/PointField.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/sensor_msgs/PointCloud2.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/sensor_msgs/Image.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/../std_msgs" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/std_msgs/Header.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/common" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/angles.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/bivariate_polynomial.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/centroid.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/concatenate.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/common.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/common_headers.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/distances.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/eigen.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/file_io.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/intersections.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/norms.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/piecewise_linear_function.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/point_correspondence.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/polynomial_calculations.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/poses_from_matches.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/rigid_transforms.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/time.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/time_trigger.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/transform.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/transformation_from_correspondences.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/vector_average.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/pca.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/synchronizer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/common/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/angles.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/bivariate_polynomial.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/centroid.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/common.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/file_io.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/norms.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/piecewise_linear_function.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/polynomial_calculations.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/rigid_transforms.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/pca.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/transform.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/transformation_from_correspondences.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/common/impl/vector_average.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/impl/instantiate.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/impl/point_types.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/ros" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/ros/conversions.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/ros/for_each_type.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/ros/point_traits.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/ros/register_point_struct.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/console" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/console/parse.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/console/print.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/common/include/pcl/console/time.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

