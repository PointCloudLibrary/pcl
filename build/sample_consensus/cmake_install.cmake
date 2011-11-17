# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/libpcl_sample_consensus.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/libpcl_sample_consensus.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/libpcl_sample_consensus.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/pcl_sample_consensus-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/sample_consensus" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/lmeds.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/method_types.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/mlesac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/model_types.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/msac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/prosac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/ransac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/rmsac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/rransac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_circle.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_cylinder.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_line.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_normal_parallel_plane.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_normal_plane.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_line.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_plane.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_perpendicular_plane.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_plane.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_registration.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/sac_model_sphere.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/sample_consensus/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/lmeds.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/mlesac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/msac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/prosac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/ransac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/rmsac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/rransac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_line.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_plane.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_line.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_plane.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/sample_consensus/include/pcl/sample_consensus/impl/sac_model_sphere.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "sample_consensus")

