# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/libpcl_registration.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/libpcl_registration.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/libpcl_registration.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/pcl_registration-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/registration" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_estimation.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection_distance.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection_one_to_one.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection_reciprocal.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection_sample_consensus.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_rejection_trimmed.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_sorting.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/correspondence_types.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/ia_ransac.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/icp.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/icp_nl.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/pyramid_feature_matching.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/registration.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/transformation_estimation.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/transformation_estimation_svd.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/transforms.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/registration/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_estimation.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_rejection_distance.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_rejection_one_to_one.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_rejection_reciprocal.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_rejection_trimmed.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/correspondence_types.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/ia_ransac.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/icp.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/icp_nl.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/pyramid_feature_matching.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/registration.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/transformation_estimation_svd.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/registration/include/pcl/registration/impl/transforms.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "registration")

