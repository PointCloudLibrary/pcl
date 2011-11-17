# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_features.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_features.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_features.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/pcl_features-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/features" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/cvfh.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/feature.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/fpfh.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/fpfh_omp.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/integral_image_2d.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/integral_image_normal.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/intensity_gradient.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/intensity_spin.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/moment_invariants.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/multiscale_feature_persistence.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/narf.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/narf_descriptor.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/normal_3d.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/normal_3d_omp.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/pfh.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/ppf.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/shot.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/shot_omp.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/principal_curvatures.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/rift.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/rsd.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/statistical_multiscale_interest_region_extraction.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/vfh.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/features/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/cvfh.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/feature.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/fpfh.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/fpfh_omp.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/integral_image_2d.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/integral_image_normal.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/intensity_gradient.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/intensity_spin.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/moment_invariants.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/multiscale_feature_persistence.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/narf.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/normal_3d.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/normal_3d_omp.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/pfh.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/ppf.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/shot.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/shot_omp.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/principal_curvatures.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/rift.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/rsd.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/statistical_multiscale_interest_region_extraction.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/vfh.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_range_image_border_extractor.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_range_image_border_extractor.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/libpcl_range_image_border_extractor.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_range_image_border_extractor.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/pcl_range_image_border_extractor-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/features" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/boundary.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/range_image_border_extractor.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/features/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/boundary.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/features/include/pcl/features/impl/range_image_border_extractor.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "features")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/test/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

