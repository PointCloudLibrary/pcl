# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/include/pcl/pcl_config.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "common")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl-1.1" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/PCLConfig.cmake"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/PCLConfigVersion.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/keypoints/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/registration/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/segmentation/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/surface/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/test/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/doc/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
