# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/libpcl_visualization.so.1.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/libpcl_visualization.so.1.1"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/libpcl_visualization.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so.1.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so.1.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_visualization.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/usr/lib/openmpi/lib:"
           NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake"
         "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl/PCLDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/CMakeFiles/Export/lib/pcl/PCLDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/CMakeFiles/Export/lib/pcl/PCLDepends-relwithdebinfo.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/pcl_visualization-1.1.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/visualization" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/cloud_viewer.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/histogram_visualizer.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/interactor.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/interactor_style.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/pcl_visualizer.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/point_cloud_handlers.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/keyboard_event.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/mouse_event.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/range_image_visualizer.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/visualization/common" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/actor_map.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/common.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/io.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/ren_win_interact_map.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/shapes.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/float_image_utils.h"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/image_widget_wx.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/visualization/impl" TYPE FILE FILES
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/impl/histogram_visualizer.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/impl/pcl_visualizer.hpp"
    "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/impl/point_cloud_handlers.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.1/pcl/visualization/common/impl" TYPE FILE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/include/pcl/visualization/common/impl/shapes.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/test/cmake_install.cmake")
  INCLUDE("/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

