# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/visualization/tools

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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/pcd_viewer")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/pcd_viewer_simple")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_viewer_simple")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/octree_viewer")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/octree_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/timed_trigger_test")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/timed_trigger_test")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/pcd_grabber_viewer")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcd_grabber_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/openni_viewer_simple")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer_simple")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/oni_viewer_simple")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/oni_viewer_simple")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/openni_viewer")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/openni_change_viewer")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_change_viewer")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization/tools/pcl_stream_compression")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_stream_compression")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "visualization")

