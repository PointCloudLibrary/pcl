# Install script for directory: /wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/apps

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/nn_classification_example")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/nn_classification_example")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_voxel_grid")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_voxel_grid")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_passthrough")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_passthrough")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_stream_compression")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/octree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/usr/lib/openmpi/lib:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_stream_compression")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_planar_segmentation")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/segmentation:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_planar_segmentation")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_ii_normal_estimation")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/segmentation:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/surface:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_ii_normal_estimation")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_3d_convex_hull")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/segmentation:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/surface:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_convex_hull")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull"
         RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/apps/openni_3d_concave_hull")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull"
         OLD_RPATH "/usr/lib/vtk-5.2:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/common:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/io:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/filters:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/visualization:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/segmentation:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/sample_consensus:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/features:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/surface:/usr/lib/openmpi/lib:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/range_image:/wg/stor2a/mdixon/pcl/branches/pcl-1.1.x/build/kdtree:"
         NEW_RPATH "/usr/local/lib:/usr/lib/vtk-5.2:/usr/lib/openmpi/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/openni_3d_concave_hull")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "apps")

