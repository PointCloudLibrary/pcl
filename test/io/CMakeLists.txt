set(SUBSYS_NAME tests_io)
set(SUBSYS_DESC "Point cloud library io module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS io)
set(OPT_DEPS visualization)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(io_io test_io
              FILES test_io.cpp
              LINK_WITH pcl_gtest pcl_io)

PCL_ADD_TEST(io_split test_split
              FILES test_split.cpp
              LINK_WITH pcl_gtest pcl_io)

PCL_ADD_TEST(io_iterators test_iterators
              FILES test_iterators.cpp
              LINK_WITH pcl_gtest pcl_io)

PCL_ADD_TEST(compression_range_coder test_range_coder
          FILES test_range_coder.cpp
          LINK_WITH pcl_gtest pcl_io)

PCL_ADD_TEST (io_grabbers test_grabbers
              FILES test_grabbers.cpp
              LINK_WITH pcl_gtest pcl_io
              ARGUMENTS "${PCL_SOURCE_DIR}/test/grabber_sequences")

PCL_ADD_TEST(io_ply_io test_ply_io
              FILES test_ply_io.cpp
              LINK_WITH pcl_gtest pcl_io)

# Uses VTK readers to verify
if(VTK_FOUND AND NOT ANDROID)
  include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
  PCL_ADD_TEST (io_ply_mesh_io test_ply_mesh_io
                FILES test_ply_mesh_io.cpp
                LINK_WITH pcl_gtest pcl_io
                ARGUMENTS "${PCL_SOURCE_DIR}/test/tum_rabbit.vtk")
endif()

PCL_ADD_TEST(point_cloud_image_extractors test_point_cloud_image_extractors
             FILES test_point_cloud_image_extractors.cpp
             LINK_WITH pcl_gtest pcl_io)

PCL_ADD_TEST(buffers test_buffers
             FILES test_buffers.cpp
             LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(io_octree_compression test_octree_compression
        FILES test_octree_compression.cpp
        LINK_WITH pcl_gtest pcl_common pcl_io pcl_octree
        ARGUMENTS "${PCL_SOURCE_DIR}/test/milk_color.pcd")

PCL_ADD_TEST (io_tim_grabber test_tim_grabber
              FILES test_tim_grabber.cpp
              LINK_WITH pcl_gtest pcl_io
              ARGUMENTS "${PCL_SOURCE_DIR}/test/io/tim_sequences")
