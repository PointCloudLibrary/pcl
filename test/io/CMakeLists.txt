if(OPENNI_FOUND)
    PCL_ADD_TEST(io_io test_io
                  FILES test_io.cpp
                  LINK_WITH pcl_gtest pcl_io)
endif(OPENNI_FOUND)
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

PCL_ADD_TEST (octree_compression test_octree_compression
               FILES test_octree_compression.cpp
               LINK_WITH pcl_gtest pcl_io pcl_common pcl_octree)

# Uses VTK readers to verify            
if (VTK_FOUND AND NOT ANDROID)
  PCL_ADD_TEST (io_ply_mesh_io test_ply_mesh_io
                FILES test_ply_mesh_io.cpp
                LINK_WITH pcl_gtest pcl_io
                ARGUMENTS "${PCL_SOURCE_DIR}/test/tum_rabbit.vtk")
endif ()

PCL_ADD_TEST(point_cloud_image_extractors test_point_cloud_image_extractors
             FILES test_point_cloud_image_extractors.cpp
             LINK_WITH pcl_gtest pcl_io)
