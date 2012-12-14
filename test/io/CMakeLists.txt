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
              ARGUMENTS ${PCL_SOURCE_DIR}/test/grabber_sequences)
