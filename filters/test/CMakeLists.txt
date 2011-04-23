if(BUILD_TESTS)
    PCL_ADD_TEST(filters_filters test_filters
                 FILES test_filters.cpp
                 LINK_WITH pcl_filters pcl_io pcl_sample_consensus pcl_kdtree
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
endif(BUILD_TESTS)

