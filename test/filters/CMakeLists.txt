#PCL_ADD_TEST(common_convolution test_convolution FILES test_convolution.cpp LINK_WITH pcl_filters pcl_common)

PCL_ADD_TEST(filters_filters test_filters
             FILES test_filters.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io pcl_sample_consensus pcl_kdtree pcl_features
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    

