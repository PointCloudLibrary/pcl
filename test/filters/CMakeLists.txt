#PCL_ADD_TEST(common_convolution test_convolution FILES test_convolution.cpp LINK_WITH pcl_filters pcl_common)

PCL_ADD_TEST(filters_filters test_filters
             FILES test_filters.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io pcl_sample_consensus pcl_kdtree pcl_features pcl_segmentation
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

PCL_ADD_TEST(filters_sampling test_filters_sampling
             FILES test_sampling.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io pcl_kdtree pcl_features
             ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd" "${PCL_SOURCE_DIR}/test/cturtle.pcd")

PCL_ADD_TEST(filters_bilateral test_filters_bilateral
             FILES test_bilateral.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io pcl_kdtree
             ARGUMENTS "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

    

