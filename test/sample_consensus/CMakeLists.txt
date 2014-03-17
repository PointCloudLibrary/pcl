PCL_ADD_TEST(a_sample_consensus_test test_sample_consensus
             FILES test_sample_consensus.cpp
             LINK_WITH pcl_gtest pcl_io pcl_sample_consensus pcl_kdtree pcl_features
             ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd")
