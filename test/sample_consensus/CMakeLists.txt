PCL_ADD_TEST(sample_consensus test_sample_consensus
             FILES test_sample_consensus.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus)

PCL_ADD_TEST(sample_consensus_plane_models test_sample_consensus_plane_models
             FILES test_sample_consensus_plane_models.cpp
             LINK_WITH pcl_gtest pcl_io pcl_sample_consensus
             ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd")

PCL_ADD_TEST(sample_consensus_quadric_models test_sample_consensus_quadric_models
             FILES test_sample_consensus_quadric_models.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus)

PCL_ADD_TEST(sample_consensus_line_models test_sample_consensus_line_models
             FILES test_sample_consensus_line_models.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus)
