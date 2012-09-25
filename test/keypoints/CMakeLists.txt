PCL_ADD_TEST(pcl_keypoints test_iss_3d
             FILES test_iss_3d.cpp
             LINK_WITH pcl_gtest pcl_keypoints pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
