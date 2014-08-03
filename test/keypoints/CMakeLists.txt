PCL_ADD_TEST(keypoints_general test_keypoints
                 FILES test_keypoints.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_filters pcl_keypoints
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/cturtle.pcd")

PCL_ADD_TEST(keypoints_iss_3d test_iss_3d
             FILES test_iss_3d.cpp
             LINK_WITH pcl_gtest pcl_keypoints pcl_io
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
