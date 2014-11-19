PCL_ADD_TEST (kdtree_kdtree test_kdtree
              FILES test_kdtree.cpp
              LINK_WITH pcl_gtest pcl_kdtree pcl_io
              ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd" "${PCL_SOURCE_DIR}/test/kdtree/kdtree_unit_test_results.xml")
