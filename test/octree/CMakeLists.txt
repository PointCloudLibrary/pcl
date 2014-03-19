PCL_ADD_TEST(octree test_octree
             FILES test_octree.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)

PCL_ADD_TEST(octree_pointcloud_search test_octree_pointcloud_search
             FILES test_octree_pointcloud_search.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)
