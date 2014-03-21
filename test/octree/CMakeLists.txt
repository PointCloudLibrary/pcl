PCL_ADD_TEST(octree_base test_octree_base
             FILES test_octree_base.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)

PCL_ADD_TEST(octree_pointcloud test_octree_pointcloud
             FILES test_octree_pointcloud.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)

PCL_ADD_TEST(octree_pointcloud_search test_octree_pointcloud_search
             FILES test_octree_pointcloud_search.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)

PCL_ADD_TEST(octree_2buf test_octree_2buf
             FILES test_octree_2buf.cpp
             LINK_WITH pcl_gtest pcl_common pcl_octree)
