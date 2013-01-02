PCL_ADD_TEST(kdtree_search test_kdtree_search
              FILES test_kdtree.cpp
              LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree)

  PCL_ADD_TEST(flann_search test_flann_search
               FILES test_flann_search.cpp
               LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree)
  PCL_ADD_TEST(organized_neighbor test_organized_search
               FILES test_organized.cpp
               LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree)

PCL_ADD_TEST(octree_search test_octree_search
              FILES test_octree.cpp
              LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree)
