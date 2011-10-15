if(BUILD_TESTS)
	PCL_ADD_TEST(kdtree_search test_kdtree_search
               FILES test_kdtree.cpp
               LINK_WITH pcl_search pcl_io)
							 
  PCL_ADD_TEST(organized_neighbor_search test_organized_neighbor_search
               FILES test_organized_neighbor_search.cpp
               LINK_WITH pcl_search pcl_io)
                 
  PCL_ADD_TEST(octree octree 
               FILES test_octree.cpp
               LINK_WITH pcl_search pcl_io)
endif(BUILD_TESTS)

