    
if(BUILD_TESTS)

	PCL_ADD_TEST(kdtree_search test_kdtree_search
                 FILES test_kdtree.cpp
                 LINK_WITH pcl_search pcl_io)
                 
	PCL_ADD_TEST(organized_index test_organized_index 
                 FILES test_organized_index.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
	
     PCL_ADD_TEST(octree octree 
                FILES test_octree.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
     
	PCL_ADD_TEST(auto_tuned_search auto_tuned_search
                FILES test_auto_search.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)

#	PCL_ADD_TEST(auto_tuned_octree auto_tuned_octree
 #               FILES test_auto_search_octree.cpp
  #               LINK_WITH pcl_search pcl_io pcl_filters)
endif(BUILD_TESTS)

