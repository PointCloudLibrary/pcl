    
if(BUILD_TESTS)

	PCL_ADD_TEST(search_generic_search test_search 
                 FILES test_search.cpp
                 LINK_WITH pcl_search pcl_io)
                 
	PCL_ADD_TEST(search_neighbor_search test_organized_index 
                 FILES test_organized_index.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
	
     PCL_ADD_TEST(octree octree 
                FILES test_octree.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
     
	PCL_ADD_TEST(auto_tuned_search auto_tuned_search
                FILES test_auto_search.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
	PCL_ADD_TEST(auto_tuned_octree auto_tuned_octree
                FILES test_auto_search_octree.cpp
                 LINK_WITH pcl_search pcl_io pcl_filters)
endif(BUILD_TESTS)

