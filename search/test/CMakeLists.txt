    
if(BUILD_TESTS)
	PCL_ADD_TEST(search_generic_search test_search 
                 FILES test_search.cpp
                 LINK_WITH pcl_search)
                 
	PCL_ADD_TEST(search_neighbor_search test_organized_neighbor_search 
                 FILES test_organized_neighbor_search.cpp
                 LINK_WITH pcl_search)
endif(BUILD_TESTS)

