if(BUILD_TESTS)
    PCL_ADD_TEST(kdtree_kdtree test_kdtree 
                 FILES test_kdtree.cpp
                 LINK_WITH pcl_kdtree)
    link_ros_libs (test_kdtree)
endif(BUILD_TESTS)

