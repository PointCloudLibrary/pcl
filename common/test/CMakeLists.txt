if(BUILD_TESTS)
    # Args: name, executable_name
    PCL_ADD_TEST(common_vector_average test_vector_average FILES test_vector_average.cpp)
    PCL_ADD_TEST(common_common test_common FILES test_common.cpp)
    PCL_ADD_TEST(common_pca test_pca FILES test_pca.cpp LINK_WITH pcl_common)
    link_ros_libs (test_pca)
endif(BUILD_TESTS)

