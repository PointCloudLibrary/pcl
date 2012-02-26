# Args: name, executable_name
PCL_ADD_TEST(common_test_wrappers test_wrappers FILES test_wrappers.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_test_macros test_macros FILES test_macros.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_vector_average test_vector_average FILES test_vector_average.cpp)
PCL_ADD_TEST(common_common test_common FILES test_common.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_pca test_pca FILES test_pca.cpp LINK_WITH pcl_common)
#PCL_ADD_TEST(common_spring test_spring FILES test_spring.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_gaussian test_gaussian FILES test_gaussian.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_operators test_operators FILES test_operators.cpp LINK_WITH pcl_common)
#PCL_ADD_TEST(common_convolution test_convolution FILES test_convolution.cpp LINK_WITH pcl_common)
PCL_ADD_TEST(common_eigen test_eigen FILES test_eigen.cpp LINK_WITH pcl_features)

