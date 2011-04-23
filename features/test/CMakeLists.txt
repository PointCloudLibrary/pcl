if(BUILD_TESTS)
    PCL_ADD_TEST(features_narf test_narf 
                 FILES test_narf.cpp
                 LINK_WITH pcl_features ${FLANN_LIBRARIES})
endif(BUILD_TESTS)

