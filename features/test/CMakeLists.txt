if(BUILD_TESTS)
    add_executable (test_narf test_narf.cpp)
    PCL_ADD_OPENMP_FLAGS(test_narf)
    target_link_libraries (test_narf pcl_features ${GTEST_BOTH_LIBRARIES}
        flann)
    PCL_LINK_OPENMP(test_narf)
    add_test(features_narf test_narf)
endif(BUILD_TESTS)

