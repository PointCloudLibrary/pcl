if(BUILD_TESTS)
    add_executable(test_filters test_filters.cpp)
    target_link_libraries(test_filters pcl_filters pcl_io pcl_sample_consensus
        pcl_kdtree ${GTEST_BOTH_LIBRARIES})
    add_test(filters_filters test_filters ${PCL_SOURCE_DIR}/test/bun0.pcd)
endif(BUILD_TESTS)

