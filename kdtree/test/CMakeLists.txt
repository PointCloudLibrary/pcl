if(BUILD_TESTS)
    add_executable(test_kdtree test_kdtree.cpp)
    PCL_ADD_OPENMP_FLAGS(test_kdtree)
    target_link_libraries(test_kdtree pcl_kdtree ${GTEST_BOTH_LIBRARIES}
        ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
    PCL_LINK_OPENMP(test_kdtree)
    add_test(kdtree_kdtree test_kdtree)
endif(BUILD_TESTS)

