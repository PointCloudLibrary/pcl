if(BUILD_TESTS)
    add_executable(test_surface test_surface.cpp)
    PCL_ADD_OPENMP_FLAGS(test_surface)
    target_link_libraries(test_surface pcl_io pcl_kdtree pcl_surface
        pcl_features ${GTEST_BOTH_LIBRARIES})
    PCL_LINK_OPENMP(test_surface)
    add_test(surface_surface test_surface)

    add_executable(test_mls test_mls.cpp)
    target_link_libraries(test_mls pcl_io pcl_kdtree pcl_surface
        ${GTEST_BOTH_LIBRARIES})
    add_test(surface_mls test_mls)
endif(BUILD_TESTS)

