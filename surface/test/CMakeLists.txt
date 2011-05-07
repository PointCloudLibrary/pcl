if(BUILD_TESTS)
    if(QHULL_FOUND)
        PCL_ADD_TEST(surface_surface test_surface
                     FILES test_surface.cpp
                     LINK_WITH pcl_io pcl_kdtree pcl_surface pcl_features
                     ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    endif(QHULL_FOUND)

    PCL_ADD_TEST(surface_reconstruction surface_example 
                 FILES surface_example.cpp
                 LINK_WITH pcl_common pcl_io pcl_kdtree pcl_surface
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
endif(BUILD_TESTS)

