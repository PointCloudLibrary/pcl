if(BUILD_TESTS)
    add_executable(test_pcl_vis test.cpp)
    target_link_libraries(test_pcl_vis pcl_common pcl_io
        pcl_kdtree pcl_range_image pcl_visualization)
    add_test(vis_test test_pcl_vis)

    # Depends on filters
    #add_executable(test_geometry test_geometry.cpp)
    #target_link_libraries(test_geometry pcl_common pcl_features pcl_filters pcl_io
        #pcl_kdtree pcl_range_image pcl_visualization)
    #add_test(vis_test_geometry test_geometry)

    add_executable(test_shapes test_shapes.cpp)
    target_link_libraries(test_shapes pcl_common pcl_io
        pcl_kdtree pcl_range_image pcl_visualization)
    add_test(vis_test_shapes test_shapes)
endif(BUILD_TESTS)

