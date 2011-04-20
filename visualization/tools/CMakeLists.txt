PCL_ADD_EXECUTABLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_common pcl_io pcl_kdtree pcl_range_image
    pcl_visualization ${Boost_LIBRARIES})

PCL_ADD_EXECUTABLE(pcd_viewer_simple ${SUBSYS_NAME} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization ${Boost_LIBRARIES})
    
PCL_ADD_EXECUTABLE(octree_viewer ${SUBSYS_NAME} octree_viewer.cpp)
target_link_libraries(octree_viewer pcl_common pcl_io pcl_octree pcl_visualization pcl_kdtree ${Boost_LIBRARIES})   

PCL_ADD_EXECUTABLE(timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)
target_link_libraries(timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_range_image pcl_visualization ${Boost_LIBRARIES})

if(OPENNI_FOUND)
    PCL_ADD_EXECUTABLE(pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)
    target_link_libraries(pcd_grabber_viewer pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization ${Boost_LIBRARIES})
    
    PCL_ADD_EXECUTABLE(openni_viewer_simple ${SUBSYS_NAME}
        openni_viewer_simple.cpp)
    target_link_libraries(openni_viewer_simple pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})

    PCL_ADD_EXECUTABLE(openni_viewer ${SUBSYS_NAME} openni_viewer.cpp)
    target_link_libraries(openni_viewer pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})

    PCL_ADD_EXECUTABLE(openni_change_viewer ${SUBSYS_NAME} openni_change_viewer.cpp)
    target_link_libraries(openni_change_viewer pcl_common pcl_io pcl_kdtree pcl_octree
        pcl_range_image pcl_visualization pcl_filters ${Boost_LIBRARIES})
endif(OPENNI_FOUND)

