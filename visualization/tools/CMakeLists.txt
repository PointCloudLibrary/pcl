PCL_ADD_EXECUTABLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_common pcl_io pcl_kdtree pcl_range_image
    pcl_visualization ${Boost_LIBRARIES})

PCL_ADD_EXECUTABLE(pcd_viewer_simple ${SUBSYS_NAME} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization ${Boost_LIBRARIES})

PCL_ADD_EXECUTABLE(timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)
target_link_libraries(timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_range_image pcl_visualization ${Boost_LIBRARIES})

if(OPENNI_FOUND)
    PCL_ADD_EXECUTABLE(pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)
    target_link_libraries(pcd_grabber_viewer pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization ${Boost_LIBRARIES})
    
    PCL_ADD_EXECUTABLE(kinect_viewer_simple ${SUBSYS_NAME}
        kinect_viewer_simple.cpp)
    target_link_libraries(kinect_viewer_simple pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})

    PCL_ADD_EXECUTABLE(kinect_viewer ${SUBSYS_NAME} kinect_viewer.cpp)
    target_link_libraries(kinect_viewer pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})

    PCL_ADD_EXECUTABLE(kinect_change_viewer ${SUBSYS_NAME} kinect_change_viewer.cpp)
    target_link_libraries(kinect_change_viewer pcl_common pcl_io pcl_kdtree pcl_octree
        pcl_range_image pcl_visualization pcl_filters ${Boost_LIBRARIES})
endif(OPENNI_FOUND)

