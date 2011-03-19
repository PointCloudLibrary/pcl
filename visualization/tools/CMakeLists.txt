PCL_ADD_EXECUTABLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_common pcl_io pcl_kdtree pcl_range_image
    pcl_visualization ${Boost_LIBRARIES})

PCL_ADD_EXECUTABLE(pcd_viewer_simple ${SUBSYS_NAME} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization ${Boost_LIBRARIES})

#PCL_ADD_EXECUTABLE(bag_viewer_simple ${SUBSYS_NAME} bag_viewer_simple.cpp)
#target_link_libraries(bag_viewer_simple pcl_visualization)

if(OPENNI_FOUND)
    PCL_ADD_EXECUTABLE(kinect_viewer_simple ${SUBSYS_NAME}
        kinect_viewer_simple.cpp)
    target_link_libraries(kinect_viewer_simple pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})

    PCL_ADD_EXECUTABLE(kinect_viewer ${SUBSYS_NAME} kinect_viewer.cpp)
    target_link_libraries(kinect_viewer pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization ${Boost_LIBRARIES})
endif(OPENNI_FOUND)

