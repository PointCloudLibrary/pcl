PCL_ADD_EXECUTABLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_common pcl_io pcl_kdtree pcl_range_image
    pcl_visualization)
    
PCL_ADD_EXECUTABLE(pcd_viewer_simple ${SUBSYS_NAME} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization)
    
PCL_ADD_EXECUTABLE(timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)
target_link_libraries(timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_range_image pcl_visualization)


#PCL_ADD_EXECUTABLE(image_viewer_example ${SUBSYS_NAME} image_viewer_example.cpp)
#target_link_libraries(image_viewer_example pcl_visualization)

#PCL_ADD_EXECUTABLE(image_viewer_window ${SUBSYS_NAME} image_viewer_window.cpp)
#target_link_libraries(image_viewer_window pcl_io pcl_visualization)

if(OPENNI_FOUND AND BUILD_OPENNI)

    PCL_ADD_EXECUTABLE(pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)
    target_link_libraries(pcd_grabber_viewer pcl_common pcl_io pcl_kdtree
    pcl_range_image pcl_visualization)
      
    #PCL_ADD_EXECUTABLE(openni_viewer_simple ${SUBSYS_NAME}
    #    openni_viewer_simple.cpp)
    #target_link_libraries(openni_viewer_simple pcl_common pcl_io pcl_kdtree
    #    pcl_range_image pcl_visualization)

    PCL_ADD_EXECUTABLE(oni_viewer_simple ${SUBSYS_NAME}
        oni_viewer_simple.cpp)
    target_link_libraries(oni_viewer_simple pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization)

    PCL_ADD_EXECUTABLE(openni_viewer ${SUBSYS_NAME} openni_viewer.cpp)
    target_link_libraries(openni_viewer pcl_common pcl_io pcl_kdtree pcl_surface
        pcl_range_image pcl_visualization)

    PCL_ADD_EXECUTABLE(openni_image ${SUBSYS_NAME} openni_image.cpp)
    target_link_libraries(openni_image pcl_common pcl_io pcl_kdtree
        pcl_range_image pcl_visualization)
endif()

