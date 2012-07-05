PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcl_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

#PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_image_viewer ${SUBSYS_NAME} image_viewer.cpp)
#target_link_libraries(pcl_pcd_image_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)
    
PCL_ADD_EXECUTABLE(pcl_timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)
target_link_libraries(pcl_timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_visualization)

if(OPENNI_FOUND AND BUILD_OPENNI)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)
    target_link_libraries(pcl_pcd_grabber_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)
      
    #PCL_ADD_EXECUTABLE(pcl_openni_viewer_simple ${SUBSYS_NAME} openni_viewer_simple.cpp)
    #target_link_libraries(pcl_openni_viewer_simple pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_oni_viewer ${SUBSYS_NAME} oni_viewer_simple.cpp)
    target_link_libraries(pcl_oni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_viewer ${SUBSYS_NAME} openni_viewer.cpp)
    target_link_libraries(pcl_openni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_image ${SUBSYS_NAME} openni_image.cpp)
    target_link_libraries(pcl_openni_image pcl_common pcl_io pcl_kdtree pcl_visualization)
endif()

