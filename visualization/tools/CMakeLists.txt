PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_viewer ${SUBSYS_NAME} pcd_viewer.cpp)

PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_image_viewer ${SUBSYS_NAME} image_viewer.cpp)

PCL_ADD_EXECUTABLE(pcl_timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)

PCL_ADD_EXECUTABLE(pcl_hdl_viewer_simple ${SUBSYS_NAME} hdl_viewer_simple.cpp)

PCL_ADD_EXECUTABLE(pcl_vlp_viewer ${SUBSYS_NAME} vlp_viewer.cpp)

if(WITH_OPENNI)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_image_grabber_saver ${SUBSYS_NAME} image_grabber_saver.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_image_grabber_viewer ${SUBSYS_NAME} image_grabber_viewer.cpp)

    #PCL_ADD_EXECUTABLE(pcl_openni_viewer_simple ${SUBSYS_NAME} openni_viewer_simple.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_oni_viewer ${SUBSYS_NAME} oni_viewer_simple.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_viewer ${SUBSYS_NAME} openni_viewer.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_image ${SUBSYS_NAME} openni_image.cpp)
endif()

if(WITH_OPENNI2)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni2_viewer ${SUBSYS_NAME} openni2_viewer.cpp)
endif()

if(WITH_ENSENSO)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ensenso_viewer ${SUBSYS_NAME} ensenso_viewer.cpp)
endif()

if(WITH_DAVIDSDK)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_davidsdk_viewer ${SUBSYS_NAME} davidsdk_viewer.cpp)
endif()

if(WITH_DSSDK)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_depth_sense_viewer ${SUBSYS_NAME} depth_sense_viewer.cpp)
endif()

if(WITH_RSSDK)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_real_sense_viewer ${SUBSYS_NAME} real_sense_viewer.cpp)
endif()

