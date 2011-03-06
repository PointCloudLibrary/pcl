PCL_ADD_EXECUTABLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_visualization)

PCL_ADD_EXECUTABLE(pcd_viewer_simple ${SUBSYS_NAME} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_visualization)

PCL_ADD_EXECUTABLE(bag_viewer_simple ${SUBSYS_NAME} bag_viewer_simple.cpp)
target_link_libraries(bag_viewer_simple pcl_visualization)

