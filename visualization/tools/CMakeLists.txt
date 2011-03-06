PCL_ADD_EXECUTABLE(pcd_viewer ${VIS_COMPONENT} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_visualization)

PCL_ADD_EXECUTABLE(pcd_viewer_simple ${VIS_COMPONENT} pcd_viewer_simple.cpp)
target_link_libraries(pcd_viewer_simple pcl_visualization)

PCL_ADD_EXECUTABLE(bag_viewer_simple ${VIS_COMPONENT} bag_viewer_simple.cpp)
target_link_libraries(bag_viewer_simple pcl_visualization)

