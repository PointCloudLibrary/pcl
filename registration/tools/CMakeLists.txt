PCL_ADD_EXECUTABLE(icp ${SUBSYS_NAME} icp.cpp)
target_link_libraries(icp pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(elch ${SUBSYS_NAME} elch.cpp)
target_link_libraries(elch pcl_common pcl_io pcl_registration)
